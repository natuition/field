from typing import List, Dict, Tuple, Optional, Union

import json
from dataclasses import dataclass
from math import cos, pi, sin
from pathlib import Path
from typing import Any, Iterable

import numpy as np
from pyproj import CRS, Transformer
from scipy.optimize import differential_evolution
from shapely import affinity
from shapely.geometry import (
    LineString,
    MultiPolygon,
    Point,
    Polygon,
    mapping,
    shape,
)
from shapely.geometry.base import BaseGeometry
from shapely.ops import transform
from shapely.validation import make_valid


@dataclass(frozen=True)
class InscribedRectangleResult:
    # Rectangle retourné en coordonnées GeoJSON (EPSG:4326 par défaut).
    rectangle: Polygon

    # Mesures calculées dans le repère métrique local.
    area: float
    width: float
    height: float
    angle_deg: float

    # Centre dans le système de coordonnées de sortie.
    center_x: float
    center_y: float

    metric_crs: str
    optimizer_success: bool
    optimizer_message: str
    debug_directory: Path

    corners: List[Tuple[float, float]]

    def to_geojson_feature(self) -> Dict[str, Any]:
        return {
            "type": "Feature",
            "properties": {
                "area_m2": self.area,
                "width_m": self.width,
                "height_m": self.height,
                "angle_deg": self.angle_deg,
                "metric_crs": self.metric_crs,
                "optimizer_success": self.optimizer_success,
            },
            "geometry": mapping(self.rectangle),
        }


def parse_geojson_linestring(
    raw_geojson: Union[str, Dict[str, Any]],
) -> LineString:
    """Convertit une géométrie ou Feature GeoJSON brute en LineString."""
    if isinstance(raw_geojson, str):
        try:
            geojson_object = json.loads(raw_geojson)
        except json.JSONDecodeError as exc:
            raise ValueError(
                f"Le GeoJSON fourni n'est pas un JSON valide : {exc}"
            ) from exc
    elif isinstance(raw_geojson, dict):
        geojson_object = raw_geojson
    else:
        raise TypeError(
            "Le GeoJSON doit être une chaîne JSON ou un dictionnaire Python."
        )

    object_type = geojson_object.get("type")

    if object_type == "Feature":
        geometry_object = geojson_object.get("geometry")
        if not isinstance(geometry_object, dict):
            raise ValueError("La Feature GeoJSON ne contient pas de géométrie valide.")
    elif object_type == "LineString":
        geometry_object = geojson_object
    else:
        raise ValueError(
            "Le GeoJSON doit être une LineString ou une Feature "
            "contenant une LineString."
        )

    if geometry_object.get("type") != "LineString":
        raise ValueError(
            "La géométrie doit être de type LineString, "
            f"pas {geometry_object.get('type')!r}."
        )

    geometry = shape(geometry_object)

    if not isinstance(geometry, LineString):
        raise ValueError("Impossible de convertir le GeoJSON en LineString.")
    if geometry.is_empty:
        raise ValueError("La LineString GeoJSON est vide.")
    if len(geometry.coords) < 3:
        raise ValueError("La LineString doit contenir au moins trois coordonnées.")

    return geometry


def feature(
    geometry: BaseGeometry,
    role: str,
    **properties: Any,
) -> Dict[str, Any]:
    return {
        "type": "Feature",
        "geometry": mapping(geometry),
        "properties": {
            "role": role,
            **properties,
        },
    }


class NullDebugger:
    """Implémentation vide de GeoJSONDebugger, pour désactiver le débogage."""

    def write(
        self,
        name: str,
        features: Iterable[Dict[str, Any]],
        *,
        explanation: str,
    ) -> None: ...


class GeoJSONDebugger:
    """
    Écrit des FeatureCollection GeoJSON numérotées.

    Tous les fichiers produits par l'algorithme principal sont reprojetés
    dans le CRS de sortie, donc directement visualisables ensemble.
    """

    def __init__(self, output_directory: Union[str, Path]) -> None:
        self.output_directory = Path(output_directory)
        self.output_directory.mkdir(parents=True, exist_ok=True)
        self.step = 0

    def write(
        self,
        name: str,
        features: Iterable[Dict[str, Any]],
        *,
        explanation: str,
    ) -> None:
        path = self.output_directory / f"{self.step:02d}_{name}.geojson"
        collection = {
            "type": "FeatureCollection",
            "name": name,
            "properties": {
                "step": self.step,
                "explanation": explanation,
            },
            "features": list(features),
        }
        path.write_text(
            json.dumps(collection, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        self.step += 1


def close_linestring(line: LineString) -> LineString:
    coordinates = list(line.coords)

    if coordinates[0] != coordinates[-1]:
        coordinates.append(coordinates[0])

    return LineString(coordinates)


def select_largest_polygon(geometry: BaseGeometry) -> Polygon:
    if isinstance(geometry, Polygon):
        return geometry

    if isinstance(geometry, MultiPolygon):
        return max(geometry.geoms, key=lambda polygon: polygon.area)

    raise ValueError(
        f"La réparation produit une géométrie {geometry.geom_type}, "
        "sans Polygon exploitable."
    )


def determine_local_metric_crs(
    geographic_geometry: BaseGeometry,
    geographic_crs: CRS,
) -> CRS:
    """
    Choisit un CRS métrique local.

    Pour une entrée EPSG:4326, une zone UTM est choisie à partir du centre.
    Pour un autre CRS géographique, le centre est d'abord converti en WGS84.
    """
    wgs84 = CRS.from_epsg(4326)

    if geographic_crs != wgs84:
        to_wgs84 = Transformer.from_crs(
            geographic_crs,
            wgs84,
            always_xy=True,
        )
        center_wgs84 = transform(
            to_wgs84.transform,
            geographic_geometry.centroid,
        )
    else:
        center_wgs84 = geographic_geometry.centroid

    longitude = center_wgs84.x
    latitude = center_wgs84.y

    if not (-180.0 <= longitude <= 180.0):
        raise ValueError(f"Longitude invalide : {longitude}")
    if not (-80.0 <= latitude <= 84.0):
        raise ValueError(
            "La sélection UTM automatique nécessite une latitude entre -80° et 84°."
        )

    zone_number = int((longitude + 180.0) // 6.0) + 1
    zone_number = min(max(zone_number, 1), 60)

    epsg = 32600 + zone_number if latitude >= 0.0 else 32700 + zone_number
    return CRS.from_epsg(epsg)


def project_geometry(
    geometry: BaseGeometry,
    transformer: Transformer,
) -> BaseGeometry:
    return transform(transformer.transform, geometry)


def normalize_geometry(
    geometry: BaseGeometry,
    offset_x: float,
    offset_y: float,
    scale: float,
) -> BaseGeometry:
    """
    x_norm = (x - offset_x) / scale
    y_norm = (y - offset_y) / scale
    """

    def normalize_coordinates(
        x: Any,
        y: Any,
        z: Any = None,
    ) -> Any:
        normalized_x = (np.asarray(x) - offset_x) / scale
        normalized_y = (np.asarray(y) - offset_y) / scale

        if z is None:
            return normalized_x, normalized_y
        return normalized_x, normalized_y, z

    return transform(normalize_coordinates, geometry)


def denormalize_geometry(
    geometry: BaseGeometry,
    offset_x: float,
    offset_y: float,
    scale: float,
) -> BaseGeometry:
    """
    Inverse exacte :
    x = x_norm * scale + offset_x
    y = y_norm * scale + offset_y
    """

    def denormalize_coordinates(
        x: Any,
        y: Any,
        z: Any = None,
    ) -> Any:
        original_x = np.asarray(x) * scale + offset_x
        original_y = np.asarray(y) * scale + offset_y

        if z is None:
            return original_x, original_y
        return original_x, original_y, z

    return transform(denormalize_coordinates, geometry)


def normalize_polygon(
    polygon: Polygon,
) -> Tuple[Polygon, float, float, float]:
    min_x, min_y, max_x, max_y = polygon.bounds
    scale = max(max_x - min_x, max_y - min_y)

    if scale <= 0:
        raise ValueError("La géométrie possède une étendue nulle.")

    normalized = normalize_geometry(
        polygon,
        offset_x=min_x,
        offset_y=min_y,
        scale=scale,
    )

    if not isinstance(normalized, Polygon):
        raise RuntimeError("La normalisation n'a pas produit un Polygon.")

    return normalized, min_x, min_y, scale


def rectangle_from_parameters(
    center_x: float,
    center_y: float,
    half_width: float,
    half_height: float,
    angle_rad: float,
) -> Polygon:
    ux = cos(angle_rad)
    uy = sin(angle_rad)
    vx = -uy
    vy = ux

    return Polygon(
        [
            (
                center_x + half_width * ux + half_height * vx,
                center_y + half_width * uy + half_height * vy,
            ),
            (
                center_x + half_width * ux - half_height * vx,
                center_y + half_width * uy - half_height * vy,
            ),
            (
                center_x - half_width * ux - half_height * vx,
                center_y - half_width * uy - half_height * vy,
            ),
            (
                center_x - half_width * ux + half_height * vx,
                center_y - half_width * uy + half_height * vy,
            ),
        ]
    )


def shrink_until_covered(
    rectangle: Polygon,
    zone: Polygon,
    iterations: int = 60,
) -> Tuple[Polygon, float]:
    if zone.covers(rectangle):
        return rectangle, 1.0

    center = rectangle.centroid
    lower = 0.0
    upper = 1.0
    best: Union[Polygon, None] = None
    best_factor = 0.0

    for _ in range(iterations):
        factor = (lower + upper) / 2.0
        candidate = affinity.scale(
            rectangle,
            xfact=factor,
            yfact=factor,
            origin=(center.x, center.y),
        )

        if zone.covers(candidate):
            best = candidate
            best_factor = factor
            lower = factor
        else:
            upper = factor

    if best is None or best.is_empty:
        raise RuntimeError("Impossible de produire un rectangle inscrit valide.")

    return best, best_factor


def rectangle_properties(
    rectangle: Polygon,
) -> Tuple[float, float, float, float, float]:
    coordinates = list(rectangle.exterior.coords)[:4]

    p0 = np.asarray(coordinates[0], dtype=float)
    p1 = np.asarray(coordinates[1], dtype=float)
    p2 = np.asarray(coordinates[2], dtype=float)

    side_1 = p1 - p0
    side_2 = p2 - p1

    length_1 = float(np.linalg.norm(side_1))
    length_2 = float(np.linalg.norm(side_2))

    if length_2 > length_1:
        main_side = side_2
        width = length_2
        height = length_1
    else:
        main_side = side_1
        width = length_1
        height = length_2

    angle_deg = float(np.degrees(np.arctan2(main_side[1], main_side[0])) % 180.0)
    center = rectangle.centroid

    return center.x, center.y, angle_deg, width, height


def largest_inscribed_rectangle(
    raw_geojson: Union[str, Dict[str, Any]],
    *,
    input_crs: Union[str, int, CRS] = "EPSG:4326",
    output_crs: Union[str, int, CRS, None] = None,
    metric_crs: Union[str,  int, CRS, None] = None,
    debug: bool = False,
    debug_directory: Union[str, Path] = "debug",
    max_iterations: int = 1_500,
    population_size: int = 25,
    seed: Union[int, None] = 42,
    workers: int = 1,
    polish: bool = True,
    snapshot_every: int = 50,
    roundtrip_tolerance: float = 1e-10,
) -> InscribedRectangleResult:
    """
    Cherche un rectangle maximal inscrit.

    Le GeoJSON d'entrée est supposé en EPSG:4326 par défaut. L'optimisation
    est effectuée dans un CRS métrique local, puis le résultat est reprojeté
    dans `output_crs` (égal à `input_crs` par défaut).
    """
    if not debug:
        debugger = NullDebugger()
    elif debug_directory is None:
        raise ValueError("Le débogage est activé mais aucun répertoire n'est fourni.")
    elif debug_directory is not None:
        debugger = GeoJSONDebugger(debug_directory)

    source_crs = CRS.from_user_input(input_crs)
    destination_crs = CRS.from_user_input(output_crs or source_crs)

    input_line = parse_geojson_linestring(raw_geojson)

    debugger.write(
        "input_linestring",
        [
            feature(
                input_line,
                "input_boundary",
                crs=source_crs.to_string(),
                is_closed=input_line.is_ring,
                point_count=len(input_line.coords),
            )
        ],
        explanation=("LineString GeoJSON reçue, sans modification des coordonnées."),
    )

    closed_line = close_linestring(input_line)

    debugger.write(
        "closed_linestring",
        [
            feature(input_line, "original_boundary"),
            feature(
                closed_line,
                "closed_boundary",
                is_closed=closed_line.is_ring,
            ),
        ],
        explanation=("Fermeture du contour en reliant le dernier point au premier."),
    )

    raw_polygon = Polygon(closed_line.coords)

    debugger.write(
        "raw_polygon",
        [
            feature(closed_line, "closed_boundary"),
            feature(
                raw_polygon,
                "raw_polygon",
                is_valid=raw_polygon.is_valid,
                source_area_units_squared=raw_polygon.area,
            ),
        ],
        explanation=("Construction directe du Polygon avant réparation topologique."),
    )

    repaired_geometry = make_valid(raw_polygon)
    geographic_zone = select_largest_polygon(repaired_geometry)

    debugger.write(
        "valid_polygon",
        [
            feature(
                raw_polygon,
                "before_repair",
                is_valid=raw_polygon.is_valid,
            ),
            feature(
                repaired_geometry,
                "after_make_valid",
                geometry_type=repaired_geometry.geom_type,
            ),
            feature(
                geographic_zone,
                "selected_zone",
                is_valid=geographic_zone.is_valid,
            ),
        ],
        explanation=("Réparation topologique puis sélection du plus grand Polygon."),
    )

    if geographic_zone.is_empty or geographic_zone.area <= 0:
        raise ValueError("La zone valide est vide ou sans surface.")

    local_metric_crs = (
        CRS.from_user_input(metric_crs)
        if metric_crs is not None
        else determine_local_metric_crs(geographic_zone, source_crs)
    )

    to_metric = Transformer.from_crs(
        source_crs,
        local_metric_crs,
        always_xy=True,
    )
    metric_to_source = Transformer.from_crs(
        local_metric_crs,
        source_crs,
        always_xy=True,
    )
    metric_to_output = Transformer.from_crs(
        local_metric_crs,
        destination_crs,
        always_xy=True,
    )

    metric_zone_geometry = project_geometry(
        geographic_zone,
        to_metric,
    )

    if not isinstance(metric_zone_geometry, Polygon):
        raise RuntimeError("La projection métrique n'a pas produit un Polygon.")

    metric_zone = metric_zone_geometry

    # Fichier GeoJSON toujours visualisable : la géométrie métrique est
    # reprojetée vers le CRS source avant écriture.
    metric_zone_visual = project_geometry(
        metric_zone,
        metric_to_source,
    )

    debugger.write(
        "metric_projection",
        [
            feature(
                geographic_zone,
                "source_zone",
                source_crs=source_crs.to_string(),
            ),
            feature(
                metric_zone_visual,
                "metric_roundtrip_zone",
                metric_crs=local_metric_crs.to_string(),
                area_m2=metric_zone.area,
            ),
        ],
        explanation=(
            "La zone est projetée dans un CRS métrique pour les calculs, "
            "puis reprojetée uniquement pour la visualisation. Les deux "
            "couches doivent presque parfaitement se superposer."
        ),
    )

    normalized_zone, offset_x, offset_y, scale = normalize_polygon(metric_zone)

    # Vérification explicite que la normalisation est inversible.
    metric_zone_roundtrip = denormalize_geometry(
        normalized_zone,
        offset_x,
        offset_y,
        scale,
    )
    normalization_roundtrip_error_m = metric_zone.hausdorff_distance(
        metric_zone_roundtrip
    )

    if normalization_roundtrip_error_m > roundtrip_tolerance:
        raise RuntimeError(
            "La normalisation/dénormalisation de la zone n'est pas "
            f"inversible : erreur={normalization_roundtrip_error_m:.3e} m."
        )

    debugger.write(
        "normalization_roundtrip",
        [
            feature(
                geographic_zone,
                "source_zone",
            ),
            feature(
                project_geometry(
                    metric_zone_roundtrip,
                    metric_to_source,
                ),
                "denormalized_zone",
                roundtrip_error_m=normalization_roundtrip_error_m,
                normalization_scale_m=scale,
                offset_x_m=offset_x,
                offset_y_m=offset_y,
            ),
        ],
        explanation=(
            "Test aller-retour de la normalisation. Les deux couches doivent "
            "se superposer ; l'erreur est exprimée en mètres."
        ),
    )

    min_x, min_y, max_x, max_y = normalized_zone.bounds
    max_dimension = float(np.hypot(max_x - min_x, max_y - min_y))

    bounds = [
        (min_x, max_x),
        (min_y, max_y),
        (1e-8, max_dimension / 2.0),
        (1e-8, max_dimension / 2.0),
        (0.0, pi / 2.0),
    ]

    def normalized_to_source(
        geometry: BaseGeometry,
    ) -> BaseGeometry:
        metric_geometry = denormalize_geometry(
            geometry,
            offset_x,
            offset_y,
            scale,
        )
        return project_geometry(metric_geometry, metric_to_source)

    debugger.write(
        "optimizer_domain",
        [
            feature(
                geographic_zone,
                "zone",
            ),
            feature(
                normalized_to_source(
                    Polygon.from_bounds(
                        min_x,
                        min_y,
                        max_x,
                        max_y,
                    )
                ),
                "center_search_bounds",
            ),
        ],
        explanation=(
            "Emprise de recherche du centre, reconvertie dans les "
            "coordonnées GeoJSON pour la visualisation."
        ),
    )

    def objective(parameters: np.ndarray) -> float:
        center_x, center_y, half_width, half_height, angle = parameters

        rectangle = rectangle_from_parameters(
            center_x,
            center_y,
            half_width,
            half_height,
            angle,
        )
        rectangle_area = rectangle.area

        if rectangle_area <= 0:
            return 1e12

        outside_area = rectangle.difference(normalized_zone).area
        score = -rectangle_area

        if outside_area > 0:
            relative_outside = outside_area / max(
                rectangle_area,
                1e-15,
            )
            score += (
                10_000.0 * outside_area
                + 10_000.0 * relative_outside**2 * normalized_zone.area
            )

        return score

    iteration = 0
    snapshot_index = 0

    def callback(
        x: np.ndarray,
        convergence: Union[float, None] = None,
    ) -> bool:
        nonlocal iteration, snapshot_index
        iteration += 1

        if snapshot_every <= 0 or iteration % snapshot_every != 0:
            return False

        cx, cy, hw, hh, angle = np.asarray(x, dtype=float)
        candidate = rectangle_from_parameters(
            cx,
            cy,
            hw,
            hh,
            angle,
        )
        outside = candidate.difference(normalized_zone)

        debugger.write(
            f"optimization_{snapshot_index:03d}",
            [
                feature(
                    geographic_zone,
                    "zone",
                ),
                feature(
                    normalized_to_source(candidate),
                    "current_best_rectangle",
                    iteration=iteration,
                    normalized_area=candidate.area,
                    angle_deg=float(np.degrees(angle)),
                    covered=normalized_zone.covers(candidate),
                    normalized_outside_area=outside.area,
                    convergence=convergence,
                ),
                feature(
                    normalized_to_source(outside),
                    "outside_part",
                    iteration=iteration,
                ),
                feature(
                    normalized_to_source(Point(cx, cy)),
                    "rectangle_center",
                    iteration=iteration,
                ),
            ],
            explanation=(
                "Meilleure solution courante. Toutes les couches ont été "
                "reprojetées dans le CRS GeoJSON d'entrée."
            ),
        )

        snapshot_index += 1
        return False

    optimization_result = differential_evolution(
        objective,
        bounds=bounds,
        strategy="best1bin",
        maxiter=max_iterations,
        popsize=population_size,
        tol=1e-9,
        atol=1e-11,
        mutation=(0.5, 1.0),
        recombination=0.8,
        seed=seed,
        workers=workers,
        updating="immediate" if workers == 1 else "deferred",
        polish=polish,
        callback=callback,
    )

    cx, cy, hw, hh, angle = optimization_result.x
    raw_normalized_rectangle = rectangle_from_parameters(
        cx,
        cy,
        hw,
        hh,
        angle,
    )
    raw_outside = raw_normalized_rectangle.difference(normalized_zone)

    debugger.write(
        "optimizer_result_before_shrink",
        [
            feature(geographic_zone, "zone"),
            feature(
                normalized_to_source(raw_normalized_rectangle),
                "optimizer_rectangle",
                normalized_area=raw_normalized_rectangle.area,
                covered=normalized_zone.covers(raw_normalized_rectangle),
                normalized_outside_area=raw_outside.area,
            ),
            feature(
                normalized_to_source(raw_outside),
                "outside_part",
            ),
        ],
        explanation=(
            "Résultat brut de l'optimiseur avant correction des très petits "
            "dépassements numériques."
        ),
    )

    normalized_rectangle, normalized_factor = shrink_until_covered(
        raw_normalized_rectangle,
        normalized_zone,
    )

    debugger.write(
        "normalized_rectangle_after_shrink",
        [
            feature(geographic_zone, "zone"),
            feature(
                normalized_to_source(raw_normalized_rectangle),
                "before_shrink",
            ),
            feature(
                normalized_to_source(normalized_rectangle),
                "after_shrink",
                shrink_factor=normalized_factor,
                covered=normalized_zone.covers(normalized_rectangle),
            ),
        ],
        explanation=(
            "Réduction uniforme du rectangle jusqu'à ce qu'il soit couvert "
            "par la zone normalisée."
        ),
    )

    metric_rectangle_geometry = denormalize_geometry(
        normalized_rectangle,
        offset_x,
        offset_y,
        scale,
    )

    if not isinstance(metric_rectangle_geometry, Polygon):
        raise RuntimeError("La dénormalisation n'a pas produit un Polygon.")

    metric_rectangle = metric_rectangle_geometry

    # Test décisif : dénormaliser puis renormaliser doit restituer exactement
    # le rectangle avant dénormalisation.
    renormalized_rectangle = normalize_geometry(
        metric_rectangle,
        offset_x,
        offset_y,
        scale,
    )
    rectangle_roundtrip_error = normalized_rectangle.hausdorff_distance(
        renormalized_rectangle
    )

    if rectangle_roundtrip_error > roundtrip_tolerance:
        raise RuntimeError(
            "La dénormalisation du rectangle l'a modifié : "
            f"erreur aller-retour={rectangle_roundtrip_error:.3e}."
        )

    debugger.write(
        "denormalized_rectangle",
        [
            feature(
                geographic_zone,
                "zone",
            ),
            feature(
                project_geometry(
                    metric_rectangle,
                    metric_to_source,
                ),
                "denormalized_rectangle",
                area_m2=metric_rectangle.area,
                covered=metric_zone.covers(metric_rectangle),
                normalized_roundtrip_error=rectangle_roundtrip_error,
                metric_crs=local_metric_crs.to_string(),
            ),
        ],
        explanation=(
            "Rectangle dénormalisé dans le repère métrique, puis reprojeté "
            "dans les coordonnées GeoJSON uniquement pour l'affichage."
        ),
    )

    metric_rectangle, final_factor = shrink_until_covered(
        metric_rectangle,
        metric_zone,
    )

    center_metric_x, center_metric_y, angle_deg, width, height = rectangle_properties(
        metric_rectangle
    )

    output_rectangle_geometry = project_geometry(
        metric_rectangle,
        metric_to_output,
    )

    if not isinstance(output_rectangle_geometry, Polygon):
        raise RuntimeError("La reprojection finale n'a pas produit un Polygon.")

    output_rectangle = output_rectangle_geometry
    output_center = output_rectangle.centroid

    final_zone_output = (
        geographic_zone
        if destination_crs == source_crs
        else project_geometry(
            geographic_zone,
            Transformer.from_crs(
                source_crs,
                destination_crs,
                always_xy=True,
            ),
        )
    )

    final_outside_metric = metric_rectangle.difference(metric_zone)

    debugger.write(
        "final_result",
        [
            feature(
                final_zone_output,
                "zone",
                output_crs=destination_crs.to_string(),
                area_m2=metric_zone.area,
            ),
            feature(
                output_rectangle,
                "largest_inscribed_rectangle",
                area_m2=metric_rectangle.area,
                width_m=width,
                height_m=height,
                angle_deg=angle_deg,
                shrink_factor=final_factor,
                covered=metric_zone.covers(metric_rectangle),
                metric_crs=local_metric_crs.to_string(),
            ),
            feature(
                Point(output_center.x, output_center.y),
                "rectangle_center",
            ),
            feature(
                project_geometry(
                    final_outside_metric,
                    metric_to_output,
                ),
                "outside_part",
                area_m2=final_outside_metric.area,
            ),
        ],
        explanation=(
            "Résultat final. La rectangularité, les dimensions et la surface "
            "sont évaluées dans le CRS métrique. La géométrie est fournie "
            "dans le CRS de sortie."
        ),
    )

    return InscribedRectangleResult(
        rectangle=output_rectangle,
        area=metric_rectangle.area,
        center_x=output_center.x,
        center_y=output_center.y,
        angle_deg=angle_deg,
        width=width,
        height=height,
        metric_crs=local_metric_crs.to_string(),
        optimizer_success=optimization_result.success,
        optimizer_message=str(optimization_result.message),
        debug_directory=Path(debug_directory),
        corners=list(output_rectangle.exterior.coords)[:4],
    )


if __name__ == "__main__":
    input_geojson = {
        "type": "LineString",
        "coordinates": [
            [-1.11862, 46.15436],
            [-1.11825, 46.15472],
            [-1.11781, 46.15448],
            [-1.11818, 46.15410],
            [-1.11862, 46.15436],
        ],
    }

    result = largest_inscribed_rectangle(
        input_geojson,
        debug=True,
        debug_directory="debug",
        max_iterations=500,
        population_size=20,
        snapshot_every=50,
        seed=42,
    )

    print(json.dumps(result.to_geojson_feature(), indent=2))
    print(f"Surface : {result.area:.3f} m²")
    print(f"Dimensions : {result.width:.3f} × {result.height:.3f} m")
    print(f"CRS métrique : {result.metric_crs}")
