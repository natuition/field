var wms_layers = [];


        var lyr_GoogleMapSattelite_0 = new ol.layer.Tile({
            'title': 'GoogleMapSattelite',
            'type': 'base',
            'opacity': 1.000000,
            
            
            source: new ol.source.XYZ({
    attributions: ' ',
                url: 'http://www.google.cn/maps/vt?lyrs=s@189&gl=cn&x={x}&y={y}&z={z}'
            })
        });
var format_qgislocation_point_1 = new ol.format.GeoJSON();
var features_qgislocation_point_1 = format_qgislocation_point_1.readFeatures(json_qgislocation_point_1, 
            {dataProjection: 'EPSG:4326', featureProjection: 'EPSG:3857'});
var jsonSource_qgislocation_point_1 = new ol.source.Vector({
    attributions: ' ',
});
jsonSource_qgislocation_point_1.addFeatures(features_qgislocation_point_1);
var lyr_qgislocation_point_1 = new ol.layer.Vector({
                declutter: true,
                source:jsonSource_qgislocation_point_1, 
                style: style_qgislocation_point_1,
                interactive: false,
                title: '<img src="styles/legend/qgislocation_point_1.png" /> qgis.location_point'
            });
var format_field_contour_2 = new ol.format.GeoJSON();
var features_field_contour_2 = format_field_contour_2.readFeatures(json_field_contour_2, 
            {dataProjection: 'EPSG:4326', featureProjection: 'EPSG:3857'});
var jsonSource_field_contour_2 = new ol.source.Vector({
    attributions: ' ',
});
jsonSource_field_contour_2.addFeatures(features_field_contour_2);
var lyr_field_contour_2 = new ol.layer.Vector({
                declutter: true,
                source:jsonSource_field_contour_2, 
                style: style_field_contour_2,
                interactive: false,
                title: '<img src="styles/legend/field_contour_2.png" /> field_contour'
            });
var format_field_balise_3 = new ol.format.GeoJSON();
var features_field_balise_3 = format_field_balise_3.readFeatures(json_field_balise_3, 
            {dataProjection: 'EPSG:4326', featureProjection: 'EPSG:3857'});
var jsonSource_field_balise_3 = new ol.source.Vector({
    attributions: ' ',
});
jsonSource_field_balise_3.addFeatures(features_field_balise_3);
var lyr_field_balise_3 = new ol.layer.Vector({
                declutter: true,
                source:jsonSource_field_balise_3, 
                style: style_field_balise_3,
                interactive: false,
                title: '<img src="styles/legend/field_balise_3.png" /> field_balise'
            });
var format_field_corner_4 = new ol.format.GeoJSON();
var features_field_corner_4 = format_field_corner_4.readFeatures(json_field_corner_4, 
            {dataProjection: 'EPSG:4326', featureProjection: 'EPSG:3857'});
var jsonSource_field_corner_4 = new ol.source.Vector({
    attributions: ' ',
});
jsonSource_field_corner_4.addFeatures(features_field_corner_4);
var lyr_field_corner_4 = new ol.layer.Vector({
                declutter: true,
                source:jsonSource_field_corner_4, 
                style: style_field_corner_4,
                interactive: false,
                title: '<img src="styles/legend/field_corner_4.png" /> field_corner'
            });

lyr_GoogleMapSattelite_0.setVisible(true);lyr_qgislocation_point_1.setVisible(true);lyr_field_contour_2.setVisible(true);lyr_field_balise_3.setVisible(true);lyr_field_corner_4.setVisible(true);
var layersList = [lyr_GoogleMapSattelite_0,lyr_qgislocation_point_1,lyr_field_contour_2,lyr_field_balise_3,lyr_field_corner_4];
lyr_qgislocation_point_1.set('fieldAliases', {'id': 'id', 'id_resultat': 'id_resultat', 'angle': 'angle', 'robot_direction': 'robot_direction', 'session': 'session', });
lyr_field_contour_2.set('fieldAliases', {'id': 'id', 'iscorner': 'iscorner', 'robot': 'robot', 'isbase': 'isbase', });
lyr_field_balise_3.set('fieldAliases', {'id': 'id', 'iscorner': 'iscorner', 'robot': 'robot', 'isbase': 'isbase', });
lyr_field_corner_4.set('fieldAliases', {'id': 'id', 'iscorner': 'iscorner', 'robot': 'robot', 'isbase': 'isbase', });
lyr_qgislocation_point_1.set('fieldImages', {'id': '', 'id_resultat': '', 'angle': '', 'robot_direction': '', 'session': '', });
lyr_field_contour_2.set('fieldImages', {'id': 'TextEdit', 'iscorner': 'CheckBox', 'robot': 'TextEdit', 'isbase': '', });
lyr_field_balise_3.set('fieldImages', {'id': 'TextEdit', 'iscorner': 'CheckBox', 'robot': 'TextEdit', 'isbase': 'TextEdit', });
lyr_field_corner_4.set('fieldImages', {'id': 'TextEdit', 'iscorner': 'CheckBox', 'robot': 'TextEdit', 'isbase': '', });
lyr_qgislocation_point_1.set('fieldLabels', {'id': 'no label', 'id_resultat': 'no label', 'angle': 'no label', 'robot_direction': 'no label', 'session': 'no label', });
lyr_field_contour_2.set('fieldLabels', {'id': 'no label', 'iscorner': 'no label', 'robot': 'no label', 'isbase': 'no label', });
lyr_field_balise_3.set('fieldLabels', {'id': 'no label', 'iscorner': 'no label', 'robot': 'no label', 'isbase': 'no label', });
lyr_field_corner_4.set('fieldLabels', {'id': 'no label', 'iscorner': 'no label', 'robot': 'no label', 'isbase': 'no label', });
lyr_field_corner_4.on('precompose', function(evt) {
    evt.context.globalCompositeOperation = 'normal';
});