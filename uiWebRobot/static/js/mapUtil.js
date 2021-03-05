var socketMap = io.connect('http://' + document.domain + ':' + location.port + '/map');

if (typeof coords !== 'undefined') {
    var line = coords
    line.push(coords[0])

    x_center = (coords[0][0]+coords[1][0]+coords[2][0]+coords[3][0])/4
    y_center = (coords[0][1]+coords[1][1]+coords[2][1]+coords[3][1])/4 
}
//x_center = myCoords[0]
//y_center = myCoords[1]

mapboxgl.accessToken = 'pk.eyJ1IjoidmluY2VudGxiIiwiYSI6ImNrY2F2YTA5NjF5c3kzMG8wbG5zbjk5cjcifQ.p9V3BtVZngNW1L8MRoALaw';
var map = new mapboxgl.Map({
    container: 'map',
    style: 'mapbox://styles/mapbox/satellite-v9',
    center: [x_center,y_center],
    zoom: 17
});

//map.addControl(new mapboxgl.NavigationControl());

map.on('load', function () {
    //Field zone
    map.addSource('field', {
        'type': 'geojson',
        'data': {
            'type': 'Feature',
            'geometry': {
                'type': 'Polygon',
                'coordinates': [
                    line
                ]
            }
        }
    }); 
    map.addLayer({
        'id': 'fieldLayer',
        'type': 'fill',
        'source': 'field',
        'layout': {},
        'paint': {
            'fill-color': '#088',
            'fill-opacity': 0.3
        }
    });
    //Field line
    map.addSource('field_corner', {
        'type': 'geojson',
        'data': {
            'type': 'Feature',
            'geometry': {
                'type': 'LineString',
                'coordinates': line
            }
        }
    });    
    map.addLayer({
        'id': 'field_cornerLayer',
        'type': 'line',
        'source': 'field_corner',
        'layout': {
            'line-join': 'round',
            'line-cap': 'round',
        },
        'paint': {
            'line-color': '#088',
            'line-width': 4
        }
    });
    //My pos
    map.addSource('my_pos', {
        'type': 'geojson',
        'data': { 
            'type': 'Feature',
            'geometry': {
                'type': 'Point',
                'coordinates': myCoords
            }
        }
    });
    map.addLayer({
        'id': 'my_posLayer',
        'type': 'circle',
        'source': 'my_pos',
        'paint': {
            'circle-radius': 6,
            'circle-color': 'darkred'
        }
    });
});

socketMap.on('newPos', function(dataServ) {
    dataServ = JSON.parse(dataServ)
    map.getSource('my_pos').setData({ 
        'type': 'Feature',
        'geometry': {
            'type': 'Point',
            'coordinates': dataServ
        }
    });
});

socketMap.on('newField', function(dataServ) {
    dataServ = JSON.parse(dataServ)

    if(typeof(map.getSource('field')) == "undefined"){
        mapboxgl.accessToken = 'pk.eyJ1IjoidmluY2VudGxiIiwiYSI6ImNrY2F2YTA5NjF5c3kzMG8wbG5zbjk5cjcifQ.p9V3BtVZngNW1L8MRoALaw';
        map = new mapboxgl.Map({
            container: 'map',
            style: 'mapbox://styles/mapbox/satellite-v9',
            center: [x_center,y_center],
            zoom: 17
        });

        map.on('load', function () {
            map.addSource('field', {
                'type': 'geojson',
                'data': {
                    'type': 'Feature',
                    'geometry': {
                        'type': 'Polygon',
                        'coordinates': [
                            dataServ
                        ]
                    }
                }
            }); 
            map.addLayer({
                'id': 'fieldLayer',
                'type': 'fill',
                'source': 'field',
                'layout': {},
                'paint': {
                    'fill-color': '#088',
                    'fill-opacity': 0.3
                }
            });
            //Field line
            map.addSource('field_corner', {
                'type': 'geojson',
                'data': {
                    'type': 'Feature',
                    'geometry': {
                        'type': 'LineString',
                        'coordinates': dataServ
                    }
                }
            });    
            map.addLayer({
                'id': 'field_cornerLayer',
                'type': 'line',
                'source': 'field_corner',
                'layout': {
                    'line-join': 'round',
                    'line-cap': 'round',
                },
                'paint': {
                    'line-color': '#088',
                    'line-width': 4
                }
            });
        });
    }else{
        map.getSource('field').setData({
            'type': 'Feature',
            'geometry': {
                'type': 'Polygon',
                'coordinates': [
                    dataServ
                ]
            }
        });
        map.getSource('field_corner').setData({
            'type': 'Feature',
            'geometry': {
                'type': 'LineString',
                'coordinates': dataServ
            }
        });
    }

    
    coords = dataServ
    x_center = (coords[0][0]+coords[1][0]+coords[2][0]+coords[3][0])/4
    y_center = (coords[0][1]+coords[1][1]+coords[2][1]+coords[3][1])/4
    map.flyTo({
        center: [x_center,y_center],
        speed: 1,
        zoom: 17
    });
});