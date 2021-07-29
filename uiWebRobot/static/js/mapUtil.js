var socketMap = io.connect('http://' + document.domain + ':' + location.port + '/map');

var map;

document.addEventListener("DOMContentLoaded",createMap(coords));

function createMap(coords){

    if (typeof coords !== 'undefined') {
        var line = coords
        line.push(coords[0])

        x_center = (coords[0][0]+coords[1][0]+coords[2][0]+coords[3][0])/4
        y_center = (coords[0][1]+coords[1][1]+coords[2][1]+coords[3][1])/4 
    }

    mapboxgl.accessToken = 'pk.eyJ1IjoidmluY2VudGxiIiwiYSI6ImNrY2F2YTA5NjF5c3kzMG8wbG5zbjk5cjcifQ.p9V3BtVZngNW1L8MRoALaw';
    map = new mapboxgl.Map({
        container: 'map',
        style: 'mapbox://styles/mapbox/satellite-v9',
        center: [x_center,y_center],
        zoom: 17
    });

    //map.addControl(new mapboxgl.NavigationControl());

    map.on('load', function () {
        //Field zone
        if(typeof(map.getSource('field')) == "undefined"){
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
        }
        //Field line
        if(typeof(map.getSource('field_corner')) == "undefined"){
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
        }
        //Path line
        if(typeof(map.getSource('pathRobot')) == "undefined"){
            map.addSource('pathRobot', {
                'type': 'geojson',
                'data': {
                    'type': 'Feature',
                    'geometry': {
                        'type': 'LineString',
                        'coordinates': []
                    }
                }
            });
            map.addLayer({
                'id': 'pathRobotLayer',
                'type': 'line',
                'source': 'pathRobot',
                'layout': {
                    'line-join': 'round',
                    'line-cap': 'round',
                },
                'paint': {
                    'line-color': 'red',
                    'line-width': 3
                }
            });
        }
        //Last pos
        if(typeof(map.getSource('lastPos')) == "undefined"){
            map.addSource('lastPos', {
                'type': 'geojson',
                'data': { 
                    'type': 'Feature',
                    'geometry': {
                        'type': 'Point',
                        'coordinates': []
                    }
                }
            });
            map.addLayer({
                'id': 'lastPosLayer',
                'type': 'circle',
                'source': 'lastPos',
                'paint': {
                    'circle-radius': 6,
                    'circle-color': 'darkred'
                }
            });
        }
    });

}

socketMap.on('updatePath', function(dataServ) {
    dataServ = JSON.parse(dataServ)
    coords = dataServ
    x_center = coords[coords.length - 1][0]
    y_center = coords[coords.length - 1][1]

    if(typeof(map.getSource('pathRobot')) == "undefined" || typeof(map.getSource('lastPos')) == "undefined"){
        createMap(dataServ);
    }
    
    map.getSource('pathRobot').setData({
        'type': 'Feature',
        'geometry': {
            'type': 'LineString',
            'coordinates': dataServ
        }
    });
    
    map.getSource('lastPos').setData({ 
        'type': 'Feature',
        'geometry': {
            'type': 'Point',
            'coordinates': [x_center,y_center]
        }
    });

    map.panTo([x_center,y_center]);
});

socketMap.on('newField', function(dataServ) {
    dataServ = JSON.parse(dataServ)

    if(typeof(map.getSource('field')) == "undefined"){
        createMap(dataServ);
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
        coords = dataServ
        x_center = (coords[0][0]+coords[1][0]+coords[2][0]+coords[3][0])/4
        y_center = (coords[0][1]+coords[1][1]+coords[2][1]+coords[3][1])/4
        map.flyTo({
            center: [x_center,y_center],
            speed: 1,
            zoom: 17
        });
    }
    
});