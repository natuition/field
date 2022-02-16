const socketMap = io.connect('http://' + document.domain + ':' + location.port + '/map');
const socketBroadcast = io.connect('http://' + document.domain + ':' + location.port + '/broadcast');
socketMap.on("reconnect_attempt", (attempt) => {
    if(attempt > 2) location.reload();
});

var map;

var firstFocus = false

window.onload = ()=>{
    if (typeof coords_field != "undefined"){
        if (typeof coords_other != "undefined"){
            document.addEventListener("DOMContentLoaded",createMap(coords_field,coords_other));
        }else{
            document.addEventListener("DOMContentLoaded",createMap(coords_field,[]));
        }
    }else{
        if (typeof coords_other != "undefined"){
            document.addEventListener("DOMContentLoaded",createMap([],coords_other));
        }else{
            document.addEventListener("DOMContentLoaded",createMap([],[]));
        }
    }

}

function createMap(coords_field,coords_other){
    var x_center = 0
    var y_center = 0

    if (coords_field.length > 0) {

        let cpt = 0
        
        for(const coord of coords_field){
            x_center += coord[0]
            y_center += coord[1]
            cpt += 1
        }
        x_center /= cpt
        y_center /= cpt

        var zoom = 17;

    }else{
        y_center = 48.85304;
        x_center = 2.3499075;

        var zoom = 16.5;

    }

    mapboxgl.accessToken = 'pk.eyJ1IjoidmluY2VudGxiIiwiYSI6ImNrY2F2YTA5NjF5c3kzMG8wbG5zbjk5cjcifQ.p9V3BtVZngNW1L8MRoALaw';
    map = new mapboxgl.Map({
        container: 'map',
        style: 'mapbox://styles/mapbox/satellite-v9',
        center: [x_center,y_center],
        zoom: zoom
    });

    var coords_other_multipolygon = []

    for (const other of coords_other) { 
        coords_other_multipolygon.push([other]);
    }

    map.on('load', function () {
        //Other field zone
        if(typeof(map.getSource('other_field')) == "undefined"){
            map.addSource('other_field', {
                'type': 'geojson',
                'data': {
                    'type': 'Feature',
                    'geometry': {
                        'type': 'MultiPolygon',
                        'coordinates': coords_other_multipolygon
                    }
                }
            }); 
            map.addLayer({
                'id': 'otherFieldLayer',
                'type': 'fill',
                'source': 'other_field',
                'layout': {},
                'paint': {
                    'fill-color': '#888',
                    'fill-opacity': 0.4
                }
            });
        }
        //Other field line
        if(typeof(map.getSource('other_field_corner')) == "undefined"){
            map.addSource('other_field_corner', {
                'type': 'geojson',
                'data': {
                    'type': 'Feature',
                    'geometry': {
                        'type': 'MultiLineString',
                        'coordinates': coords_other
                    }
                }
            });    
            map.addLayer({
                'id': 'other_field_cornerLayer',
                'type': 'line',
                'source': 'other_field_corner',
                'layout': {
                    'line-join': 'round',
                    'line-cap': 'round',
                },
                'paint': {
                    'line-color': '#888',
                    'line-width': 4
                }
            });
        }
        //Field zone
        if(typeof(map.getSource('field')) == "undefined"){
            map.addSource('field', {
                'type': 'geojson',
                'data': {
                    'type': 'Feature',
                    'geometry': {
                        'type': 'Polygon',
                        'coordinates': [
                            coords_field
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
                    'fill-color': '#0620FB',
                    'fill-opacity': 0.4
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
                        'coordinates': coords_field
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
                    'line-color': '#0620FB',
                    'line-width': 4
                }
            });
        }
        //Instruction_line
        if(typeof(map.getSource('instruction_line')) == "undefined"){
            map.addSource('instruction_line', {
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
                'id': 'instruction_lineLayer',
                'type': 'line',
                'source': 'instruction_line',
                'layout': {
                    'line-join': 'round',
                    'line-cap': 'round',
                },
                'paint': {
                    'line-color': '#FF8C15',
                    'line-width': 2
                }
            });
        }
        //Instruction_point
        if(typeof(map.getSource('instruction_point')) == "undefined"){
            map.addSource('instruction_point', {
                'type': 'geojson',
                'data': {
                    'type': 'Feature',
                    'geometry': {
                        'type': 'MultiPoint',
                        'coordinates': []
                    }
                }
            });    
            map.addLayer({
                'id': 'instruction_pointLayer',
                'type': 'circle',
                'source': 'instruction_point',
                'paint': {
                    'circle-radius': 3,
                    'circle-color': '#FF8C15'
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
                    'line-color': '#e55e5e',
                    'line-width': 2
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
                    'circle-radius': 3,
                    'circle-color': [
                        'match',
                        ['get', 'Quality'],
                        '4',
                        '#e55e5e',// red for quality 4
                        '#fbb03b' // orange for other
                    ]
                }
            });
        }

        socketMap.on('updatePath', function(dataServ) {
            dataServ = JSON.parse(dataServ)
            var coords = dataServ[0]
            var last_coord = coords[coords.length - 1]
            var quality = dataServ[1]
        
            if(coords.length>1){
                console.log("ok")
                map.getSource('pathRobot').setData({
                    'type': 'Feature',
                    'geometry': {
                        'type': 'LineString',
                        'coordinates': coords
                    }
                });
            }
            
            map.getSource('lastPos').setData({ 
                'type': 'Feature',
                'geometry': {
                    'type': 'Point',
                    'coordinates': last_coord
                },
                "properties": Object.assign({}, {'Quality':Object.keys(quality)[0]}, quality)
            });
        
            if(coords.length>1 || !firstFocus){
                map.panTo(last_coord);
                firstFocus = true;
            }
        });

    });

}

socketMap.on('updateDisplayInstructionPath', function(dataServ) {
    dataServ = JSON.parse(dataServ)
    //Instruction_line
    if(typeof(map.getSource('instruction_line')) == "undefined"){
        map.addSource('instruction_line', {
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
            'id': 'instruction_lineLayer',
            'type': 'line',
            'source': 'instruction_line',
            'layout': {
                'line-join': 'round',
                'line-cap': 'round',
            },
            'paint': {
                'line-color': '#FF8C15',
                'line-width': 2
            }
        });
    }else{
        map.getSource('instruction_line').setData({
            'type': 'Feature',
            'geometry': {
                'type': 'LineString',
                'coordinates': dataServ
            }
        });
    }
    //Instruction_point
    if(typeof(map.getSource('instruction_point')) == "undefined"){
        map.addSource('instruction_point', {
            'type': 'geojson',
            'data': {
                'type': 'Feature',
                'geometry': {
                    'type': 'MultiPoint',
                    'coordinates': dataServ
                }
            }
        });    
        map.addLayer({
            'id': 'instruction_pointLayer',
            'type': 'circle',
            'source': 'instruction_point',
            'paint': {
                'circle-radius': 3,
                'circle-color': '#FF8C15'
            }
        });
    }else{
        map.getSource('instruction_point').setData({
            'type': 'Feature',
            'geometry': {
                'type': 'MultiPoint',
                'coordinates': dataServ
            }
        });
    }
});

socketMap.on('newField', function(dataServ) {
    dataServ = JSON.parse(dataServ);

    if(dataServ["current_field_name"] == ""){
        parent.document.location.reload()
    }

    if(typeof(map.getSource('field')) == "undefined"){
        createMap(dataServ);
    }else{
        
        var coords_other_multipolygon = []

        for (const other of dataServ["other_fields"]) { 
            coords_other_multipolygon.push([other]);
        }
        if(map.getSource('other_field') == undefined){
            map.addSource('other_field', {
                'type': 'geojson',
                'data': {
                    'type': 'Feature',
                    'geometry': {
                        'type': 'MultiPolygon',
                        'coordinates': [
                            dataServ["other_fields"]
                        ]
                    }
                }
            }); 
            map.addLayer({
                'id': 'otherFieldLayer',
                'type': 'fill',
                'source': 'other_field',
                'layout': {},
                'paint': {
                    'fill-color': '#888',
                    'fill-opacity': 0.4
                }
            });
        }else{
            map.getSource('other_field').setData({
                'type': 'Feature',
                'geometry': {
                    'type': 'MultiPolygon',
                    'coordinates': coords_other_multipolygon
                }
            });
        }
        if(map.getSource('other_field') == undefined){
            map.addSource('other_field_corner', {
                'type': 'geojson',
                'data': {
                    'type': 'Feature',
                    'geometry': {
                        'type': 'MultiLineString',
                        'coordinates': dataServ["other_fields"]
                    }
                }
            });    
            map.addLayer({
                'id': 'other_field_cornerLayer',
                'type': 'line',
                'source': 'other_field_corner',
                'layout': {
                    'line-join': 'round',
                    'line-cap': 'round',
                },
                'paint': {
                    'line-color': '#888',
                    'line-width': 4
                }
            });
        }else{
            map.getSource('other_field_corner').setData({
                'type': 'Feature',
                'geometry': {
                    'type': 'MultiLineString',
                    'coordinates': dataServ["other_fields"]
                }
            });
        }

        map.getSource('field').setData({
            'type': 'Feature',
            'geometry': {
                'type': 'Polygon',
                'coordinates': [
                    dataServ["field"]
                ]
            }
        });
        map.getSource('field_corner').setData({
            'type': 'Feature',
            'geometry': {
                'type': 'LineString',
                'coordinates': dataServ["field"]
            }
        });

        if (dataServ["field"].length > 0){
            var coords = dataServ["field"]
            let cpt = 0
            let x_center = 0
            let y_center = 0
            for(const coord of coords){
                x_center += coord[0]
                y_center += coord[1]
                cpt += 1
            }
            x_center /= cpt
            y_center /= cpt
            map.flyTo({
                center: [x_center,y_center],
                speed: 2,
                zoom: 17
            });
        }

        var sel = parent.document.getElementById('field_selector');
        var opts = sel.options;

        var current_field_name_found = false;

        for (var j = 0; j < sel.options.length ; j++) {
            var opt = opts[j];
            if(typeof dataServ["fields_list"] != "undefined"){
                if(!dataServ["fields_list"].includes(opt.value) && !opt.disabled){
                    sel.remove(j);
                }
            }
        }

        for (var j = 0; j < sel.options.length ; j++) {
            var opt = opts[j];
            if (opt.value == dataServ["current_field_name"]) {
                sel.selectedIndex = j;
                current_field_name_found = true;
            }
        }

        if(!current_field_name_found){
            if(typeof dataServ["fields_list"] != "undefined"){
                if(dataServ["fields_list"].includes(dataServ["current_field_name"])){
                    var option = document.createElement('option');
                    option.value = option.text = dataServ["current_field_name"];
                    sel.add(option);
                    var opts = sel.options;
                    for (var opt, j = 0; opt = opts[j]; j++) {
                        if (opt.value == dataServ["current_field_name"]) {
                            sel.selectedIndex = j;
                            break;
                        }
                    }
                }
            }
            
        }
    }

    socketBroadcast.emit('data', {type: "reloader", status : false});
    
});