var url_step = ["/", "/vesc_foc", "/vesc_z", "/x_y_dir", "/camera_focus", "/camera_crop_picture", "/camera_target_detection", "/client_config", "/end"]

function get_next_page_ref(current_step) {
    var next = url_step[current_step]
    return 'http://' + document.domain + ':' + location.port + next;
}

function set_step_url_on_breadcrumb_trail() {
    var index = 0;
    for (const child of document.getElementById("breadcrumb").children) {
        child.href = url_step[index];
        index++;
    }
}