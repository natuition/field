from config import config
import detection
import adapters

# undistorted area px from center
ul = -225
ut = -225
ur = 225
ub = 225

# working area px from center
wl = -520
wt = -520
wr = 520
wb = 450


def px_to_smoohie_value(value_px, center_px, one_mm_in_px):
    """Converts px into mm-s"""

    return (value_px - center_px) / one_mm_in_px


def sort_plant_boxes_dist(boxes: list, current_px_x, current_px_y):
    """Returns sorted list making closest to center plants first"""

    return sorted(boxes, key=lambda box: box.get_distance_from(current_px_x, current_px_y))


def sort_plant_boxes_conf(boxes: list):
    """Returns list sorted by descending plant confidence"""

    return sorted(boxes, key=lambda box: box.get_confidence(), reverse=True)


def point_is_in_rect(x, y, left, top, right, bottom):
    """Returns True if (x,y) point in the rectangle or on it's border, else False"""

    return True if x >= left and x <= right and y >= top and y <= bottom else False


def main():
    sma = adapters.SmoothieAdapter(config.SMOOTHIE_HOST)
    pca = adapters.PiCameraAdapter()
    det = detection.YoloOpenCVDetection()
    sma.ext_align_cork_center(config.XY_F_MAX)
    sma.wait_for_all_actions_done()

    img = pca.get_image()
    plant_boxes = det.detect(img)
    plant_boxes = sort_plant_boxes_dist(plant_boxes, config.CORK_CENTER_X, config.CORK_CENTER_Y)

    for box in plant_boxes:
        sma.ext_align_cork_center(config.XY_F_MAX)
        sma.wait_for_all_actions_done()

        box_x, box_y = box.get_center_points()
        # if inside the working zone
        if point_is_in_rect(box_x, box_y, wl, wt, wr, wb):
            while True:
                box_x, box_y = box.get_center_points()
                sm_x = px_to_smoohie_value(box_x, config.CORK_CENTER_X, config.ONE_MM_IN_PX)
                sm_y = px_to_smoohie_value(box_y, config.CORK_CENTER_Y, config.ONE_MM_IN_PX)
                res = sma.custom_move_for(config.XY_F_MAX, X=sm_x, Y=sm_y)
                sma.wait_for_all_actions_done()

                # check if no movement errors
                if res != sma.RESPONSE_OK:
                    print("Smoothie error occurred:", res)
                    exit()

                # if inside undistorted zone
                if point_is_in_rect(box_x, box_y, ul, ut, ur, ub):
                    input("Ready to plant extraction, press enter to begin")
                    sma.ext_do_extraction(config.Z_F_MAX)
                    sma.wait_for_all_actions_done()
                    break
                # if outside undistorted zone but in working zone
                else:
                    temp_img = pca.get_image()
                    temp_plant_boxes = det.detect(temp_img)
                    # get closest box
                    box = sort_plant_boxes_dist(temp_plant_boxes, config.CORK_CENTER_X, config.CORK_CENTER_Y)[0]
        # if not in working zone
        else:
            print(str(box), "is not in working area, switching to next")


def test():
    test_boxes = [
        detection.DetectedPlantBox(5, 5, 60, 60, ["Test_1"], 0, 0.89),
        detection.DetectedPlantBox(200, 200, 250, 250, ["Test_2"], 0, 0.74),
        detection.DetectedPlantBox(400, 400, 450, 450, ["Test_3"], 0, 0.67)
    ]

    # test px to mm converter
    px_x = px_to_smoohie_value(460, 500, config.ONE_MM_IN_PX)
    px_y = px_to_smoohie_value(450, 500, config.ONE_MM_IN_PX)
    print("Px values x, y: ", px_x, px_y)

    # test sort box by dist
    print()
    cur_x = 1000
    cur_y = 1000

    res = sort_plant_boxes_dist(test_boxes, cur_x, cur_y)

    for plant_box in res:
        print(plant_box.get_name(), plant_box.get_confidence())

    # test sort box by confidence
    print()
    res = sort_plant_boxes_conf(test_boxes)

    for plant_box in res:
        print(plant_box.get_name(), plant_box.get_confidence())


if __name__ == "__main__":
    main()
