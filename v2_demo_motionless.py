from config import config
import detection
import adapters
import time
import math

# circle working area
working_zone_radius = 850
# circle undistorted area
undistorted_zone_radius = 240


def px_to_smoohie_value(target_px, center_px, one_mm_in_px):
    """Converts px into mm-s"""

    return (center_px - target_px) / one_mm_in_px


def sort_plant_boxes_dist(boxes: list, current_px_x, current_px_y):
    """Returns sorted list making closest to center plants first"""

    return sorted(boxes, key=lambda box: box.get_distance_from(current_px_x, current_px_y))


def min_plant_box_dist(boxes: list, current_px_x, current_px_y):
    """Returns closest to current coordinates plant box"""

    return min(boxes, key=lambda box: box.get_distance_from(current_px_x, current_px_y))


def sort_plant_boxes_conf(boxes: list):
    """Returns list sorted by descending plant confidence"""

    return sorted(boxes, key=lambda box: box.get_confidence(), reverse=True)


def is_point_in_rect(point_x, point_y, left, top, right, bottom):
    """Returns True if (x,y) point in the rectangle or on it's border, else False"""

    return point_x >= left and point_x <= right and point_y >= top and point_y <= bottom


def is_point_in_circle(point_x, point_y, circle_center_x, circle_center_y, circle_radius):
    """Returns True if (x,y) point in the circle or on it's border, False otherwise"""

    return math.sqrt((point_x - circle_center_x) ** 2 + (point_y - circle_center_y) ** 2) <= circle_radius


def main():
    smoothie = adapters.SmoothieAdapter(config.SMOOTHIE_HOST)
    camera = adapters.CameraAdapterIMX219_170()
    time.sleep(2)
    detector = detection.YoloOpenCVDetection()
    smoothie.ext_align_cork_center(config.XY_F_MAX)
    smoothie.wait_for_all_actions_done()

    image = camera.get_image()
    img_y_c, img_x_c = int(image.shape[0] / 2), int(image.shape[1] / 2)
    plant_boxes = detector.detect(image)
    plant_boxes = sort_plant_boxes_dist(plant_boxes, config.CORK_CENTER_X, config.CORK_CENTER_Y)

    for box in plant_boxes:
        smoothie.ext_align_cork_center(config.XY_F_MAX)  # camera in real center
        smoothie.wait_for_all_actions_done()
        box_x, box_y = box.get_center_points()

        # if inside the working zone
        if is_point_in_circle(box_x, box_y, img_x_c, img_y_c, working_zone_radius):
            while True:
                box_x, box_y = box.get_center_points()

                # if inside undistorted zone
                if is_point_in_circle(box_x, box_y, img_x_c, img_y_c, undistorted_zone_radius):
                    # calculate values to move camera over a plant
                    sm_x = px_to_smoohie_value(box_x, config.CORK_CENTER_X, config.ONE_MM_IN_PX)
                    sm_y = px_to_smoohie_value(box_y, config.CORK_CENTER_Y, config.ONE_MM_IN_PX)

                    # move camera over a plant
                    res = smoothie.custom_move_for(config.XY_F_MAX, X=sm_x, Y=sm_y)
                    smoothie.wait_for_all_actions_done()
                    if res != smoothie.RESPONSE_OK:
                        print("Couldn't move camera over plant, smoothie error occurred:", res)
                        exit(1)

                    # move cork to the camera position
                    res = smoothie.custom_move_for(config.XY_F_MAX, Y=57)
                    smoothie.wait_for_all_actions_done()
                    if res != smoothie.RESPONSE_OK:
                        print("Couldn't move cork over plant, smoothie error occurred:", res)
                        exit(1)

                    # waiting confirmation for extraction (just to make people see how it's going on)
                    input("Ready to plant extraction, press enter to begin")

                    # extraction, cork down
                    res = smoothie.custom_move_for(config.Z_F_MAX, Z=-30)
                    smoothie.wait_for_all_actions_done()
                    if res != smoothie.RESPONSE_OK:
                        print("Couldn't move the extractor down, smoothie error occurred:", res)
                        exit(1)

                    # extraction, cork up
                    res = smoothie.ext_cork_up()
                    smoothie.wait_for_all_actions_done()
                    if res != smoothie.RESPONSE_OK:
                        print("Couldn't move the extractor up, smoothie error occurred:", res)
                        exit(1)
                    break

                # if outside undistorted zone but in working zone
                else:
                    # calculate values for move camera closer to a plant
                    sm_x = px_to_smoohie_value(box_x, img_x_c, config.ONE_MM_IN_PX)
                    sm_y = px_to_smoohie_value(box_y, img_y_c, config.ONE_MM_IN_PX)
                    # move for a half distance, dist is not < 10
                    sm_x = int(sm_x / 2) if sm_x / 2 > 10 else 10
                    sm_y = int(sm_y / 2) if sm_y / 2 > 10 else 10

                    # move camera closer to a plant
                    res = smoothie.custom_move_for(config.XY_F_MAX, X=sm_x, Y=sm_y)
                    smoothie.wait_for_all_actions_done()
                    if res != smoothie.RESPONSE_OK:
                        print("Couldn't move to plant, smoothie error occurred:", res)
                        exit(1)

                    # make new photo and re-detect plants
                    temp_img = camera.get_image()
                    temp_plant_boxes = detector.detect(temp_img)
                    # get closest box (exactly update current box from main list coordinates after moving closer)
                    box = min_plant_box_dist(temp_plant_boxes, img_x_c, img_y_c)

        # if not in working zone
        else:
            print("skipped", str(box), "(not in working area)")

    print("Done.")


def tools_test():
    test_boxes = [
        detection.DetectedPlantBox(5, 5, 60, 60, ["Test_1"], 0, 0.89),
        detection.DetectedPlantBox(200, 200, 250, 250, ["Test_2"], 0, 0.74),
        detection.DetectedPlantBox(400, 400, 450, 450, ["Test_3"], 0, 0.67)
    ]

    # test px to mm converter
    """
    px_x = px_to_smoohie_value(460, 500, config.ONE_MM_IN_PX)
    px_y = px_to_smoohie_value(450, 500, config.ONE_MM_IN_PX)
    print("Px values x, y: ", px_x, px_y)
    """

    # test sort box by dist
    print()
    cur_x = 1000
    cur_y = 1000

    res_min = min_plant_box_dist(test_boxes, cur_x, cur_y)
    print(res_min.get_name(), res_min.get_confidence())

    res = sort_plant_boxes_dist(test_boxes, cur_x, cur_y)

    for plant_box in res:
        print(plant_box.get_name(), plant_box.get_confidence())

    print(res == res_min)

    # test sort box by confidence
    """
    print()
    res = sort_plant_boxes_conf(test_boxes)

    for plant_box in res:
        print(plant_box.get_name(), plant_box.get_confidence())
    """


if __name__ == "__main__":
    main()
