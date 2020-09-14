"""Changes px coordinates of poly points, adapting them from using on original image to cropped"""

from config import config


POLY_POINTS = [[1611, 1292], [804, 1286], [803, 1236], [804, 1135], [812, 1057], [821, 999], [840, 923], [872, 801], [908, 715], [1010, 663], [1085, 629], [1181, 592], [1279, 565], [1361, 548], [1473, 534], [1572, 527], [1670, 529], [1791, 542], [1893, 558], [1983, 582], [2077, 614], [2154, 646], [2254, 695], [2281, 750], [2314, 844], [2347, 960], [2367, 1057], [2377, 1153], [2382, 1279]]
OUTPUT_FILE = "adapted poly points.txt"


def main():
    for i in range(len(POLY_POINTS)):
        # """
        POLY_POINTS[i][0] -= config.CROP_W_FROM
        POLY_POINTS[i][1] -= config.CROP_H_FROM
        """
        POLY_POINTS[i][0] = POLY_POINTS[i][0] - config.CROP_W_FROM + 114 + 5
        POLY_POINTS[i][1] = POLY_POINTS[i][1] - config.CROP_H_FROM + 266 - 55
        """

    print(POLY_POINTS)

    with open(OUTPUT_FILE, "w") as file:
        file.write(str(POLY_POINTS))


if __name__ == '__main__':
    main()
