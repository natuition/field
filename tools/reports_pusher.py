"""Loads given json and pushes all the data to remote DB using web API"""

import json
import requests
import datetime
import pytz
import contextlib
import websocket
import time
import sys

# === SETTINGS ===
# === PATHS ===
# INPUT_JSON_PATH = "database.json"
INPUT_JSON_PATH = "D:/Projects/Natuition/TASKS/n3 parsing data to database/bd/database.json"

# === REMOTE ===
# SERVER_HTTP_ADDRESS = "http://localhost:8080"
# SERVER_WS_IP_ADDRESS = "localhost"
SERVER_HTTP_ADDRESS = "http://172.16.3.5:8080"
SERVER_WS_IP_ADDRESS = "172.16.3.5"

SERVER_WS_PORT = 8080

POST_WEED_TYPE = "/api/v1/data_gathering/weed_type"     # {"label": "Daisy"}
GET_WEED_TYPES = "/api/v1/data_gathering/weeds_types"   # [{"label": "Daisy", "id": 0}]

POST_ROBOT_SN = "/api/v1/data_gathering/robot"          # {"serial_number": "SNXXX"}
GET_ROBOTS_SN = "/api/v1/data_gathering/robots"         # [{"serial_number": "SN002"}]

POST_FIELD = "/api/v1/data_gathering/field"             # {"label": "Field 1", "robot_serial_number": "SNXXX"} # 200-201
GET_FIELD = "/api/v1/data_gathering/fields"             # [{"label": "Field 1","robot_serial_number": "SNXXX","id": 1}]

POST_GPS_POINT = "/api/v1/data_gathering/gps_point"     # {"quality": 4, "latitude": 46.15763837573922, "longitude": -1.1350485428751012, "id": 1} # 201
GET_GPS_POINTS = "/api/v1/data_gathering/gps_points"    # [{"quality": 4,"latitude": 46.15763837573922, "longitude": -1.1350485428751012, "id": 1}] # 201

POST_FIELD_CORNER = "/api/v1/data_gathering/field_corner"  # {"field_id": 1, "gps_point_id": 1, "id": 1} # 201
GET_FIELDS_CORNERS = "/api/v1/data_gathering/fields_corners"

POST_SESSION = "/api/v1/data_gathering/session"  # 201
GET_SESSIONS = "/api/v1/data_gathering/sessions"  # 200

POST_VESC_STAT = "/api/v1/data_gathering/vesc_statistic"  # 201
GET_VESC_STATS = "/api/v1/data_gathering/vesc_statistics"  # 200

POST_PATH_RECORD = "/api/v1/data_gathering/point_of_path"  # 201
GET_PATHS_RECORDS = "/api/v1/data_gathering/points_of_paths"  # 200

POST_EXTRACTED_WEED = "/api/v1/data_gathering/extracted_weed"  # 201
GET_EXTRACTED_WEEDS = "/api/v1/data_gathering/extracted_weeds"  # 200

# === APP ===
REQUESTS_TIMEOUT = 10  # seconds; raise an exception if no cnnection or response more than this value
EXISTING_CODE = 200
CREATED_CODE = 201
SUCCESS_CODES = [EXISTING_CODE, CREATED_CODE]
SHOW_PROGRESS = True
PROGRESS_STEP = 1
POINTS_PROGRESS_STEP = 100
DELAY_SESSIONS_SENDING = 1
SN_SKIP_LIST = []
# True: use faster web socket connection instead of slower http for gps points and extractions data sending
# note: WS api has no sending success confirmation
USE_WEB_SOCKET = True
WS_MAX_PTS_PER_OPERATION = 50  # max amount of points sent via single 'package' of web socket
WS_SENDING_TICK = 0.25  # sec; will not send points more often than this value


def get_current_time():
    """Returns current time as formatted string"""

    return datetime.datetime.now(pytz.timezone('Europe/Berlin')).strftime("%d-%m-%Y %H-%M-%S %f")


def format_db_timestamp(d: dict):
    """Formats given time dict and returns as UTC time format str.

    data: {
        "year": int,
        "month": int,
        "day": int,
        "hour": int,
        "minute": int,
        "second": int
    }
    "2023-01-20T13:09:17.397Z"
    """

    return f"{str(d['year'])}-{str(d['month'])}-{str(d['day'])}T{str(d['hour'])}:{str(d['minute'])}:{str(d['second'])}.0Z"


def send_path_http(cur_session_data: dict, cur_session_id: int, weeds_types: dict):
    # post trajectory (path) points and extractions if were any
    for cur_path_pt_rec_idx, cur_path_pt_rec in enumerate(cur_session_data["path_and_ext"]):
        if SHOW_PROGRESS and cur_path_pt_rec_idx % POINTS_PROGRESS_STEP == 0:
            print(f"Pushing {cur_path_pt_rec_idx + 1}th of {len(cur_session_data['path_and_ext'])} points")

        # post path gps point to GPS_points
        res = requests.post(
            SERVER_HTTP_ADDRESS + POST_GPS_POINT,
            json={
                "quality": cur_path_pt_rec["point"][2],
                "latitude": cur_path_pt_rec["point"][0],
                "longitude": cur_path_pt_rec["point"][1]},
            timeout=REQUESTS_TIMEOUT)
        if res.status_code != CREATED_CODE:
            msg = f"Couldn't post path's GPS point " \
                  f"'{str({'quality': cur_path_pt_rec['point'][2], 'latitude': cur_path_pt_rec['point'][0], 'longitude': cur_path_pt_rec['point'][1]})}'\n" \
                  f"to {SERVER_HTTP_ADDRESS + POST_GPS_POINT}\n" \
                  f"Response code: {str(res.status_code)}\n" \
                  f"Response content: {str(res.content)}"
            print(msg)
            exit()
        cur_path_pt_id = json.loads(res.content)["id"]

        # post record to Points_of_paths
        res = requests.post(
            SERVER_HTTP_ADDRESS + POST_PATH_RECORD,
            timeout=REQUESTS_TIMEOUT,
            json={"point_number": cur_path_pt_rec_idx,
                  "session_id": cur_session_id,
                  "gps_point_id": cur_path_pt_id})
        if res.status_code != CREATED_CODE:
            msg = f"Couldn't post path's point record " \
                  f"{str({'point_number': cur_path_pt_rec_idx, 'session_id': cur_session_id, 'gps_point_id': cur_path_pt_id})}\n" \
                  f"to {SERVER_HTTP_ADDRESS + POST_PATH_RECORD}\n" \
                  f"Response code: {str(res.status_code)}\n" \
                  f"Response content: {str(res.content)}"
            print(msg)
            exit()
        cur_path_rec_id = json.loads(res.content)["id"]

        # post extractions data if there were any
        if "extractions" in cur_path_pt_rec:
            # loop over extracted plants types and amounts list
            for weed_type in cur_path_pt_rec["extractions"]:
                # post weed type to Weed_types if it is not exist in DB
                if weed_type not in weeds_types:
                    res = requests.post(
                        SERVER_HTTP_ADDRESS + POST_WEED_TYPE,
                        timeout=REQUESTS_TIMEOUT,
                        json={"label": weed_type}
                    )
                    if res.status_code not in SUCCESS_CODES:
                        msg = f"Couldn't post weed {str({'label': weed_type})}\n" \
                              f"to {SERVER_HTTP_ADDRESS + POST_WEED_TYPE}\n" \
                              f"Response code: {str(res.status_code)}\n" \
                              f"Response content: {str(res.content)}"
                        print(msg)
                        exit()
                    weeds_types[weed_type] = json.loads(res.content)["id"]

                # post point's extracted cur weed type amount to Extracted_weeds
                res = requests.post(
                    SERVER_HTTP_ADDRESS + POST_EXTRACTED_WEED,
                    timeout=REQUESTS_TIMEOUT,
                    json={
                        "point_of_path_id": cur_path_rec_id,
                        "weed_type_id": weeds_types[weed_type],
                        "session_id": cur_session_id,
                        "number": cur_path_pt_rec["extractions"][weed_type]}
                )
                if res.status_code != CREATED_CODE:
                    msg = f"Couldn't post extracted weeds " \
                          f"{str({'point_of_path_id': cur_path_rec_id, 'weed_type_id': weeds_types[weed_type], 'session_id': cur_session_id, 'number': cur_path_pt_rec['extractions'][weed_type]})}\n" \
                          f"to {SERVER_HTTP_ADDRESS + POST_EXTRACTED_WEED}\n" \
                          f"Response code: {str(res.status_code)}\n" \
                          f"Response content: {str(res.content)}"
                    print(msg)
                    exit()


def send_path_websocket(ip, port, cur_sn, cur_session_id, cur_session_data: dict):
    ws: websocket.WebSocket
    with contextlib.closing(websocket.create_connection(
            f"ws://{ip}:{port}/api/v1/data_gathering/ws/{cur_sn}/{cur_session_id}")) as ws:

        pts_buf = []
        last_send_t = 0

        # post trajectory (path) points and extractions if were any
        for cur_path_pt_rec_idx, cur_path_pt_rec in enumerate(cur_session_data["path_and_ext"]):
            if SHOW_PROGRESS and cur_path_pt_rec_idx % POINTS_PROGRESS_STEP == 0:
                print(f"Pushing {cur_path_pt_rec_idx + 1}th of {len(cur_session_data['path_and_ext'])} points")

            # add gps point and its number
            cur_pt_for_ws = {  # forming point in web socket api format
                "current_coordinate": [
                    cur_path_pt_rec["point"][0],
                    cur_path_pt_rec["point"][1],
                    cur_path_pt_rec["point"][2]
                ],
                "path_point_number": cur_path_pt_rec_idx
            }

            # add extractions to this point if there were any
            if "extractions" in cur_path_pt_rec:
                cur_pt_for_ws["extracted_weeds"] = cur_path_pt_rec["extractions"]

            # add point to buffer (for sending multiple points per single 'package')
            pts_buf.append(cur_pt_for_ws)

            # send no more than max allowed points and send anyway if it is last point
            if len(pts_buf) == WS_MAX_PTS_PER_OPERATION or \
                    cur_path_pt_rec_idx + 1 == len(cur_session_data["path_and_ext"]):

                delay_t = WS_SENDING_TICK - (time.time() - last_send_t)
                if delay_t > 0:
                    time.sleep(delay_t)

                data_to_send = json.dumps({"coordinate_with_extracted_weed": pts_buf})
                # print(data_to_send, "\n")
                # bytes_to_send = sys.getsizeof(data_to_send)
                bytes_sent = ws.send(data_to_send)
                # if bytes_to_send != bytes_sent:
                    # print("Data:", bytes_to_send, "Sent:", bytes_sent)
                last_send_t = time.time()
                pts_buf.clear()


def main():
    # websocket.enableTrace(True)

    # load data from json
    print("Loading json...")
    with open(INPUT_JSON_PATH) as json_file:
        data = json.loads(json_file.read())
    print("Loaded.")

    # make lists of commonly used ids, etc.
    # get DB weeds list
    res = requests.get(SERVER_HTTP_ADDRESS + GET_WEED_TYPES, timeout=REQUESTS_TIMEOUT)
    if res.status_code != EXISTING_CODE:
        msg = f"Couldn't get weeds ids from {SERVER_HTTP_ADDRESS + GET_WEED_TYPES}\n" \
              f"Response code: {str(res.status_code)}\n" \
              f"Response content: {str(res.content)}"
        print(msg)
        exit()
    res = json.loads(res.content)  # [{"label": "Weed_1", "id": 1}, ...]
    weeds_types = dict()
    for d in res:
        weeds_types[d["label"]] = d["id"]

    # get SNs list
    res = requests.get(SERVER_HTTP_ADDRESS + GET_ROBOTS_SN, timeout=REQUESTS_TIMEOUT)
    if res.status_code != EXISTING_CODE:
        msg = f"Couldn't get robots SNs ids from {SERVER_HTTP_ADDRESS + GET_ROBOTS_SN}\n" \
              f"Response code: {str(res.status_code)}\n" \
              f"Response content: {str(res.content)}"
        print(msg)
        exit()
    res = json.loads(res.content)  # [{"serial_number": "SN002"}, ...]
    robots_sns = [d["serial_number"] for d in res]

    # loop over robots
    for cur_sn_idx, cur_sn in enumerate(data):
        if SHOW_PROGRESS:
            print(f"Pushing {cur_sn} ({cur_sn_idx + 1}th of {len(data)})")

        # check for non emptiness
        if len(data[cur_sn]) == 0:
            continue

        # db ignore list
        if cur_sn in SN_SKIP_LIST:
            continue

        cur_sn_data = data[cur_sn]

        # post robot SN to Robots
        if cur_sn not in robots_sns:
            res = requests.post(
                SERVER_HTTP_ADDRESS + POST_ROBOT_SN,
                json={"serial_number": cur_sn},
                timeout=REQUESTS_TIMEOUT)
            if res.status_code != CREATED_CODE:
                if res.status_code == 400:  # in case sn was added during script work
                    pass
                else:
                    msg = f"Failed to post current robot SN '{cur_sn}'\n" \
                          f"to {SERVER_HTTP_ADDRESS + POST_ROBOT_SN}\n" \
                          f"Response code: {str(res.status_code)}\n" \
                          f"Response content: {str(res.content)}"
                    print(msg)
                    exit()
            robots_sns.append(cur_sn)

        # loop over current SN robot's sessions
        for cur_session_name_idx, cur_session_name in enumerate(cur_sn_data):
            if SHOW_PROGRESS and cur_session_name_idx % PROGRESS_STEP == 0:
                print(f"Pushing {cur_session_name_idx + 1}th of {len(cur_sn_data)} sessions")

            # check for non emptiness
            if len(cur_sn_data[cur_session_name]) == 0:
                continue

            cur_session_data = cur_sn_data[cur_session_name]

            # post field to Fields
            cur_field_name = cur_session_data["field_name"]
            if cur_field_name == "None":
                cur_field_name += get_current_time()
            res = requests.post(
                SERVER_HTTP_ADDRESS + POST_FIELD,
                json={"label": cur_field_name, "robot_serial_number": cur_sn},
                timeout=REQUESTS_TIMEOUT)
            if res.status_code == CREATED_CODE:  # if field wasn't existing and was just created
                cur_field_id = json.loads(res.content)["id"]

                # post field corners to GPS_points
                cur_field_gps_points_ids = []
                for cur_field_gps_point in cur_session_data["field"]:
                    res = requests.post(
                        SERVER_HTTP_ADDRESS + POST_GPS_POINT,
                        json={"quality": 4, "latitude": cur_field_gps_point[0], "longitude": cur_field_gps_point[1]},
                        timeout=REQUESTS_TIMEOUT)
                    if res.status_code != CREATED_CODE:
                        msg = f"Couldn't post GPS point " \
                              f"'{str({'quality': 4, 'latitude': cur_field_gps_point[0], 'longitude': cur_field_gps_point[1]})}'\n" \
                              f"to {SERVER_HTTP_ADDRESS + POST_GPS_POINT}\n" \
                              f"Response code: {str(res.status_code)}\n" \
                              f"Response content: {str(res.content)}"
                        print(msg)
                        exit()
                    cur_field_gps_points_ids.append(json.loads(res.content)["id"])

                # post field corners to Fields_corners
                for cur_field_gps_point_id in cur_field_gps_points_ids:
                    res = requests.post(
                        SERVER_HTTP_ADDRESS + POST_FIELD_CORNER,
                        timeout=REQUESTS_TIMEOUT,
                        json={"field_id": cur_field_id, "gps_point_id": cur_field_gps_point_id})
                    if res.status_code != CREATED_CODE:
                        msg = f"Couldn't post field corner\n" \
                              f"'{str({'field_id': cur_field_id, 'gps_point_id': cur_field_gps_point_id})}'\n" \
                              f"to {SERVER_HTTP_ADDRESS + POST_FIELD_CORNER}\n" \
                              f"Response code: {str(res.status_code)}\n" \
                              f"Response content: {str(res.content)}"
                        print(msg)
                        exit()
            elif res.status_code == EXISTING_CODE:
                cur_field_id = json.loads(res.content)["id"]
            else:
                msg = f"Couldn't post field '{str({'label': cur_field_name, 'robot_serial_number': cur_sn})}'\n" \
                      f"to {SERVER_HTTP_ADDRESS + POST_FIELD}\n" \
                      f"Response code: {str(res.status_code)}\n" \
                      f"Response content: {str(res.content)}"
                print(msg)
                exit()

            # post session to Sessions
            session_post_data = {
                "start_time": format_db_timestamp(cur_session_data["start_time"]),
                "end_time": format_db_timestamp(cur_session_data["end_time"]),
                "robot_serial_number": cur_sn,
                "field_id": cur_field_id,
                "previous_sessions_id": 0}  # TODO need to know what ID is using when session doesn't point to others
            res = requests.post(
                SERVER_HTTP_ADDRESS + POST_SESSION,
                timeout=REQUESTS_TIMEOUT,
                json=session_post_data)
            if res.status_code != CREATED_CODE:
                msg = f"Couldn't post session '{str(session_post_data)}'\n" \
                      f"to {SERVER_HTTP_ADDRESS + POST_SESSION}\n" \
                      f"Response code: {str(res.status_code)}\n" \
                      f"Response content: {str(res.content)}"
                print(msg)
                exit()
            cur_session_id = json.loads(res.content)["id"]

            # post vesc voltage to Vesc_statistics
            res = requests.post(
                SERVER_HTTP_ADDRESS + POST_VESC_STAT,
                timeout=REQUESTS_TIMEOUT,
                json={
                    "session_id": cur_session_id,
                    "voltage": cur_session_data["start_voltage"],
                    "timestamp": format_db_timestamp(cur_session_data["start_time"])
                }
            )

            # send points and extractions data via web socket or http
            if USE_WEB_SOCKET:
                send_path_websocket(
                    SERVER_WS_IP_ADDRESS,
                    SERVER_WS_PORT,
                    cur_sn,
                    cur_session_id,
                    cur_session_data)
            else:
                send_path_http(
                    cur_session_data,
                    cur_session_id,
                    weeds_types)

            if DELAY_SESSIONS_SENDING > 0:
                time.sleep(DELAY_SESSIONS_SENDING)

    print("Done!")


if __name__ == '__main__':
    main()
