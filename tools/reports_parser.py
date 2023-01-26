"""Parses robots reports and stocks parsed data as json

Expected dirs tree: data_root_dir/any_robot_SN/any_session_name/txt_session_files

Output format:

{
    robot_sn: {
        robot_session: {
            "field": [lat: float, lon: float],
            "field_name": str,
            "path_and_ext": [
                {
                    "point": [point_1_lat: float, point_1_lon: float, point_1_quality: int],
                    "extractions": {"plant_name_1": amount: int, "plant_name_2": amount: int, ...}
                },
                {
                    "point": [point_2_lat: float, point_2_lon: float, point_2_quality: int],
                    "extractions": {"plant_name_1": amount: int, "plant_name_2": amount: int, ...}
                }, ...
            ],
            "start_time": {
                "year": int,
                "month": int,
                "day": int,
                "hour": int,
                "minute": int,
                "second": int
            },
            "end_time": {
                "year": int,
                "month": int,
                "day": int,
                "hour": int,
                "minute": int,
                "second": int
            },
            "start_voltage": float
        }
    }
}
"""

import json
import glob
import platform
import os
import re

# PATHS
INPUT_DATA_ROOT_DIR = "D:/Projects/Natuition/TASKS/n3 parsing data to database/bd/"
OUTPUT_DB_JSON = "D:/Projects/Natuition/TASKS/n3 parsing data to database/bd/database.json"
OUTPUT_ERRORS_LOG_FILE = "D:/Projects/Natuition/TASKS/n3 parsing data to database/bd/errors.txt"
OUTPUT_ABSENT_PATH_LOG_FILE = "D:/Projects/Natuition/TASKS/n3 parsing data to database/bd/absent_path_gps_with_extract.txt"

# SETTINGS
PRINT_EVERY_SESSIONS = 10
OS_WINDOWS = platform.system() == "Windows"
DIR_SLASH = "\\" if OS_WINDOWS else "/"
SESSION_FILE_SEP = " : "
GPS_EXT_SEP = " : "
IGNORE_PLANTS = 5  # do not add ext data of 1rst point in path_gps_with_extract.txt if this 1rst point has more extractions than this value
SESSION_DATE_PATTERN = "[0-9][0-9]-[0-9][0-9]-[0-9][0-9][0-9][0-9]"
SESSION_TIME_PATTERN = "[0-9][0-9]-[0-9][0-9]-[0-9][0-9]"
LAT_LON_PATTERN = r"-?[0-9]+\.[0-9]*"
PTS_QUALITY_PATTERN = "[0-9]"

# SYSTEMS SETTINGS, DON'T EDIT
FILES_HANDLERS = [
    "field.txt",
    "field_name.txt",
    "path_gps_with_extract.txt",
    "session_resume.txt",
    "session_resume_error.txt"]
SESSION_RESUME_KEYS_HANDLERS = [
    "Start time",
    "Voltage at start",
    "Treated plant",
    "Extraction number",
    "End time"]


# FILE HANDLERS

def field_handler(file_path: str):
    """Processes by given path field.txt file.

    Returns list of 4 points [lat, lon] and list of errors encountered
    """

    errors = []
    field = []

    with open(file_path) as field_file:
        for idx, line in enumerate(field_file):
            if line != "" and line != "\n":
                if "[" in line and "]" in line:
                    point = line[line.find("[") + 1:line.find("]")].strip().split(", ")
                    if len(point) != 2:
                        msg = f"File {file_path} line {str(idx + 1)} contains corrupted data, parsed " \
                              f"{str(len(point))} points instead of two."
                        print(msg)
                        errors.append(msg)
                        ask_for_exit()
                        continue
                    point[0], point[1] = float(point[0]), float(point[1])
                    field.append(point)
                else:
                    msg = f"File {file_path} line {str(idx + 1)} contains corrupted or out of format data, " \
                          f"couldn't find list brackets '[]' of points list, parsing this point is failed."
                    print(msg)
                    errors.append(msg)
                    ask_for_exit()
                    continue

    if len(field) != 4:
        msg = f"Parsed {str(len(field))} field points instead of 4 in file {file_path}"
        print(msg)
        errors.append(msg)
        ask_for_exit()

    return field, errors


def field_name_handler(file_path: str):
    """Processes by given path field_name.txt file.

    Returns str name. Name may be empty "" str.
    """

    with open(file_path) as field_name_file:
        return field_name_file.readline().replace("\n", "")


def path_with_gps_extract_handler(file_path: str):
    """Processes by given path path_with_gps_extract.txt file.

    Returns list of dicts {"point": [lat, lon, quality], "extractions": {"plant_name_1": count, ...}} and errors list
    "extractions" is optionally-present key. May be absent if there's no records about extractions at the point.
    """

    errors = []
    path = []

    with open(file_path) as gps_ext_file:
        for idx, record in enumerate(gps_ext_file):
            record = record.strip()
            if record != "":
                record = record.split(GPS_EXT_SEP)

                # process gps point
                if "[" in record[0] and "]" in record[0]:
                    record[0] = record[0][record[0].find("[") + 1:record[0].find("]")].replace("'", "").split(", ")

                    if len(record[0]) != 3:
                        msg = f"File {file_path} line {str(idx + 1)} contains corrupted data, couldn't extract 3 items"
                        print(msg)
                        errors.append(msg)
                        # ask_for_exit()
                        continue
                    if not (re.match(LAT_LON_PATTERN, record[0][0]) and re.match(LAT_LON_PATTERN, record[0][1])):
                        msg = f"File {file_path} line {str(idx + 1)} lat '{str(record[0][0])}' or lon " \
                              f"'{str(record[0][1])}' are not matching '{LAT_LON_PATTERN}' format pattern"
                        print(msg)
                        errors.append(msg)
                        print("This point will be skipped")
                        # ask_for_exit()
                        continue
                    if not re.match(PTS_QUALITY_PATTERN, record[0][2][0]):  # TODO cur reg ex patterns doesn't work strictly so [0] is used
                        msg = f"File {file_path} line {str(idx + 1)} point quality '{record[0][2]}' is not matching" \
                              f"'{PTS_QUALITY_PATTERN}' format pattern"
                        print(msg)
                        errors.append(msg)
                        print("This point will be skipped")
                        # ask_for_exit()
                        continue

                    record[0][0], record[0][1], record[0][2] = float(record[0][0]), float(record[0][1]), int(
                        record[0][2][0])
                else:
                    msg = f"File {file_path} contains no GPS points at {str(idx + 1)} line!"
                    print(msg)
                    errors.append(msg)
                    print("This point will be skipped")
                    # ask_for_exit()
                    continue

                # process extractions if present
                if len(record) == 2:
                    if record[1].startswith("{") and record[1].endswith("}"):
                        # check 1rst line if it contains previous sessions data (1rst point total weeds > settings val)
                        record[1] = json.loads(record[1].replace("'", '"'))
                        if idx == 0 and sum(record[1].values()) > IGNORE_PLANTS:
                            del record[1]
                    else:
                        msg = f"File {file_path} contains corrupted extractions data at {str(idx + 1)} line!"
                        print(msg)
                        errors.append(msg)
                        print("This point will be skipped")
                        # ask_for_exit()
                        continue

                # save to path
                point = {"point": record[0]}
                if len(record) == 2:
                    point["extractions"] = record[1]
                path.append(point)

    return path, errors


def session_resume_handler(file_path: str):
    """
    """

    data = dict()
    errors = []

    with open(file_path) as session_file:
        for idx, line in enumerate(session_file):
            line = line.strip()
            split_line = line.split(SESSION_FILE_SEP)

            # extract Start time
            if line.startswith("Start time"):
                if len(split_line) < 2:
                    msg = f"File {file_path} line {str(idx + 1)} contains corrupted data or separators, found " \
                          f"'Start time' key, separator '{SESSION_FILE_SEP}', but couldn't access value."
                    print(msg)
                    errors.append(msg)
                    ask_for_exit()
                    continue

                split_line = split_line[1].split(" ")[:2]  # ['24-08-2022', '07-35-36']

                if not (re.match(SESSION_DATE_PATTERN, split_line[0]) and re.match(SESSION_TIME_PATTERN, split_line[1])):
                    msg = f"File {file_path} line {str(idx + 1)} date or time is not matching expected " \
                          f"date '{split_line[0]}' to '{SESSION_DATE_PATTERN}' or time '{split_line[1]}' to " \
                          f"'{SESSION_TIME_PATTERN}' format, can't parse it"
                    print(msg)
                    errors.append(msg)
                    ask_for_exit()
                    continue

                day, month, year = map(int, split_line[0].split("-"))
                hour, minute, second = map(int, split_line[1].split("-"))
                data["start_time"] = dict()
                data["start_time"]["day"] = day
                data["start_time"]["month"] = month
                data["start_time"]["year"] = year
                data["start_time"]["hour"] = hour
                data["start_time"]["minute"] = minute
                data["start_time"]["second"] = second

            # extract Voltage at start
            elif line.startswith("Voltage at start"):
                if len(split_line) < 2:
                    msg = f"File {file_path} line {str(idx + 1)} contains corrupted data or separators, found " \
                          f"'Voltage at start' key, separator '{SESSION_FILE_SEP}', but couldn't access value."
                    print(msg)
                    errors.append(msg)
                    ask_for_exit()
                    continue

                try:
                    data["start_voltage"] = float(split_line[1])
                except ValueError:
                    msg = f"File {file_path} line {str(idx + 1)} contains wrong voltage value " \
                          f"(failed to convert to float)"
                    print(msg)
                    errors.append(msg)
                    ask_for_exit()
                    continue

                """
                # extract Extraction number
                elif line.startswith("Extraction number"):
                    if len(line) < 2:
                        msg = f"File {file_path} line {str(idx + 1)} contains corrupted data or separators, found " \
                              f"'Extraction number' key, separator '{SESSION_FILE_SEP}', but couldn't access value."
                        print(msg)
                        ask_for_exit()
                        continue
                """

            # extract End time
            elif line.startswith("End time"):
                if len(split_line) < 2:
                    msg = f"File {file_path} line {str(idx + 1)} contains corrupted data or separators, found " \
                          f"'End time' key, separator '{SESSION_FILE_SEP}', but couldn't access value."
                    print(msg)
                    errors.append(msg)
                    ask_for_exit()
                    continue

                split_line = split_line[1].split(" ")[:2]  # ['24-08-2022', '07-35-36']

                if not (re.match(SESSION_DATE_PATTERN, split_line[0]) and re.match(SESSION_TIME_PATTERN, split_line[1])):
                    msg = f"File {file_path} line {str(idx + 1)} date or time is not matching expected " \
                          f"date '{SESSION_DATE_PATTERN}' and time '{SESSION_TIME_PATTERN}' format, can't parse it"
                    print(msg)
                    errors.append(msg)
                    ask_for_exit()
                    continue

                day, month, year = map(int, split_line[0].split("-"))
                hour, minute, second = map(int, split_line[1].split("-"))
                data["end_time"] = dict()
                data["end_time"]["day"] = day
                data["end_time"]["month"] = month
                data["end_time"]["year"] = year
                data["end_time"]["hour"] = hour
                data["end_time"]["minute"] = minute
                data["end_time"]["second"] = second

            elif split_line[0] in SESSION_RESUME_KEYS_HANDLERS:
                pass
            else:
                msg = f"File {file_path} line {str(idx + 1)} contains unrecognized key {split_line[0]}"
                print(msg)
                errors.append(msg)
                ask_for_exit()
                continue

    return data, errors


def ask_for_exit():
    cmd = input("Want to continue? y/n: ")
    if cmd.lower() == "n":
        exit()


def write_errors_to_log(errors: list, file_obj):
    for line in errors:
        file_obj.write(line + "\n")
    file_obj.flush()


def main():
    global INPUT_DATA_ROOT_DIR
    if not INPUT_DATA_ROOT_DIR.endswith("/"):
        INPUT_DATA_ROOT_DIR += "/"

    data = dict()

    with \
            open(OUTPUT_ERRORS_LOG_FILE, "w") as errors_file, \
            open(OUTPUT_ABSENT_PATH_LOG_FILE, "w") as absent_gps_file:

        # loop over robots
        robot_sn_dirs = list(filter(os.path.isdir, glob.glob(INPUT_DATA_ROOT_DIR + "*")))
        for sn_idx, robot_sn_dir in enumerate(robot_sn_dirs):
            print(f"Processing {str(sn_idx + 1)} of {str(len(robot_sn_dirs))} robots")

            robot_sn = robot_sn_dir[robot_sn_dir.rfind(DIR_SLASH) + len(DIR_SLASH):].strip()
            data[robot_sn] = dict()
            robot_sn_dir += "/"

            # loop over robot sessions
            robot_session_dirs = list(filter(os.path.isdir, glob.glob(robot_sn_dir + "*")))
            for session_idx, robot_session_dir in enumerate(robot_session_dirs):
                if session_idx % PRINT_EVERY_SESSIONS == 0:
                    msg = f"Processing {str(session_idx + 1)} of {str(len(robot_session_dirs))} sessions " \
                          f"of {str(sn_idx + 1)} robot"
                    print(msg)

                robot_session = robot_session_dir[robot_session_dir.rfind(DIR_SLASH) + len(DIR_SLASH):].strip()
                data[robot_sn][robot_session] = dict()
                cur_session_files = []
                robot_session_dir += "/"

                # loop over files in a session
                for session_file in glob.glob(robot_session_dir + "*.txt"):
                    cur_session_files.append(
                        session_file[session_file.rfind(DIR_SLASH) + len(DIR_SLASH):].strip().lower())

                if "path_gps_with_extract.txt" not in cur_session_files:
                    absent_gps_file.write(robot_session_dir + "\n")
                    absent_gps_file.flush()
                    continue

                field, field_errors = None, []
                field_name = None
                gps_path, gps_path_errors = None, []
                session_resume, session_resume_errors = None, []

                for session_file in cur_session_files:
                    if session_file == "field.txt":
                        field, field_errors = field_handler(robot_session_dir + session_file)
                    elif session_file == "field_name.txt":
                        field_name = field_name_handler(robot_session_dir + session_file)
                    elif session_file == "path_gps_with_extract.txt":
                        gps_path, gps_path_errors = path_with_gps_extract_handler(robot_session_dir + session_file)
                    elif session_file == "session_resume.txt":
                        session_resume, session_resume_errors = session_resume_handler(robot_session_dir + session_file)
                    elif session_file == "session_resume_error.txt":
                        pass
                    else:
                        msg = f"Found an unexpected txt file: {robot_session_dir + session_file}"
                        print(msg)
                        errors_file.write(msg + "\n")
                        errors_file.flush()

                # check parsed data
                if field is None or len(field_errors) != 0:
                    msg = f"SKIPPED SESSION {robot_session_dir} due to field.txt file absense or parsing errors."
                    print(msg)
                    field_errors.append(msg)
                    write_errors_to_log(field_errors, errors_file)
                    continue
                elif field_name is None or field_name == "":
                    field_name = "None"
                elif session_resume is None or len(session_resume_errors) != 0:
                    msg = f"SKIPPED SESSION {robot_session_dir} due to session_resume.txt file absense or parsing " \
                          f"errors."
                    print(msg)
                    session_resume_errors.append(msg)
                    write_errors_to_log(session_resume_errors, errors_file)
                    continue
                elif gps_path is None:
                    msg = f"SKIPPED SESSION {robot_session_dir} due to path_gps_with_extract.txt file absense or" \
                          f"parsing errors."
                    print(msg)
                    gps_path_errors.append(msg)
                    write_errors_to_log(gps_path_errors, errors_file)
                    continue
                elif len(gps_path_errors) != 0:
                    write_errors_to_log(gps_path_errors, errors_file)

                # add parsed data
                # data[robot_sn][robot_session] = dict()
                data[robot_sn][robot_session]["field"] = field
                data[robot_sn][robot_session]["field_name"] = field_name
                data[robot_sn][robot_session]["path_and_ext"] = gps_path
                data[robot_sn][robot_session]["start_time"] = session_resume["start_time"]
                data[robot_sn][robot_session]["end_time"] = session_resume["end_time"]
                data[robot_sn][robot_session]["start_voltage"] = session_resume["start_voltage"]

    print("Saving DB...")
    with open(OUTPUT_DB_JSON, "w") as db_json_file:
        db_json_file.write(json.dumps(data))
    print("Done!")


if __name__ == '__main__':
    main()
