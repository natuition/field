"""Makes list of all files names variations and encountered in session_resume.txt file data keys

Saves data in txt files
Expected dirs tree: data_root_dir/any_robot_SN/any_session_name/txt_session_files
"""

import glob
import os
import platform

# PATHS
INPUT_DATA_ROOT_DIR = "D:/Projects/Natuition/TASKS/n3 parsing data to database/bd/"
OUTPUT_FILES_CASES_FILE = "files_cases.txt"
OUTPUT_KEYS_CASES_FILE = "keys_cases.txt"

# SETTINGS
OS_WINDOWS = platform.system() == "Windows"
DIR_SLASH = "\\" if OS_WINDOWS else "/"
SESSION_FILE_SEP = " : "


def combine_path(path: str):
    """Input full path, output file name and full path"""

    return path[path.rfind(DIR_SLASH):] + " : " + path


def get_file_name(path: str):
    return path[path.rfind(DIR_SLASH) + 1 if OS_WINDOWS else 0:]


def analyze_input_data_cases():
    """Makes list of all files names variations and encountered in session_resume.txt file data keys

    Saves data in txt files
    Expected dirs tree: data_root_dir/any_robot_SN/any_session_name/txt_session_files
    """

    global INPUT_DATA_ROOT_DIR
    if not INPUT_DATA_ROOT_DIR.endswith("/"):
        INPUT_DATA_ROOT_DIR += "/"

    files_cases = set()
    keys_cases = []

    for robot_sn_dir in filter(os.path.isdir, glob.glob(INPUT_DATA_ROOT_DIR + "*")):
        robot_sn_dir += "/"
        for robot_session_dir in filter(os.path.isdir, glob.glob(robot_sn_dir + "*")):
            robot_session_dir += "/"

            # add encountered txts as tuple into files cases set (saves only unique variations of txt files)
            files_cases.add(
                tuple(sorted(map(get_file_name, filter(os.path.isfile, glob.glob(robot_session_dir + "*.txt")))))
            )

            # add encountered in session_resume.txt file data keys into set (each key is saved only 1 time)
            session_file_path = robot_session_dir + "session_resume.txt"
            if os.path.isfile(session_file_path):
                with open(session_file_path) as session_file:
                    for line in session_file:
                        if line != "" and line != "\n":
                            line_items = line.split(SESSION_FILE_SEP)
                            if len(line_items) > 0:
                                key_case = line_items[0].strip()
                                if key_case not in keys_cases:
                                    keys_cases.append(key_case)

    # save sessions files cases
    with open(INPUT_DATA_ROOT_DIR + OUTPUT_FILES_CASES_FILE, "w") as files_cases_rep_file:
        files_cases_total = set()
        session_items: tuple
        for session_items in files_cases:
            for file_name in session_items:
                files_cases_total.add(file_name)
                files_cases_rep_file.write(file_name + "\n")
            files_cases_rep_file.write("\n\n")

        files_cases_rep_file.write("=== Files total cases ===\n")
        for file_name in sorted(files_cases_total):
            files_cases_rep_file.write(file_name + "\n")

    # save session resume (stats file) encountered keys
    with open(INPUT_DATA_ROOT_DIR + OUTPUT_KEYS_CASES_FILE, "w") as keys_cases_rep_file:
        keys_cases_rep_file.write("=== Encountered keys in all session_resume.txt files ===\n")
        for key_case in keys_cases:
            keys_cases_rep_file.write(key_case + "\n")

    print("Done!")


if __name__ == '__main__':
    analyze_input_data_cases()
