"""Makes a smaller copy of json for test purposes (to use smaller amounts of data)"""

import json

# SETTINGS
INPUT_JSON_PATH = "D:/Projects/Natuition/TASKS/n3 parsing data to database/bd/database_SN003.json"
OUTPUT_JSON_PATH = "D:/Projects/Natuition/TASKS/n3 parsing data to database/bd/database_SN003_19-20.json"

SN_TO_SAVE = ["SN003"]  # works on filter_sn call
SN_TO_CUT = ["SN002"]  # works on cut_specific_sns call
SESSION_TO_SAVE = [19, 20]  # works on filter_sessions call


def filter_sn():
    print("Loading input json...")
    with open(INPUT_JSON_PATH) as in_json_file:
        inp_data = json.loads(in_json_file.read())
    print("Loading input json is done.")

    print("Writing piece of json...")
    out_data = dict()
    for sn_key in inp_data:
        if sn_key in SN_TO_SAVE:
            out_data[sn_key] = inp_data[sn_key]

    with open(OUTPUT_JSON_PATH, "w") as out_json_file:
        out_json_file.write(json.dumps(out_data))
    print("Done!")


def filter_sessions():
    print("Loading input json...")
    with open(INPUT_JSON_PATH) as inp_file:
        inp_data = json.loads(inp_file.read())
    print("Loaded.")

    out_data = dict()
    for sn_key in inp_data:
        for session_idx, session_key in enumerate(inp_data[sn_key]):
            if session_idx in SESSION_TO_SAVE:
                if sn_key not in out_data:
                    out_data[sn_key] = dict()
                out_data[sn_key][session_key] = inp_data[sn_key][session_key]
        break  # process only one sn

    print("Saving out json...")
    with open(OUTPUT_JSON_PATH, "w") as out_file:
        out_file.write(json.dumps(out_data))
    print("Done!")


def cut_specific_sns():
    raise NotImplementedError


def show_sns_list():
    with open(INPUT_JSON_PATH) as db_file:
        data = json.loads(db_file.read())
    for sn in data:
        print(sn)


if __name__ == '__main__':
    # filter_sn()
    filter_sessions()
    # show_sns_list()
