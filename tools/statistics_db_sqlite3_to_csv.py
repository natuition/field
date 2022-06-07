import sqlite3
import sys
sys.path.append("../")

# SETTINGS
INPUT_DB_FULL_PATH = "../1_test_db.sqlite3"
OUTPUT_CSV_FULL_PATH = "../1_test_db.csv"


def main():
    try:
        db_connection = sqlite3.connect(INPUT_DB_FULL_PATH)
        db_cursor = db_connection.cursor()

        # get data from DB
        sql_query = """SELECT * FROM working_times;"""
        db_cursor.execute(sql_query)
        data = db_cursor.fetchall()
        sql_query = """PRAGMA table_info(working_times);"""
        db_cursor.execute(sql_query)
        headers = db_cursor.fetchall()

        # write data to csv file
        with open(OUTPUT_CSV_FULL_PATH, "w") as csv_file:
            # write headers
            header_line = ""
            for header in headers:
                header_line += str(header[1]) + ","
            csv_file.write(header_line[:-1] + "\n")

            # write data
            for row in data:
                data_line = ""
                for item in row:
                    if item == "":
                        item = "None"
                    elif type(item) == float:
                        item = round(item, 3)
                    data_line += str(item) + ","
                csv_file.write(data_line[:-1] + "\n")

        print("Done!")

    finally:
        db_cursor.close()
        db_connection.close()


if __name__ == '__main__':
    main()
