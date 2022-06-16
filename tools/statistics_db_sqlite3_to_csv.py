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
        working_times_sql = "SELECT * FROM working_times;"
        db_cursor.execute(working_times_sql)
        working_times_data = db_cursor.fetchall()

        delta_scans_amount_sql = "SELECT COUNT(*) FROM delta_scans;"
        db_cursor.execute(delta_scans_amount_sql)
        delta_scans_amount = db_cursor.fetchone()

        delta_scans_total_time_sql = "SELECT SUM(time_spent) FROM delta_scans;"
        db_cursor.execute(delta_scans_total_time_sql)
        delta_scans_total_time = db_cursor.fetchone()

        delta_scans_total_shifts_sql = "SELECT SUM(shifts_count) FROM delta_scans;"
        db_cursor.execute(delta_scans_total_shifts_sql)
        delta_scans_total_shifts = db_cursor.fetchone()

        # get tables structure
        tables_structure_sql = "PRAGMA table_info(working_times);"
        db_cursor.execute(tables_structure_sql)
        headers = db_cursor.fetchall()

        # write data to csv file
        with open(OUTPUT_CSV_FULL_PATH, "w") as csv_file:
            # write headers
            header_line = ""
            for header in headers:
                header_line += str(header[1]) + ","
            csv_file.write(header_line[:-1] + "\n")

            # write working_times table's data
            for row in working_times_data:
                data_line = ""
                for item in row:
                    if item == "":
                        item = "None"
                    elif type(item) == float:
                        item = round(item, 3)
                    data_line += str(item) + ","
                csv_file.write(data_line[:-1] + "\n")

            # write delta_scans table's data
            csv_file.write("\n")
            csv_file.write("Delta scans amount,Delta scans total time,Delta scans total shifts\n")
            csv_file.write(f"{str(delta_scans_amount[0])},{str(delta_scans_total_time[0])},{str(delta_scans_total_shifts[0])}")

        print("Done!")

    finally:
        db_cursor.close()
        db_connection.close()


if __name__ == '__main__':
    main()
