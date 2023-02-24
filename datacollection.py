"""This module provides tools for robot data and statistic collection on client (robot control script) side.
"""

import time
import datetime
import os
import pickle
import json
import sqlite3


class DataCollector:
    """This class stores and saves collected data as txt file, or (will be added later) sends data to the local
    database.

    In case of loading data errors will ignore that data and start counting from scratch.
    """

    def __init__(self,
                 db_full_path: str,
                 notification,
                 load_from_file=False,
                 file_path="",
                 ui_msg_queue=None,
                 dump_at_receiving=True):

        file_loading_fail = False

        if load_from_file:
            try:
                structure = self.__load_from_file(file_path)
                # data
                self.__detected_plants = structure["__detected_plants"]
                self.__extracted_plants = structure["__extracted_plants"]
                self.__vesc_moving_time = structure["__vesc_moving_time"]
                self.__cork_moving_time = structure["__cork_moving_time"]
                self.__previous_sessions_working_time = structure["__previous_sessions_working_time"]
            except KeyboardInterrupt:
                raise KeyboardInterrupt
            except EOFError:
                file_loading_fail = True
            except FileNotFoundError:
                file_loading_fail = True

        if not load_from_file or file_loading_fail:
            # data
            self.__detected_plants = dict()
            self.__extracted_plants = dict()
            self.__vesc_moving_time = 0
            self.__cork_moving_time = 0
            self.__previous_sessions_working_time = 0

        self.__start_time = time.time()
        self.__notification = notification
        self.__ui_msg_queue = ui_msg_queue
        self.__dump_at_receiving = dump_at_receiving
        self.__dump_file_path = file_path

        # init local DB
        self.__db_connection = self.__init_database_sqlite3(db_full_path)
        self.__db_cursor = self.__db_connection.cursor()
        # init data IDs
        self.__moving_t_id = self.__get_moving_t_id()
        self.__stopped_t_id = self.__get_stopped_t_id()
        self.__pdz_scan_t_id = self.__get_pdz_scan_t_id()
        self.__all_ext_t_id = self.__get_all_ext_t_id()
        # self.__all_ext_delta_t_id = self.__get_all_ext_delta_t_id()
        self.__all_ext_xy_t_id = self.__get_all_ext_xy_t_id()
        self.__all_ext_img_t_id = self.__get_all_ext_img_t_id()
        self.__all_ext_z_t_id = self.__get_all_ext_z_t_id()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()

    def close(self):
        self.set_stopped_t(self.__previous_sessions_working_time + self.get_session_working_time())

        self.__db_cursor.close()
        self.__db_connection.close()

    @staticmethod
    def __init_database_sqlite3(db_full_path):
        """Prepares to using a database.

        Creates local sqlite3 database and it's tables structure or connects if DB exists and makes sure it's structure
        is correct for further statistics saving.

        :return: connection - sqlite3.Connection
        """

        connection = sqlite3.connect(db_full_path)
        cursor = connection.cursor()

        # create DB structure
        sql_working_times = """
        CREATE TABLE IF NOT EXISTS working_times (
            id INTEGER PRIMARY KEY,
            name TEXT NOT NULL UNIQUE,
            time_spent REAL NOT NULL,
            description TEXT
        );
        """
        sql_working_times_relations = """
        CREATE TABLE IF NOT EXISTS working_times_relations (
            id INTEGER PRIMARY KEY,
            working_times_id_child INTEGER NOT NULL,
            working_times_id_parent INTEGER NOT NULL CHECK(working_times_id_child != working_times_id_parent),
            FOREIGN KEY (working_times_id_child) REFERENCES working_times (id),
            FOREIGN KEY (working_times_id_parent) REFERENCES working_times (id)
        );
        """
        sql_delta_scans = """
        CREATE TABLE IF NOT EXISTS delta_scans (
            id INTEGER PRIMARY KEY,
            time_spent REAL NOT NULL,
            shifts_count INTEGER NOT NULL
        );
        """

        cursor.execute(sql_working_times)
        cursor.execute(sql_working_times_relations)
        cursor.execute(sql_delta_scans)
        cursor.close()
        return connection

    def __get_moving_t_id(self):
        """moving time
        """

        get_db_id_sql = """SELECT id FROM working_times WHERE name = "moving time";"""
        db_id = self.__db_cursor.execute(get_db_id_sql).fetchall()

        if len(db_id) > 1:
            raise ValueError("expected single id, got multiple ids")

        if len(db_id) == 0:
            insert_db_row_sql = "INSERT INTO working_times(name, time_spent, description) VALUES(?, ?, ?);"
            self.__db_cursor.execute(insert_db_row_sql, ("moving time", 0, ""))
            self.__db_connection.commit()
            db_id = self.__db_cursor.execute(get_db_id_sql).fetchall()

        return db_id[0][0]

    def __get_stopped_t_id(self):
        """stopped time
        """

        get_db_id_sql = """SELECT id FROM working_times WHERE name = "stopped time";"""
        db_id = self.__db_cursor.execute(get_db_id_sql).fetchall()

        if len(db_id) > 1:
            raise ValueError("expected single id, got multiple ids")

        if len(db_id) == 0:
            insert_db_row_sql = "INSERT INTO working_times(name, time_spent, description) VALUES(?, ?, ?);"
            self.__db_cursor.execute(insert_db_row_sql, ("stopped time", 0, ""))
            self.__db_connection.commit()
            db_id = self.__db_cursor.execute(get_db_id_sql).fetchall()

        return db_id[0][0]

    def __get_pdz_scan_t_id(self):
        """pdz scan time
        """

        get_db_id_sql = """SELECT id FROM working_times WHERE name = "pdz scan time";"""
        db_id = self.__db_cursor.execute(get_db_id_sql).fetchall()

        if len(db_id) > 1:
            raise ValueError("expected single id, got multiple ids")

        if len(db_id) == 0:
            insert_db_row_sql = "INSERT INTO working_times(name, time_spent, description) VALUES(?, ?, ?);"
            self.__db_cursor.execute(insert_db_row_sql, ("pdz scan time", 0, ""))
            self.__db_connection.commit()
            db_id = self.__db_cursor.execute(get_db_id_sql).fetchall()

        return db_id[0][0]

    def __get_all_ext_t_id(self):
        """all extractions time
        """

        get_db_id_sql = """SELECT id FROM working_times WHERE name = "all extractions time";"""
        db_id = self.__db_cursor.execute(get_db_id_sql).fetchall()

        if len(db_id) > 1:
            raise ValueError("expected single id, got multiple ids")

        if len(db_id) == 0:
            insert_db_row_sql = "INSERT INTO working_times(name, time_spent, description) VALUES(?, ?, ?);"
            self.__db_cursor.execute(insert_db_row_sql, ("all extractions time", 0, 'this includes "pdz scan time"'))
            self.__db_connection.commit()
            db_id = self.__db_cursor.execute(get_db_id_sql).fetchall()

        return db_id[0][0]

    '''
    def __get_all_ext_delta_t_id(self):
        """all extractions delta scan time
        """

        get_db_id_sql = """SELECT id FROM working_times WHERE name = "all extractions delta scan time";"""
        db_id = self.__db_cursor.execute(get_db_id_sql).fetchall()

        if len(db_id) > 1:
            raise ValueError("expected single id, got multiple ids")

        if len(db_id) == 0:
            insert_db_row_sql = "INSERT INTO working_times(name, time_spent, description) VALUES(?, ?, ?);"
            self.__db_cursor.execute(insert_db_row_sql, ("all extractions delta scan time", 0, ""))
            self.__db_connection.commit()
            db_id = self.__db_cursor.execute(get_db_id_sql).fetchall()

        return db_id[0][0]
    '''

    def __get_all_ext_xy_t_id(self):
        """all extractions XY movement time
        """

        get_db_id_sql = """SELECT id FROM working_times WHERE name = "all extractions XY movement time";"""
        db_id = self.__db_cursor.execute(get_db_id_sql).fetchall()

        if len(db_id) > 1:
            raise ValueError("expected single id, got multiple ids")

        if len(db_id) == 0:
            insert_db_row_sql = "INSERT INTO working_times(name, time_spent, description) VALUES(?, ?, ?);"
            self.__db_cursor.execute(insert_db_row_sql, ("all extractions XY movement time", 0, ""))
            self.__db_connection.commit()
            db_id = self.__db_cursor.execute(get_db_id_sql).fetchall()

        return db_id[0][0]

    def __get_all_ext_img_t_id(self):
        """all extractions image analysis time
        """

        get_db_id_sql = """SELECT id FROM working_times WHERE name = "all extractions image analysis time";"""
        db_id = self.__db_cursor.execute(get_db_id_sql).fetchall()

        if len(db_id) > 1:
            raise ValueError("expected single id, got multiple ids")

        if len(db_id) == 0:
            insert_db_row_sql = "INSERT INTO working_times(name, time_spent, description) VALUES(?, ?, ?);"
            self.__db_cursor.execute(insert_db_row_sql, ("all extractions image analysis time", 0, ""))
            self.__db_connection.commit()
            db_id = self.__db_cursor.execute(get_db_id_sql).fetchall()

        return db_id[0][0]

    def __get_all_ext_z_t_id(self):
        """all extractions Z extraction time
        """

        get_db_id_sql = """SELECT id FROM working_times WHERE name = "all extractions Z extraction time";"""
        db_id = self.__db_cursor.execute(get_db_id_sql).fetchall()

        if len(db_id) > 1:
            raise ValueError("expected single id, got multiple ids")

        if len(db_id) == 0:
            insert_db_row_sql = "INSERT INTO working_times(name, time_spent, description) VALUES(?, ?, ?);"
            self.__db_cursor.execute(insert_db_row_sql, ("all extractions Z extraction time", 0, ""))
            self.__db_connection.commit()
            db_id = self.__db_cursor.execute(get_db_id_sql).fetchall()

        return db_id[0][0]

    @staticmethod
    def __load_from_file(file_path: str):
        """Using pickle module loads data structure from a binary file with a given name

        Uses structure designed for this class (structure keys), shouldn't be used somewhere else except debug purposes.
        """

        with open(file_path, "rb") as input_file:
            return pickle.load(input_file)

    def __send_to_ui(self):
        if self.__ui_msg_queue is not None:
            self.__ui_msg_queue.send(json.dumps({"datacollector": [
                self.__detected_plants,
                self.__extracted_plants,
                self.__previous_sessions_working_time]}))

    @staticmethod
    def __format_time(seconds):
        """Returns given seconds as formatted DD:HH:MM:SS MSS str time
        """

        return str(datetime.timedelta(seconds=seconds))

    @staticmethod
    def __swap_files(old_file_path, new_file_path):
        """Removes old file, then renames new file to the old one's name
        """

        if os.path.exists(old_file_path):
            os.remove(old_file_path)
        os.rename(new_file_path, old_file_path)

    def add_moving_t(self, seconds: float):
        """Adds given robot moving time to statistics database.

        NOTE: This function is mostly the same f-ion as add_vesc_moving_time_data, but works with local DB instead of
        a txt file. This f-ion is going to replace add_vesc_moving_time_data in the future.
        """

        if type(seconds) not in [float, int]:
            msg = f"seconds should be int or float, got {str(type(seconds))} instead"
            raise TypeError(msg)
        if seconds < 0:
            msg = f"seconds must be equal or greater than zero, got {str(seconds)}"
            raise ValueError(msg)

        sql = """
        UPDATE working_times
        SET time_spent = time_spent + ?
        WHERE id = ?;
        """

        self.__db_cursor.execute(sql, (seconds, self.__moving_t_id))
        self.__db_connection.commit()

    def add_stopped_t(self, seconds: float):
        """Adds given robot stopped time to statistics database.
        """

        if type(seconds) not in [float, int]:
            msg = f"seconds should be int or float, got {str(type(seconds))} instead"
            raise TypeError(msg)
        if seconds < 0:
            msg = f"seconds must be equal or greater than zero, got {str(seconds)}"
            raise ValueError(msg)

        sql = """
        UPDATE working_times
        SET time_spent = time_spent + ?
        WHERE id = ?;
        """

        self.__db_cursor.execute(sql, (seconds, self.__stopped_t_id))
        self.__db_connection.commit()

    def set_stopped_t(self, seconds: float):
        """Sets given robot stopped time to statistics database.
        """

        if type(seconds) not in [float, int]:
            msg = f"seconds should be int or float, got {str(type(seconds))} instead"
            raise TypeError(msg)
        if seconds < 0:
            msg = f"seconds must be equal or greater than zero, got {str(seconds)}"
            raise ValueError(msg)

        sql = """
        UPDATE working_times
        SET time_spent = ?
        WHERE id = ?;
        """

        self.__db_cursor.execute(sql, (seconds, self.__stopped_t_id))
        self.__db_connection.commit()

    def add_pdz_scan_t(self, seconds: float):
        """Adds given robot pdz scan time to statistics database.
        """

        if type(seconds) not in [float, int]:
            msg = f"seconds should be int or float, got {str(type(seconds))} instead"
            raise TypeError(msg)
        if seconds < 0:
            msg = f"seconds must be equal or greater than zero, got {str(seconds)}"
            raise ValueError(msg)

        sql = """
        UPDATE working_times
        SET time_spent = time_spent + ?
        WHERE id = ?;
        """

        self.__db_cursor.execute(sql, (seconds, self.__pdz_scan_t_id))
        self.__db_connection.commit()

    def add_all_ext_t(self, seconds: float):
        """Adds given robot all extractions time to statistics database.
        """

        if type(seconds) not in [float, int]:
            msg = f"seconds should be int or float, got {str(type(seconds))} instead"
            raise TypeError(msg)
        if seconds < 0:
            msg = f"seconds must be equal or greater than zero, got {str(seconds)}"
            raise ValueError(msg)

        sql = """
        UPDATE working_times
        SET time_spent = time_spent + ?
        WHERE id = ?;
        """

        self.__db_cursor.execute(sql, (seconds, self.__all_ext_t_id))
        self.__db_connection.commit()

    '''
    def add_all_ext_delta_t(self, seconds: float):
        """Adds given robot all extractions delta scan time to statistics database.
        """

        if type(seconds) not in [float, int]:
            msg = f"seconds should be int or float, got {str(type(seconds))} instead"
            raise TypeError(msg)
        if seconds < 0:
            msg = f"seconds must be equal or greater than zero, got {str(seconds)}"
            raise ValueError(msg)

        sql = """
        UPDATE working_times
        SET time_spent = time_spent + ?
        WHERE id = ?;
        """

        self.__db_cursor.execute(sql, (seconds, self.__all_ext_delta_t_id))
        self.__db_connection.commit()
    '''

    def add_all_ext_delta_info(self, seconds: float, shifts_count: int):
        """Adds a record into DB about weed delta seeking. Each row is single using case.

        delta_scans (
            id INTEGER PRIMARY KEY,
            time_spent REAL NOT NULL,
            shifts_count INTEGER NOT NULL
        )
        """

        if type(seconds) not in [float, int]:
            msg = f"seconds must be int or float, got {str(type(seconds))} instead"
            raise TypeError(msg)
        if seconds < 0:
            msg = f"seconds must be equal or greater than zero, got {str(seconds)}"
            raise ValueError(msg)
        if type(shifts_count) != int:
            msg = f"shifts_count must be int, got {str(type(shifts_count))} instead"
            raise TypeError(msg)
        if shifts_count < 0:
            msg = f"shifts_count must be equal or greater than zero, got {str(seconds)}"
            raise ValueError(msg)

        insert_row_sql = "INSERT INTO delta_scans(time_spent, shifts_count) VALUES(?, ?);"
        self.__db_cursor.execute(insert_row_sql, (seconds, shifts_count))
        self.__db_connection.commit()

    def add_all_ext_xy_t(self, seconds: float):
        """Adds given robot all extractions XY movement time to statistics database.
        """

        if type(seconds) not in [float, int]:
            msg = f"seconds should be int or float, got {str(type(seconds))} instead"
            raise TypeError(msg)
        if seconds < 0:
            msg = f"seconds must be equal or greater than zero, got {str(seconds)}"
            raise ValueError(msg)

        sql = """
        UPDATE working_times
        SET time_spent = time_spent + ?
        WHERE id = ?;
        """

        self.__db_cursor.execute(sql, (seconds, self.__all_ext_xy_t_id))
        self.__db_connection.commit()

    def add_all_ext_img_t(self, seconds: float):
        """Adds given robot all extractions image analysis time to statistics database.
        """

        if type(seconds) not in [float, int]:
            msg = f"seconds should be int or float, got {str(type(seconds))} instead"
            raise TypeError(msg)
        if seconds < 0:
            msg = f"seconds must be equal or greater than zero, got {str(seconds)}"
            raise ValueError(msg)

        sql = """
        UPDATE working_times
        SET time_spent = time_spent + ?
        WHERE id = ?;
        """

        self.__db_cursor.execute(sql, (seconds, self.__all_ext_img_t_id))
        self.__db_connection.commit()

    def add_all_ext_z_t(self, seconds: float):
        """Adds given robot all extractions Z extraction time to statistics database.
        """

        if type(seconds) not in [float, int]:
            msg = f"seconds should be int or float, got {str(type(seconds))} instead"
            raise TypeError(msg)
        if seconds < 0:
            msg = f"seconds must be equal or greater than zero, got {str(seconds)}"
            raise ValueError(msg)

        sql = """
        UPDATE working_times
        SET time_spent = time_spent + ?
        WHERE id = ?;
        """

        self.__db_cursor.execute(sql, (seconds, self.__all_ext_z_t_id))
        self.__db_connection.commit()

    def add_detections_data(self, type_label: str, count: int):
        """Adds given detections data to the stored values
        """

        if type(count) is not int:
            raise TypeError("'count' type should be int, got " + type(count).__name__)
        if count <= 0:
            raise ValueError("'count' value should be greater than 0, got " + str(count))

        if type_label in self.__detected_plants:
            self.__detected_plants[type_label] += count
        else:
            self.__detected_plants[type_label] = count

        self.__send_to_ui()

        if self.__dump_at_receiving:
            self.dump_to_file(self.__dump_file_path)

    def add_extractions_data(self, type_label: str, count: int):
        """Adds given extractions data to the stored values
        """

        if type(count) is not int:
            raise TypeError("'count' type should be int, got " + type(count).__name__)
        if count <= 0:
            raise ValueError("'count' value should be greater than 0, got " + str(count))

        if type_label in self.__extracted_plants:
            self.__extracted_plants[type_label] += count
        else:
            self.__extracted_plants[type_label] = count

        if self.__notification.is_continuous_information_sending():
            self.__notification.set_extracted_plants(self.__extracted_plants)

        self.__send_to_ui()

        if self.__dump_at_receiving:
            self.dump_to_file(self.__dump_file_path)

    def add_vesc_moving_time_data(self, seconds: float):
        """Adds given vesc moving time to the stored value
        """

        if type(seconds) not in [float, int]:
            msg = f"seconds should be int or float, got {str(type(seconds))} instead"
            raise TypeError(msg)
        if seconds < 0:
            msg = f"seconds must be equal or greater than zero, got {str(seconds)}"
            raise ValueError(msg)
        self.__vesc_moving_time += seconds

        # save to DB (TODO further DB way should replace txt way completely)
        self.add_moving_t(seconds)

        if self.__dump_at_receiving:
            self.dump_to_file(self.__dump_file_path)

    def add_cork_moving_time_data(self, seconds: float):
        """Adds given cork moving time to the stored value
        """

        if type(seconds) not in [float, int]:
            msg = f"seconds should be int or float, got {str(type(seconds))} instead"
            raise TypeError(msg)
        if seconds < 0:
            msg = f"seconds must be equal or greater than zero, got {str(seconds)}"
            raise ValueError(msg)
        self.__cork_moving_time += seconds

        if self.__dump_at_receiving:
            self.dump_to_file(self.__dump_file_path)

    def get_session_working_time(self):
        """Returns how many seconds is past after launch till this function call moment
        """

        return time.time() - self.__start_time

    def save_all_data(self, output_file_path: str):
        """Saves data as formatted txt file
        """

        new_file = output_file_path + ".new"  # temp file to avoid data loss if any error occurred during saving
        with open(new_file, "w") as file:
            file.write("Last session robot working time: " + self.__format_time(self.get_session_working_time()) + "\n")
            file.write("Total field robot working time: " +
                       self.__format_time(self.get_session_working_time() +
                                          self.__previous_sessions_working_time) + "\n\n")

            # detections statistics
            file.write("Total field detections\n")
            for label in self.__detected_plants:
                file.write(label + ": " + str(self.__detected_plants[label]) + "\n")
            if len(self.__detected_plants) > 0:
                file.write("\n")

            # extractions statistic
            file.write("Total field extractions\n")
            for label in self.__extracted_plants:
                file.write(label + ": " + str(self.__extracted_plants[label]) + "\n")
            if len(self.__extracted_plants) > 0:
                file.write("\n")

            # vesc and cork statistic
            file.write("Total field vesc moving time: " + self.__format_time(self.__vesc_moving_time) + "\n")
            file.write("Total field cork moving time: " + self.__format_time(self.__cork_moving_time) + "\n")

        # swap old file with the new
        self.__swap_files(output_file_path, new_file)

    def dump_to_file(self, file_path: str):
        """Using pickle module dumps stored data into a binary file with a given name

        Uses structure designed for this class (structure keys).
        """

        structure = dict()
        structure["__detected_plants"] = self.__detected_plants
        structure["__extracted_plants"] = self.__extracted_plants
        structure["__vesc_moving_time"] = self.__vesc_moving_time
        structure["__cork_moving_time"] = self.__cork_moving_time
        structure["__previous_sessions_working_time"] = \
            self.__previous_sessions_working_time + self.get_session_working_time()

        with open(file_path, "wb") as output_file:
            pickle.dump(structure, output_file)
