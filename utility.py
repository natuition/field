import platform
import datetime
import os
import serial.tools.list_ports
import threading
from time import sleep
import psutil
import glob
import detection
import cv2 as cv
import math
from config import config
import time
from pytz import timezone
import socket
import subprocess


class ImageSaver:
    """Implements flexible ways to save images and detected objects on them

    Supports user-defined counters.
    Has two built-in counters:
    "main" - default counter which is used if no counter was set;
    "total" - contains total count of saved images during this class instance life, this counter is always increasing
    during saving independent on other counters."""

    def __init__(self, counter: int = 0):
        if type(counter) is not int:
            raise TypeError("'counter' type should be int, got " + type(counter).__name__)

        self.__counters = {"total": 0, "main": counter}

    def has_counter(self, counter_key):
        return counter_key in self.__counters

    def get_counter(self, counter_key="main"):
        return self.__counters[counter_key]

    def set_counter(self, counter: int, counter_key="main"):
        if type(counter) is not int:
            raise TypeError("'counter' type should be int, got " + type(counter).__name__)
        if counter_key == "total":
            raise ValueError("'total' counter is class internal counter and can't be changed")

        self.__counters[counter_key] = counter

    def save_image(self,
                   image,
                   directory: str,
                   extension: str = "jpg",
                   sep: str = "_",
                   specific_name=None,
                   label=None,
                   plants_boxes=None,
                   counter_key="main"):
        """
        Saves image in different ways.

        extension should not contain point separator: "jpg"
        specific_name should not contain file extension
        """

        # define file name (image and txt if plants_boxes are passed)
        if specific_name:
            file_name = specific_name
        else:
            cur_dt = get_current_time()
            file_name = cur_dt[:cur_dt.rfind(" ")] + sep + str(self.__counters[counter_key])
            self.__counters[counter_key] += 1
            if label:
                file_name += sep + label
        self.__counters["total"] += 1

        file_name = file_name.replace(" ", "_")

        # save image
        cv.imwrite(directory + file_name + "." + extension, image)

        # save plants boxes if passed
        if type(plants_boxes) is list and len(plants_boxes) > 0:
            with open(directory + file_name + ".txt", "w") as txt_file:
                plant_box: detection.DetectedPlantBox
                for plant_box in plants_boxes:
                    txt_file.write(plant_box.get_as_yolo(return_as_text=True) + "\n")

    @staticmethod
    def draw_data_in_frame(frame, undistorted_zone_radius=None, poly_zone_points_cv=None, pdz_cv_rect=None, plants_boxes=None):
        if undistorted_zone_radius is not None:
            frame = ImageSaver.draw_zone_circle(frame, config.SCENE_CENTER_X, config.SCENE_CENTER_Y, undistorted_zone_radius, (255, 40, 162))
            frame = ImageSaver.draw_zone_circle(frame, config.SCENE_CENTER_X, config.SCENE_CENTER_Y, 5, (255, 40, 162), -1)
        if poly_zone_points_cv is not None:
            frame = ImageSaver.draw_zone_poly(frame, poly_zone_points_cv, (255, 0, 0))
        if plants_boxes is not None:
            frame = detection.draw_boxes(frame, plants_boxes)
        if pdz_cv_rect is not None:
            frame = cv.rectangle(frame, pdz_cv_rect[0], pdz_cv_rect[1], (16, 127, 237), 3)
        return frame

    @staticmethod
    def draw_zone_circle(image, circle_center_x, circle_center_y, circle_radius, color=(0, 0, 255), thickness=3):
        """Draws received circle on image. Used for drawing undistorted zone edges on photo"""
        return cv.circle(image, (circle_center_x, circle_center_y), circle_radius, color, thickness=thickness)

    @staticmethod
    def draw_zone_poly(image, np_poly_points, color=(0, 0, 255)):
        """Draws received polygon on image. Used for drawing working zone edges on photo"""
        return cv.polylines(image, [np_poly_points], isClosed=True, color=color, thickness=5)


class TrajectorySaver:
    """Provides safe gps points saving (robot's trajectory)"""

    def __init__(self, full_path, append_file=False):
        self.__full_path = full_path
        self.__last_received_point = None
        self.__output_file = open(full_path, "a" if append_file else "w")
        if append_file:
            self.__output_file.write("\n")

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()

    def close(self):
        self.__output_file.close()

    def save_point(self, point: list, save_raw=False, flush_immediately=True):
        """
        Saves given point if it is different from previous received gps point.
        """

        if not str(point) == self.__last_received_point:
            self.__last_received_point = str(point)

            #if len(point)==3:
            #    str_point = str(point[0]) + " " + str(point[1])+ " " + point[2] + "\n" if save_raw else str(point) + "\n"
            #else :
            str_point = str(point[0]) + " " + str(point[1]) + "\n" if save_raw else str(point) + "\n"
                
            self.__output_file.write(str_point)

            if flush_immediately:
                self.__output_file.flush()


class MemoryManager:
    """
    Provides tools for removing obsolete files from given directory.
    Supports:
    - manual blocking cleaning: called manually from code, blocks caller thread until completely executed
    - manual non-blocking cleaning: called manually, does single time cleaning in a separate thread, can be stopped at
      any time
    - auto-cleaning: automatically calls for cleaner each N seconds (set as parameter), can be stopped at any time
    """

    def __init__(self, path: str, files_to_keep_count: int = 600, check_frequency_seconds: int = 60,
                 memory_threshold: int = 95368):
        self.__files_to_keep_count = files_to_keep_count
        self.__path = path
        self.__check_frequency_seconds = check_frequency_seconds
        self.__memory_threshold = memory_threshold

        self.__auto_cleaner_thread = threading.Thread(target=self.__auto_cleaner_tf, daemon=True)
        self.__keep_auto_cleaner_thread_alive = False

        self.__manual_cleaner_thread = threading.Thread(target=self.__manual_cleaner_tf, daemon=True)
        self.__keep_manual_cleaner_thread_alive = False

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop_auto_cleaning()
        self.stop_clean_manual_non_blocking()

    def start_auto_cleaning(self):
        self.__keep_auto_cleaner_thread_alive = True
        self.__auto_cleaner_thread.start()

    def stop_auto_cleaning(self):
        self.__keep_auto_cleaner_thread_alive = False

    def start_clean_manual_non_blocking(self):
        self.__keep_manual_cleaner_thread_alive = True
        self.__manual_cleaner_thread.run()

    def stop_clean_manual_non_blocking(self):
        self.__keep_manual_cleaner_thread_alive = False

    def start_clean_manual_blocking(self):
        """This function does disk cleaning, blocking until executed"""

        files_to_delete = self.__get_files_to_delete_list()
        for file in files_to_delete:
            os.remove(file)

    def __get_files_to_delete_list(self):
        # TODO: can be optimized (seems like getmtime is called multiple times during sorting)
        full_files_list = sorted(glob.glob(self.__path + "*"), key=os.path.getmtime, reverse=True)
        return full_files_list[self.__files_to_keep_count:]

    def __manual_cleaner_tf(self):
        """Manual cleaner thread target function"""

        files_to_delete = self.__get_files_to_delete_list()
        for file in files_to_delete:
            if self.__keep_manual_cleaner_thread_alive:
                os.remove(file)
            else:
                break

    def __auto_cleaner_tf(self):
        """Auto cleaner thread target function"""

        while self.__keep_auto_cleaner_thread_alive:
            if (round((psutil.disk_usage("/").used / (2 ** 20)), 2)) > self.__memory_threshold:
                files_to_delete = self.__get_files_to_delete_list()
                for file in files_to_delete:
                    if self.__keep_auto_cleaner_thread_alive:
                        os.remove(file)
                    else:
                        return
            sleep(self.__check_frequency_seconds)


class Logger:
    """
    Writes into the file with specified name str data, flushing data on each receiving
    """

    def __init__(self, file_name, add_time=True, time_sep=" ", append_file=False):
        self._file = open(file_name, "a" if append_file else "w")
        self.__add_time = add_time
        self.__time_sep = time_sep

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self._file.__exit__(exc_type, exc_val, exc_tb)

    def write_and_flush(self, s):
        if self.__add_time:
            s = get_current_time() + self.__time_sep + s
        self._file.write(s)
        self._file.flush()

    def write(self, s):
        if self.__add_time:
            s = get_current_time() + self.__time_sep + s
        self._file.write(s)

    def close(self):
        self._file.close()


class DemoPauseServer:
    """Instance of this class is accepting incoming connection. When called to block from code, will block until
    resume command is read from any client.

    Initially created to resume robot during demo.
    Implementation and purpose of this class may be changed in the future.
    """

    def __init__(self, host, port, tick_delay=0.005, buffer_size=1024):
        self.__clients_conn_listener = socket.socket()
        self.__clients_conn_listener.bind((host, port))
        self.__clients_conn_listener.listen(5)
        self.__keep_clients_conn_listener_alive = True
        self.__current_clients = []
        self.__current_clients_locker = threading.Lock()
        # new clients connections accept thread
        self.__new_clients_conn_listener_th = threading.Thread(
            target=self.__new_clients_conn_listener_tf,
            name="__new_clients_conn_listener_th",
            daemon=True
        )

        # clients commands reader
        self.__keep_clients_cmd_reader_alive = True
        self.__clients_cmd_reader_th = threading.Thread(
            target=self.__clients_cmd_reader_tf,
            name="__clients_cmd_reader_tf",
            daemon=True
        )

        self.__is_paused = False
        self.__is_paused_locker = threading.Lock()
        self.__tick_delay = tick_delay
        self.__buffer_size = buffer_size

        self.__new_clients_conn_listener_th.start()
        self.__clients_cmd_reader_th.start()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()

    def __del__(self):
        self.close()

    def close(self):
        self.__keep_clients_conn_listener_alive = False
        self.__keep_clients_cmd_reader_alive = False

        try:
            self.__clients_conn_listener.close()
        except KeyboardInterrupt:
            raise KeyboardInterrupt
        except:
            pass

        with self.__current_clients_locker:
            for client in self.__current_clients:
                try:
                    client.shutdown(socket.SHUT_RDWR)
                except KeyboardInterrupt:
                    raise KeyboardInterrupt
                except:
                    pass

                try:
                    client.close()
                except KeyboardInterrupt:
                    raise KeyboardInterrupt
                except:
                    pass

    def __new_clients_conn_listener_tf(self):
        while self.__keep_clients_conn_listener_alive:
            try:
                client, address = self.__clients_conn_listener.accept()
                with self.__current_clients_locker:
                    self.__current_clients.append(client)
            except KeyboardInterrupt:
                print("__new_clients_conn_listener_th for some reason got KeyboardInterrupt!")
                raise KeyboardInterrupt
            except Exception as ex:
                if self.__keep_clients_conn_listener_alive:
                    print("__new_clients_conn_listener_th error when accepting new client:", ex)

    def __clients_cmd_reader_tf(self):
        connections_to_close = []

        while self.__keep_clients_cmd_reader_alive:
            time.sleep(self.__tick_delay)

            with self.__current_clients_locker:
                # read from each client, allow robot to resume is resume cmd is receiced
                for idx, current_client in enumerate(self.__current_clients):
                    current_client: socket.socket

                    if not self.__keep_clients_cmd_reader_alive:
                        break

                    try:
                        data = current_client.recv(self.__buffer_size, socket.MSG_DONTWAIT)
                        if len(data) != 0:
                            if data == b"resume":
                                with self.__is_paused_locker:
                                    self.__is_paused = False
                        else:
                            if idx not in connections_to_close:
                                connections_to_close.append(idx)
                    except BlockingIOError:
                        pass
                    except (ConnectionResetError, socket.error, socket.herror, socket.gaierror):
                        if idx not in connections_to_close:
                            connections_to_close.append(idx)
                    except KeyboardInterrupt:
                        print("__clients_cmd_reader_th for some reason got KeyboardInterrupt!")
                        raise KeyboardInterrupt
                    except Exception as ex:
                        if self.__keep_clients_cmd_reader_alive:
                            # print("__clients_cmd_reader_th error when reading from client:", ex)
                            pass

                # try to close, and remove client objects which likely loss a connection
                for idx in reversed(connections_to_close):
                    if not self.__keep_clients_cmd_reader_alive:
                        break

                    try:
                        self.__current_clients[idx].shutdown(socket.SHUT_RDWR)
                    except KeyboardInterrupt:
                        print("__clients_cmd_reader_th for some reason got KeyboardInterrupt!")
                        raise KeyboardInterrupt
                    except Exception as ex:
                        if self.__keep_clients_cmd_reader_alive:
                            # print("__clients_cmd_reader_th error when shutting down connection:", ex)
                            pass

                    try:
                        self.__current_clients[idx].close()
                    except KeyboardInterrupt:
                        print("__clients_cmd_reader_th for some reason got KeyboardInterrupt!")
                        raise KeyboardInterrupt
                    except Exception as ex:
                        if self.__keep_clients_cmd_reader_alive:
                            # print("__clients_cmd_reader_th error when closing connection:", ex)
                            pass

                    del self.__current_clients[idx]

                if len(connections_to_close) > 0:
                    connections_to_close.clear()

    def wait_for_resume_cmd(self):
        with self.__is_paused_locker:
            self.__is_paused = True

        while True:
            with self.__is_paused_locker:
                if not self.__is_paused:
                    return
            time.sleep(self.__tick_delay)


class DemoPauseClient:
    """Sends resume requests to DemoPauseServer"""

    def __init__(self, host, port, tick_delay=0.005):
        self.__tick_delay = tick_delay

        self.__host = host
        self.__port = port
        self.__server_conn = None

        self.__keep_sender_alive = True
        self.__sender_th = threading.Thread(
            target=self.__sender_tf,
            name="__sender_th",
            daemon=True
        )

        self.__resume_request = False
        self.__resume_request_locker = threading.Lock()

        self.__sender_th.start()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()

    def __del__(self):
        self.close()

    def close(self):
        self.__keep_sender_alive = False

        try:
            self.__server_conn.close()
        except KeyboardInterrupt:
            raise KeyboardInterrupt
        except:
            pass

    def send_resume_cmd(self):
        with self.__resume_request_locker:
            self.__resume_request = True

    def __reconnect(self):
        if self.__server_conn is not None:
            try:
                self.__server_conn.close()
            except KeyboardInterrupt:
                raise KeyboardInterrupt
            except:
                pass

        while self.__keep_sender_alive:
            try:
                self.__server_conn = socket.socket()
                error_indicator = self.__server_conn.connect_ex((self.__host, self.__port))
                if error_indicator == 0:
                    break
            except KeyboardInterrupt:
                raise KeyboardInterrupt
            except:
                pass

    def __sender_tf(self):
        self.__reconnect()
        need_to_reconnect = False

        while self.__keep_sender_alive:
            time.sleep(self.__tick_delay)

            with self.__resume_request_locker:
                if self.__resume_request:
                    self.__resume_request = False
                else:
                    continue

            while self.__keep_sender_alive:
                if need_to_reconnect:
                    self.__reconnect()
                    need_to_reconnect = False

                try:
                    self.__server_conn.send(b"resume")
                    break
                except KeyboardInterrupt:
                    print("__sender_tf for some reason got KeyboardInterrupt!")
                    raise KeyboardInterrupt
                except (socket.error, socket.herror, socket.gaierror):
                    need_to_reconnect = True
                    continue
                except Exception as ex:
                    if self.__keep_sender_alive:
                        # print("__sender_tf error when sending resume cmd:", ex)
                        pass


def get_current_time():
    """Returns current time as formatted string"""

    return datetime.datetime.now(timezone('Europe/Berlin')).strftime("%d-%m-%Y %H-%M-%S %f")


def create_directories(*args):
    """Creates directories, receives any args count, each arg is separate dir"""

    for path in args:
        if not os.path.exists(path):
            try:
                os.mkdir(path)
            except OSError:
                print("Creation of the directory %s failed" % path)
            else:
                print("Successfully created the directory %s " % path)
        else:
            print("Directory %s is already exists" % path)


def get_path_slash():
    return "\\" if platform.system() == "Windows" else "/"


def get_smoothie_vesc_addresses():
    equipment_by_port = dict()
    for port, desc, other in sorted(serial.tools.list_ports.comports()):
        if "Smoothie" in desc:
            equipment_by_port["smoothie"] = port
        if "ChibiOS/RT" in desc:
            equipment_by_port["vesc"] = port
    return equipment_by_port

def get_ublox_address():
    for port, desc, other in sorted(serial.tools.list_ports.comports()):
        if "u-blox" in desc:
            return port
    return None

def mu_sigma(samples: list):
    
    mu =0
    sigma =0
    sign =1

    #moyenne arithmétique
    
    for x in samples:
        mu+=x**2
    
    mu/=len(samples)
    if samples[len(samples)-1]<0 :
        sign=-1
    mu = sign * math.sqrt(mu)   # mu is ready
    
    
    
    # perform sigma calculation
    for x in samples:       
            sigma+=math.pow(math.fabs(x-mu),2)
    sigma/=len(samples)
    sigma = math.sqrt(sigma)
    
    #print ( "mu =%2.13f"%mu, " sigma =%E"%sigma)

    stat= [mu, sigma]
    return stat



def distribution_of_values(samples: list, mu, sigma):
    leg=[10,9,8,7,6,5,4,3,2,1,1,2,3,4,5,6,7,8,9,10]  # legend
    stat=[0, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
    junk=0

    for x in samples:
            if int(math.fabs((x-mu)/sigma))<10:
                if x>=mu:
                    n_sigma=int(((x-mu)/sigma))+10       #n_sigma[10] contient le nombre de valeur entre mu et mu-sigma
                                                                #n_sigma[11] contient le nombre de valeur entre mu et mu-2sigma
                else:
                    n_sigma=-int(((mu-x)/sigma))+9      #n_sigma[9] contient le nombre de valeur entre mu et mu-1sigma
                stat[n_sigma]+=1                         #n_sigma[8] contient le nombre de valeur entre mu et mu-2sigma
                #print("nsigma %2d"%n_sigma,"x %2.4f"%x) 
            else: 
                junk+=1                                 # stupid value exceeding +10 -9 sigma are only counted
                #print("junk sample %2.2f"%x)
    print(leg)
    print(stat)
    print("junk total", junk)
   
    distrib= [stat, junk]
    return distrib



def average_point(gps,trajectory_saver: TrajectorySaver,nav, logger: Logger=None):

    #ORIGIN POINT SAVING
    lat = []     #latitude history
    long = []    #longitude history
    distances = []
    

    for i in range(0,config.ORIGIN_AVERAGE_SAMPLES):
        prev_maneuver_time = time.time()
        try:
            prev_pos = gps.get_fresh_position()
            msg = f"Get {i+1}/{config.ORIGIN_AVERAGE_SAMPLES} point in {time.time()-prev_maneuver_time} for average_point."
            if logger is not None:
                logger.write_and_flush(msg+"\n")
            #if config.VERBOSE:
            #    print(msg)
        except TimeoutError:
            msg = f"Erro waiting time too long for the {i+1} point in average_point !"
            if logger is not None:
                logger.write_and_flush(msg+"\n")
            if config.VERBOSE:
                print(msg)
            raise TimeoutError
        lat.append(prev_pos[0])
        long.append(prev_pos[1])
        mu_lat, sigma_lat = mu_sigma(lat)
        mu_long, sigma_long = mu_sigma(long)
        distance = nav.get_distance([mu_lat,mu_long,'1'], prev_pos)
        #print("| ",get_current_time()," | %2.2f"%distance, " | ", prev_pos, "|")
        distances.append(distance)
    
    mu_distance, sigma_distance = mu_sigma(distances)
    #print("stat lattitude : \n")
    mu_lat, sigma_lat = mu_sigma(lat)
    #distribution_of_values(lat, mu_lat, sigma_lat)
    #print("stat longitude : \n")
    mu_long, sigma_long = mu_sigma(long)
    #distribution_of_values(long, mu_long, sigma_long)
    #print("stat distance : \n")
    mu_distance, sigma_distance =  mu_sigma(distances)
    #distribution_of_values(distances, mu_distance, sigma_distance)
    #print("Average origin point:  %2.13f"%mu_lat," ","%2.13f"%mu_long, "standard deviation (mm) %2.2f"%sigma_distance)    
    prev_pos[0]=mu_lat      #replace the instantaneous value by the average latitude
    prev_pos[1]=mu_long     #replace the instantaneous value by the average longitude
    prev_pos.append("Origin_with_" + str(config.ORIGIN_AVERAGE_SAMPLES) + "_samples")
    #print("prev_pos syntax : ",prev_pos)   #debug
    if trajectory_saver is not None:
        trajectory_saver.save_point(prev_pos)
    
    return prev_pos


def get_last_dir_name(parent_dir_path: str):
    """Looks for directories in a given parent directory, returns the last created dir (Windows) or last changed dir
    (Linux), or None if no directories were found at given path.
    """

    all_parent_dir_objects = os.listdir(parent_dir_path)
    last_dir = None
    last_dir_creation_time = None

    for cur_obj_name in all_parent_dir_objects:
        cur_obj_full_path = parent_dir_path + cur_obj_name
        if os.path.isdir(cur_obj_full_path):
            cur_dir_creation_time = os.path.getctime(cur_obj_full_path)
            if last_dir:
                if cur_dir_creation_time > last_dir_creation_time:
                    last_dir = cur_obj_name
                    last_dir_creation_time = cur_dir_creation_time
            else:
                last_dir = cur_obj_name
                last_dir_creation_time = cur_dir_creation_time
    return last_dir

def life_line_reset():
    dir_gpio = f"/sys/class/gpio/gpio{config.LIFE_LINE_PIN}"
    if os.path.isdir(dir_gpio):
        print(f"The directory '{dir_gpio}' exist.")
    else:
        print(f"The directory '{dir_gpio}' not exist, creating...")
        subprocess.run(f'echo {config.LIFE_LINE_PIN} > /sys/class/gpio/export',shell=True)

    check_direction = subprocess.run(f'grep -q "out" "/sys/class/gpio/gpio{config.LIFE_LINE_PIN}/direction"',shell=True).returncode

    if check_direction==0:
        print(f"Gpio {config.LIFE_LINE_PIN} is already out.")
    else:
        print(f"Gpio {config.LIFE_LINE_PIN} is not output, output adjustment in progress...")
        subprocess.run(f'echo out > /sys/class/gpio/gpio{config.LIFE_LINE_PIN}/direction',shell=True)

    subprocess.run(f'echo 1 > /sys/class/gpio/gpio{config.LIFE_LINE_PIN}/value',shell=True)
    sleep(0.5)
    subprocess.run(f'echo 0 > /sys/class/gpio/gpio{config.LIFE_LINE_PIN}/value',shell=True)

    for time_int in range(10):
        if "vesc" in get_smoothie_vesc_addresses():
            break
        else:
            print(f"Not find vesc sleeping {time_int+1}/10 secs.")
        sleep(1)


def load_coordinates(file_path):
    positions_list = []
    with open(file_path) as file:
        for line in file:
            if line != "":
                positions_list.append(list(map(float, line.split(" "))))
    return positions_list