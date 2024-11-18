from config import config
import posix_ipc
import json
import time
import threading
from penetrometry import extractions_data_pb2

class PenetrometryAnalyse :
    """Class use to get a penetrometry analyse.

    This class accumulate data about motor erpm (rpm) and current (torque) 
    during an extraction. Those datas will be usefull to make a analyse of 
    the ground

    Attributes:
        __vescAdapter (VescAdapterV4): The vesc adapter use in main.
        __analyse_frequency (float): The frequency of vesc data acquisition.
        __extraction_engine_can_id (int): The can id of the vesc use to control the extraction motor.
        __field_to_analyse (list): Names of parameters to get data from vesc.
        __is_saving_mode_active (bool): Is the saving mode wanted by the user.
        __file_name (str): Name of the file where data will be save.
        __file_path (str): The repository where the file will be placed.
        __data_queue (MessageQueue) : The IPC queue where datas are placed to exchange with UI State machine.
        __data_queue_name (str) : The name of the IPC queue for datas.
        __data_queue_max_message (int) : The maximal number of message that queue for datas can contains.
        __data_queue_buffer_slice_size (int) : The maximal size of datas in a message in queue for data.
        __params_queue (MessageQueue) : The IPC queue where params are placed to exchange with UI State machine.
        __params_queue_name (str) : The name of the IPC queue for params.
        __params_queue_max_message (int) : The maximal number of message that queue for params can contains.
        __nb_capture_before (int) : The number of capture before the trigger that have to be show and save.
        __nb_capture_over (int) : The number of capture over the threshold to activate the trigger.
        __nb_capture_after (int) : The number of capture after the trigger that have to be show and save.
        __threshold (int) : The RPM value that can activate the trigger if is go over.
        __buffer (list) : The list where datas will be placed temporaly.
        __buffer_size (int) : The size of the list where datas will be placed temporaly.
        __count_capture_over_threshold (int) : The count of capture straight over threshold.
        __extraction_trigger (bool) : If a extraction is detected, set to True.
        __count_capture_after_trigger (int) : The count of capture after the trigger activation.
        __current_coordinates (dict) : Current latitude and longitude of the robot.
        __penetrometry_thread_alive (bool) : Should the thread be alive.
        __penetrometry_thread (Thread) : The thread that gather datas about extractions.
    """
    def __init__(self, vescAdapter):
        """Create a PenetrometryAnalyse object.

        Args:
            vescAdapter (VescAdapterV4): The vesc adapter use in main.
        """
        self.__vescAdapter = vescAdapter

        self.__analyse_frequency = config.PENETROMETRY_ANALYSE_FREQUENCY
        self.__extraction_engine_can_id = config.VESC_EXTRACTION_CAN_ID
        self.__field_to_analyse = ["temp_motor_filtered", 
                                    "avg_motor_current", 
                                    "rpm",] # Name fields according to GetValues message if raess1/PyVESC-FW3.33 (use in this robot)

        self.__is_saving_mode_active = config.PENETROMETRY_SAVING_MODE
        self.__file_name = f"data_{time.strftime('%Y-%m-%d-%H-%M-%S')}.pb" # Timestamp of the class creating for naming the saving file
        self.__file_path = f"/home/violette/field/penetrometry/datas/{self.__file_name}"

        self.__data_queue = None
        self.__data_queue_name = config.PENETROMETRY_DATA_QUEUE_NAME
        self.__data_queue_max_message = config.PENETROMETRY_DATA_QUEUE_MAX_MESSAGE
        self.__data_queue_buffer_slice_size = config.PENETROMETRY_DATA_QUEUE_BUFFER_SLICE_SIZE
        self.__params_queue = None
        self.__params_queue_name = config.PENETROMETRY_PARAMS_QUEUE_NAME
        self.__params_queue_max_message = config.PENETROMETRY_PARAMS_QUEUE_MAX_MESSAGE

        self.__nb_capture_before = config.PENETROMETRY_RPM_TRIGGER_NB_CAPTURE_BEFORE
        self.__nb_capture_over = config.PENETROMETRY_RPM_TRIGGER_NB_CAPTURE_OVER
        self.__nb_capture_after = config.PENETROMETRY_RPM_TRIGGER_NB_CAPTURE_AFTER
        self.__threshold = config.PENETROMETRY_RPM_TRIGGER_THRESHOLD

        self.__buffer = []
        self.__buffer_size = self.__nb_capture_before + self.__nb_capture_over + self.__nb_capture_after

        self.__count_capture_over_threshold = 0
        self.__extraction_trigger = False
        self.__count_capture_after_trigger = 0

        self.__current_coordinates = {}
        self.__current_coordinates['latitude'] = 0
        self.__current_coordinates['longitude'] = 0

        self.__penetrometry_thread_alive = True
        self.__penetrometry_thread = threading.Thread(target=self.__penetrometry_thread_function, daemon=True)
        self.__penetrometry_thread.start()

    def __del__(self):
        """Stop the penetrometry thread.
        """
        self.__penetrometry_thread_alive = False
        self.__penetrometry_thread.join(1)


    def __save_in_file(self, file_path, buffer, coords):
        """Save datas about an extraction into a protocol buffer file.

        Args:
            file_path (str): The name and the path of the file use to save date.
            buffer (list): The buffer that contains all datas about an extraction.
            coords (dict): The dict that contains a latitude and a longitude.
        """
        # Creating an object, according to .proto, for saving an extraction into the list in file
        extraction_data = extractions_data_pb2.ExtractionList()

        rpm_list = []
        torque_list = []
        timestamp_list = []
        for data in buffer:
            rpm_list.append(data['rpm_meca'])
            torque_list.append(data['torque'])
            timestamp_list.append(data['timestamp'])

        # Adding the new extraction datas
        extraction = extraction_data.extractions.add()
        extraction.rpm.extend(rpm_list)
        extraction.torque.extend(torque_list)
        extraction.timestamp.extend(timestamp_list) 
        extraction.latitude = coords['latitude']
        extraction.longitude = coords['longitude']

        # Writing into binary file
        with open(file_path, "ab") as file:
            file.write(extraction_data.SerializeToString())

    def __open_queues(self):
        """Ensure that queues does not already exist and then open those
        """
        # Be sure that there is no queue already existing
        try:
            posix_ipc.unlink_message_queue(self.__data_queue_name)
        except:
            pass
        try:
            posix_ipc.unlink_message_queue(self.__params_queue_name)
        except:
            pass

        # Creating the queue for sending extraction data
        try:
            self.__data_queue = posix_ipc.MessageQueue(self.__data_queue_name, posix_ipc.O_CREAT, max_messages=self.__data_queue_max_message)
        except Exception as e:
            print("PENETROMETRY, queue creating:", e)
            return False
        # Creating the queue for receiving trigger params
        try:
            self.__params_queue = posix_ipc.MessageQueue(self.__params_queue_name, posix_ipc.O_CREAT, max_messages=self.__params_queue_max_message)
        except Exception as e:
            print("PENETROMETRY, queue creating:", e)
            return False
        
        return True

    def __close_queues(self):
        """Closing descriptor file and removing queue if it exist
        """
        if self.__data_queue is not None:
            try:
                self.__data_queue.close()
            except Exception as e:
                print("PENETROMETRY, closing queue descriptor:", e)
            try:
                posix_ipc.unlink_message_queue(self.__data_queue_name)
            except Exception as e:
                print("PENETROMETRY, suppressing queue:", e)
        if self.__params_queue is not None:
            try:
                self.__params_queue.close()
            except Exception as e:
                print("PENETROMETRY, closing queue descriptor:", e)
            try:
                posix_ipc.unlink_message_queue(self.__params_queue_name)
            except Exception as e:
                print("PENETROMETRY, suppressing queue:", e)

    def __search_new_params(self):
        """Read the params_queue and apply new trigger parameters if there is data in
           the queue.
        """
        try:
            new_params = self.__params_queue.receive(timeout=0.01)
            new_params_json = json.loads(new_params[0])

            self.__nb_capture_before = new_params_json['nb_capture_before']
            self.__nb_capture_over = new_params_json['nb_capture_over']
            self.__nb_capture_after = new_params_json['nb_capture_after']
            self.__threshold = new_params_json['threshold']

            self.__buffer_size  = self.__nb_capture_before + self.__nb_capture_over + self.__nb_capture_after
        except posix_ipc.BusyError:
            pass # If queue is empty it is not a problem, continue


    def __send_to_queue(self, content_json):
        """Send a json content into the data queue, if the queue is full it remove the 
           older message and try to send again. 

        Returns:
            content_json (str): The JSON formatted str to be send in queue.
        """
        try:
            self.__data_queue.send(content_json, timeout=0.01)
        except posix_ipc.BusyError:
            try:
                self.__data_queue.receive(timeout=0.01)
                self.__data_queue.send(content_json, timeout=0.01)
            except posix_ipc.BusyError as e:
                print("PENETROMETRY, sending data in queue", e)
            except ValueError as e :
                print("PENETROMETRY, unable to send a message too long in queue", e)
        except ValueError as e :
            print("PENETROMETRY, unable to send a message too long in queue", e)

    
    def __capture_data(self):
        """Get datas from vesc, calculate mecanical rpm and torque from those, add a 
           timestamp and store those data into a buffer.

        Returns:
            capture (dict): The dict wich contains a vesc data capture.
        """
        capture = self.__vescAdapter.get_sensors_data_of_can_id(self.__field_to_analyse, self.__extraction_engine_can_id)
        if capture is not None:
            capture['timestamp'] = time.time()
            capture['rpm_meca'] = capture['rpm'] / config.POLARY_POLE_COUNT
            capture['torque'] = capture['avg_motor_current'] * config.TORQUE_CONST
            del capture['rpm']
            del capture['avg_motor_current']

            self.__buffer.append(capture)
            while len(self.__buffer) > self.__buffer_size :
                self.__buffer.pop(0)

        return capture

    def __detect_threshold_surpassing(self, capture):
        """Update variables for extraction detecting. It will set self.__extraction_trigger 
           to True if an extraction is detected by the algorithm.

        Args:
            capture (dict): The dict wich contains a vesc data capture.
        """
        if ((capture.get('rpm_meca', 0)) >= self.__threshold):
            self.__count_capture_over_threshold += 1
        else:
            self.__count_capture_over_threshold = 0

        if self.__count_capture_over_threshold >= self.__nb_capture_over:
            self.__extraction_trigger = True

    def __slice_buffer(self, buffer, size_slice):
        """Slice a buffer into smaller parts of a given size and add it to a json format.

        Args:
            buffer (list): The buffer to slice.
            size_slice (int): The size max of a buffer slice.

        Returns:
            list: The list wich contains each slice of the buffer and the json data added.
        """
        # Slicing the buffer
        buffer_slices = [buffer[i:i + size_slice] for i in range(0, len(buffer), size_slice)]
        
        custom_slices = []
        for i, slice in enumerate(buffer_slices):
            is_last_slice = (i == len(buffer_slices) - 1) # Check if it is the last slice
            # Creating a json objet with vesc data and trigger's param
            extraction_data = {
                "captures": slice,
                "detection_parameters": {
                    "captures_frequency": config.PENETROMETRY_ANALYSE_FREQUENCY,
                    "threshold": self.__threshold,
                    "nb_capture_over": self.__nb_capture_over,
                    "nb_capture_before": self.__nb_capture_before,
                    "nb_capture_after": self.__nb_capture_after,
                },
                "is_last_slice": is_last_slice,
            }
            custom_slices.append(extraction_data)
        
        return custom_slices
    
    def set_current_coordinates(self, latitude, longitude):
        """Set the value of attribute current_coordinate with a latitude value and a longitude value.

        Args:
            latitude (float/int): The latitude value to be set.
            longitude (float/int): The longitude value to be set.
        """
        self.__current_coordinates['latitude'] = latitude
        self.__current_coordinates['longitude'] = longitude
        

    def __penetrometry_thread_function(self):
        """The main loop of the thread, it get data from vesc, detect an extraction, send 
           datas in queue IPC and save it to a file.
        """
        if (self.__open_queues() == False):
            self.__penetrometry_thread_alive = False

        # Thread loop
        while self.__penetrometry_thread_alive:
            initialTime = time.time()
            self.__search_new_params()
            capture = self.__capture_data()
            if capture is not None:
                self.__detect_threshold_surpassing(capture)
                if self.__extraction_trigger:
                    self.__count_capture_after_trigger += 1

                # When the datas are nicely placed in the buffer
                if self.__count_capture_after_trigger == self.__nb_capture_after:
                    slices = self.__slice_buffer(self.__buffer, self.__data_queue_buffer_slice_size)
                    for slice in slices:
                        self.__send_to_queue(json.dumps(slice))
                            
                    if(self.__is_saving_mode_active):
                        self.__save_in_file(self.__file_path, self.__buffer, self.__current_coordinates)
                        
                    # Reinitialise variable for the trigger
                    self.__buffer.clear()
                    self.__count_capture_over_threshold = 0
                    self.__extraction_trigger = False
                    self.__count_capture_after_trigger = 0
                
            # Do not saturate the CPU
            time_to_sleep = self.__analyse_frequency - (time.time()- initialTime)
            if (time_to_sleep > 0):
                time.sleep(time_to_sleep)

        self.__close_queues()