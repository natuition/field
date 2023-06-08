from enum import Enum
import threading
import multiprocessing
import time

LISTENER_CHECK_FREQUENCE_HZ = 4

class MessageType(Enum):
    TOO_SMALL_AREA = 1

class Message:

    def __init__(self, message_type: MessageType, content: object = None) -> None:
        self.__message_type: MessageType = message_type
        self.__content: object = content

    def get_message_type(self) -> MessageType:
        return self.__message_type
    
    def get_content(self) -> object:
        return self.__content
    
class MessageListener:

    def __init__(self) -> None:
        multiprocessing.set_start_method('spawn')
        self.__queue = multiprocessing.JoinableQueue()

        self.__keep_thread_alive = True
        self.__listener_thread = threading.Thread(target=self.__listener_thread_tf, daemon=True)
        self.__listener_thread.start()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()

    def close(self):
        self.__queue.close()
        self.__queue.join_thread()

    def get_multiprocessing_joinable_queue(self) -> multiprocessing.JoinableQueue:
        return self.__queue
    
    def create_the_process(self, target, *args) -> None:
        self.__process = multiprocessing.Process(target=target, args=args)
        
    def get_process(self) -> multiprocessing.Process:
        return self.__process

    def __listener_thread_tf(self) -> None:
        while self.__keep_thread_alive:
            if not self.__queue.empty():
                message: Message = self.__queue.get_nowait()
                print(message.get_message_type())
                print(message.get_content())
                self.__queue.task_done()
            else:
                time.sleep(1/LISTENER_CHECK_FREQUENCE_HZ)