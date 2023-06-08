import multiprocessing_communication
import multiprocessing
import main

if __name__ == '__main__':
    message_listener = multiprocessing_communication.MessageListener()

    p = multiprocessing.Process(target=main.main, args=(message_listener.get_multiprocessing_joinable_queue(),))
    p.start()

    print("Main running...")

    p.join()

    print("Main finish")