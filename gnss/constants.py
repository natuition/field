from safe_import_of_config import config


IPC_ENDPOINT = "ipc://{}".format(
    config.GPS_IPC_ZMQ_PATH
)

TOPIC = config.GPS_IPC_ZMQ_TOPIC