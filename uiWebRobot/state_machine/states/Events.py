import sys
sys.path.append('../')

from enum import Enum

class Events(Enum):
    ERROR = -1
    LIST_VALIDATION = 0
    CREATE_FIELD = 1
    VALIDATE_FIELD = 2
    START_MAIN = 3
    CONTINUE_MAIN = 4
    START_AUDIT = 5
    CONTINUE_AUDIT = 6
    CONFIG_IS_SET = 7
    WORK_IS_DONE = 8
    STOP = 9
    WHEEL = 10
    AUDIT_ENABLE = 11
    AUDIT_DISABLE = 12
    VALIDATE_FIELD_NAME = 13