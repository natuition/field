from enum import Enum
from typing import List

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
    CLOSE_APP = 14
    CALIBRATION = 15
    CALIBRATION_DETECT = 16
    CALIBRATION_MOVE = 17
    CALIBRATION_VALIDATE = 18
    CALIBRATION_CANCEL = 19
    ACTUATOR_SCREENING = 20
    ACTUATOR_SCREENING_START = 21
    ACTUATOR_SCREENING_PAUSE = 22
    ACTUATOR_SCREENING_STOP = 23
    PHYSICAL_BLOCAGE = 24

    @staticmethod
    def from_str(events_name: str) -> 'Events':
        return Events[events_name.upper()]
    
    @staticmethod
    def from_value(events_name: int) -> 'Events':
        return Events(events_name)
    
    @staticmethod
    def event_list_to_str_list(list_events: List['Events']) -> List[str]:
        return [str(i) for i in list_events]

    def __str__(self):
        return self.name.lower()

    def __eq__(self, other):
        return self.value == other.value