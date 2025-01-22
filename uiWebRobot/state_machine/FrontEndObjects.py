import sys
sys.path.append('../')

from enum import Enum

class ButtonState(Enum):
    ENABLE = True
    DISABLE = False
    CHARGING = "charging"
    VALIDATE = "validate"
    NOT_HERE = None

class PhysicalBlocageFEO(Enum):
    DISABLE = False
    DETECTED = True
    REVERSING = "reversing"
    BLOCKED = "blocked"
    RELOADING = "reloading"

class AuditButtonState(Enum):
    EXTRACTION_DISABLE = True
    EXTRACTION_ENABLE = False
    IN_USE = "use"
    NOT_IN_USE = "not-use"
    BUTTON_DISABLE = "disable"

    @staticmethod
    def get_by_bool(self, status: bool):
        if status:
            return AuditButtonState.EXTRACTION_DISABLE
        else:
            return AuditButtonState.EXTRACTION_ENABLE

class FrontEndObjects:

    def __init__(self, fieldButton: ButtonState, startButton: ButtonState, continueButton: ButtonState, stopButton: ButtonState, wheelButton: ButtonState, removeFieldButton: ButtonState, joystick: bool, slider: float, physicalBlocage: PhysicalBlocageFEO = PhysicalBlocageFEO.DISABLE, audit: AuditButtonState = AuditButtonState.BUTTON_DISABLE):
        self.fieldButton: ButtonState = fieldButton
        self.startButton: ButtonState = startButton
        self.continueButton: ButtonState = continueButton
        self.stopButton: ButtonState = stopButton
        self.wheelButton: ButtonState = wheelButton
        self.removeFieldButton: ButtonState = removeFieldButton
        self.joystick: bool = joystick
        self.slider: float = slider
        self.audit: AuditButtonState = audit
        self.physicalBlocage: PhysicalBlocageFEO = physicalBlocage

    def to_json(self):
        return{
            "joystick": self.joystick,
            "fieldButton": self.fieldButton.value,
            "startButton": self.startButton.value,
            "continueButton": self.continueButton.value,
            "stopButton": self.stopButton.value,
            "wheelButton": self.wheelButton.value,
            "audit": self.audit.value,
            "slider": self.slider,
            "removeFieldButton": self.removeFieldButton.value,
            "physicalBlocage": self.physicalBlocage.value
        }

    def __str__(self):
        return str({
            "joystick": self.joystick,
            "fieldButton": self.fieldButton,
            "startButton": self.startButton,
            "continueButton": self.continueButton,
            "stopButton": self.stopButton,
            "wheelButton": self.wheelButton,
            "audit": self.audit,
            "slider": self.slider,
            "removeFieldButton": self.removeFieldButton,
            "physicalBlocage": self.physicalBlocage
        })