import sys
sys.path.append('../')

from flask_socketio import SocketIO

import utility
from state_machine.states import CheckState
from state_machine.states import ErrorState
from state_machine import Events
from state_machine import State
from state_machine.FrontEndObjects import FrontEndObjects


class StateMachine:

    def __init__(self, socketio):
        utility.create_directories("logs/")
        self.logger = utility.Logger("logs/"+utility.get_current_time())
        sys.stderr = ErrorLogger(self.logger)
        self.socketio: SocketIO = socketio
        self.currentState: State.State = CheckState.CheckState(socketio,self.logger)
        msg = f"Current state : {self.currentState}."
        self.logger.write_and_flush(msg+"\n")
        print(msg)

    def on_event(self, event: Events.Events):
        print()
        msg = f"{self.currentState} received event : {event}."
        self.logger.write_and_flush(msg+"\n")
        print(msg)

        try:
            newState = self.currentState.on_event(event)
        except Exception as e:
            self.logger.write_and_flush("[Error] "+str(e)+"\n")
            newState = ErrorState.ErrorState(self.socketio,self.logger,str(e))

        if newState is None:
            msg = f"Error last state : {self.currentState}."
            self.logger.write_and_flush(msg+"\n")
            newState = ErrorState.ErrorState(self.socketio,self.logger,msg)

        if str(newState) in ["StartingState","ResumeState"]:
            self.change_current_state(newState.on_event(Events.Events.CONFIG_IS_SET))
        else:
            self.change_current_state(newState)

        msg = f"Current state : {self.currentState}."
        self.logger.write_and_flush(msg+"\n")
        print(msg)
    
    def on_socket_data(self, data):
        try:
            self.currentState = self.currentState.on_socket_data(data)
        except Exception as e:
            self.logger.write_and_flush("[Error] "+str(e)+"\n")
            self.change_current_state(ErrorState.ErrorState(self.socketio,self.logger,str(e)))
    
    def change_current_state(self, newState):
        self.currentState = newState
        msg = f"Current state : {self.currentState}."
        self.logger.write_and_flush(msg+"\n")
        print(msg)

    def getStatusOfControls(self):
        frontEndObjects: FrontEndObjects|dict = self.currentState.getStatusOfControls()
        #print(f"FrontEndObjects = {frontEndObjects}")
        if isinstance(frontEndObjects,dict):
            return frontEndObjects
        else:
            return frontEndObjects.to_json()
    
    def getField(self):
        return self.currentState.getField()

    def close(self):
        pass

class ErrorLogger:

    def __init__(self, logger: utility.Logger):
        self.logger = logger

    def write (self, s):
        print(s)
        self.logger.write_and_flush(s+"\n")