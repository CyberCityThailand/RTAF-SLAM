''' Master Agent Status Class '''

from dataclasses import dataclass
from datetime import datetime
from distutils.log import error
from collections import namedtuple
import os
import serial.tools.list_ports
import yaml
import json

# Custom Tuple for position data
Position = namedtuple('Position', 'lat lon alt')

class AgentPosition():

    def __init__(self, lat=0.0, lon=0.0, alt=0.0, relative_alt=0.0, 
                 vx=0.0, vy=0.0, vz=0.0, hdg=0.0):

        self.lat = lat
        self.lon = lon
        self.alt = alt
        self.relative_alt = relative_alt
        self.vx = vx
        self.vy = vy
        self.vz = vz
        self.hdg = hdg

class AgentStatus():

    ''' Establishes an Agent Status class/object that is passing the current status around 
        to the different managers within the agent '''

    def __init__(self, d=None):

        self.flight_mode_changed = False
        self._flight_mode = "MAVLINK Lost"
        self.arm_status_changed = False
        self._arm_status = 1 #[0: prearm-fail, 1: MAVLINK Lost, 2: prearm-good, 3: armed-idle, 4: armed-above idle]
        self.agent_position_changed = False
        self._agent_position = AgentPosition()

        self.agent_id = None
        self.time_stamp = datetime.now().strftime("%m/%d/%Y, %H:%M:%S.%f")
        self.throttle_position = 0 # [0: idle (limited arm time), 1: above idle (unlimited time)]
        self.battery = 100

        ### These are the network ports that this agent uses to setup push-pull and publisher ports
        ### unique to this agent
        self.agent_server_host = None
        self.agent_server_port = None
        self.agent_publisher_port = None

        if d is not None:
            for key, value in d.items():
                setattr(self, key, value)


    ###################### Property Changes #########################

    ### arm_status (current arm status from ardupilot) ###
    @property
    def arm_status(self):
        return self._arm_status

    @arm_status.setter
    def arm_status(self, value):
        if value != self._arm_status:
            self._arm_status = value
            self.arm_status_changed = True

    def check_for_arm_status_changed(self):
        if self.arm_status_changed:
            self.arm_status_changed = False
            return True
        else:
            return False


    ### flight_mode (current flight mode from ardupilot) ###

    @property
    def flight_mode(self):
        return self._flight_mode
    
    @flight_mode.setter
    def flight_mode(self, value):
        if value != self._flight_mode:
            self._flight_mode = value
            self.flight_mode_changed = True

    def check_for_flight_mode_changed(self):
        if self.flight_mode_changed:
            self.flight_mode_changed = False
            return True
        else:
            return False


    ### agent_position (current position from ardupilot) ###
    @property
    def agent_position(self):
        return self._agent_position
    
    @agent_position.setter
    def agent_position(self, value):
        if value != self._agent_position:
            self._agent_position = value
            self.agent_position_changed = True

    def check_for_position_change(self):
        if self.agent_position_changed:
            self.agent_position_changed = False
            return True
        else:
            return False


    ################ Set and Get parameters to and from JSON ##################

    def __str__(self):
        
        return str(self.get_json())

    def get_json(self):
        ''' Returns the JSON string of the Agent_Status class variables 
            This is used to send the agent as a JSON over zmq '''

        final_attribute_dict = self.get_dict()

        final_attribute_json = json.dumps(final_attribute_dict)
        return final_attribute_json

    def get_dict(self):
        ''' Returns the JSON string of the Agent_Status class variables 
            This is used to send the agent as a JSON over zmq '''

        final_attribute_list = [a for a in dir(self) if not (a.startswith('__') or a.startswith('_'))
                                            and not callable(getattr(self, a))]
        final_attribute_dict = dict()
        for attribute in final_attribute_list:
            try:
                json.dumps(getattr(self, attribute))
                attribute_value = getattr(self, attribute)
            except:
                attribute_value = getattr(self, attribute).__dict__
            final_attribute_dict.update({attribute: attribute_value})

        return final_attribute_dict

    @classmethod
    def timestamped_agent_obj_dict_to_str_dict(cls, agent_status_obj_dict, new_time_stamp=False):
        ''' Takes a dictionary of time_stamp and list of agent_status objects and
            turns it into a dictionary of time_stamp and list of agent_status dictionaries.
            This is used to convert the object dictionary to a text dictionary to be sent via JSON '''
        
        agent_status_dict = dict()
        if new_time_stamp is True:
            agent_status_dict['time_stamp'] = datetime.now().strftime("%m/%d/%Y, %H:%M:%S.%f")
        else:
            agent_status_dict['time_stamp'] = agent_status_obj_dict['time_stamp']
        agent_status_dict['agent_status_list'] = []
        for agent in agent_status_obj_dict['agent_status_list']:
            agent_status_obj: AgentStatus = agent
            agent_status_dict['agent_status_list'].append(agent_status_obj.get_dict())

        return agent_status_dict