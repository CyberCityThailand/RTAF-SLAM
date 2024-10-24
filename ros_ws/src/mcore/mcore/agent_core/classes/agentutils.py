''' Utility Functions and Methods for the Agent '''

from dataclasses import dataclass
from datetime import datetime
from distutils.log import error
import os
import serial.tools.list_ports
import yaml
import json

import zmq
import pymavlink
from pymavlink import mavutil

@dataclass
class Position:

    lat: float = 0.0
    lon: float = 0.0
    alt: float = 0.0

class AgentUtils:

    def __init__(self):

        self.config = self.get_config_dict()

    @classmethod
    def get_config_dict(cls, yaml_file):
        ''' Gets the config file for this agent '''
        # Load config yaml file and store it in config dictionary
        here = os.path.dirname(os.path.abspath(__file__))
        # here = here[:here.find('/classes')]
        here = os.path.normpath(here)
        here = os.path.dirname(here)
        print(here)
        with open(os.path.join(here, yaml_file), "r") as f:
            config = yaml.safe_load(f)
        return config

    @classmethod
    def return_json(cls, obj):
        ''' Returns the JSON string of the Agent_Status class variables 
            This is used to send the agent as a JSON over zmq '''

        final_attribute_list = [a for a in dir(obj) if not a.startswith('__')
                                            and not callable(getattr(obj, a))]
        final_attribute_dict = dict()
        for attribute in final_attribute_list:
            try:
                json.dumps(getattr(obj, attribute))
                attribute_value = getattr(obj, attribute)
            except:
                attribute_value = getattr(obj, attribute).__str__()
            final_attribute_dict.update({attribute: attribute_value})

        final_attribute_json = json.dumps(final_attribute_dict)
        return final_attribute_json
