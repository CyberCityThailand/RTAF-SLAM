'''
    This class inherits from the AgentCommandManager class and is where the
    developer puts all the intermediate logic for their custom commands.
'''

import os
import json
from pymavlink import mavutil
from classes.agent_command_manager import AgentCommandManager

os.environ['MAVLINK20'] = '1'

##############################################################################
# These are the commands that are currently managed by the AgentCommandManager
#
#   all mode changes via:                   MAV_CMD_DO_SET_MODE
#   'arm' w/ override_safety &
#      delay_before_disarm"& 'disarm' via:  MAV_CMD_COMPONENT_ARM_DISARM
#   'takeoff' w/ alt via:                   MAV_CMD_NAV_TAKEOFF
#
# If you have a custom command that does not conform


class CustomAgentCommandManager(AgentCommandManager):

    def handle_custom_command(self, _cmd, _params=None):

        # print("The child's custom command = " + str(_cmd) + " and the params = " + str(_params))
        print(_cmd)
        # print(self.agent_status_obj.__dict__)
