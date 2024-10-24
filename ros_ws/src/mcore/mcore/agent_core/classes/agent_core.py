'''
    The agent_core.py file is the main file that runs on the agent.  It both listens
    for incoming command messages as well as changes to the agent's states such as
    position and modes that are registered in the agent_status_object.  This is also
    where developers will place their command logic to send commands to the Ardupilot
    using a custom inheritance of the AgentCommandManager class. 
'''

import time
import threading

from .agentutils import AgentUtils
from .agent_status_class import AgentStatus
from .agent_hub import AgentHub
from .custom_agent_command_mgr import CustomAgentCommandManager


class AgentCore:

    def __init__(self, loop_delay=0, log_file=False, verbose=False):

        self._loop_delay = loop_delay

        # Initialize the agent's status object which is used to store all the current
        # information about the status information about the agent
        self.agent_status_obj = AgentStatus()
        # Get the agent configuration from the agent_configuration.yaml file
        self.config = AgentUtils.get_config_dict('agent_configuration.yaml')
        # Initialize the agent hub that manages all internal and external communications
        self.agent_hub = AgentHub(self.config, self.agent_status_obj, verbose=verbose)
        # Initialize the agent's command object that is used to send commands to the
        # Ardupilot
        self.agent_command_mgr = CustomAgentCommandManager(self.config, self.agent_hub, self.agent_hub.mav_mgr.mav_connection, verbose=False)

        # self.run_agent_core_loop()

        self.stopFlag = threading.Event()
        thread = threading.Thread(target=self.run_agent_core_loop_thread, args=(self.stopFlag, self._loop_delay))
        thread.start()

    def agent_core_loop_functions(self):
        pass

    def run_agent_core_loop_thread(self, event, loop_delay):
        # Let the agent_status_obj and agent_hub get settled
        # with a 3 sec sleep before starting the command loop
        time.sleep(3)
        while True:
            if not event.wait(loop_delay):
                # print("my thread")
                self.agent_core_loop_functions()

    # Manage the start and stop of the log file
    def start_logging(self):
        self.agent_hub.mav_mgr.start_logging.set()

    def stop_logging(self):
        self.agent_hub.mav_mgr.start_logging.clear()

    def log_string(self, string):
        self.agent_hub.mav_mgr.log_str(string)
