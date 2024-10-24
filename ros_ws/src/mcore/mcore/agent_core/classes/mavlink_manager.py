''' Defines the mavlink manager class'''

import os
import math
from typing import Dict
import serial.tools.list_ports
from datetime import datetime, timedelta
from threading import Thread, Timer, Event
import time

from collections import namedtuple

os.environ['MAVLINK20'] = '1'
from pymavlink import mavutil
import zmq

from .agent_status_class import AgentStatus
from .agent_status_class import AgentPosition
# from agent_hub import AgentHub

class MavlinkManager:

    ''' This class is used to talk between the agent.py process and the Flight Controller.
        It will listen for commands from agent.py and convert it to MAVLINK messages and
        send it to the Flight Controller over the conneted port.
        It will also listen to the flight controller's port and send flight controller heartbeat,
        status and ACK messages back to the agent.py file for processing'''
    
    def __init__(self, config, agent_hub, verbose=False):

        # class settings and parameters
        self.config = config
        # self._agent_hub: AgentHub = agent_hub
        self._agent_hub = agent_hub
        self.agent_status_obj: AgentStatus = self._agent_hub.agent_status_obj
        self._verbose = verbose

        # aurdupilot connectivity status parameters
        self.mav_connection = None
        self.port_connected = False
        self.running_in_sim = False
        self.mavlink_ready = False
        self._time_of_last_ardupilot_heartbeat = datetime.now()
        self.wait_for_ack = False
        self.current_mavlink_message_dict = dict()

        self._mavlink_manager_messenger_bind = self._agent_hub.mavlink_manager_messenger_binding
        self._mavlink_bind_poller = self._agent_hub.mavlink_bind_poller

        # default mavlink messaging parameters
        self._message_names = []
        self._base_mode = 0
        self._custom_mode = 0
        self._system_status = 0
        self._prearm_status = False
        self._mode_string = ""

        self._initialize_the_port()

        ### Sets up the listener for the Ardupilot Heartbeat and other designated messages with intervals (GPS, etc.)
        self._mavlink_manager_heartbeat_listener_loop = Thread(target=self.mavlink_manager_message_queue)
        self._mavlink_manager_heartbeat_listener_loop.daemon = True

        self.initialize_logging()


    def vprint(self, print_string):
        if self._verbose:
            print(print_string)


    def initialize_instance_variables(self):

        self._message_names = list(self.config['MESSAGE_INTERVALS'].keys())
        for message_name in self._message_names:
            self.current_mavlink_message_dict[message_name] = None


    ##################################### Manage Logging ##########################################
    
    def initialize_logging(self):

        def write_to_file(filename, string_list, start_logging, log_interval):
            while True:
                # print("start_log = " + str(vars(start_logging)))
                if start_logging.is_set():
                    
                    if string_list[0] != "":
                        try:
                            with open(filename, 'a') as f:
                                f.write(string_list[0])
                            string_list[0] = ""
                        except IOError:
                            print(IOError)
                            pass
                
                time.sleep(log_interval)
        
        self.log_timer_dict = {}
        for key, value in self.config['MESSAGE_INTERVALS'].items():
            if value['LOG_INTERVAL'] != 0:
                self.log_timer_dict[key] = datetime.now() + timedelta(microseconds = int(value['LOG_INTERVAL']))

        # Create the subdirectory if it doesn't exist
        os.makedirs('logs', exist_ok=True)
        filename = os.path.join('logs', str(datetime.now().strftime('%Y%m%d_%H%M%S') + '.txt'))
        self.log_str = [""]
        self.start_logging = Event()
        self.start_logging.clear()
        t = Thread(target=write_to_file, args=(filename, self.log_str, self.start_logging, 1))
        t.start()


    def log_string(self, string):

        if self.start_logging.is_set():
            self.log_str[0] = str(self.log_str[0] + 
                                    str(self.agent_status_obj.time_stamp) + 
                                    " --- " + 
                                    str(string + "\n"))


    ############################ Manage General Execution ##################################

    def run(self):
        ''' Starts the thread that listens for messages from the agent.py and flight controller port '''
        self._mavlink_manager_heartbeat_listener_loop.start()

    def stop(self):
        ''' Stops the thread that listens for messges from the agent and flight controller port'''
        self._mavlink_manager_heartbeat_listener_loop.join()


    def mavlink_manager_message_queue(self):

        while True:

            # If the mav_connection is not established yet, then don't run the function
            if self.mav_connection == None:
                return

            # If the mav_connection is not established yet, then don't run the function
            if self.mav_connection == None:
                return
            
            # get all the desired messages in the MESSAGE_INTERVALS ymal dictionary
            message_names = list(self.config['MESSAGE_INTERVALS'].keys())
            
            # Check if the ardupilot has stopped talking
            self._lost_connection_to_flightcontoller_check()

            # LISTEN for the desired messages from the ardupilot
            msg_from_ardupilot = self.mav_connection.recv_match(blocking=True)
            msg_type = msg_from_ardupilot.get_type()
            self.current_mavlink_message_dict[msg_type] = msg_from_ardupilot

            # Update agent_status from HEARTBEAT
            if msg_type == 'HEARTBEAT' and msg_from_ardupilot.type == 2:

                # print("Heartbeat from system {} component {}".format(self.mav_connection.target_system, self.mav_connection.target_component))

                # self.vprint("Cube Heartbeat")
                # self.vprint(msg_from_ardupilot)
                self._time_of_last_ardupilot_heartbeat = datetime.now()
                mode = mavutil.mode_string_v10(msg_from_ardupilot)
                # print("The mode = " + mode)

                # self.vprint(str(msg_from_ardupilot.base_mode) + " / " + 
                #       str(msg_from_ardupilot.custom_mode) + " / " + 
                #       str(msg_from_ardupilot.system_status))
                # if the ardupilot heartbeat shows the current mode has changed
                if (msg_from_ardupilot.base_mode != self._base_mode) or (
                            msg_from_ardupilot.custom_mode != self._custom_mode) or (
                            msg_from_ardupilot.system_status != self._system_status):
                    
                    self._base_mode = msg_from_ardupilot.base_mode
                    self._custom_mode = msg_from_ardupilot.custom_mode
                    self._system_status = msg_from_ardupilot.system_status
                    self._update_agent_arm_status()

                if mode != self.agent_status_obj.flight_mode:
                    self.agent_status_obj.flight_mode = mode
                    self.agent_status_obj.time_stamp = datetime.now().strftime("%m/%d/%Y, %H:%M:%S.%f")
                    # self._thread_pair_mavlink_manager_message.send_multipart(["change_flt_mode".encode()])

            # else you are getting the SYS_STATUS information you requested at the desired refresh rate
            if msg_type == 'SYS_STATUS':
                # print(msg_from_ardupilot)
                bitmask_health = int(msg_from_ardupilot.to_dict()['onboard_control_sensors_health'])
                status_bits = format(bitmask_health, "032b")
                status_to_check = [mavutil.mavlink.MAV_SYS_STATUS_PREARM_CHECK]
                new_prearm_status = self.check_MAV_SYS_STATUS_SENSOR_prearm_status(status_bits, status_to_check)[268435456]
                # print(new_prearm_status)
                # if your prearm_status has changed (such as a timeout without launch) then self.update_agent_arm_status()
                if new_prearm_status != self._prearm_status:
                    self._prearm_status = new_prearm_status
                    self._update_agent_arm_status()

            # else you are getting the position data you requested at the desired refresh rate
            if msg_type == 'GLOBAL_POSITION_INT':

                self.agent_status_obj.time_stamp = datetime.now().strftime("%m/%d/%Y, %H:%M:%S.%f")
                self.agent_status_obj.agent_position = AgentPosition(float(msg_from_ardupilot.to_dict()["lat"])/10000000.0, 
                                                  float(msg_from_ardupilot.to_dict()["lon"])/10000000.0, 
                                                  float(msg_from_ardupilot.to_dict()["alt"])/1000.0,
                                                  float(msg_from_ardupilot.to_dict()["relative_alt"])/1000.0,
                                                  float(msg_from_ardupilot.to_dict()["vx"])/100.0,
                                                  float(msg_from_ardupilot.to_dict()["vy"])/100.0,
                                                  float(msg_from_ardupilot.to_dict()["vz"])/100.0,
                                                  float(msg_from_ardupilot.to_dict()["hdg"])/100.0)

            # log the timestamped data to the text file
            if msg_type in self.log_timer_dict:
                if self.log_timer_dict[msg_type] < datetime.now():
                    self.log_string(str(msg_from_ardupilot.to_dict()))
                    self.log_timer_dict[msg_type] = (datetime.now() + 
                                    timedelta(microseconds=int(self.config['MESSAGE_INTERVALS'][msg_type]['LOG_INTERVAL'])))



    ######################################## Initialize the Ports #######################################

    def _lost_connection_to_flightcontoller_check(self):
        ''' Checks to see if the time since the last heartbeat has past the lost connection parameter and 
            sends a message to the agent if it is '''
        delta_time = (datetime.now() - self._time_of_last_ardupilot_heartbeat).seconds
        if delta_time > self.config['CUBE_LOST_COMM_LIMIT']:
            self._time_of_last_ardupilot_heartbeat = datetime.now()
            self.agent_status_obj.flight_mode = "NO MAVLINK"
            self.agent_status_obj.arm_status = 1
            self.agent_status_obj.time_stamp = datetime.now().strftime("%m/%d/%Y, %H:%M:%S.%f")
            # Send a message back to the agent to let the agent know there is no MAVLINK connection to the flight controller
            # and to initiate an immediate update back to the UIs
            self.vprint("I have  ")
            self._mavlink_manager_messenger_bind.send_multipart(["change_arm_status".encode()])


    def _initialize_the_port(self):
        ''' Gets the port the Cube is connected to '''

        def _wait_for_heartbeat():

            try:
                self.vprint("waiting for initial adupilot heartbeat")
                self.mav_connection.wait_heartbeat()
                self.vprint("received intitial ardupilit heartbeat")
                self.mavlink_ready = True
                self.vprint("Heartbeat from ardupilot: " + str(self.mav_connection.target_system) +  " / " + str(self.mav_connection.target_component))
                return True
            except:
                self.vprint("Failed to get heartbeat from ardupilot")
                return False

        def _set_message_intervals():

            requested_message_interval_dict = self.config['MESSAGE_INTERVALS']
            for requested_message in requested_message_interval_dict.keys():

                self.vprint(str(requested_message) + " / " + str(requested_message_interval_dict[requested_message]))

                self.mav_connection.mav.command_long_send(
                    self.mav_connection.target_system,
                    self.mav_connection.target_component,
                    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                    0, int(requested_message_interval_dict[requested_message]['ID']),
                    requested_message_interval_dict[requested_message]['INTERVAL'], 0, 0, 0, 0, 0
                )
                ackmsg = self.mav_connection.recv_match(type='COMMAND_ACK', blocking=True)
                self.vprint(ackmsg)

        # self.running_in_sim = False
        ports = list(serial.tools.list_ports.comports())
        for p in ports:
            self.vprint(p)
            #if any(flt_ctlr in p.description for flt_ctlr in ["CubeBlack", "CubeOrange", "Cube", "CUBE", "USB Serial Device"]) and self.port_connected is False:
            if any(
                flt_ctlr in p.description
                for flt_ctlr in ["CubeBlack", "Cube", "CUBE", "USB Serial Device"]
            ):    
                print("#################### Establishing Connection to Cube ##########################")
                _serial_port = serial.Serial(p.device)
                _serial_port.close()    # Need to close for window's devices / will deny access when trying to set mav_connection
                #self.port_connected = True
                self.vprint("Serial Port = "+ str(_serial_port))
                print("Serial Port = "+ str(_serial_port))
                self.serial_port = _serial_port
                # setup the mavconnection to that port and ask for a heartbeat
                if self.mav_connection:
                    self.mav_connection = None
                self.mav_connection = mavutil.mavlink_connection(self.serial_port.port, baud=9600) #baud=9600) ### Used to connect directly to Ardupilot
                # self.mav_connection = mavutil.mavlink_connection('udp:127.0.0.1:14551') ### Used to connect through MavProxy
                self.running_in_sim = False

        if not self.port_connected:
            self.vprint("looking for an ardupilot connection on: " + self.config['SIMULATOR_ADDRESS'])
            print("#################### Establishing Connection to Mavproxy ##########################")
            self.mav_connection = mavutil.mavlink_connection(self.config['SIMULATOR_ADDRESS'])
            if self.mav_connection != None:
                self.vprint("There is a connection to the ardupilot")
                self.running_in_sim = True
            else:
                self.vprint("There is not a created mav_connection")
                self.running_in_sim = False

        self.mav_connection.target_system = self.config['SYS_ID']

        # If wait_for_heartbeat works, you are ready to start talking across the mav_connection
        if _wait_for_heartbeat() is True:
            self.vprint("I heard the first heartbeat from the ardupilot")
            print("Connection Established")
            print("Address: ", self.config['SIMULATOR_ADDRESS'])
            self.mavlink_ready = True
            _set_message_intervals()
        # else the port you've connected to is no longer valid (no heartbeat) so try and reconnect to the port again
        else:
            self.port_connected = False
            self.mavlink_ready = False


    def _update_agent_arm_status(self):
        
        # Set this agent's agent_status_obj's arm status to it's current arm value
        if  (self._prearm_status) is False and (self.running_in_sim == False):
            self.agent_status_obj.arm_status = 0
        else:
            if self._base_mode == 89 or self._base_mode == 81:
                self.agent_status_obj.arm_status = 2
            elif self._base_mode == 209 or self._base_mode == 217:
                self.agent_status_obj.arm_status = 3
        if self._system_status == 4:
            self.agent_status_obj.arm_status = 4

        # Update the current timestamp of the agent's agent_status_obj's timestamp
        self.agent_status_obj.time_stamp = datetime.now().strftime("%m/%d/%Y, %H:%M:%S.%f")
        # Send a message back to the agent to let the agent know the flight controller's arm status has changed
        # and to initiate an immediate update back to the UIs
        self.vprint("mm - I should be changing arm status")
        # self._mavlink_manager_messenger_bind.send_multipart(["change_arm_status".encode()])


    def check_MAV_SYS_STATUS_SENSOR_prearm_status(self, status_bits, check_status_ids):
        running_status = dict()
        for status_id in check_status_ids:
            pos = int(math.log(status_id, 2)) + 1
            if str(status_bits)[32-int(pos)] == '1':
                running_status[status_id] = True
            else:
                running_status[status_id] = False
        return running_status
