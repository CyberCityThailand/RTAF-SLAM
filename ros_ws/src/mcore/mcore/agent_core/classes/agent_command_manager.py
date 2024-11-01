'''
    Defines the Agent Command Manager class.  Contains the following commands:

    send_mode_change_to_flight_controller: MAV_CMD_DO_SET_MODE
    send_arm_disarm_command: MAV_CMD_COMPONENT_ARM_DISARM

    For any custom commands, inherit this class and override
    the handle_custom_command function

'''

import os
import json
from pymavlink import mavutil

from .agent_status_class import AgentStatus
from .agent_hub import AgentHub

# from mavlink_manager import MavlinkManager

os.environ['MAVLINK20'] = '1'

current_position = 0
current_altitude = 0


class AgentCommandManager:

    ''' This class is where the commands from the UI are translated into
        Mavlink commands and sent to the Ardupilot through the
        mavlink_manager's mav_connection
    '''

    def __init__(self, config, agent_hub, mav_connection, verbose=False):

        # class settings and parameters
        self.config = config
        self._agent_hub: AgentHub = agent_hub
        self.agent_status_obj: AgentStatus = self._agent_hub.agent_status_obj
        self._mav_connection = mav_connection
        self._verbose = verbose

    def vprint(self, print_string):
        if self._verbose:
            print(print_string)

    def process_command(self, cmd):

        _cmd = None
        _params = None

        # Log the command in the log file (assuming the start_logging is active)
        self._agent_hub.mav_mgr.log_string(str(cmd))

        # cmd is a simple string such as a flight mode
        if isinstance(cmd, str):
            _cmd = cmd
        # cmd is a dictionary with the command as the key and the params as the value
        elif isinstance(cmd, dict):
            pass
            _cmd = next(iter(cmd))
            _params = cmd[_cmd]
        # cmd is a zmq message that is encoded for network messaging and needs to be decoded
        else:
            _cmd = cmd[1].decode()
            _params = cmd[2].decode()

        # First try to see if the command is a mode change by
        # seeing if mode_mapping() returns a valid response
        try:
            mode_id = self._mav_connection.mode_mapping()[_cmd.upper()]
            self.vprint("The new mode = " + str(mode_id))
            self.send_mode_change_to_flight_controller(mode_id)

        # If the command is not a mode command, then run an if
        # sequence to decide what to do with it.
        except:
            self.vprint("Not a mode change: " + str(_cmd))
            if _cmd == 'arm':
                self.vprint(_cmd)
                self.vprint(cmd[2].decode())
                # print(cmd[2].decode().strip("\""))
                arm_params = json.loads(json.loads(_params))
                self.vprint(arm_params)
                self.send_arm_disarm_command(
                    _cmd)#,
                    #bool(arm_params["override_safety"]),
                    #int(arm_params["delay_before_disarm"]))
            elif _cmd == 'disarm':
                self.send_arm_disarm_command(_cmd)
            elif _cmd == 'takeoff':
                print(_params)
                takeoff_height = int(
                    #json.loads(json.loads(_params["takeoff_height"]))
                    _params["takeoff_height"]
                    )
                self.send_takeoff_command(takeoff_height)
            elif _cmd == 'goto':
                print(_params)
                self.send_goto_command(_params)
            elif _cmd == 'start_mission':
                self.send_start_mission_command(_params)
            else:
                self.handle_custom_command(_cmd, _params)

    def handle_custom_command(self, _cmd, _params=None):

        print("The custom command = " + str(_cmd) +
              " and the params = " + str(_params))

    def send_mode_change_to_flight_controller(self, mode_id):

        if self._mav_connection is not None:
            self._mav_connection.mav.command_long_send(
                self._mav_connection.target_system,
                self._mav_connection.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                0,
                209,
                mode_id,
                0,
                0,
                0,
                0,
                0
            )
            msg = self._mav_connection.recv_match(
                type='COMMAND_ACK',
                blocking=True,
                timeout=3.0)
            self.vprint("The message is: " + str(msg))
        else:
            self.vprint("There is no mav_connection")
        self._agent_hub.immediate_response_queue.put(True)

    def send_arm_disarm_command(self, arm_disarm, override=False, delay=10):

        self.vprint("I should be setting " + arm_disarm)

        if arm_disarm == 'arm':
            arm_disarm_cmd = 1
        elif arm_disarm == 'disarm':
            arm_disarm_cmd = 0

        if override:
            override_code = 21196
        else:
            override_code = 0
        self._mav_connection.mav.param_set_send(
            self._mav_connection.target_system,
            self._mav_connection.target_component,
            b'DISARM_DELAY', float(delay),
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        self._mav_connection.mav.command_long_send(
            self._mav_connection.target_system,
            self._mav_connection.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            arm_disarm_cmd,
            override_code,
            0,
            0,
            0,
            0,
            0
        )
        # msg = self._mav_connection.recv_match(
        #     type='COMMAND_ACK',
        #     blocking=True,
        #     timeout=1.0)
        # self.vprint(msg)
        # if msg.result == 0:
        #     return True
        # else:
        #     return False

    def send_takeoff_command(self, height=10):
        
        global current_altitude
        self._mav_connection.mav.command_long_send(
            self._mav_connection.target_system,
            self._mav_connection.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            int(height)
        )
        #Update Current Altitude
        current_altitude = height


        # msg = self._mav_connection.recv_match(
        #     type='COMMAND_ACK',
        #     blocking=True)
        # self.vprint(msg)
        # if msg.result == 0:
        #     return True
        # else:
        #     return False
    
    #Writer Chanon Mallanoo
    def send_move_command(self, x=0, y=0, z=0):
        
        self._mav_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, self._mav_connection.target_system, self._mav_connection.target_component, 
                                                                                      mavutil.mavlink.MAV_FRAME_BODY_FRD, int(0b110111111000), 
                                                                                      x, #Forward
                                                                                      y, #Left
                                                                                      z, #Down Altitude is negative in NED
                                                                                      0, 0, 0, #XYZ Velocity
                                                                                      0, 0, 0, #XYZ Acceleration
                                                                                      0, 0)) #Yaw setpoint and Yaw rate setpoint
    
        
    
    def send_new_speed(self, speed=3):
        
        self._mav_connection.mav.command_long_send(self._mav_connection.target_system, self._mav_connection.target_component, 
                                                                                      mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, 
                                                                                      0,
                                                                                      0, #Param 1
                                                                                      speed, #Speed -1:No change, -2 Return to default speed
                                                                                      -1, #Throttle -1:No change, -2 Return to default speed
                                                                                      0,
                                                                                      0,
                                                                                      0,
                                                                                      0)



    #Writer Chanon Mallanoo
    def send_yaw_command(self, angle=45):
        self._mav_connection.mav.command_long_send(
            self._mav_connection.target_system,
            self._mav_connection.target_component,
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,
            0,
            angle,  #1 Angle
            25,     #2 omega deg/sec 
            0,      #3 -1:CCW 0:Shortest Distance 1:CW
            1,      #4 0 for Absolute Angle, 1 for Relative offset
            0,
            0,
            0
        )
    
    


    # def send_goto_command(self, target):
    #     # If target format is a string then follow the JSON form:
    #     #   coming from a python script: target =
    #     #      '{"lat": 39.0179776, "lon": -104.8936, "alt": 2170.0}'
    #     #   other languages that don't distinguish between " and ': target =
    #     #      "{\"lat\": 39.0179776, \"lon\": -104.8936, \"alt\": 2170.0}"

    #     # create target_dict from input target (dict or string)
    #     print("Made it to goto function")
    #     if isinstance(target, dict):
    #         target_dict = target
    #     elif isinstance(target, str):
    #         target_dict = json.loads(target)
    #         if isinstance(target_dict, str):
    #             target_dict = json.loads(target_dict)
    #         print(str(type(target_dict)) + " / " + str(target_dict))

    #     lat = int(target_dict['lat'] * 10 ** 7)
    #     lat_pre = int(39.0179776 * 10 ** 7)
    #     lon = int(target_dict['lon'] * 10 ** 7)
    #     lon_pre = int(-104.8936 * 10 ** 7)

    #     # check if relative or absolute altitude sent
    #     if 'alt' in target:
    #         alt = int(target_dict['alt'])
    #         ref_frame = mavutil.mavlink.MAV_FRAME_GLOBAL_INT
    #     elif 'relative_alt' in target:
    #         alt = int(target_dict['relative_alt'])
    #         ref_frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT

    #     alt_pre = 2180
    #     self.vprint(str(lat) + " : " + str(lat_pre) + " / " +
    #                 str(lon) + " : " + str(lon_pre) + " / " +
    #                 str(alt) + " : " + str(alt_pre))

    #     # for the mask to change params, use:
    #     #   Use Position : 0b110111111000 / 0x0DF8 / 3576 (decimal)
    #     #   Use Velocity : 0b110111000111 / 0x0DC7 / 3527 (decimal)
    #     #   Use Acceleration : 0b110000111000 / 0x0C38 / 3128 (decimal)
    #     #   Use Pos+Vel : 0b110111000000 / 0x0DC0 / 3520 (decimal)
    #     #   Use Pos+Vel+Accel : 0b110000000000 / 0x0C00 / 3072 (decimal)
    #     #   Use Yaw : 0b100111111111 / 0x09FF / 2559 (decimal)
    #     #   Use Yaw Rate : 0b010111111111 / 0x05FF / 1535 (decimal)

    #     self._mav_connection.mav.send(
    #         mavutil.mavlink.MAVLink_set_position_target_global_int_message(
    #             10,
    #             self._mav_connection.target_system,
    #             self._mav_connection.target_component,
    #             ref_frame,   # Frame of Reference
    #             int(0b110111111000),    # Mask to change params
    #             lat,            # latitude * 1e7
    #             lon,            # lon * 1e7
    #             alt,            # alt in meters MSL
    #             0,              # X velocity in m/s (positive is North)
    #             0,              # Y velocity in m/s (positive is East)
    #             0,              # Z velocity in m/s (positive is down)
    #             0,              # X acceleration in m/s/s (positive is North)
    #             0,              # Y acceleration in m/s/s (positive is East)
    #             0,              # Z acceleration in m/s/s (positive is Down)
    #             1.57,           # yaw or heading in radians (0 is forward)
    #             0.5             # yaw rate in rad/s

    #         )
    #     )

    def send_goto_command(self, target):
        # If target format is a string then follow the JSON form:
        #   coming from a python script: target =
        #      '{"lat": 39.0179776, "lon": -104.8936, "alt": 2170.0}'
        #   other languages that don't distinguish between " and ': target =
        #      "{\"lat\": 39.0179776, \"lon\": -104.8936, \"alt\": 2170.0}"

        # create target_dict from input target (dict or string)
        print("Made it to goto function")
        if isinstance(target, dict):
            target_dict = target
        elif isinstance(target, str):
            target_dict = json.loads(target)
            if isinstance(target_dict, str):
                target_dict = json.loads(target_dict)
            print(str(type(target_dict)) + " / " + str(target_dict))

        lat = int(target_dict['lat'] * 10 ** 7)
        lat_pre = int(39.0179776 * 10 ** 7)
        lon = int(target_dict['lon'] * 10 ** 7)
        lon_pre = int(-104.8936 * 10 ** 7)

        # check if relative or absolute altitude sent
        if 'alt' in target:
            alt = int(target_dict['alt'])
            ref_frame = mavutil.mavlink.MAV_FRAME_GLOBAL_INT
        elif 'relative_alt' in target:
            alt = int(target_dict['relative_alt'])
            ref_frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT

        alt_pre = 2180
        self.vprint(str(lat) + " : " + str(lat_pre) + " / " +
                    str(lon) + " : " + str(lon_pre) + " / " +
                    str(alt) + " : " + str(alt_pre))

        # for the mask to change params, use:
        #   Use Position : 0b110111111000 / 0x0DF8 / 3576 (decimal)
        #   Use Velocity : 0b110111000111 / 0x0DC7 / 3527 (decimal)
        #   Use Acceleration : 0b110000111000 / 0x0C38 / 3128 (decimal)
        #   Use Pos+Vel : 0b110111000000 / 0x0DC0 / 3520 (decimal)
        #   Use Pos+Vel+Accel : 0b110000000000 / 0x0C00 / 3072 (decimal)
        #   Use Yaw : 0b100111111111 / 0x09FF / 2559 (decimal)
        #   Use Yaw Rate : 0b010111111111 / 0x05FF / 1535 (decimal)

        self._mav_connection.mav.send(
            mavutil.mavlink.MAVLink_set_position_target_global_int_message(
                10,
                self._mav_connection.target_system,
                self._mav_connection.target_component,
                ref_frame,   # Frame of Reference
                int(0b111111111000),    # Mask to change params
                lat,            # latitude * 1e7
                lon,            # lon * 1e7
                alt,            # alt in meters MSL
                0,              # X velocity in m/s (positive is North)
                0,              # Y velocity in m/s (positive is East)
                0,              # Z velocity in m/s (positive is down)
                0,              # X acceleration in m/s/s (positive is North)
                0,              # Y acceleration in m/s/s (positive is East)
                0,              # Z acceleration in m/s/s (positive is Down)
                1.57,           # yaw or heading in radians (0 is forward)
                0.5             # yaw rate in rad/s

            )
        )


    def send_start_mission_command(self):
        self._mav_connection.mav.command_long_send(
            self._mav_connection.target_system,
            self._mav_connection.target_component,
            mavutil.mavlink.MAV_CMD_MISSION_START,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0
        )


    def send_land_command(self):
        """Sends the land command to the vehicle."""
        self.vprint("Initiating landing")

        if self._mav_connection is not None:
            self._mav_connection.mav.command_long_send(
                self._mav_connection.target_system,
                self._mav_connection.target_component,
                mavutil.mavlink.MAV_CMD_NAV_LAND,  # Command for landing
                0,  # Confirmation
                0, 0, 0, 0,  # Param1-4: Unused
                0,  # Latitude (optional, 0 to use current position)
                0,  # Longitude (optional, 0 to use current position)
                0   # Altitude (optional, 0 for immediate descent)
            )
            msg = self._mav_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3.0)
            self.vprint(f"The message is: {msg}")
        else:
            self.vprint("No MAVLink connection")

        self._agent_hub.immediate_response_queue.put(True)

        
        # msg = self._mav_connection.recv_match(
        #     type='COMMAND_ACK',
        #     blocking=True,
        #     timeout=1.0)
        # self.vprint(msg)
        # if msg.result == 0:
        #     return True
        # else:
        #     return False

        
    # def set_rc_channel_pwm(self, channel_id, pwm=1550):
    #     """Set RC channel pwn value
    #     Arguments:
    #         - channel_id (int): Servo Channel ID
    #         - pwm(int, optional): Channel pwm 0%-100% values are 1000-2000
    #         -- best practice is to set the pwm values between 1100-1900
    #         -- value of 0 means hand it back to handheld controller
    #         -- value of UINT16_MAX (65535) means ignore the channel

    #     """

    #     # check for valid input channel
    #     if channel_id < 1 or channel_id > 18:
    #         print("Channel does not exist")
    #         return

    #     # Mavlink 2 supports up to 18 channels:
    #     # https://mavlink.io/en/messages/common.html#RC_CHANNELS_OVERRIDE
    #     # Initialize all the channels to UINT16_MAX which tells mavlink the channel is unused
    #     rc_channel_values = [65535 for _ in range(18)]
    #     rc_channel_values[channel_id - 1] = pwm
    #     self._mav_connection.mav.rc_channels_override_send(
    #         self._mav_connection.target_system,
    #         self._mav_connection.target_component,
    #         *rc_channel_values
    #     )

