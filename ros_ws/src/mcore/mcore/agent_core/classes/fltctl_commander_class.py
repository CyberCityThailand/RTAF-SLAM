""" Defines the Flight Controller Commander class"""

import os
import json

os.environ["MAVLINK20"] = "1"
from pymavlink import mavutil

# import .commands.transmit_mission as tm
from .commands.transmit_mission import *

# import transmit_mission as tm


class FltCtlCommander:
    def __init__(self, mav_connection, agent_direct_request_sock=None, verbose=False):
        self._verbose = verbose
        self._mav_connection = mav_connection
        self._agent_direct_request_sock = agent_direct_request_sock

    # reads message sent from ui_client/worker and translates it to mavlink message
    def process_command(self, multipart_message):
        """Read message, send to flight controller and update status"""
        command = multipart_message[0].decode()
        # print(command)

        if command == "arm":
            self.send_arm_disarm_command(1, False)
        elif command == "disarm":
            self.send_arm_disarm_command(0)
        elif command == "takeoff":
            self.send_takeoff_command(10)
        elif command == "start-mission":
            self.send_start_mission_command()
        elif command == "rtl":
            mode_id = self._mav_connection.mode_mapping()["RTL"]
            self.send_mode_change_to_flt_ctlr(mode_id)
        elif command == "stabilize":
            mode_id = self._mav_connection.mode_mapping()["STABILIZE"]
            self.send_mode_change_to_flt_ctlr(mode_id)
        elif command == "auto":
            mode_id = self._mav_connection.mode_mapping()["AUTO"]
            self.send_mode_change_to_flt_ctlr(mode_id)
        elif command == "guided":
            mode_id = self._mav_connection.mode_mapping()["GUIDED"]
            self.send_mode_change_to_flt_ctlr(mode_id)
        elif command == "fly_to_target":
            # target = self._mav_connection
            self.send_guided_target_to_flt_ctlr(multipart_message[1].decode())

        elif command == "upload_route":
            transmit_missions.write_mission_waypoints_to_agent(
                multipart_message[1].decode(), self._mav_connection
            )
        elif command == "read_route":
            route_dict = transmit_missions.read_mission_waypoints_from_agent(
                self._mav_connection
            )
            if self._agent_direct_request_sock:
                self._agent_direct_request_sock.send_json(route_dict)
            print(route_dict)

    def send_mode_change_to_flt_ctlr(self, mode_id):
        if self._verbose:
            print("I should be setting " + str(mode_id))
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
            0,
        )
        msg = self._mav_connection.recv_match(
            type="COMMAND_ACK", blocking=True, timeout=1.0
        )
        try:
            if msg.result == 0:
                return True
            else:
                return False
        except:
            return True

    def send_arm_disarm_command(self, arm_disarm, override=False):
        if override:
            override_code = 21196
        else:
            override_code = 0
        self._mav_connection.mav.command_long_send(
            self._mav_connection.target_system,
            self._mav_connection.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            arm_disarm,
            override_code,
            0,
            0,
            0,
            0,
            0,
        )
        msg = self._mav_connection.recv_match(
            type="COMMAND_ACK", blocking=True, timeout=1.0
        )
        if msg.result == 0:
            return True
        else:
            return False

    def send_takeoff_command(self, altitude=10):
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
            altitude,
        )
        msg = self._mav_connection.recv_match(
            type="COMMAND_ACK", blocking=True, timeout=1.0
        )
        if msg.result == 0:
            return True
        else:
            return False

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
            0,
        )
        msg = self._mav_connection.recv_match(
            type="COMMAND_ACK", blocking=True, timeout=1.0
        )
        if msg.result == 0:
            return True
        else:
            return False

    def send_guided_target_to_flt_ctlr(self, target):
        # If target format is a string then follow the JSON form:
        #   coming from a python script: target = '{"lat": 39.0179776, "lon": -104.8936, "alt": 2170.0}'
        #   other languages that don't distinguish between " and ': target = "{\"lat\": 39.0179776, \"lon\": -104.8936, \"alt\": 2170.0}"

        # create target_dict from input target (dict or string)
        if isinstance(target, dict):
            target_dict = target
        elif isinstance(target, str):
            target_dict = json.loads(target)
            if isinstance(target_dict, str):
                target_dict = json.loads(target_dict)
            print(str(type(target_dict)) + " / " + str(target_dict))

        lat = int(target_dict["lat"] * 10**7)
        lat_pre = int(39.0179776 * 10**7)
        lon = int(target_dict["lon"] * 10**7)
        lon_pre = int(-104.8936 * 10**7)
        alt = int(target_dict["alt"])
        alt_pre = 2180
        print(
            str(lat)
            + " : "
            + str(lat_pre)
            + " / "
            + str(lon)
            + " : "
            + str(lon_pre)
            + " / "
            + str(alt)
            + " : "
            + str(alt_pre)
        )

        self._mav_connection.mav.send(
            mavutil.mavlink.MAVLink_set_position_target_global_int_message(
                10,
                self._mav_connection.target_system,
                self._mav_connection.target_component,
                mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                int(0b110111111000),
                lat,
                lon,
                alt,
                0,
                0,
                0,
                0,
                0,
                0,
                1.57,
                0.5,
            )
        )

    def send_move_at_velocity(self, speed=(2.0, 2.0, 2.0)):
        # https://ardupilot.org/dev/docs/mavlink-rover-commands.html

        # Send the SET_POSITION_TARGET_LOCAL_NED command
        self._mav_connection.mav.set_position_target_local_ned_send(
            0,  # system time in milliseconds
            self._mav_connection.target_system,
            self._mav_connection.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            3559,  # mask specifying which dimensions should be ignored
            0,
            0,
            0,  # x, y, z positions (not used)
            speed[0],
            speed[1],
            speed[2],  # x, y, z velocity
            0,
            0,
            0,  # x, y, z acceleration (not used)
            0,
            0,
        )  # yaw, yaw_rate (not used)

    def send_sequenced_commands(self, command_list):
        # for command in command_list:
        #   send command
        #   receive ack and verify command was accepted
        pass

    def set_rc_channel_pwm(self, channel_id, pwm=1550):
        """Set RC channel pwn value
        Arguments:
            - channel_id (int): Servo Channel ID
            - pwm(int, optional): Channel pwm 0%-100% values are 1000-2000
            -- best practice is to set the pwm values between 1100-1900
            -- value of 0 means hand it back to handheld controller
            -- value of UINT16_MAX (65535) means ignore the channel

        """

        # check for valid input channel
        if channel_id < 1 or channel_id > 18:
            print("Channel does not exist")
            return

        # Mavlink 2 supports up to 18 channels:
        # https://mavlink.io/en/messages/common.html#RC_CHANNELS_OVERRIDE
        # Initialize all the channels to UINT16_MAX which tells mavlink the channel is unused
        rc_channel_values = [65535 for _ in range(18)]
        rc_channel_values[channel_id - 1] = pwm
        self._mav_connection.mav.rc_channels_override_send(
            self._mav_connection.target_system,
            self._mav_connection.target_component,
            *rc_channel_values
        )
