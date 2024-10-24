import os
import json
from pymavlink import mavutil
from .agent_command_manager import AgentCommandManager

os.environ['MAVLINK20'] = '1'

class CustomAgentCommandManager(AgentCommandManager):
    
    def __init__(self, config, agent_hub, mav_connection, verbose=False):
        super().__init__(config, agent_hub, mav_connection, verbose)

    # Function to arm the vehicle
    def arm(self, override_safety=False, delay_before_disarm=0):
        """Arm the vehicle with optional safety override and delay before disarming."""
        self.send_arm_disarm_command('arm', override=override_safety, delay=delay_before_disarm)

    # Function to disarm the vehicle
    def disarm(self):
        """Disarm the vehicle."""
        self.send_arm_disarm_command('disarm')

    # Function to set the flight mode
    def set_mode(self, mode):
        """Set the flight mode using a simple mode string."""
        try:
            mode_id = self._mav_connection.mode_mapping()[mode.upper()]
            self.send_mode_change_to_flight_controller(mode_id)
        except KeyError:
            print(f"Mode {mode} is not recognized.")

    # Function to take off
    def takeoff(self, altitude=10):
        """Command the vehicle to take off to a specified altitude."""
        self.send_takeoff_command(altitude)

    #Function to land
    def landing(self):
        """Command the vehicle to land."""
        self.send_land_command()

    # Function to move to a target position (local NED frame)
    def move(self, x, y, z=None):
        """Move the vehicle to a relative position (x, y, z)."""
        if z is None:
            z = self.agent_status_obj.current_altitude  # Use current altitude if not provided
        self.send_move_command(x, y, z)

    # Function to change speed
    def change_speed(self, speed):
        """Change the speed of the vehicle."""
        self.send_new_speed(speed)

    # Function to rotate to a specific yaw angle
    def yaw(self, angle, clockwise=True):
        """Rotate the vehicle to a specified yaw angle. Clockwise by default."""
        if clockwise:
            self.send_yaw_command_CW(angle)
        else:
            self.send_yaw_command_CCW(angle)

    # Function to move to a specific GPS coordinate
    def goto(self, latitude, longitude, altitude=None):
        """Go to a specific GPS coordinate (latitude, longitude, altitude)."""
        target = {
            'lat': latitude,
            'lon': longitude,
            'alt': altitude or self.agent_status_obj.current_altitude  # Use current altitude if not specified
        }
        self.send_goto_command(target)

    # Function to start a mission
    def start_mission(self):
        """Start the mission."""
        self.send_start_mission_command()

    
