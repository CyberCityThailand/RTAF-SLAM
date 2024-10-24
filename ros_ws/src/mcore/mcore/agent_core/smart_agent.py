from pymavlink import mavutil
import time

the_connection = mavutil.mavlink_connection('udpin:localhost:14551')

the_connection.wait_heartbeat()
print("Heartbeat from system")
'''
the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, the_connection.target_system, the_connection.target_component, 
                                                                                      mavutil.mavlink.MAV_FRAME_LOCAL_NED, 3576, 
                                                                                      30,
                                                                                      0,
                                                                                      -10,
                                                                                      0, 0, 0,
                                                                                      0, 0, 0,
                                                                                      0, 0))
'''
def send_arm_disarm_command(arm_disarm, override=False, delay=10):

    
    if arm_disarm == 'arm':
        arm_disarm_cmd = 1
    elif arm_disarm == 'disarm':
        arm_disarm_cmd = 0

    if override:
        override_code = 21196
    else:
        override_code = 0
    the_connection.mav.param_set_send(
        the_connection.target_system,
        the_connection.target_component,
        b'DISARM_DELAY', float(delay),
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    the_connection.mav.command_long_send(
        the_connection.target_system,
        the_connection.target_component,
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
    # msg = the_connection.recv_match(
    #     type='COMMAND_ACK',
    #     blocking=True,
    #     timeout=1.0)
    # self.vprint(msg)
    # if msg.result == 0:
    #     return True
    # else:
    #     return False

def send_takeoff_command(height=10):
    
    the_connection.mav.command_long_send(
        the_connection.target_system,
        the_connection.target_component,
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

def send_goto_command(latitude, longitude, altitude):
    cmd = mavutil.mavlink.MAVLink_command_long_message(
        the_connection.target_system,  # Target system ID
        the_connection.target_component,  # Target component ID
        mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM,  # Command ID for loiter unlimited
        0,  # Confirmation (not used)
        0,  # Param1 (not used)
        0,  # Param2 (not used)
        0,  # Param3 (not used)
        0,  # Param4 (not used)
        latitude,  # Param5: Latitude of the target location
        longitude,  # Param6: Longitude of the target location
        altitude,  # Param7: Altitude in meters above ground
    )

    the_connection.mav.send(cmd)
print("Arm")
send_arm_disarm_command("arm")
time.sleep(2)

print("Taking off")
send_takeoff_command(20)
time.sleep(10)

print("Goto")
send_goto_command(20, -104, 20000)
time.sleep(5)

