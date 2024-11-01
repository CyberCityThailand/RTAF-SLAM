import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import math
from .agent_core.classes.agent_core import AgentCore
from .agent_core.classes.custom_agent_command_mgr import CustomAgentCommandManager
import time
from geometry_msgs.msg import Twist
import threading
import sys


class Controller(AgentCore):
    
    def __init__(self, x=0, y=0, z=0):
        AgentCore.__init__(self, loop_delay=0, log_file=False, verbose=False)
        # Simulated placeholders for agent_hub and mav_connection (replace these with actual objects in your code)
        config = self.config
        agent_hub = self.agent_hub
        mav_connection = self.agent_hub.mav_mgr.mav_connection

        # Initialize the command manager
        self.command_manager = CustomAgentCommandManager(config, agent_hub, mav_connection, verbose=True)

    
    def arm(self, override_safety=True, delay_before_disarm=0):
        """Arm the vehicle with optional safety override and delay before disarming."""
        self.command_manager.arm(override_safety=override_safety, delay_before_disarm=delay_before_disarm)

    # Function to disarm the vehicle
    def disarm(self):
        """Disarm the vehicle."""
        self.command_manager.disarm()

    # Function to set the flight mode
    def set_mode_guided(self):
        self.command_manager.set_mode('GUIDED')

    # Function to take off
    def takeoff(self, altitude=5):
        """Command the vehicle to take off to a specified altitude."""
        self.command_manager.takeoff(altitude)

    # Function to move to a target position (local NED frame)
    # X is Forward Y is Left Z is Down (So provide negative value to go up)
    def move(self, x, y, z=0):
        self.command_manager.move(x, y, z)

    def yaw(self, angle=45):
        self.command_manager.yaw(angle)

    #Function to land
    def land(self):
        """Command the vehicle to land."""
        self.command_manager.set_mode('LAND')

class Operation(Node):

    def __init__(self):
        super().__init__('mallanoo_agent')
        # Initialize ROS Inputs, queue max = 10
        self.goal_heading_sub = self.create_subscription(
            Twist, "cmd_vel", self.callback_cmd, 10
        )
        self.controller = Controller()
        self.sem = threading.Semaphore()

        print("---Command Precheck---")
        isPrechecked = input("Do you want to do precheck?")
        if (isPrechecked != '0'):
            self.sem.acquire()
            input("Set mode GUIDED")
            self.controller.set_mode_guided()

            if (self.controller.agent_status_obj.flight_mode != 'GUIDED'):
                input("Cannot change to Mode Guided")

            input("Press any keys to takeoff")
            self.controller.takeoff(3)
            time.sleep(5)
            self.sem.release()

            self.sem.acquire()
            input("Ready to move ?")
            self.controller.move(x=3,y=3,z=0)
            input("Mode Guided?")
            self.controller.set_mode_guided()
            time.sleep(3)
            self.sem.release()

            self.sem.acquire()
            input("Ready to yaw ?")
            self.controller.yaw()
            time.sleep(5)
            self.sem.release()

            self.sem.acquire()
            input("Ready to land ?")
            self.controller.land()
            time.sleep(5)
            self.sem.release()
        else:
            print("Mission started")
            print("Taking movement from cmd_vel")
        



    def callback_cmd(self, twist: Twist):
        
        print("---Ready to Move---")
        forward = twist.linear.x
        left = twist.linear.y
        turn = twist.angular.z
        print("Forward: " + str(forward) + " Left: " + str(left) + " Turn: " + str(turn))
        print('\n')

        print("---Angular Movement---")
        self.sem.acquire()
        self.controller.yaw(turn)
        time.sleep(5)
        self.sem.release()

        print('---Linear Movement---')
        self.sem.acquire()
        self.controller.move(x=forward, y=left, z=0)
        time.sleep(5)
        self.sem.release()








        


def main(args=None):
    rclpy.init(args=args)
    mallanoo_agent = Operation()
    rclpy.spin(mallanoo_agent)
    print("[Controller] main boot sequence finished")

    # Run the node until a shutdown condition occurs
    try:
        rclpy.spin(mallanoo_agent)
    except KeyboardInterrupt:
        pass
    
    finally:
        #mallanoo_agent.cmd_vel_pub.publish(Twist())
        rclpy.try_shutdown()

    mallanoo_agent.destroy_node()
    print("[controller.py] Node shutdown")
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
