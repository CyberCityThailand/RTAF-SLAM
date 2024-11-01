import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import math
import time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import threading
import sys



class Obeject_Avoidance(Node):

    def __init__(self):
        super().__init__('avoidance_node')
        # Initialize ROS Inputs, queue max = 10
        self.collision_sub = self.create_subscription(
            LaserScan, "scan", self.scan_cb, 10
        )

        # self.odom_sub = self.create_subscription(
        #     Odomertry, "odom_rf2o", self.scan_cb, 10
        # )

        self.out = Twist()

        #Initialize Publisher

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pub_timer = self.create_timer(1 / 20, self.publish)

        print("Test")

    


    def scan_cb(self, laser_2D_scan):
        current_2D_scan = laser_2D_scan.ranges
        current_heading = 0
        avoidance_vector_x = 0
        avoidance_vector_y = 0
        avoid = False
        for i in range(len(current_2D_scan)):
            d0 = 3
            k = 0.5
            if (current_2D_scan[i] < d0 and current_2D_scan[i] > .35):
                avoid = True
                X = math.cos(laser_2D_scan.angle_increment*i)
                Y = math.sin(laser_2D_scan.angle_increment*i)
                U = -0.5*k*pow( ((1/current_2D_scan[i]) - (1/d0)),  2)

                avoidance_vector_x = avoidance_vector_x + (X * U)
                avoidance_vector_y = avoidance_vector_y + (Y * U)
        
        deg2Rad = (3.14159265/180)

        avoidance_vector_x = avoidance_vector_x*math.cos((current_heading)*deg2Rad) - avoidance_vector_y * math.sin((current_heading)*deg2Rad)
        avoidance_vector_y = avoidance_vector_y*math.sin((current_heading)*deg2Rad) + avoidance_vector_y * math.cos((current_heading)*deg2Rad)


        if(avoid):
            
            z = math.sqrt(pow(avoidance_vector_x,2) + pow(avoidance_vector_y,2))
            if(z > 3):
                avoidance_vector_x = 3*(avoidance_vector_x/z) 
                avoidance_vector_y = 3*(avoidance_vector_y/z)
            
            self.out.linear.x = avoidance_vector_x
            self.out.linear.y = avoidance_vector_y



    def publish(self):
        self.cmd_vel_pub.publish(self.out)















def main(args=None):
    rclpy.init(args=args)
    obj = Obeject_Avoidance()
    rclpy.spin(obj)
    print("Obstacle Avoidance Ready")

    # Run the node until a shutdown condition occurs
    try:
        rclpy.spin(obj)
    except KeyboardInterrupt:
        pass
    
    finally:
        obj.cmd_vel_pub.publish(Twist())
        rclpy.try_shutdown()

    obj.destroy_node()
    print("[controller.py] Node shutdown")
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()