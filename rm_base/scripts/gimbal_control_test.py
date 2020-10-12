#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from rm_msgs.msg import GimbalControl

                        
def main(args=None):
    rclpy.init(args=args)
    node = Node("gimbal_control_test_node")
    pub = node.create_publisher(GimbalControl, 'gimbal_control', 10)
    info=GimbalControl()
    while True:
        print("\n[absolute mode],please intput tagret angle (float) ")
        try:
            info.pitch = float(input("pitch_angle: "))
            info.yaw = float(input("yaw_angle: "))
        except:
            break
        pub.publish(info)
        print("send--------------------------------\n")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
