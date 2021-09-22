#!/usr/bin/python3
import sys
if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty

import rclpy
from rmoss_interfaces.msg import ChassisCmd

msg = """
This node takes keypresses from the keyboard and publishes them
as Chassis messages.
---------------------------
Moving around:
        w    
   a    s    d
turn : '[' for left  ']' for right
stop : space key
---------------------------
CTRL-C to quit
"""

def getKey(settings):
    if sys.platform == 'win32':
        # getwch() returns a string on Windows
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        # sys.stdin.read() returns a string on Linux
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)


def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def getChassisContolMsg(x,y,w):
    control_info = ChassisCmd()
    control_info.twist.linear.x = x
    control_info.twist.linear.y = y
    control_info.twist.linear.z = 0.0
    control_info.twist.angular.x = 0.0
    control_info.twist.angular.y = 0.0
    control_info.twist.angular.z = w
    return control_info

def main():
    settings = saveTerminalSettings()
    rclpy.init()
    node = rclpy.create_node('control_chassis_test')
    #get params
    node.declare_parameter('v',1.0)
    node.declare_parameter('w',1.0)
    v=node.get_parameter('v').value
    w=node.get_parameter('w').value
    pub = node.create_publisher(ChassisCmd, 'cmd_chassis', 10)
    print("node params v:%f,w:%f"%(v,w))
    print(msg)
    vel_x=vel_y=vel_w=0.0
    while True:
        key=getKey(settings)
        if key == 'w':
            vel_x=1.0 * v
        elif key == 's':
            vel_x=-1.0 * v
        elif key == 'a':
            vel_y=1.0 * v
        elif key == 'd':
            vel_y=-1.0 * v
        elif key == '[':
            vel_w=1.0 * w
        elif key == ']':
            vel_w=-1.0 * w
        elif key == ' ':
            vel_x=vel_y=vel_w=0.0
        elif key == '\x03':
            break
        info=getChassisContolMsg(vel_x,vel_y,vel_w)
        pub.publish(info)

if __name__ == '__main__':
    main()