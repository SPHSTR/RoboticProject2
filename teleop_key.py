import os
import select
import sys

import rclpy
from rclpy.qos import QoSProfile
from rclpy.node import Node

from geometry_msgs.msg import Vector3 
# from std_msgs.msg import String
# from std_msgs.msg import Int32

if os.name == 'nt':
    import msvcrt
else:
    import termios
    import tty


min_linx_vel = 0.163
min_liny_vel = 0.163
min_ang_vel = 1.63
max_linx_vel = 0.49
max_liny_vel = 0.49
max_ang_vel = 4.9  #rad/s

msgs = """ 
________________________________________________________
        move/rotate     +/- vel
        q  w  e           r
         a s d             f
________________________________________________________
"""

def get_key(settings):
    if os.name == 'nt':
        return msvcrt.getch().decode('utf-8')
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key




def print_vel(linx_vel,liny_vel,ang_vel):
    print(f"current X velocity: {linx_vel}  current Y velocity: {liny_vel}  current ang velocity: {ang_vel}")

def check_linx_vel(linx_vel):
    return max(min_linx_vel, min(linx_vel, max_linx_vel))

def check_liny_vel(liny_vel):
    return max(min_liny_vel, min(liny_vel, max_liny_vel))

def check_ang_vel(ang_vel):
    return max(min_ang_vel, min(ang_vel, max_ang_vel))


def main(arg=None):
    settings = None
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rclpy.init()
    qos = QoSProfile(depth=10)
    node = rclpy.create_node('teleop_keyboard')
    pub = node.create_publisher(Vector3, 'cnt_vel', qos)

    status = 0

    linx_vel = min_linx_vel
    liny_vel = min_liny_vel
    ang_vel = min_ang_vel

    linx_dir = 0
    liny_dir = 0
    ang_dir = 0

    msg = Vector3()

    print(msgs)

    while (1):
        key = get_key(settings)
        if key == "r" :
            linx_vel = check_linx_vel(linx_vel + 0.163)
            liny_vel = check_liny_vel(liny_vel + 0.163)
            ang_vel = check_ang_vel(ang_vel + 1.63)
            print_vel(linx_vel,liny_vel,ang_vel)
            status+=1
        elif key == "f" :
            linx_vel = check_linx_vel(linx_vel - 0.163)
            liny_vel = check_liny_vel(liny_vel - 0.163)
            ang_vel = check_ang_vel(ang_vel - 1.63)
            print_vel(linx_vel,liny_vel,ang_vel)
            status+=1
        elif key == "w":
            linx_dir = linx_vel
        elif key == "s":
            linx_dir = -linx_vel
        elif key == "d":
            liny_dir = liny_vel
        elif key == "a":
            liny_dir = -liny_vel
        elif key == "e":
            ang_dir = ang_vel
        elif key == "q":
            ang_dir = -ang_vel
        elif not key:
            linx_dir = 0.0
            liny_dir = 0.0
            ang_dir = 0.0
        elif (key == '\x03'):
            break

        msg.x = linx_dir
        msg.y = liny_dir
        msg.z = ang_dir

        if key in ["w", "a", "s", "d", "q", "e"]:
            print(f"Publishing: x={msg.x}, y={msg.y}, z={msg.z}")
            status+=1

        if status == 10:
            print(msgs)
            status = 0

        pub.publish(msg)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()