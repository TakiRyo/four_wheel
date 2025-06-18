#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, termios, tty

# ======= Configuration =======
MAX_LIN_VEL = 0.2
MAX_ANG_VEL = 1.5
LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1
# =============================

def constrain(val, min_val, max_val):
    return max(min(val, max_val), min_val)

def make_simple_profile(output, input, slop):
    if input > output:
        output = min(input, output + slop)
    elif input < output:
        output = max(input, output - slop)
    return output

def check_linear_limit_velocity(vel):
    return constrain(vel, -MAX_LIN_VEL, MAX_LIN_VEL)

def check_angular_limit_velocity(vel):
    return constrain(vel, -MAX_ANG_VEL, MAX_ANG_VEL)

class TeleopNode(Node):
    def __init__(self):
        super().__init__('four_wheel_teleop')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.settings = termios.tcgetattr(sys.stdin)
        self.print_instructions()

    def print_instructions(self):
        print("""
Four-Wheel Robot Teleop
---------------------------
Move your robot using:
    w
  a s d       (q to quit)
    x

w/x: Forward/Backward
a/d: Turn Left/Right
s: Stop
---------------------------
""")

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        twist = Twist()
        target_linear = 0.0
        target_angular = 0.0
        control_linear = 0.0
        control_angular = 0.0

        try:
            while rclpy.ok():
                key = self.get_key()

                if key == 'w':
                    target_linear += LIN_VEL_STEP_SIZE
                    target_linear = check_linear_limit_velocity(target_linear)
                elif key == 'x':
                    target_linear -= LIN_VEL_STEP_SIZE
                    target_linear = check_linear_limit_velocity(target_linear)
                elif key == 'a':
                    target_angular += ANG_VEL_STEP_SIZE
                    target_angular = check_angular_limit_velocity(target_angular)
                elif key == 'd':
                    target_angular -= ANG_VEL_STEP_SIZE
                    target_angular = check_angular_limit_velocity(target_angular)
                elif key == 's':
                    target_linear = 0.0
                    target_angular = 0.0
                elif key == 'q' or key == '\x03':
                    break
                else:
                    continue

                control_linear = make_simple_profile(control_linear, target_linear, LIN_VEL_STEP_SIZE / 2.0)
                control_angular = make_simple_profile(control_angular, target_angular, ANG_VEL_STEP_SIZE / 2.0)

                twist.linear.x = control_linear
                twist.angular.z = control_angular

                self.publisher.publish(twist)
                print(f"[linear: {control_linear:.2f} | angular: {control_angular:.2f}]")

        except Exception as e:
            print(e)

        finally:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher.publish(twist)
            print("👋 Shutdown complete.")

def main():
    rclpy.init()
    node = TeleopNode()
    node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
