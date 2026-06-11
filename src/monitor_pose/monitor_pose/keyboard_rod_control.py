#!/usr/bin/env python3

import sys
import select
import termios
import tty
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_rod_teleop')
        self.publisher_ = self.create_publisher(Float32MultiArray, '/rod_target', 10)
        
        # Initialized to match the C++ offboard_control defaults
        self.x = 0.0
        self.y = 2.0
        self.z = 2.5
        self.yaw = 0.0
        self.pitch = 0.0
        
        # Step sizes for each key press
        self.step_trans = 0.1  # meters
        self.step_rot = 0.05   # radians
        
    def publish_target(self):
        msg = Float32MultiArray()
        msg.data = [self.x, self.y, self.z, self.yaw, self.pitch]
        self.publisher_.publish(msg)
        self.get_logger().info(
            f"Published: X:{self.x:.2f} Y:{self.y:.2f} Z:{self.z:.2f} Yaw:{self.yaw:.2f} Pitch:{self.pitch:.2f}"
        )

# Helper function to read raw terminal input instantly
def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
        # Handle multi-byte escape sequences for arrow keys
        if key == '\x1b':
            key += sys.stdin.read(2)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main(args=None):
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init(args=args)
    node = KeyboardTeleop()
    
    instructions = """
    Rod Target Keyboard Teleop Started!
    -----------------------------------
    Translation Controls:
        8 (+Y)    
    4 (-X)  6 (+X) 
        2 (-Y)    
    
    Up Arrow   : +Z (Up)
    Down Arrow : -Z (Down)
    
    Rotation Controls:
    7 (-Pitch) / 9 (+Pitch)
    1 (-Yaw)   / 3 (+Yaw)
    
    CTRL-C to quit
    """
    print(instructions)
    
    try:
        while rclpy.ok():
            key = get_key(settings)
            if key != '':
                # Translation mapping
                if key == '4': node.x -= node.step_trans
                elif key == '6': node.x += node.step_trans
                elif key == '8': node.y += node.step_trans
                elif key == '2': node.y -= node.step_trans
                elif key == '\x1b[A': node.z += node.step_trans # Up Arrow
                elif key == '\x1b[B': node.z -= node.step_trans # Down Arrow
                
                # Rotation mapping
                elif key == '7': node.pitch -= node.step_rot
                elif key == '9': node.pitch += node.step_rot
                elif key == '1': node.yaw -= node.step_rot
                elif key == '3': node.yaw += node.step_rot
                
                # Quit condition
                elif key == '\x03': # CTRL-C
                    break 
                
                node.publish_target()
                
    except Exception as e:
        print(e)
    finally:
        # Restore terminal settings upon exit
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()