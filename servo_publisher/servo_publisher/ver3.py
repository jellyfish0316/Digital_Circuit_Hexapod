#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import sys
import termios
import tty
import select
import math
import time

# Constants
NUM_SERVOS = 12
# Servo mapping assumption:
# Legs 1-3: Right side (Front to Back) -> IDs 1,2; 3,4; 5,6
# Legs 4-6: Left side (Front to Back) -> IDs 7,8; 9,10; 11,12
# Odd IDs: Horizontal (Coxa), Even IDs: Vertical (Femur)

# Neutral positions
NEUTRAL_COXA = 120.0
NEUTRAL_FEMUR = 40.0

# Gait parameters
LIFT_HEIGHT = 30.0  # Degrees to lift leg
SWING_ANGLE = 10.0  # Degrees to swing leg forward/backward
FB_SWING_ANGLE = 30.0 # Degrees front and back leg swings
STEP_DELAY = 0.5    # Seconds between gait steps (slower for safety)

# Example: Map Leg 0 to Servos 11,12; Leg 1 to Servos 9,10...
LEG_SERVO_MAP = [
    (4, 11), # Leg 0: Coxa Index 10, Femur Index 11 (Servo 11, 12)
    (3, 9),   # Leg 1: Coxa Index 8, Femur Index 9
    (8, 2),
    (5, 10),
    (6, 12),
    (1, 7)
]

class HexapodTeleop(Node):
    def __init__(self):
        super().__init__('hexapod_teleop')
        self.pub = self.create_publisher(JointTrajectory, '/servo_trajectory', 10)
        self.settings = termios.tcgetattr(sys.stdin)
        self.get_logger().info('Hexapod Teleop Started')
        self.get_logger().info('Use WASD to move, Space to stop, Q to quit')
        
        # State
        # State
        self.reset_to_neutral()
        self.gait_phase = 0
        self.current_cmd = 'stop' # 'forward', 'backward', 'left', 'right', 'stop'
        
        # Timer for gait loop
        self.timer = self.create_timer(STEP_DELAY, self.gait_loop)

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def gait_loop(self):
        key = self.getKey()
        if key == 'w':
            self.current_cmd = 'forward'
        elif key == 's':
            self.current_cmd = 'backward'
        elif key == 'a':
            self.current_cmd = 'left'
        elif key == 'd':
            self.current_cmd = 'right'
        elif key == ' ':
            self.current_cmd = 'stop'
        elif key == 'q':
            self.get_logger().info('Quitting...')
            raise KeyboardInterrupt

        if self.current_cmd == 'stop':
            self.reset_to_neutral()
        else:
            self.step_gait()
            
        self.publish_joints()

    def reset_to_neutral(self):
        self.positions = [0.0] * NUM_SERVOS
        for leg_idx in range(6):
            coxa_id, femur_id = LEG_SERVO_MAP[leg_idx]
            self.positions[coxa_id - 1] = NEUTRAL_COXA
            self.positions[femur_id - 1] = NEUTRAL_FEMUR
        self.gait_phase = 0

    def step_gait(self):

        if self.current_cmd == 'forward':
            phase = self.gait_phase % 4
            if phase == 0:
                for leg_idx in range(6):
                    coxa_id, femur_id = LEG_SERVO_MAP[leg_idx]
                    if leg_idx == 0:
                        self.positions[coxa_id - 1] = NEUTRAL_COXA
                        self.positions[femur_id - 1] = NEUTRAL_FEMUR
                    elif leg_idx == 1:
                        self.positions[coxa_id - 1] = NEUTRAL_COXA - SWING_ANGLE
                        self.positions[femur_id - 1] = NEUTRAL_FEMUR + LIFT_HEIGHT
                    elif leg_idx == 2:
                        self.positions[coxa_id - 1] = NEUTRAL_COXA
                        self.positions[femur_id - 1] = NEUTRAL_FEMUR 
                    elif leg_idx == 3:
                        self.positions[coxa_id - 1] = NEUTRAL_COXA + FB_SWING_ANGLE
                        self.positions[femur_id - 1] = NEUTRAL_FEMUR + LIFT_HEIGHT
                    elif leg_idx == 4:
                        self.positions[coxa_id - 1] = NEUTRAL_COXA
                        self.positions[femur_id - 1] = NEUTRAL_FEMUR 
                    elif leg_idx == 5:
                        self.positions[coxa_id - 1] = NEUTRAL_COXA + FB_SWING_ANGLE
                        self.positions[femur_id - 1] = NEUTRAL_FEMUR + LIFT_HEIGHT
            elif phase == 1:
                for leg_idx in range(6):
                    coxa_id, femur_id = LEG_SERVO_MAP[leg_idx]
                    if leg_idx == 0:
                        self.positions[coxa_id - 1] = NEUTRAL_COXA
                        self.positions[femur_id - 1] = NEUTRAL_FEMUR + LIFT_HEIGHT
                    elif leg_idx == 1:
                        self.positions[coxa_id - 1] = NEUTRAL_COXA - SWING_ANGLE
                        self.positions[femur_id - 1] = NEUTRAL_FEMUR
                    elif leg_idx == 2:
                        self.positions[coxa_id - 1] = NEUTRAL_COXA 
                        self.positions[femur_id - 1] = NEUTRAL_FEMUR + LIFT_HEIGHT
                    elif leg_idx == 3:
                        self.positions[coxa_id - 1] = NEUTRAL_COXA + FB_SWING_ANGLE
                        self.positions[femur_id - 1] = NEUTRAL_FEMUR
                    elif leg_idx == 4:
                        self.positions[coxa_id - 1] = NEUTRAL_COXA 
                        self.positions[femur_id - 1] = NEUTRAL_FEMUR + LIFT_HEIGHT
                    elif leg_idx == 5:
                        self.positions[coxa_id - 1] = NEUTRAL_COXA + FB_SWING_ANGLE
                        self.positions[femur_id - 1] = NEUTRAL_FEMUR
            elif phase == 2:
                for leg_idx in range(6):
                    coxa_id, femur_id = LEG_SERVO_MAP[leg_idx]
                    if leg_idx == 0:
                        self.positions[coxa_id - 1] = NEUTRAL_COXA - FB_SWING_ANGLE
                        self.positions[femur_id - 1] = NEUTRAL_FEMUR + LIFT_HEIGHT
                    elif leg_idx == 1:
                        self.positions[coxa_id - 1] = NEUTRAL_COXA
                        self.positions[femur_id - 1] = NEUTRAL_FEMUR
                    elif leg_idx == 2:
                        self.positions[coxa_id - 1] = NEUTRAL_COXA - FB_SWING_ANGLE
                        self.positions[femur_id - 1] = NEUTRAL_FEMUR + LIFT_HEIGHT
                    elif leg_idx == 3:
                        self.positions[coxa_id - 1] = NEUTRAL_COXA
                        self.positions[femur_id - 1] = NEUTRAL_FEMUR
                    elif leg_idx == 4:
                        self.positions[coxa_id - 1] = NEUTRAL_COXA + SWING_ANGLE
                        self.positions[femur_id - 1] = NEUTRAL_FEMUR + LIFT_HEIGHT
                    elif leg_idx == 5:
                        self.positions[coxa_id - 1] = NEUTRAL_COXA
                        self.positions[femur_id - 1] = NEUTRAL_FEMUR
            elif phase == 3:
                for leg_idx in range(6):
                    coxa_id, femur_id = LEG_SERVO_MAP[leg_idx]
                    if leg_idx == 0:
                        self.positions[coxa_id - 1] = NEUTRAL_COXA - FB_SWING_ANGLE
                        self.positions[femur_id - 1] = NEUTRAL_FEMUR
                    elif leg_idx == 1:
                        self.positions[coxa_id - 1] = NEUTRAL_COXA
                        self.positions[femur_id - 1] = NEUTRAL_FEMUR + LIFT_HEIGHT
                    elif leg_idx == 2:
                        self.positions[coxa_id - 1] = NEUTRAL_COXA - FB_SWING_ANGLE
                        self.positions[femur_id - 1] = NEUTRAL_FEMUR
                    elif leg_idx == 3:
                        self.positions[coxa_id - 1] = NEUTRAL_COXA
                        self.positions[femur_id - 1] = NEUTRAL_FEMUR + LIFT_HEIGHT
                    elif leg_idx == 4:
                        self.positions[coxa_id - 1] = NEUTRAL_COXA + SWING_ANGLE
                        self.positions[femur_id - 1] = NEUTRAL_FEMUR
                    elif leg_idx == 5:
                        self.positions[coxa_id - 1] = NEUTRAL_COXA
                        self.positions[femur_id - 1] = NEUTRAL_FEMUR + LIFT_HEIGHT

        if self.current_cmd == 'left':
            phase = self.gait_phase % 4
            if phase == 0:
                for leg_idx in range(6):
                    coxa_id, femur_id = LEG_SERVO_MAP[leg_idx]
                    if leg_idx == 0:
                        self.positions[coxa_id - 1] = NEUTRAL_COXA
                        self.positions[femur_id - 1] = NEUTRAL_FEMUR
                    elif leg_idx == 1:
                        self.positions[coxa_id - 1] = NEUTRAL_COXA - SWING_ANGLE
                        self.positions[femur_id - 1] = NEUTRAL_FEMUR + LIFT_HEIGHT
                    elif leg_idx == 2:
                        self.positions[coxa_id - 1] = NEUTRAL_COXA
                        self.positions[femur_id - 1] = NEUTRAL_FEMUR 
                    elif leg_idx == 3:
                        self.positions[coxa_id - 1] = NEUTRAL_COXA - FB_SWING_ANGLE
                        self.positions[femur_id - 1] = NEUTRAL_FEMUR + LIFT_HEIGHT
                    elif leg_idx == 4:
                        self.positions[coxa_id - 1] = NEUTRAL_COXA
                        self.positions[femur_id - 1] = NEUTRAL_FEMUR 
                    elif leg_idx == 5:
                        self.positions[coxa_id - 1] = NEUTRAL_COXA - FB_SWING_ANGLE
                        self.positions[femur_id - 1] = NEUTRAL_FEMUR + LIFT_HEIGHT
            elif phase == 1:
                for leg_idx in range(6):
                    coxa_id, femur_id = LEG_SERVO_MAP[leg_idx]
                    if leg_idx == 0:
                        self.positions[coxa_id - 1] = NEUTRAL_COXA
                        self.positions[femur_id - 1] = NEUTRAL_FEMUR + LIFT_HEIGHT
                    elif leg_idx == 1:
                        self.positions[coxa_id - 1] = NEUTRAL_COXA - SWING_ANGLE
                        self.positions[femur_id - 1] = NEUTRAL_FEMUR
                    elif leg_idx == 2:
                        self.positions[coxa_id - 1] = NEUTRAL_COXA 
                        self.positions[femur_id - 1] = NEUTRAL_FEMUR + LIFT_HEIGHT
                    elif leg_idx == 3:
                        self.positions[coxa_id - 1] = NEUTRAL_COXA - FB_SWING_ANGLE
                        self.positions[femur_id - 1] = NEUTRAL_FEMUR
                    elif leg_idx == 4:
                        self.positions[coxa_id - 1] = NEUTRAL_COXA 
                        self.positions[femur_id - 1] = NEUTRAL_FEMUR + LIFT_HEIGHT
                    elif leg_idx == 5:
                        self.positions[coxa_id - 1] = NEUTRAL_COXA - FB_SWING_ANGLE
                        self.positions[femur_id - 1] = NEUTRAL_FEMUR
            elif phase == 2:
                for leg_idx in range(6):
                    coxa_id, femur_id = LEG_SERVO_MAP[leg_idx]
                    if leg_idx == 0:
                        self.positions[coxa_id - 1] = NEUTRAL_COXA - FB_SWING_ANGLE
                        self.positions[femur_id - 1] = NEUTRAL_FEMUR + LIFT_HEIGHT
                    elif leg_idx == 1:
                        self.positions[coxa_id - 1] = NEUTRAL_COXA
                        self.positions[femur_id - 1] = NEUTRAL_FEMUR
                    elif leg_idx == 2:
                        self.positions[coxa_id - 1] = NEUTRAL_COXA - FB_SWING_ANGLE
                        self.positions[femur_id - 1] = NEUTRAL_FEMUR + LIFT_HEIGHT
                    elif leg_idx == 3:
                        self.positions[coxa_id - 1] = NEUTRAL_COXA
                        self.positions[femur_id - 1] = NEUTRAL_FEMUR
                    elif leg_idx == 4:
                        self.positions[coxa_id - 1] = NEUTRAL_COXA - SWING_ANGLE
                        self.positions[femur_id - 1] = NEUTRAL_FEMUR + LIFT_HEIGHT
                    elif leg_idx == 5:
                        self.positions[coxa_id - 1] = NEUTRAL_COXA
                        self.positions[femur_id - 1] = NEUTRAL_FEMUR
            elif phase == 3:
                for leg_idx in range(6):
                    coxa_id, femur_id = LEG_SERVO_MAP[leg_idx]
                    if leg_idx == 0:
                        self.positions[coxa_id - 1] = NEUTRAL_COXA - FB_SWING_ANGLE
                        self.positions[femur_id - 1] = NEUTRAL_FEMUR
                    elif leg_idx == 1:
                        self.positions[coxa_id - 1] = NEUTRAL_COXA
                        self.positions[femur_id - 1] = NEUTRAL_FEMUR + LIFT_HEIGHT
                    elif leg_idx == 2:
                        self.positions[coxa_id - 1] = NEUTRAL_COXA - FB_SWING_ANGLE
                        self.positions[femur_id - 1] = NEUTRAL_FEMUR
                    elif leg_idx == 3:
                        self.positions[coxa_id - 1] = NEUTRAL_COXA
                        self.positions[femur_id - 1] = NEUTRAL_FEMUR + LIFT_HEIGHT
                    elif leg_idx == 4:
                        self.positions[coxa_id - 1] = NEUTRAL_COXA - SWING_ANGLE
                        self.positions[femur_id - 1] = NEUTRAL_FEMUR
                    elif leg_idx == 5:
                        self.positions[coxa_id - 1] = NEUTRAL_COXA
                        self.positions[femur_id - 1] = NEUTRAL_FEMUR + LIFT_HEIGHT

        if self.current_cmd == 'right':
            phase = self.gait_phase % 4
            if phase == 0:
                for leg_idx in range(6):
                    coxa_id, femur_id = LEG_SERVO_MAP[leg_idx]
                    if leg_idx == 0:
                        self.positions[coxa_id - 1] = NEUTRAL_COXA
                        self.positions[femur_id - 1] = NEUTRAL_FEMUR
                    elif leg_idx == 1:
                        self.positions[coxa_id - 1] = NEUTRAL_COXA + SWING_ANGLE
                        self.positions[femur_id - 1] = NEUTRAL_FEMUR + LIFT_HEIGHT
                    elif leg_idx == 2:
                        self.positions[coxa_id - 1] = NEUTRAL_COXA
                        self.positions[femur_id - 1] = NEUTRAL_FEMUR 
                    elif leg_idx == 3:
                        self.positions[coxa_id - 1] = NEUTRAL_COXA + FB_SWING_ANGLE
                        self.positions[femur_id - 1] = NEUTRAL_FEMUR + LIFT_HEIGHT
                    elif leg_idx == 4:
                        self.positions[coxa_id - 1] = NEUTRAL_COXA
                        self.positions[femur_id - 1] = NEUTRAL_FEMUR 
                    elif leg_idx == 5:
                        self.positions[coxa_id - 1] = NEUTRAL_COXA + FB_SWING_ANGLE
                        self.positions[femur_id - 1] = NEUTRAL_FEMUR + LIFT_HEIGHT
            elif phase == 1:
                for leg_idx in range(6):
                    coxa_id, femur_id = LEG_SERVO_MAP[leg_idx]
                    if leg_idx == 0:
                        self.positions[coxa_id - 1] = NEUTRAL_COXA
                        self.positions[femur_id - 1] = NEUTRAL_FEMUR + LIFT_HEIGHT
                    elif leg_idx == 1:
                        self.positions[coxa_id - 1] = NEUTRAL_COXA + SWING_ANGLE
                        self.positions[femur_id - 1] = NEUTRAL_FEMUR
                    elif leg_idx == 2:
                        self.positions[coxa_id - 1] = NEUTRAL_COXA 
                        self.positions[femur_id - 1] = NEUTRAL_FEMUR + LIFT_HEIGHT
                    elif leg_idx == 3:
                        self.positions[coxa_id - 1] = NEUTRAL_COXA + FB_SWING_ANGLE
                        self.positions[femur_id - 1] = NEUTRAL_FEMUR
                    elif leg_idx == 4:
                        self.positions[coxa_id - 1] = NEUTRAL_COXA 
                        self.positions[femur_id - 1] = NEUTRAL_FEMUR + LIFT_HEIGHT
                    elif leg_idx == 5:
                        self.positions[coxa_id - 1] = NEUTRAL_COXA + FB_SWING_ANGLE
                        self.positions[femur_id - 1] = NEUTRAL_FEMUR
            elif phase == 2:
                for leg_idx in range(6):
                    coxa_id, femur_id = LEG_SERVO_MAP[leg_idx]
                    if leg_idx == 0:
                        self.positions[coxa_id - 1] = NEUTRAL_COXA + FB_SWING_ANGLE
                        self.positions[femur_id - 1] = NEUTRAL_FEMUR + LIFT_HEIGHT
                    elif leg_idx == 1:
                        self.positions[coxa_id - 1] = NEUTRAL_COXA
                        self.positions[femur_id - 1] = NEUTRAL_FEMUR
                    elif leg_idx == 2:
                        self.positions[coxa_id - 1] = NEUTRAL_COXA + FB_SWING_ANGLE
                        self.positions[femur_id - 1] = NEUTRAL_FEMUR + LIFT_HEIGHT
                    elif leg_idx == 3:
                        self.positions[coxa_id - 1] = NEUTRAL_COXA
                        self.positions[femur_id - 1] = NEUTRAL_FEMUR
                    elif leg_idx == 4:
                        self.positions[coxa_id - 1] = NEUTRAL_COXA + SWING_ANGLE
                        self.positions[femur_id - 1] = NEUTRAL_FEMUR + LIFT_HEIGHT
                    elif leg_idx == 5:
                        self.positions[coxa_id - 1] = NEUTRAL_COXA
                        self.positions[femur_id - 1] = NEUTRAL_FEMUR
            elif phase == 3:
                for leg_idx in range(6):
                    coxa_id, femur_id = LEG_SERVO_MAP[leg_idx]
                    if leg_idx == 0:
                        self.positions[coxa_id - 1] = NEUTRAL_COXA + FB_SWING_ANGLE
                        self.positions[femur_id - 1] = NEUTRAL_FEMUR
                    elif leg_idx == 1:
                        self.positions[coxa_id - 1] = NEUTRAL_COXA
                        self.positions[femur_id - 1] = NEUTRAL_FEMUR + LIFT_HEIGHT
                    elif leg_idx == 2:
                        self.positions[coxa_id - 1] = NEUTRAL_COXA + FB_SWING_ANGLE
                        self.positions[femur_id - 1] = NEUTRAL_FEMUR
                    elif leg_idx == 3:
                        self.positions[coxa_id - 1] = NEUTRAL_COXA
                        self.positions[femur_id - 1] = NEUTRAL_FEMUR + LIFT_HEIGHT
                    elif leg_idx == 4:
                        self.positions[coxa_id - 1] = NEUTRAL_COXA + SWING_ANGLE
                        self.positions[femur_id - 1] = NEUTRAL_FEMUR
                    elif leg_idx == 5:
                        self.positions[coxa_id - 1] = NEUTRAL_COXA
                        self.positions[femur_id - 1] = NEUTRAL_FEMUR + LIFT_HEIGHT

        if self.current_cmd == 'backward':
            phase = self.gait_phase % 4
            if phase == 0:
                for leg_idx in range(6):
                    coxa_id, femur_id = LEG_SERVO_MAP[leg_idx]
                    if leg_idx == 0:
                        self.positions[coxa_id - 1] = NEUTRAL_COXA
                        self.positions[femur_id - 1] = NEUTRAL_FEMUR
                    elif leg_idx == 1:
                        self.positions[coxa_id - 1] = NEUTRAL_COXA + SWING_ANGLE
                        self.positions[femur_id - 1] = NEUTRAL_FEMUR + LIFT_HEIGHT
                    elif leg_idx == 2:
                        self.positions[coxa_id - 1] = NEUTRAL_COXA
                        self.positions[femur_id - 1] = NEUTRAL_FEMUR 
                    elif leg_idx == 3:
                        self.positions[coxa_id - 1] = NEUTRAL_COXA - FB_SWING_ANGLE
                        self.positions[femur_id - 1] = NEUTRAL_FEMUR + LIFT_HEIGHT
                    elif leg_idx == 4:
                        self.positions[coxa_id - 1] = NEUTRAL_COXA
                        self.positions[femur_id - 1] = NEUTRAL_FEMUR 
                    elif leg_idx == 5:
                        self.positions[coxa_id - 1] = NEUTRAL_COXA - FB_SWING_ANGLE
                        self.positions[femur_id - 1] = NEUTRAL_FEMUR + LIFT_HEIGHT
            elif phase == 1:
                for leg_idx in range(6):
                    coxa_id, femur_id = LEG_SERVO_MAP[leg_idx]
                    if leg_idx == 0:
                        self.positions[coxa_id - 1] = NEUTRAL_COXA
                        self.positions[femur_id - 1] = NEUTRAL_FEMUR + LIFT_HEIGHT
                    elif leg_idx == 1:
                        self.positions[coxa_id - 1] = NEUTRAL_COXA + SWING_ANGLE
                        self.positions[femur_id - 1] = NEUTRAL_FEMUR
                    elif leg_idx == 2:
                        self.positions[coxa_id - 1] = NEUTRAL_COXA 
                        self.positions[femur_id - 1] = NEUTRAL_FEMUR + LIFT_HEIGHT
                    elif leg_idx == 3:
                        self.positions[coxa_id - 1] = NEUTRAL_COXA - FB_SWING_ANGLE
                        self.positions[femur_id - 1] = NEUTRAL_FEMUR
                    elif leg_idx == 4:
                        self.positions[coxa_id - 1] = NEUTRAL_COXA 
                        self.positions[femur_id - 1] = NEUTRAL_FEMUR + LIFT_HEIGHT
                    elif leg_idx == 5:
                        self.positions[coxa_id - 1] = NEUTRAL_COXA - FB_SWING_ANGLE
                        self.positions[femur_id - 1] = NEUTRAL_FEMUR
            elif phase == 2:
                for leg_idx in range(6):
                    coxa_id, femur_id = LEG_SERVO_MAP[leg_idx]
                    if leg_idx == 0:
                        self.positions[coxa_id - 1] = NEUTRAL_COXA + FB_SWING_ANGLE
                        self.positions[femur_id - 1] = NEUTRAL_FEMUR + LIFT_HEIGHT
                    elif leg_idx == 1:
                        self.positions[coxa_id - 1] = NEUTRAL_COXA
                        self.positions[femur_id - 1] = NEUTRAL_FEMUR
                    elif leg_idx == 2:
                        self.positions[coxa_id - 1] = NEUTRAL_COXA + FB_SWING_ANGLE
                        self.positions[femur_id - 1] = NEUTRAL_FEMUR + LIFT_HEIGHT
                    elif leg_idx == 3:
                        self.positions[coxa_id - 1] = NEUTRAL_COXA
                        self.positions[femur_id - 1] = NEUTRAL_FEMUR
                    elif leg_idx == 4:
                        self.positions[coxa_id - 1] = NEUTRAL_COXA - SWING_ANGLE
                        self.positions[femur_id - 1] = NEUTRAL_FEMUR + LIFT_HEIGHT
                    elif leg_idx == 5:
                        self.positions[coxa_id - 1] = NEUTRAL_COXA
                        self.positions[femur_id - 1] = NEUTRAL_FEMUR
            elif phase == 3:
                for leg_idx in range(6):
                    coxa_id, femur_id = LEG_SERVO_MAP[leg_idx]
                    if leg_idx == 0:
                        self.positions[coxa_id - 1] = NEUTRAL_COXA + FB_SWING_ANGLE
                        self.positions[femur_id - 1] = NEUTRAL_FEMUR
                    elif leg_idx == 1:
                        self.positions[coxa_id - 1] = NEUTRAL_COXA
                        self.positions[femur_id - 1] = NEUTRAL_FEMUR + LIFT_HEIGHT
                    elif leg_idx == 2:
                        self.positions[coxa_id - 1] = NEUTRAL_COXA + FB_SWING_ANGLE
                        self.positions[femur_id - 1] = NEUTRAL_FEMUR
                    elif leg_idx == 3:
                        self.positions[coxa_id - 1] = NEUTRAL_COXA
                        self.positions[femur_id - 1] = NEUTRAL_FEMUR + LIFT_HEIGHT
                    elif leg_idx == 4:
                        self.positions[coxa_id - 1] = NEUTRAL_COXA - SWING_ANGLE
                        self.positions[femur_id - 1] = NEUTRAL_FEMUR
                    elif leg_idx == 5:
                        self.positions[coxa_id - 1] = NEUTRAL_COXA
                        self.positions[femur_id - 1] = NEUTRAL_FEMUR + LIFT_HEIGHT
            self.gait_phase += 1
  

    def publish_joints(self):
        msg = JointTrajectory()
        msg.joint_names = [f'servo_{i}' for i in range(1, NUM_SERVOS+1)]
        pt = JointTrajectoryPoint()
        pt.positions = self.positions.copy()
        pt.time_from_start.sec = 0
        pt.time_from_start.nanosec = 0
        msg.points = [pt]
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = HexapodTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.reset_to_neutral()
        node.publish_joints()
        node.destroy_node()
        rclpy.shutdown()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, node.settings)

if __name__ == '__main__':
    main()
