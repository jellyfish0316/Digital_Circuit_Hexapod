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
NEUTRAL_FEMUR = 120.0

# Gait parameters
LIFT_HEIGHT = 30.0  # Degrees to lift leg
SWING_ANGLE = 20.0  # Degrees to swing leg forward/backward
TIMER_PERIOD = 0.05 # Seconds (20Hz) for input polling
GAIT_INTERVAL = 0.5   # Seconds between gait steps (Walking speed)

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
        self.positions = [120.0] * NUM_SERVOS
        self.gait_phase = 0
        self.current_cmd = 'stop' # 'forward', 'backward', 'left', 'right', 'stop'
        self.last_step_time = 0
        
        # Timer for gait loop
        self.timer = self.create_timer(TIMER_PERIOD, self.gait_loop)

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
            now = time.time()
            if now - self.last_step_time > GAIT_INTERVAL:
                self.step_gait()
                self.last_step_time = now
            
        self.publish_joints()

    def reset_to_neutral(self):
        self.positions = [120.0] * NUM_SERVOS
        self.gait_phase = 0

    def step_gait(self):
        # Simple Tripod Gait
        # Group A: Legs 1, 3, 5 (Right Front, Right Rear, Left Middle)
        # Group B: Legs 2, 4, 6 (Right Middle, Left Front, Left Rear)
        
        # Leg Indices (0-based for list):
        # Leg 1 (RF): 0, 1 (ID 1,2)
        # Leg 2 (RM): 2, 3 (ID 3,4)
        # Leg 3 (RR): 4, 5 (ID 5,6)
        # Leg 4 (LF): 6, 7 (ID 7,8)
        # Leg 5 (LM): 8, 9 (ID 9,10)
        # Leg 6 (LR): 10, 11 (ID 11,12)
        
        # Tripod Groups (Indices of legs 0-5)
        # Group A: 0 (RF), 2 (RM), 4 (RR) -> Wait, Tripod is 1,3,5 vs 2,4,6?
        # Standard Tripod:
        # Set 1: RF, LM, RB (Right Front, Left Middle, Right Back) -> Legs 1, 5, 3
        # Set 2: LF, RM, LB (Left Front, Right Middle, Left Back) -> Legs 4, 2, 6
        
        # Let's map my Leg IDs to positions:
        # Leg 1: RF
        # Leg 2: RM
        # Leg 3: RB
        # Leg 4: LF
        # Leg 5: LM
        # Leg 6: LB
        
        group_a_legs = [0, 4, 2] # RF(0), LM(4), RB(2) -> Wait, Leg 5 is LM (index 4)
        # Leg indices: 0=1, 1=2, 2=3, 3=4, 4=5, 5=6
        # Leg 1 (RF) -> 0
        # Leg 2 (RM) -> 1
        # Leg 3 (RB) -> 2
        # Leg 4 (LF) -> 3
        # Leg 5 (LM) -> 4
        # Leg 6 (LB) -> 5
        
        # Tripod 1: RF(0), LM(4), RB(2)
        # Tripod 2: LF(3), RM(1), LB(5)
        
        tripod1 = [0, 4, 2]
        tripod2 = [3, 1, 5]
        
        phase = self.gait_phase % 2
        
        # Calculate swing offset
        swing_offset = SWING_ANGLE
        if self.current_cmd == 'backward':
            swing_offset = -SWING_ANGLE
            
        # Helper to set leg
        def set_leg(leg_idx, lift, swing_val):
            coxa_id, femur_id = LEG_SERVO_MAP[leg_idx]
            coxa_idx = coxa_id - 1
            femur_idx = femur_id - 1
            
            # Femur: Lift means smaller angle (up)
            femur_angle = NEUTRAL_FEMUR - LIFT_HEIGHT if lift else NEUTRAL_FEMUR
            
            # Coxa:
            # Right side (Legs 0,1,2): Forward is -angle (e.g. 90->60) or +angle?
            # Usually servos are mounted such that forward rotation is consistent?
            # Let's assume:
            # Right Side: 0=Front, 180=Back. Forward swing = Decrease angle.
            # Left Side: 0=Back, 180=Front. Forward swing = Increase angle.
            # OR symmetric: 0=Front for all? No, usually mirrored.
            # Let's assume standard mirrored mounting:
            # Right: 90 is center. <90 is forward.
            # Left: 90 is center. >90 is forward.
            
            coxa_angle = NEUTRAL_COXA
            is_right_side = leg_idx < 3
            
            if swing_val != 0:
                if self.current_cmd in ['forward', 'backward']:
                    # Forward walk
                    if is_right_side:
                        coxa_angle -= swing_val # Right: -swing to go forward
                    else:
                        coxa_angle += swing_val # Left: +swing to go forward
                
                elif self.current_cmd == 'left':
                    # Turn Left
                    # Right side moves forward (+swing effect)
                    # Left side moves backward (-swing effect)
                    if is_right_side:
                        coxa_angle -= swing_val # Right forward
                    else:
                        coxa_angle -= swing_val # Left backward (Forward is +, so Back is -)
                        
                elif self.current_cmd == 'right':
                    # Turn Right
                    if is_right_side:
                        coxa_angle += swing_val # Right backward
                    else:
                        coxa_angle += swing_val # Left forward

            self.positions[coxa_idx] = max(0.0, min(240.0, coxa_angle))
            self.positions[femur_idx] = max(0.0, min(240.0, femur_angle))

        # Execute Phase
        if phase == 0:
            # Tripod 1 Swing (Air), Tripod 2 Stance (Ground)
            for i in tripod1:
                set_leg(i, lift=True, swing_val=swing_offset) # Move to forward position in air
            for i in tripod2:
                set_leg(i, lift=False, swing_val=-swing_offset) # Move to backward position on ground
        else:
            # Tripod 2 Swing (Air), Tripod 1 Stance (Ground)
            for i in tripod1:
                set_leg(i, lift=False, swing_val=-swing_offset)
            for i in tripod2:
                set_leg(i, lift=True, swing_val=swing_offset)

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
