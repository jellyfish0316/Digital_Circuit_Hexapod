#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import sys
import termios
import tty
import select

# ==========================================
# Configuration from Arduino Code
# ==========================================

# Default Constants
C_SERVO_MIN = 100
C_SERVO_MAX = 475

# ID Mappings (Indices 0-5)
# Leg 0-2: Right Side?, Leg 3-5: Left Side?
IDS_LEG = [1, 4, 5, 7, 15, 11]  # Vertical Servos (Femur/Lift)
IDS_MID = [2, 3, 6, 8, 10, 12]  # Horizontal Servos (Coxa/Swing)

# Calibration Values (PWM Pulse Widths)
# Copied directly from Arduino initialization
VAL_SERVO_MAX = [
    C_SERVO_MAX,      C_SERVO_MAX + 30, C_SERVO_MAX,
    C_SERVO_MAX,      C_SERVO_MAX + 30, C_SERVO_MAX + 30
]

VAL_SERVO_MIN = [
    C_SERVO_MIN - 30, C_SERVO_MIN,      C_SERVO_MIN - 20,
    C_SERVO_MIN,      C_SERVO_MIN,      C_SERVO_MIN
]

VAL_SERVO_HALF = [185, 300, 200, 200, 310, 220] # Stance/Down position

VAL_MID_NEUTRAL = [300, 270, 270, 140, 250, 110] # Neutral Horizontal
VAL_MID_INC     = [220, 220, 220, 120, 280, 180] # Forward/Swing Target

NUM_LEGS = 6

class HexapodGaitController(Node):
    def __init__(self):
        super().__init__('hexapod_gait_controller')
        
        # Publisher
        self.pub = self.create_publisher(JointTrajectory, '/servo_trajectory', 10)
        
        # Keyboard Input Setup
        self.settings = termios.tcgetattr(sys.stdin)
        
        self.get_logger().info('Hexapod Gait Controller Started')
        self.get_logger().info('Mapped from Arduino Code.')
        self.get_logger().info('Controls: [W] Start Gait, [Space] Stop/Sit, [Q] Quit')
        
        # State Machine
        self.gait_phase = 0 # 0 to 3
        self.is_moving = False
        
        # Collect all unique Servo IDs for the ROS message
        self.all_servo_ids = sorted(list(set(IDS_LEG + IDS_MID)))
        
        # Current PWM state map {servo_id: pwm_value}
        self.current_pwms = {sid: 0.0 for sid in self.all_servo_ids}
        
        # Initial Pose (Sitting)
        self.execute_stop_pose()

        # Timer: The Arduino loop has delay(1000) between phases.
        # We set a timer to trigger the next phase.
        self.timer_period = 1.0 # Seconds (matches delay(1000))
        self.timer = self.create_timer(self.timer_period, self.gait_loop)

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    # ==========================================
    # Logic Helpers (Mimicking C++ Functions)
    # ==========================================
    
    def get_leg_up_val(self, idx):
        # void legUp(int id): if (id <= 2) max else min
        if idx <= 2:
            return VAL_SERVO_MAX[idx]
        else:
            return VAL_SERVO_MIN[idx]

    def get_leg_down_val(self, idx):
        # void legDown(int id): if (id <= 2) min else max
        # Note: The Arduino loop actually uses legHalf() for "Down" mostly, 
        # but legDown is defined as the inverse of Up.
        if idx <= 2:
            return VAL_SERVO_MIN[idx]
        else:
            return VAL_SERVO_MAX[idx]

    def get_leg_half_val(self, idx):
        return VAL_SERVO_HALF[idx]

    def get_leg_middle_val(self, idx):
        return VAL_MID_NEUTRAL[idx]

    def get_leg_inc_val(self, idx):
        return VAL_MID_INC[idx]

    # ==========================================
    # Main Loop Logic
    # ==========================================

    def execute_stop_pose(self):
        # Equivalent to all legs Half (Vertical) and Middle (Horizontal)
        for i in range(NUM_LEGS):
            self.current_pwms[IDS_LEG[i]] = float(self.get_leg_half_val(i))
            self.current_pwms[IDS_MID[i]] = float(self.get_leg_middle_val(i))
        self.publish_joints()

    def update_gait_state(self):
        # Mimics the Arduino loop structure with phases
        
        # Phase 0: 
        #   if (i % 2): legUp, legInc
        #   else: legHalf, legMiddle
        if self.gait_phase == 0:
            for i in range(NUM_LEGS):
                if i % 2 != 0: # Odd (1, 3, 5)
                    self.current_pwms[IDS_LEG[i]] = float(self.get_leg_up_val(i))
                    self.current_pwms[IDS_MID[i]] = float(self.get_leg_inc_val(i))
                else: # Even (0, 2, 4)
                    self.current_pwms[IDS_LEG[i]] = float(self.get_leg_half_val(i))
                    self.current_pwms[IDS_MID[i]] = float(self.get_leg_middle_val(i))

        # Phase 1: 
        #   All legHalf (Down)
        #   (Horizontal servos stay at previous position based on Arduino code logic, 
        #    since loop only calls legHalf in this block)
        elif self.gait_phase == 1:
            for i in range(NUM_LEGS):
                self.current_pwms[IDS_LEG[i]] = float(self.get_leg_half_val(i))

        # Phase 2:
        #   if (i % 2): legHalf, legMiddle
        #   else: legUp, legInc
        elif self.gait_phase == 2:
            for i in range(NUM_LEGS):
                if i % 2 != 0: # Odd
                    self.current_pwms[IDS_LEG[i]] = float(self.get_leg_half_val(i))
                    self.current_pwms[IDS_MID[i]] = float(self.get_leg_middle_val(i))
                else: # Even
                    self.current_pwms[IDS_LEG[i]] = float(self.get_leg_up_val(i))
                    self.current_pwms[IDS_MID[i]] = float(self.get_leg_inc_val(i))

        # Phase 3:
        #   All legHalf (Down)
        elif self.gait_phase == 3:
            for i in range(NUM_LEGS):
                self.current_pwms[IDS_LEG[i]] = float(self.get_leg_half_val(i))

        # Advance Phase
        self.gait_phase = (self.gait_phase + 1) % 4

    def gait_loop(self):
        key = self.get_key()
        
        if key == 'w':
            self.is_moving = True
            self.get_logger().info('Walking...')
        elif key == ' ':
            self.is_moving = False
            self.get_logger().info('Stopping...')
            self.execute_stop_pose()
            self.gait_phase = 0 # Reset phase
        elif key == 'q':
            self.get_logger().info('Quitting...')
            raise KeyboardInterrupt

        if self.is_moving:
            self.update_gait_state()
            self.publish_joints()

    def publish_joints(self):
        msg = JointTrajectory()
        # Create joint names matching the specific IDs (e.g., servo_1, servo_15)
        msg.joint_names = [f'servo_{sid}' for sid in self.all_servo_ids]
        
        pt = JointTrajectoryPoint()
        # Map current PWMs to the correct index in joint_names
        pt.positions = [self.current_pwms[sid] for sid in self.all_servo_ids]
        pt.time_from_start.sec = 0
        pt.time_from_start.nanosec = 0 # Immediate execution
        
        msg.points = [pt]
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = HexapodGaitController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.execute_stop_pose()
        node.destroy_node()
        rclpy.shutdown()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, node.settings)

if __name__ == '__main__':
    main()