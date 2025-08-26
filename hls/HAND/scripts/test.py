import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(__file__)))
from contrl.core3 import OrcaHand
import time

hand = OrcaHand()

status = hand.connect()
print(status)
if not status[0]:
    print("Failed to connect to the hand.")
    exit(1)
    
hand.enable_torque()

joint_dict = {
    "index_mcp": 90,
    "middle_pip": 30,
}

hand_pose = {
    "thumb_mcp": -13,
    "thumb_abd": 43,
    "thumb_pip": 33,
    "thumb_dip": 19,
    "index_abd": 25,
    "index_mcp": 0,
    "index_pip": 0,
    "middle_abd": -2,
    "middle_mcp": 0,
    "middle_pip": 0,
    "ring_abd": -20,
    "ring_mcp": -1,
    "ring_pip": 0,
    "pinky_abd": -55,
    "pinky_mcp": 1,
    "pinky_pip": 0,
    "wrist": 0
}

hand_pose_1 = {
    "thumb_mcp": 0,
    "thumb_abd": 0,
    "thumb_pip": 0,
    "thumb_dip": 0,
    "index_abd": 0,
    "index_mcp": 0,
    "index_pip": 0,
    "middle_abd": 0,
    "middle_mcp": 0,
    "middle_pip": 0,
    "ring_abd": 0,
    "ring_mcp": 0,
    "ring_pip": 0,
    "pinky_abd": 10,
    "pinky_mcp": 0,
    "pinky_pip": 0,
    "wrist": 0
}

hand.set_joint_pos(hand_pose_1, num_steps = 25, step_size = 0.001)
print("Hand position set to hand_pose_1.")
# hand.set_zero_position()

time.sleep(4)
hand.disable_torque()
print("Torque disabled.")

hand.disconnect()
