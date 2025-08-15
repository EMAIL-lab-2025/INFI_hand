import sys
import os
import time
import yaml
import numpy as np

# 添加 core_new_2.py 所在路径
sys.path.append(os.path.dirname(os.path.dirname(__file__)))
from core_new_2 import OrcaHand

def main():
    config_path = sys.argv[1] if len(sys.argv) > 1 else "./orcahand_v1_right/config.yaml"
    hand = OrcaHand(config_path)

    # 连接机械手
    hand.connect()

    # 关闭力矩，准备手动转动
    hand.disable_torque()
    print("Torque disabled. You can now manually move the joints.")

    print("\nPress Ctrl+C to exit. Reading joint positions every 0.5s:\n")
    try:
        while True:
            joint_pos = hand.get_joint_pos(as_list=False)
            print("Joint angles (rad):")
            for name, angle in joint_pos.items():
                print(f"  {name:>10s}: {angle:.6f}")
            print("-" * 40)
            time.sleep(0.5)

    except KeyboardInterrupt:
        print("\nTest finished.")

if __name__ == "__main__":
    main()
