import time
import yaml
import argparse
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(__file__)))
from hls.contrl.core_new_2 import OrcaHand

def main():
    parser = argparse.ArgumentParser(description='Replay joint waypoints at fixed frame rate.')
    parser.add_argument('model_path', type=str, help='Path to the orcahand config.yaml')
    parser.add_argument('replay_file', type=str, help='YAML file with waypoints')
    parser.add_argument('--step_time', type=float, default=1, help='Seconds per frame (default: 0.02s = 50Hz)')
    args = parser.parse_args()

    # === Load waypoints ===
    try:
        with open(args.replay_file, "r") as f:
            data = yaml.safe_load(f)
        waypoints = data.get("waypoints") or data.get("angles", [])
    except Exception as e:
        print("Failed to load replay file:", e)
        return

    if not waypoints:
        print("No waypoints found in the file.")
        return

    # === Initialize hand ===
    hand = OrcaHand(args.model_path)
    status = hand.connect()
    print("Connect:", status)
    if not status[0]:
        print("Failed to connect to hand.")
        return

    hand.enable_torque()
    print("Torque enabled. Replaying...")

    try:
        while True:
            for i, pose in enumerate(waypoints):
                start_time = time.time()

                print(f"\n[Frame {i}] Sending pose:")
                print(pose)

                hand.set_joint_pos(pose)

                elapsed = time.time() - start_time
                remaining = args.step_time - elapsed

                print(f"[Frame {i}] Elapsed: {elapsed:.4f}s, Sleeping: {max(remaining, 0):.4f}s")

                if remaining > 0:
                    time.sleep(remaining)

    except KeyboardInterrupt:
        print("Replay interrupted by user.")

    finally:
        hand.disable_torque()
        print("Torque disabled.")

if __name__ == "__main__":
    main()
