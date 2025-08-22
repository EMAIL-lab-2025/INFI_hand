import argparse
from core import OrcaHand  # 确保 core.py 和本文件在同一目录，或正确设置 PYTHONPATH

def main():
    parser = argparse.ArgumentParser(
        description="Automatically calibrate the InfiHand B-type servos. Provide the path to the model folder containing config.yaml."
    )
    parser.add_argument(
        "model_path",
        type=str,
        help="Path to the model folder (e.g., /path/to/bhand_model)"
    )
    args = parser.parse_args()

    hand = OrcaHand(args.model_path)

    if not hand.connect():
        print("❌ Failed to connect to the B-type hand.")
        exit(1)

    print("✅ Connected. Starting automatic calibration...")
    hand.calibrate()

if __name__ == "__main__":
    main()
