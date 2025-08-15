import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(__file__)))
from client_new_2 import BMotorClient

def test_read_positions():
    # 替换为你当前使用的舵机 ID 列表和串口
    motor_ids = list(range(1, 18))
    client = BMotorClient(motor_ids=motor_ids, port="/dev/ttyUSB1", baudrate=1000000)
    
    success, msg = client.connect()
    if not success:
        print("连接失败：", msg)
        return

    positions = client.read_positions()

    print("\n[TEST] Read positions:")
    for mid, pos in positions.items():
        print(f"Motor {mid}: pos = {pos:.4f} rad")

    client.disconnect()

if __name__ == "__main__":
    test_read_positions()
