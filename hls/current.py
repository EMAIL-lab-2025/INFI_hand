import time
import numpy as np
from hls.contrl.client_new_2 import BMotorClient  # 根据你实际的路径导入

def test_write_pos_with_raw_input():
    # 初始化参数
    SCALE = 4096 / (2 * np.pi)  # 转换比例，原始值 => 弧度
    motor_ids = [1]             # 假设我们只控制一个舵机
    raw_input = [3965]          # 例如中间位置（非弧度）

    # 创建 client（
    client = BMotorClient(motor_ids, port='/dev/ttyUSB1', baudrate=1000000)
    client.connect()
    client.enable_torque()

    # Step 1. 把 raw 转成弧度
    rad_pos = [p / SCALE for p in raw_input]
    print(f"[TEST] Raw input: {raw_input} -> radians: {rad_pos}")

    # Step 2. 构造 position_dict
    position_dict = dict(zip(motor_ids, rad_pos))

    # Step 3. 调用 write_positions_with_speed_acc_torque
    client.write_positions_with_speed_acc_torque(position_dict, speed=10, acc=10, torque=150)

    # Step 4. 等待观察
    time.sleep(2)

    client.disable_torque()
    client.disconnect()

if __name__ == "__main__":
    test_write_pos_with_raw_input()
