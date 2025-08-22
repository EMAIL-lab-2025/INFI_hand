import time
import numpy as np
from typing import Dict, Union, Optional, Sequence

from hls.contrl.client_new_2 import BMotorClient


class BMotorAdapter:
    def __init__(self, motor_ids: Sequence[int], port: str = '/dev/ttyUSB0', baudrate: int = 1000000):
        self.client = BMotorClient(motor_ids, port=port, baudrate=baudrate)

    def connect(self):
        return self.client.connect()

    def disconnect(self):
        self.client.disconnect()

    def set_operating_mode(self, motor_ids: Sequence[int], mode: int):
        # BMotor does not support operating mode switch, ignore or log warning
        import logging
        logging.warning("Operating mode is fixed on BMotor and cannot be changed.")

    def set_torque_enabled(self, motor_ids: Sequence[int], enabled: bool):
        self.client.set_torque_enabled(motor_ids, enabled)

    def enable_torque(self, motor_ids: Optional[Sequence[int]] = None):
        self.client.enable_torque(motor_ids)

    def disable_torque(self, motor_ids: Optional[Sequence[int]] = None):
        self.client.disable_torque(motor_ids)

    def write_desired_pos(self, motor_ids: Sequence[int], positions: Union[float, Sequence[float]]):
        if isinstance(positions, float):
            positions = [positions] * len(motor_ids)
        elif len(positions) != len(motor_ids):
            raise ValueError("Length of positions must match length of motor_ids")

        pos_dict = dict(zip(motor_ids, positions))
        self.client.write_positions_with_speed_acc_torque(pos_dict)

    def read_pos_vel_cur(self):
        return self.client.read_pos_vel_cur()

    @property
    def joint_ids(self):
        return self.client.motor_ids
