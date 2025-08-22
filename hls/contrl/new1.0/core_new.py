import os
import time
import numpy as np
import yaml
from client_new import BMotorClient

class OrcaHand:
    def __init__(self, model_path=None):
        self.model_path = model_path or os.path.join(
            os.path.dirname(__file__), "config.yaml"
        )
        with open(self.model_path, 'r') as f:
            self.cfg = yaml.safe_load(f) or {}

        self.motor_ids = self.cfg["motor_ids"]
        self.motor_range = np.array(self.cfg.get("motor_range", [[-1000, 1000]] * len(self.motor_ids)))
        self.motor = BMotorClient(
            motor_ids=self.motor_ids,
            port=self.cfg["port"],
            baudrate=self.cfg["baudrate"]
        )

        self.joint_names = self.cfg.get("joint_names") or list(self.cfg.get("joint_to_motor_map", {}).keys())
        self.joint_map = self.cfg.get("joint_to_motor_map")
        self.joint_offsets = self.cfg.get("joint_offsets", {k: 0.0 for k in self.joint_names})
        self.joint_roms = self.cfg.get("joint_roms", {k: [-3.14, 3.14] for k in self.joint_names})

        self.calib_sequence = self.cfg.get("calib_sequence", self.motor_ids)
        self.calib_current = self.cfg.get("calib_current", 100.0)
        self.calib_threshold = self.cfg.get("calib_threshold", 10.0)
        self.calib_window = self.cfg.get("calib_window", 5)

        self.default_speed = 20
        self.default_acc = 20
        self.default_torque = 100

        self.calibrated = False

    def _update_config(self, key, value):
        def convert(obj):
            if isinstance(obj, np.ndarray):
                return obj.tolist()
            elif isinstance(obj, np.generic):
                return obj.item()
            elif isinstance(obj, dict):
                return {k: convert(v) for k, v in obj.items()}
            elif isinstance(obj, list):
                return [convert(v) for v in obj]
            return obj

        value = convert(value)

        try:
            with open(self.model_path, 'r') as f:
                cfg = yaml.safe_load(f) or {}
            cfg[key] = value
            with open(self.model_path, 'w') as f:
                yaml.dump(cfg, f, default_flow_style=False, sort_keys=False)
        except Exception as e:
            print(f"[YAML Update Error] {e}")
            raise

    def is_connected(self):
        return self.motor.is_connected()

    def is_calibrated(self):
        return self.calibrated

    def connect(self):
        return self.motor.connect()

    def disconnect(self):
        return self.motor.disconnect()

    def enable_torque(self, motor_ids=None):
        self.motor.enable_torque(motor_ids or self.motor_ids)

    def disable_torque(self, motor_ids=None):
        self.motor.disable_torque(motor_ids or self.motor_ids)

    def get_motor_pos(self):
        return self.motor.read_positions()

    def get_motor_current(self):
        return self.motor.read_currents()

    def get_motor_temp(self):
        return self.motor.read_temperature()

    def get_joint_pos(self):
        if not self.calibrated:
            return None
        motor_pos = self.get_motor_pos()
        joint_pos = {}
        for joint in self.joint_names:
            sid = abs(self.joint_map[joint])
            direction = np.sign(self.joint_map[joint])
            offset = self.joint_offsets[joint]
            joint_pos[joint] = direction * motor_pos[sid] + offset
        return joint_pos

    def set_joint_pos(self, joint_pos):
        if not self.calibrated:
            raise RuntimeError("Hand not calibrated. Cannot send joint commands.")
        cmd = np.zeros(len(self.motor_ids))
        for joint, value in joint_pos.items():
            if joint not in self.joint_map:
                continue
            sid = abs(self.joint_map[joint])
            direction = np.sign(self.joint_map[joint])
            offset = self.joint_offsets[joint]
            cmd[sid] += direction * (value - offset)
        cmd = np.clip(cmd, self.motor_range[:, 0], self.motor_range[:, 1])
        self.motor.write_positions_with_speed_acc_torque(
            {sid: cmd[i] for i, sid in enumerate(self.motor_ids)},
            speed=self.default_speed,
            acc=self.default_acc,
            torque=self.default_torque
        )

    def calibrate(self):
        if not self.is_connected():
            raise RuntimeError("Hand not connected.")

        print("[Calib] Starting dynamic calibration...\n>>> 所有舵机断电，请手动将所有关节调整到合适位置，然后按下 ENTER 继续...")
        self.disable_torque()
        input("准备就绪后按 ENTER ▶▶▶")

        neutral_pos = self.motor.read_positions()

        pos_min = {sid: float('inf') for sid in self.motor_ids}
        pos_max = {sid: float('-inf') for sid in self.motor_ids}

        for step in self.calib_sequence:
            if isinstance(step, int):
                motor_ids = [step]
                motor_dirs = {step: -1}
            else:
                joints = step.get("joints", {})
                motor_ids = []
                motor_dirs = {}
                for joint, movement in joints.items():
                    if joint not in self.joint_map:
                        continue
                    sid = abs(self.joint_map[joint])
                    base_dir = np.sign(self.joint_map[joint])
                    motor_ids.append(sid)
                    motor_dirs[sid] = base_dir if movement == "extend" else -base_dir

            for sid in motor_ids:
                print(f"[Calib] Calibrating motor {sid}...")
                self.disable_torque()
                time.sleep(0.05)
                self.enable_torque([sid])

                buffer = []
                reached = False
                start_pos = self.motor.read_positions()[sid]
                pos = start_pos
                direction = motor_dirs.get(sid, -1)
                step_size = 1

                # Set current threshold and torque per motor
                if sid == 17 or sid == 11:
                    current_threshold = 120.0
                elif sid == 8:
                    current_threshold = 150.0
                else:
                    current_threshold = self.calib_current

                if sid == 8:
                    torque = 175.0
                elif sid == 11:
                    torque = 130.0
                else:
                    torque = self.default_torque

                while not reached:
                    pos += direction * step_size
                    self.motor.write_position_with_speed_acc_torque(
                        sid, pos,
                        speed=self.default_speed,
                        acc=self.default_acc,
                        torque=torque
                    )
                    time.sleep(0.05)
                    current = self.motor.read_currents()[sid]
                    buffer.append(current)
                    if len(buffer) > self.calib_window:
                        buffer.pop(0)
                    pos_now = self.motor.read_positions()[sid]
                    pos_min[sid] = min(pos_min[sid], pos_now)
                    pos_max[sid] = max(pos_max[sid], pos_now)
                    if np.mean(buffer) > current_threshold:
                        print(f"[Calib] Detected contact. Avg current = {np.mean(buffer):.2f}")
                        reached = True

                self.motor.write_position_with_speed_acc_torque(
                    sid, neutral_pos[sid],
                    speed=self.default_speed,
                    acc=self.default_acc,
                    torque=self.default_torque
                )
                time.sleep(2.5)
                self.disable_torque([sid])

        motor_range_list = [[pos_min[sid], pos_max[sid]] for sid in self.motor_ids]
        self.motor_range = np.array(motor_range_list)
        self._update_config('motor_range', self.motor_range)

        neutral_config = {}
        for joint in self.joint_names:
            sid = abs(self.joint_map[joint])
            base_dir = np.sign(self.joint_map[joint])
            mid = (pos_min[sid] + pos_max[sid]) / 2
            self.joint_offsets[joint] = -base_dir * mid
            self.joint_roms[joint] = [-abs((pos_max[sid] - pos_min[sid]) / 2), abs((pos_max[sid] - pos_min[sid]) / 2)]
            neutral_config[sid] = neutral_pos[sid]

        self.calibrated = True
        self._update_config('joint_offsets', self.joint_offsets)
        self._update_config('joint_roms', self.joint_roms)
        self._update_config('neutral_position', neutral_config)
        print("[Calib] Calibration complete. Offsets, ROMs, and motor range saved.")

    def calibrate_manual(self):
        if not self.is_connected():
            raise RuntimeError("Hand not connected.")

        print("[Manual Calib] Starting manual calibration...")
        self.disable_torque()
        print("\n请手动掰动所有关节达到最大最小角度，然后按 ENTER 开始记录60秒的运动范围...")
        input("准备好了按 ENTER ▶▶▶")

        pos_min = {sid: float('inf') for sid in self.motor_ids}
        pos_max = {sid: float('-inf') for sid in self.motor_ids}

        print("[Manual Calib] 正在记录60秒内的位置变化，请继续来回掰动...")
        t_end = time.time() + 60
        while time.time() < t_end:
            positions = self.motor.read_positions()
            for sid in self.motor_ids:
                pos_min[sid] = min(pos_min[sid], positions[sid])
                pos_max[sid] = max(pos_max[sid], positions[sid])
            time.sleep(0.02)

        motor_range_list = [[pos_min[sid], pos_max[sid]] for sid in self.motor_ids]
        self.motor_range = np.array(motor_range_list)
        self._update_config('motor_range', self.motor_range)

        for joint in self.joint_names:
            sid = abs(self.joint_map[joint])
            base_dir = np.sign(self.joint_map[joint])
            mid = (pos_min[sid] + pos_max[sid]) / 2
            self.joint_offsets[joint] = -base_dir * mid
            self.joint_roms[joint] = [-abs((pos_max[sid] - pos_min[sid]) / 2), abs((pos_max[sid] - pos_min[sid]) / 2)]

        self.calibrated = True
        self._update_config('joint_offsets', self.joint_offsets)
        self._update_config('joint_roms', self.joint_roms)
        print("[Manual Calib] Calibration complete. Offsets, ROMs, and motor range saved.")