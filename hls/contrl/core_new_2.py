import os
import time
import numpy as np
import yaml
from hls.contrl.client_new_2 import BMotorClient

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
        self.type: str = self.cfg.get('type', None)

        self.calib_sequence = self.cfg.get("calib_sequence", self.motor_ids)
        self.calib_current = self.cfg.get("calib_current", 100.0)
        self.calib_threshold = self.cfg.get("calib_threshold", 10.0)
        self.calib_window = self.cfg.get("calib_window", 5)

        self.default_speed = 20
        self.default_acc = 20
        self.default_torque = 150

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
        self.load_calibration()  # 自动加载校准数据
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
    
    def get_joint_pos(self, as_list=False) -> dict | list:
        """Returns current joint positions in radians, based on calibrated offsets and direction."""
        motor_pos = self.motor.read_positions()

        # 打印 motor 层返回的原始弧度值
        # print("=== Raw motor_pos (弧度) ===")
        # for mid in sorted(motor_pos):
        #     print(f"  Motor {mid:2d}: {motor_pos[mid]:.6f} rad")
        # print("============================")

        joint_pos = {}
        for joint in self.joint_names:
            sid = abs(self.joint_map[joint])
            base_dir = np.sign(self.joint_map[joint])
            offset = self.joint_offsets.get(joint, 0.0)  # 单位：弧度
            raw_rad = motor_pos[sid]  # 电机弧度值
            joint_pos[joint] = base_dir * raw_rad + offset

        return list(joint_pos.values()) if as_list else joint_pos
    
    def set_joint_pos(self, joint_angles: list, speed=15, acc=20, torque=125):
        """Set desired joint angles (in radians), auto-mapped to motor positions."""
        assert len(joint_angles) == len(self.joint_names), \
            f"Expected {len(self.joint_names)} joint angles, got {len(joint_angles)}"

        motor_cmd = {}
        for joint_name, joint_angle in zip(self.joint_names, joint_angles):
            sid = abs(self.joint_map[joint_name])
            base_dir = np.sign(self.joint_map[joint_name])
            offset = self.joint_offsets.get(joint_name, 0.0)

            # 从 joint angle 反推 motor angle（仍是弧度）
            motor_angle = (joint_angle - offset) * base_dir
            motor_cmd[sid] = motor_angle  # 保持弧度传给底层 motor 写入函数

        self.motor.write_positions_with_speed_acc_torque(
            position_dict=motor_cmd,
            speed=speed,
            acc=acc,
            torque=torque
        )
        
    def set_neutral_position(self):
        """Move all motors to neutral positions (弧度), derived from raw-position integers in config.yaml."""
        neutral_config = self.cfg.get("neutral_position", {})
        motor_range_config = self.cfg.get("motor_range", {})

        if not neutral_config:
            print("[Neutral] No neutral_position found in config.yaml.")
            return

        print("[Neutral] Moving motors to neutral_position...")

        # 舵机位置单位：原始整数 → 弧度
        cmd_dict = {}
        for sid in self.motor_ids:
            raw_pos = neutral_config.get(sid, 0)
            if sid in motor_range_config:
                low, high = motor_range_config[sid]
                raw_pos = int(np.clip(raw_pos, low, high))
            else:
                print(f"[WARN] No motor_range defined for motor {sid}, using unclipped value.")
            
            # 👇 加上 scale 换算：整数 → 弧度
            pos_rad = raw_pos * 2.0 * np.pi / 4095
            cmd_dict[sid] = pos_rad

        # 发送弧度单位位置指令
        self.motor.write_positions_with_speed_acc_torque(
            position_dict=cmd_dict,
            speed=10,
            acc=20,
            torque=125
        )

        time.sleep(3.0)

    def write_desired_pos(self, joint_ids, positions):
        self.motor.write_positions_with_speed_acc_torque(
            dict(zip(joint_ids, positions)),
            speed=self.default_speed,
            acc=self.default_acc,
            torque=self.default_torque
        )

    def read_pos_vel_cur(self):
        return self.motor.read_pos_vel_cur()

    def set_torque_enabled(self, motor_ids, enabled):
        self.motor.set_torque_enabled(motor_ids, enabled)

    def set_joint_current(self, current_dict):
        for mid, current in current_dict.items():
            self.motor.set_torque_limit(mid, current)

    def read_joint_currents(self):
        return self.motor.read_currents()

    def joint_ids(self):
        return self.motor_ids
    

    def calibrate(self):
        if not self.is_connected():
            raise RuntimeError("Hand not connected.")

        print("[Calib] Starting dynamic calibration...\n>>> 所有舵机断电，请手动将所有关节调整到合适位置，然后按下 ENTER 继续...")
        self.disable_torque()
        input("准备就绪后按 ENTER ▶▶▶")

        neutral_pos = self.motor.read_positions()
        scale = 1.0
        neutral_config = {sid: round(neutral_pos[sid] / scale, 6) for sid in self.motor_ids}
        self._update_config('neutral_position', neutral_config)
        print("[Calib] Neutral position captured and saved to config.yaml.")

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
                    current = abs(self.motor.read_currents()[sid])
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

        motor_range_list = [[round(pos_min[sid] / scale, 6), round(pos_max[sid] / scale, 6)] for sid in self.motor_ids]
        self.motor_range = np.array(motor_range_list)
        self._update_config('motor_range', {sid: r for sid, r in zip(self.motor_ids, motor_range_list)})

        for joint in self.joint_names:
            sid = abs(self.joint_map[joint])
            base_dir = np.sign(self.joint_map[joint])
            mid = (pos_min[sid] + pos_max[sid]) / 2 / scale
            self.joint_offsets[joint] = round(-base_dir * mid, 6)
            rom_half = abs((pos_max[sid] - pos_min[sid]) / 2 / scale)
            self.joint_roms[joint] = [round(-rom_half, 6), round(rom_half, 6)]

        self.calibrated = True
        self._update_config('joint_offsets', self.joint_offsets)
        self._update_config('joint_roms', self.joint_roms)
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

        scale = 1.0
        motor_range_list = [[round(pos_min[sid] / scale, 6), round(pos_max[sid] / scale, 6)] for sid in self.motor_ids]
        self.motor_range = np.array(motor_range_list)
        self._update_config('motor_range', {sid: r for sid, r in zip(self.motor_ids, motor_range_list)})

        for joint in self.joint_names:
            sid = abs(self.joint_map[joint])
            base_dir = np.sign(self.joint_map[joint])
            mid = (pos_min[sid] + pos_max[sid]) / 2 / scale
            self.joint_offsets[joint] = round(-base_dir * mid, 6)
            rom_half = abs((pos_max[sid] - pos_min[sid]) / 2 / scale)
            self.joint_roms[joint] = [round(-rom_half, 6), round(rom_half, 6)]

        self.calibrated = True
        self._update_config('joint_offsets', self.joint_offsets)
        self._update_config('joint_roms', self.joint_roms)
        print("[Manual Calib] Calibration complete. Offsets, ROMs, and motor range saved.")


    def load_calibration(self):
        """Load joint_offsets and joint_roms from config file."""
        if not os.path.exists(self.model_path):
            raise FileNotFoundError(f"Config file not found: {self.model_path}")

        with open(self.model_path, 'r') as f:
            config = yaml.safe_load(f) or {}

        joint_offsets = config.get('joint_offsets')
        joint_roms = config.get('joint_roms')

        if joint_offsets is None or joint_roms is None:
            print("[load_calibration] Missing joint_offsets or joint_roms in config.")
            return

        self.joint_offsets = {str(k): float(v) for k, v in joint_offsets.items()}
        self.joint_roms = {str(k): list(map(int, v)) for k, v in joint_roms.items()}
        self.calibrated = True
        print("[load_calibration] Calibration data loaded successfully.")


    def init_joints(self):
        return 0
