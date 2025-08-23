#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
单舵机原始位置写入测试：
示例：
  python test_single_servo_raw.py --id 1 --pos_raw 2048
可选：
  --port /dev/ttyUSB2  --baud 1000000
  --wait 1.0(写入后等待秒数)  --readback (写完读回并打印当前位置/状态)
"""

import argparse
import time
import sys

# 你的工程结构中已将 scservo_sdk 放在上级目录；如无必要可去掉这两行
sys.path.append("..")
from scservo_sdk import *
from scservo_sdk.protocol_packet_handler import protocol_packet_handler

# ---------------- 寄存器地址（与您的代码保持一致） ----------------
ADDR_TORQUE_ENABLE       = 40
ADDR_GOAL_POSITION_L     = 42   # 2 bytes
ADDR_PRESENT_POSITION_L  = 56   # 2 bytes
ADDR_PRESENT_TEMPERATURE = 63   # 1 byte
ADDR_MOVING              = 66   # 1 byte
ADDR_MODE                = 33   # 0=舵机(位置)；1=轮式(速度)
COMM_SUCCESS             = 0    # SDK 通信成功码

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--id", type=int, required=True, help="舵机ID")
    ap.add_argument("--pos_raw", type=int, required=True,
                    help="目标原始位置值（推荐 0~4095；你也可用于扩展范围实验）")
    ap.add_argument("--port", type=str, default="/dev/ttyUSB2", help="串口设备")
    ap.add_argument("--baud", type=int, default=1000000, help="波特率")
    ap.add_argument("--wait", type=float, default=1.0, help="写入后等待秒数")
    ap.add_argument("--readback", action="store_true", help="写入后读回实际位置与状态")
    args = ap.parse_args()

    motor_id = args.id
    goal_raw = int(args.pos_raw)

    # —— 基础安全钳位（按需调整）：标准单圈 0~4095 ；若你做多圈/偏移实验可放宽 —— #
    # goal_raw = max(0, min(4095, goal_raw))

    # 打开串口
    ph = PortHandler(args.port)
    if not ph.openPort():
        print(f"[ERR] 无法打开串口 {args.port}")
        sys.exit(1)
    if not ph.setBaudRate(args.baud):
        print(f"[ERR] 无法设置波特率 {args.baud}")
        ph.closePort()
        sys.exit(1)

    # 获取协议处理器（协议版本依据你的 SDK；此处与示例一致）
    pkt = protocol_packet_handler(ph, 0)

    # 确保处于“舵机(位置)模式”（ADDR_MODE=0）
    _res, _err = pkt.write1ByteTxRx(motor_id, ADDR_MODE, 0)
    if _res != COMM_SUCCESS or _err != 0:
        print(f"[WARN] 切换位置模式失败 result={_res} err={_err}（可能已在该模式，继续）")

    # 使能力矩
    res, err = pkt.write1ByteTxRx(motor_id, ADDR_TORQUE_ENABLE, 1)
    if res != COMM_SUCCESS or err != 0:
        print(f"[ERR] 使能力矩失败 result={res} err={err}")
        ph.closePort()
        sys.exit(1)

    # 发送目标位置（2字节：低/高位）
    lo = pkt.scs_lobyte(goal_raw)
    hi = pkt.scs_hibyte(goal_raw)
    res = pkt.write2ByteTxRx(motor_id, ADDR_GOAL_POSITION_L, pkt.scs_makeword(lo, hi))[1]  # 返回(result, err)
    # 说明：也可以用 write1Byte 两次写低/高位；这里用 write2ByteTxRx 更直观

    if res != 0:
        print(f"[ERR] 写入目标位置失败 err={res}")
        # 关闭力矩
        pkt.write1ByteTxRx(motor_id, ADDR_TORQUE_ENABLE, 0)
        ph.closePort()
        sys.exit(1)

    print(f"[OK] 已向ID={motor_id}写入目标原始位置：{goal_raw}")

    # 简单等待电机运动
    time.sleep(args.wait)

    if args.readback:
        # 读当前位置
        pos_raw, r1, e1 = pkt.read2ByteTxRx(motor_id, ADDR_PRESENT_POSITION_L)
        # 读是否仍在运动
        moving, r2, e2 = pkt.read1ByteTxRx(motor_id, ADDR_MOVING)
        # 读温度（可选）
        temp, r3, e3 = pkt.read1ByteTxRx(motor_id, ADDR_PRESENT_TEMPERATURE)

        if r1 == COMM_SUCCESS and e1 == 0:
            # 注意：视你的 SDK 而定，有些需要 scs_tohost 做符号扩展；对单圈0~4095通常无需
            print(f"[READ] 当前原始位置：{pos_raw}")
        else:
            print(f"[READ] 读取当前位置失败 result={r1} err={e1}")

        if r2 == COMM_SUCCESS and e2 == 0:
            print(f"[READ] MOVING状态：{moving}（1=运动中，0=停止）")
        if r3 == COMM_SUCCESS and e3 == 0:
            print(f"[READ] 温度：{temp}°C")

    # 视需要是否断电机力矩；测试结束可关闭以便手动扭动
    pkt.write1ByteTxRx(motor_id, ADDR_TORQUE_ENABLE, 0)
    ph.closePort()

if __name__ == "__main__":
    main()
