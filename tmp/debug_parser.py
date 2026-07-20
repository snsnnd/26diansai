#!/usr/bin/env python3
"""MSPM0 line-car binary parser and tuning client.

Frame: AA 55 <type> <len> <payload[len]>, all multi-byte fields little-endian.
"""

import argparse
import os
import select
import serial
import struct
import sys
import time

TASK_NAMES = ["输入", "陀螺", "循线", "传感", "控制", "菜单", "蜂鸣", "OLED", "遥测", "调试", "调参"]
STATE_NAMES = {0: "停止", 1: "运行", 2: "故障"}
FAULT_NAMES = {
    0: "无", 1: "急停", 2: "电机看门狗", 3: "电池", 4: "陀螺仪",
    5: "循迹传感器", 6: "丢线", 7: "未知",
}
HEADING_NAMES = {
    0: "未初始化", 1: "等待数据", 2: "正常", 3: "总线错误",
    4: "ID错误", 5: "配置错误", 6: "校准失败",
}
GYRO_SOURCE_NAMES = {1: "M0", 2: "MPU6050"}
CURRENT_GYRO_SOURCE = 0
I2C_NAMES = {0: "OK", 1: "NACK", 2: "参数", 3: "时钟延展超时", 4: "总线卡死"}
T8_NAMES = {0: "OK", -1: "ERROR", -2: "参数", -3: "IO", -4: "超时", -5: "坏帧", -6: "校验", -7: "未知命令"}
LINE_STATE_NAMES = {0: "循线", 1: "寻线"}
CMD_STATUS_NAMES = {0: "已接受", 1: "非停止状态", 2: "安全状态拒绝", 3: "协议校验失败"}
H2024_STATE_NAMES = {
    0: "停止", 1: "离开A", 2: "直线1", 3: "弧线1", 4: "直线2",
    5: "弧线2", 6: "对齐", 7: "完成", 8: "超时", 9: "故障",
}

PARAM_NAMES = {0: "目标转速", 1: "基础PWM", 2: "前馈增益", 3: "速度KP",
    4: "速度KI", 5: "速度KD", 6: "巡线KP", 7: "巡线KD", 8: "航向KP",
    9: "航向KD", 10: "航向限幅", 11: "左轮增益", 12: "右轮增益",
    13: "方向符号", 14: "速度环", 15: "VOFA"}
PARAM_MAP = {v: k for k, v in PARAM_NAMES.items()}
PARAM_MAP.update({"kp": 6, "kd": 7, "line_kp": 6, "line_kd": 7,
    "巡线kp": 6, "巡线kd": 7})

G = "\033[32m"
Y = "\033[33m"
C = "\033[36m"
R = "\033[0m"
W = "\033[31m"


def unpack_payload(fmt, payload):
    size = struct.calcsize(fmt)
    if len(payload) != size:
        raise ValueError(f"长度错误: got={len(payload)} expected={size}")
    return struct.unpack(fmt, payload)


def task_name(index):
    return TASK_NAMES[index] if index < len(TASK_NAMES) else f"T{index}"


def parse_debug(payload):
    if len(payload) < 6:
        raise ValueError("任务统计帧过短")
    ts, state, count = struct.unpack_from("<IBB", payload)
    expected = 6 + count * 8
    if len(payload) != expected:
        raise ValueError(f"任务统计长度错误: got={len(payload)} expected={expected}")
    parts = [f"[{ts / 1000:.1f}s {STATE_NAMES.get(state, '?')}]任务"]
    for i in range(count):
        run, skipped = struct.unpack_from("<II", payload, 6 + i * 8)
        parts.append(f"{task_name(i)}={run}/{skipped}")
    return " ".join(parts)


def parse_tune(payload):
    if len(payload) != 6 or payload[0] != 0xCC:
        return f"调参响应异常 {payload.hex()}"
    pid = payload[1]
    value = struct.unpack_from("<f", payload, 2)[0]
    return f"调参 {PARAM_NAMES.get(pid, f'id{pid}')}={value:.2f}"


def parse_cmd_response(payload):
    status, left, right = unpack_payload("<Bhh", payload)
    return f"远程CMD {CMD_STATUS_NAMES.get(status, status)} left={left} right={right}"


def parse_event(payload):
    global CURRENT_GYRO_SOURCE

    if not payload:
        raise ValueError("事件帧为空")
    event = payload[0]
    if event == 1:
        (_, ts, profile, cpr, wheel_x100, gyro_enabled, gyro_source,
         p_gpio, p_uart) = unpack_payload("<BIBHHBBBB", payload)
        CURRENT_GYRO_SOURCE = gyro_source
        return (f"启动 t={ts} profile={profile} CPR={cpr} wheel={wheel_x100 / 100:.2f}mm "
                f"GYRO={GYRO_SOURCE_NAMES.get(gyro_source, gyro_source) if gyro_enabled else '关'} "
                f"IRQ=gpio{p_gpio}/uart{p_uart}")
    if event == 2:
        _, ts, key, state, selected = unpack_payload("<BIBBB", payload)
        return f"按键 t={ts} key={key} state={STATE_NAMES.get(state, state)} selected={selected}"
    if event == 3:
        old_format = "<BIBBBBIii"
        new_format = "<BIBBBBIiiii"
        if len(payload) == struct.calcsize(old_format):
            values = unpack_payload(old_format, payload)
            _, ts, status, address, who, i2c, samples, bias_x100, variance_x1000 = values
            range_text = ""
        else:
            values = unpack_payload(new_format, payload)
            (_, ts, status, address, who, i2c, samples, bias_x100,
             variance_x1000, min_gz_x1000, max_gz_x1000) = values
            range_text = (f" range=[{min_gz_x1000 / 1000:.3f},"
                          f"{max_gz_x1000 / 1000:.3f}]dps")
        return (f"航向初始化 t={ts} src={GYRO_SOURCE_NAMES.get(CURRENT_GYRO_SOURCE, '?')} "
                f"st={HEADING_NAMES.get(status, status)} addr=0x{address:02X} "
                f"who=0x{who:02X} i2c={I2C_NAMES.get(i2c, i2c)} cal={samples} "
                f"bias={bias_x100 / 100:.2f}dps var={variance_x1000 / 1000:.3f}{range_text}")
    if event == 4:
        _, ts, state, bits, position = unpack_payload("<BIBBh", payload)
        return f"循迹状态 t={ts} {LINE_STATE_NAMES.get(state, state)} bits=0x{bits:02X} pos={position}"
    if event == 5:
        _, ts, left, right, left_edges, right_edges = unpack_payload("<BIhhII", payload)
        left_text = "NO EDGE" if left < 0 else str(left)
        right_text = "NO EDGE" if right < 0 else str(right)
        return (f"电机死区 t={ts} left={left_text} right={right_text} "
                f"edges={left_edges},{right_edges}")
    return f"未知事件 id={event} payload={payload.hex()}"


def parse_run(payload):
    old_format = "<IBBhhhhBhHIIIIIIIIIHHII"
    route_format = "<IBBhhhhBhHIIIIIIIIIHHIIBhhhIIB"
    tuning_format = "<IBBhhhhBhHIIIIIIIIIHHIIBhhhIIBii"
    diagnostic_format = tuning_format + "BBIIII"
    irq_diagnostic_format = diagnostic_format + "BBII"
    poll_diagnostic_format = diagnostic_format + "BII"
    diagnostic_text = ""
    if len(payload) == struct.calcsize(old_format):
        values = unpack_payload(old_format, payload)
        route_text = ""
    else:
        if len(payload) == struct.calcsize(route_format):
            values = unpack_payload(route_format, payload)
            tuning_text = ""
        elif len(payload) == struct.calcsize(tuning_format):
            values = unpack_payload(tuning_format, payload)
            line_kp_x100, line_kd_x100 = values[-2:]
            tuning_text = (f" linePD={line_kp_x100 / 100:.2f}/"
                           f"{line_kd_x100 / 100:.2f}")
            values = values[:-2]
        elif len(payload) == struct.calcsize(diagnostic_format):
            values = unpack_payload(diagnostic_format, payload)
            ab_l, ab_r, edges_l, edges_r, gpio_irq, gpio_events = values[-6:]
            diagnostic_text = (f" ab=L{ab_l >> 1 & 1}{ab_l & 1}/"
                               f"R{ab_r >> 1 & 1}{ab_r & 1} "
                               f"edges={edges_l},{edges_r} "
                               f"GPIO irq/evt={gpio_irq}/{gpio_events}")
            values = values[:-6]
            line_kp_x100, line_kd_x100 = values[-2:]
            tuning_text = (f" linePD={line_kp_x100 / 100:.2f}/"
                           f"{line_kd_x100 / 100:.2f}")
            values = values[:-2]
        elif len(payload) == struct.calcsize(irq_diagnostic_format):
            values = unpack_payload(irq_diagnostic_format, payload)
            primask, gpio_nvic, imask_a, imask_b = values[-4:]
            values = values[:-4]
            ab_l, ab_r, edges_l, edges_r, gpio_irq, gpio_events = values[-6:]
            diagnostic_text = (f" ab=L{ab_l >> 1 & 1}{ab_l & 1}/"
                               f"R{ab_r >> 1 & 1}{ab_r & 1} "
                               f"edges={edges_l},{edges_r} "
                               f"GPIO irq/evt={gpio_irq}/{gpio_events} "
                               f"IRQcfg=P{primask}/N{gpio_nvic} "
                               f"IMASK=A{imask_a:08X}/B{imask_b:08X}")
            values = values[:-6]
            line_kp_x100, line_kd_x100 = values[-2:]
            tuning_text = (f" linePD={line_kp_x100 / 100:.2f}/"
                           f"{line_kd_x100 / 100:.2f}")
            values = values[:-2]
        else:
            values = unpack_payload(poll_diagnostic_format, payload)
            diag_version, poll_l, poll_r = values[-3:]
            values = values[:-3]
            ab_l, ab_r, edges_l, edges_r, gpio_irq, gpio_events = values[-6:]
            diagnostic_text = (f" ab=L{ab_l >> 1 & 1}{ab_l & 1}/"
                               f"R{ab_r >> 1 & 1}{ab_r & 1} "
                               f"edges={edges_l},{edges_r} "
                               f"GPIO irq/evt={gpio_irq}/{gpio_events} "
                               f"poll{diag_version}={poll_l},{poll_r}")
            values = values[:-6]
            line_kp_x100, line_kd_x100 = values[-2:]
            tuning_text = (f" linePD={line_kp_x100 / 100:.2f}/"
                           f"{line_kd_x100 / 100:.2f}")
            values = values[:-2]
        (h_state, yaw, target, error, line_enter, line_exit,
         stable_on_line) = values[-7:]
        route_text = (f" H={H2024_STATE_NAMES.get(h_state, h_state)} "
                      f"yaw/target/error={yaw / 100:.2f}/{target / 100:.2f}/{error / 100:.2f} "
                      f"event={line_enter}/{line_exit} on={stable_on_line}"
                      f"{tuning_text}")
        values = values[:-7]
    (ts, state, mode, rpm_l, rpm_r, cmd_l, cmd_r, bits, pos, bat_mv,
     control_dt, skipped, qerr_l, qerr_r, line_age, rpm_age, overruns,
     gpio_limit, uart_limit, tx_pending, tx_high, tx_rejected,
     tx_dropped) = values
    return (f"运行 t={ts} state={STATE_NAMES.get(state, state)} mode={mode} "
            f"rpm={rpm_l},{rpm_r} cmd={cmd_l},{cmd_r} line=0x{bits:02X}/{pos} "
            f"bat={bat_mv}mV dt={control_dt}ms age(line/rpm)={line_age}/{rpm_age}ms "
            f"skip={skipped} over={overruns} qerr={qerr_l},{qerr_r} "
            f"ISRlimit(gpio/uart)={gpio_limit}/{uart_limit} "
            f"TX pending/high/reject/drop={tx_pending}/{tx_high}/{tx_rejected}/{tx_dropped}"
            f"{route_text}{diagnostic_text}")


def parse_sensor(payload):
    old_format = "<IBBHHBBhIbBBhhiiIIhh"
    new_format = "<IBBHHBBhIbBBhhiiIIhhBB"
    if len(payload) == struct.calcsize(old_format):
        values = unpack_payload(old_format, payload)
        ab_text = ""
    else:
        values = unpack_payload(new_format, payload)
        ab_l, ab_r = values[-2:]
        ab_text = f" ab=L{ab_l >> 1 & 1}{ab_l & 1}/R{ab_r >> 1 & 1}{ab_r & 1}"
        values = values[:-2]
    (ts, state, mode, bat_raw, bat_mv, bat_status, bits, pos, line_age,
     t8_status, i2c, failures, rpm_l, rpm_r, enc_l, enc_r, qerr_l,
     qerr_r, cmd_l, cmd_r) = values
    return (f"传感 t={ts} state={STATE_NAMES.get(state, state)} mode={mode} "
            f"bat={bat_raw}/{bat_mv}mV st={bat_status} line=0x{bits:02X}/{pos} age={line_age}ms "
            f"T8={T8_NAMES.get(t8_status, t8_status)} i2c={I2C_NAMES.get(i2c, i2c)} fail={failures} "
            f"rpm={rpm_l},{rpm_r} enc={enc_l},{enc_r} qerr={qerr_l},{qerr_r} "
            f"cmd={cmd_l},{cmd_r}{ab_text}")


def parse_imu(payload):
    values = unpack_payload("<IBBBBiIIiiiiiiiii", payload)
    (ts, enabled, status, who, i2c, age, samples, errors, ax, ay, az,
     gx, gy, gz, temp, yaw, wz) = values
    return (f"GYRO t={ts} src={GYRO_SOURCE_NAMES.get(CURRENT_GYRO_SOURCE, '?')} "
            f"enabled={enabled} st={HEADING_NAMES.get(status, status)} who=0x{who:02X} "
            f"i2c={I2C_NAMES.get(i2c, i2c)} age={age}ms n={samples} err={errors} "
            f"A={ax},{ay},{az}mg G={gx},{gy},{gz}mdps T={temp / 100:.2f}C "
            f"yaw={yaw / 100:.2f}deg wz={wz / 100:.2f}dps")


def parse_timing(payload):
    values = unpack_payload("<IIIIIIIBIBIIIIIIHHII", payload)
    (ts, control_dt, control_min, control_max, rpm_age, skipped, overruns,
     late_index, late_ms, runtime_index, runtime_ms, gpio_irq, gpio_events,
     gpio_limit, uart_irq, uart_limit, tx_pending, tx_high, tx_rejected,
     tx_dropped) = values
    return (f"时序 t={ts} ctrl={control_dt}/{control_min}/{control_max}ms rpm_age={rpm_age}ms "
            f"skip={skipped} over={overruns} worst_late={task_name(late_index)}:{late_ms}ms "
            f"worst_run={task_name(runtime_index)}:{runtime_ms}ms "
            f"GPIO irq/evt/limit={gpio_irq}/{gpio_events}/{gpio_limit} "
            f"UART irq/limit={uart_irq}/{uart_limit} "
            f"TX pending/high/reject/drop={tx_pending}/{tx_high}/{tx_rejected}/{tx_dropped}")


def parse_fault(payload):
    values = unpack_payload("<IBBBBIhhHHBBhIbBBhhiiIIBBIii", payload)
    (ts, reason, mode, previous_state, armed, cmd_age, cmd_l, cmd_r,
     bat_raw, bat_mv, bat_status, bits, pos, line_age, t8_status, t8_i2c,
     t8_failures, rpm_l, rpm_r, enc_l, enc_r, qerr_l, qerr_r, gyro_status,
     gyro_i2c, gyro_age, yaw, wz) = values
    return (f"故障 t={ts} reason={FAULT_NAMES.get(reason, reason)} mode={mode} "
            f"prev={STATE_NAMES.get(previous_state, previous_state)} armed={armed} cmd_age={cmd_age}ms "
            f"cmd={cmd_l},{cmd_r} bat={bat_raw}/{bat_mv}mV st={bat_status} "
            f"line=0x{bits:02X}/{pos} age={line_age}ms T8={T8_NAMES.get(t8_status, t8_status)} "
            f"i2c={I2C_NAMES.get(t8_i2c, t8_i2c)} fail={t8_failures} rpm={rpm_l},{rpm_r} "
            f"enc={enc_l},{enc_r} qerr={qerr_l},{qerr_r} gyro={HEADING_NAMES.get(gyro_status, gyro_status)} "
            f"gyro_i2c={I2C_NAMES.get(gyro_i2c, gyro_i2c)} age={gyro_age}ms "
            f"yaw={yaw / 100:.2f} wz={wz / 100:.2f}")


PARSERS = {
    0x01: parse_debug,
    0x02: parse_tune,
    0x03: parse_cmd_response,
    0x10: parse_event,
    0x11: parse_run,
    0x12: parse_sensor,
    0x13: parse_imu,
    0x14: parse_timing,
    0x15: parse_fault,
}


def find_frame(buffer):
    header = buffer.find(b"\xAA\x55")
    if header < 0:
        if len(buffer) > 1:
            del buffer[:-1]
        return None
    if header:
        del buffer[:header]
    if len(buffer) < 4:
        return None
    length = buffer[3]
    frame_length = 4 + length
    if len(buffer) < frame_length:
        return None
    frame_type = buffer[2]
    payload = bytes(buffer[4:frame_length])
    del buffer[:frame_length]
    return frame_type, payload


def send_tune(port, param_id, value):
    port.write(struct.pack("<BBf", 0xCC, param_id, value))
    port.flush()


def encode_motor_command(left, right):
    if not -10000 <= left <= 10000 or not -10000 <= right <= 10000:
        raise ValueError("cmd范围必须是 -10000..10000")
    packet = bytearray(struct.pack("<BBhh", 0xCD, 0x4D, left, right))
    packet.append(sum(packet) & 0xFF)
    return packet


def send_motor_command(port, left, right):
    port.write(encode_motor_command(left, right))
    port.flush()


def parse_motor_command(command):
    parts = command.split()
    if parts == ["stop"]:
        return 0, 0
    if not parts or parts[0] != "cmd" or len(parts) not in (2, 3):
        raise ValueError(f"未知命令: {command}")
    left = int(parts[1], 10)
    right = left if len(parts) == 2 else int(parts[2], 10)
    if not -10000 <= left <= 10000 or not -10000 <= right <= 10000:
        raise ValueError("cmd范围必须是 -10000..10000")
    return left, right


def decode_frame(frame_type, payload):
    parser = PARSERS.get(frame_type)
    try:
        return parser(payload) if parser else f"未知帧 type=0x{frame_type:02X} payload={payload.hex()}"
    except (ValueError, struct.error) as error:
        return f"帧解析失败 type=0x{frame_type:02X}: {error}; {payload.hex()}"


def parse_tune_command(command):
    parts = command.split()
    if len(parts) != 2 or parts[0] not in PARAM_MAP:
        raise ValueError(f"未知命令: {command}")
    param_id = PARAM_MAP[parts[0]]
    if param_id in (14, 15):
        value = 1.0 if parts[1].lower() in ("on", "1", "true") else 0.0
    else:
        value = float(parts[1])
    return param_id, value


def run_cli(port_name, baud):
    is_windows = os.name == "nt"
    if is_windows:
        import msvcrt

    with serial.Serial(port_name, baud, timeout=0.1) as port:
        buffer = bytearray()
        command_chars = []
        manual_cmd = None
        last_manual_send = 0.0

        def redraw_prompt():
            if is_windows:
                print(f"\r\033[2K> {''.join(command_chars)}", end="", flush=True)

        def print_cli_line(text):
            if is_windows:
                print("\r\033[2K", end="")
            print(text)
            redraw_prompt()

        print(f"打开 {port_name}@{baud}")
        print("命令: cmd <左右> | cmd <左> <右> | stop | kp <值> | kd <值> | q")
        print("CMD范围 -10000..10000，负值为当前车体前进方向，仅允许菜单停止状态\n")
        redraw_prompt()
        while True:
            if manual_cmd is not None and time.monotonic() - last_manual_send >= 0.1:
                send_motor_command(port, *manual_cmd)
                last_manual_send = time.monotonic()
            if port.in_waiting:
                buffer.extend(port.read(port.in_waiting))
                while True:
                    frame = find_frame(buffer)
                    if frame is None:
                        break
                    frame_type, payload = frame
                    text = decode_frame(frame_type, payload)
                    if frame_type == 0x03 and payload and payload[0] != 0:
                        manual_cmd = None
                    color = W if frame_type == 0x15 else Y if frame_type == 0x02 else G
                    print_cli_line(f"{color}{text}{R}")

            command = None
            if is_windows:
                while msvcrt.kbhit():
                    char = msvcrt.getwch()
                    if char in ("\x00", "\xe0"):
                        if msvcrt.kbhit():
                            _ = msvcrt.getwch()
                        continue
                    if char == "\x03":
                        command = "q"
                        command_chars.clear()
                        break
                    if char in ("\r", "\n"):
                        entered = "".join(command_chars)
                        command = entered.strip()
                        print(f"\r\033[2K> {entered}")
                        command_chars.clear()
                        break
                    if char == "\b":
                        if command_chars:
                            command_chars.pop()
                            redraw_prompt()
                        continue
                    if char.isprintable():
                        command_chars.append(char)
                        redraw_prompt()
                if command is None:
                    time.sleep(0.01)
                    continue
            else:
                ready, _, _ = select.select([sys.stdin], [], [], 0.1)
                if not ready:
                    continue
                command = input("> ").strip()
            if command == "q":
                manual_cmd = None
                send_motor_command(port, 0, 0)
                break
            if command in ("cls", "clear"):
                os.system("cls" if os.name == "nt" else "clear")
                redraw_prompt()
                continue
            if not command:
                redraw_prompt()
                continue
            try:
                if command == "stop" or command.startswith("cmd "):
                    left, right = parse_motor_command(command)
                    send_motor_command(port, left, right)
                    manual_cmd = None if left == 0 and right == 0 else (left, right)
                    last_manual_send = time.monotonic()
                    print_cli_line(f"  -> 远程CMD left={left} right={right}")
                else:
                    param_id, value = parse_tune_command(command)
                    send_tune(port, param_id, value)
                    print_cli_line(f"  -> {PARAM_NAMES[param_id]}={value}")
            except (ValueError, serial.SerialException, OSError) as error:
                print_cli_line(error)
        if is_windows:
            print("\r\033[2K", end="")


class SplitWindowApp:
    def __init__(self, port_name, baud):
        import tkinter as tk
        from tkinter import scrolledtext

        self.tk = tk
        self.port = serial.Serial(port_name, baud, timeout=0)
        self.buffer = bytearray()
        self.latest_tasks = "等待任务统计..."
        self.latest_timing = "等待时序数据..."
        self.manual_cmd = None
        self.last_manual_send = 0.0

        self.root = tk.Tk()
        self.root.title(f"MSPM0 事件与调参 - {port_name}@{baud}")
        self.root.protocol("WM_DELETE_WINDOW", self.close)

        screen_w = self.root.winfo_screenwidth()
        screen_h = self.root.winfo_screenheight()
        window_w = max(480, screen_w // 2 - 30)
        window_h = max(180, (screen_h - 110) // 3)

        self.root.geometry(f"{window_w}x{window_h}+10+10")
        self.status = tk.StringVar(value=f"已连接 {port_name}@{baud}")
        tk.Label(self.root, textvariable=self.status, anchor="w").pack(fill="x", padx=8, pady=4)

        command_row = tk.Frame(self.root)
        command_row.pack(fill="x", padx=8)
        tk.Label(command_row, text="命令:").pack(side="left")
        self.command = tk.Entry(command_row)
        self.command.pack(side="left", fill="x", expand=True, padx=6)
        self.command.bind("<Return>", self.send_command)
        tk.Button(command_row, text="发送", command=self.send_command).pack(side="right")

        self.event_text = scrolledtext.ScrolledText(
            self.root, wrap="word", font=("Consolas", 10), height=10)
        self.event_text.pack(fill="both", expand=True, padx=8, pady=8)
        self.event_text.tag_configure("fault", foreground="#c00000")
        self.event_text.tag_configure("tune", foreground="#9a6700")

        self.views = {}
        self.views["run"] = self.create_view(
            "运行状态", window_w, window_h, window_w + 20, 10)
        self.views["sensor"] = self.create_view(
            "传感器", window_w, window_h, 10, window_h + 45)
        self.views["imu"] = self.create_view(
            "IMU", window_w, window_h, window_w + 20, window_h + 45)
        self.views["timing"] = self.create_view(
            "调度与时序", window_w, window_h, 10, 2 * window_h + 80)

        self.append_event(f"已连接 {port_name}@{baud}")
        self.append_event("命令示例: cmd -3000 | cmd -3000 -3200 | stop | kp 400")
        self.root.after(20, self.poll_serial)

    def create_view(self, title, width, height, x, y):
        window = self.tk.Toplevel(self.root)
        window.title(title)
        window.geometry(f"{width}x{height}+{x}+{y}")
        window.protocol("WM_DELETE_WINDOW", window.iconify)
        text = self.tk.Text(window, wrap="word", font=("Consolas", 11))
        text.pack(fill="both", expand=True, padx=8, pady=8)
        text.configure(state="disabled")
        return text

    @staticmethod
    def replace_text(widget, text):
        widget.configure(state="normal")
        widget.delete("1.0", "end")
        widget.insert("1.0", text)
        widget.configure(state="disabled")

    def append_event(self, text, tag=None):
        self.event_text.insert("end", text + "\n", tag or ())
        self.event_text.see("end")

    def dispatch(self, frame_type, payload):
        text = decode_frame(frame_type, payload)
        if frame_type == 0x03 and payload and payload[0] != 0:
            self.manual_cmd = None
        if frame_type == 0x11:
            self.replace_text(self.views["run"], text)
        elif frame_type == 0x12:
            self.replace_text(self.views["sensor"], text)
        elif frame_type == 0x13:
            self.replace_text(self.views["imu"], text)
        elif frame_type == 0x01:
            self.latest_tasks = text
            self.replace_text(self.views["timing"], self.latest_tasks + "\n\n" + self.latest_timing)
        elif frame_type == 0x14:
            self.latest_timing = text
            self.replace_text(self.views["timing"], self.latest_tasks + "\n\n" + self.latest_timing)
        else:
            self.append_event(text, "fault" if frame_type == 0x15 else "tune" if frame_type in (0x02, 0x03) else None)
            if frame_type == 0x15:
                self.root.deiconify()
                self.root.lift()

    def poll_serial(self):
        try:
            if self.manual_cmd is not None and time.monotonic() - self.last_manual_send >= 0.1:
                send_motor_command(self.port, *self.manual_cmd)
                self.last_manual_send = time.monotonic()
            waiting = self.port.in_waiting
            if waiting:
                self.buffer.extend(self.port.read(waiting))
                while True:
                    frame = find_frame(self.buffer)
                    if frame is None:
                        break
                    self.dispatch(*frame)
        except (serial.SerialException, OSError) as error:
            self.status.set(f"串口错误: {error}")
            self.append_event(f"串口错误: {error}", "fault")
            return
        self.root.after(20, self.poll_serial)

    def send_command(self, _event=None):
        command = self.command.get().strip()
        if not command:
            return
        if command == "q":
            self.close()
            return
        try:
            if command == "stop" or command.startswith("cmd "):
                left, right = parse_motor_command(command)
                send_motor_command(self.port, left, right)
                self.manual_cmd = None if left == 0 and right == 0 else (left, right)
                self.last_manual_send = time.monotonic()
                self.append_event(f"发送远程CMD left={left} right={right}", "tune")
            else:
                param_id, value = parse_tune_command(command)
                send_tune(self.port, param_id, value)
                self.append_event(f"发送调参 {PARAM_NAMES[param_id]}={value}", "tune")
            self.command.delete(0, "end")
        except (ValueError, serial.SerialException, OSError) as error:
            self.append_event(str(error), "fault")

    def close(self):
        if self.port.is_open:
            try:
                send_motor_command(self.port, 0, 0)
            except (serial.SerialException, OSError):
                pass
            self.port.close()
        self.root.destroy()

    def run(self):
        self.root.mainloop()


def main():
    parser = argparse.ArgumentParser(description="MSPM0 line-car debug parser")
    parser.add_argument("port", help="serial port, for example COM22")
    parser.add_argument("baud", nargs="?", type=int, default=115200)
    parser.add_argument("--gui", action="store_true", help="show split GUI windows")
    args = parser.parse_args()

    if args.gui:
        SplitWindowApp(args.port, args.baud).run()
    else:
        run_cli(args.port, args.baud)


if __name__ == "__main__":
    main()
