import os
import time

def float_to_uint(x, x_min, x_max, bits):
    span = x_max - x_min
    x_clipped = max(min(x, x_max), x_min)
    scale = (1 << bits) - 1
    return int((x_clipped - x_min) / span * scale)

# Formats and packs a MIT-mode CAN command for CubeMars V3 motors
def pack_cmd_v3(controller_id, p_des, v_des, kp, kd, t_ff):
    P_MIN, P_MAX = -12.56, 12.56
    V_MIN, V_MAX = -28.0, 28.0
    T_MIN, T_MAX = -54.0, 54.0
    KP_MIN, KP_MAX = 0.0, 500.0
    KD_MIN, KD_MAX = 0.0, 5.0

    p_int = float_to_uint(p_des, P_MIN, P_MAX, 16)
    v_int = float_to_uint(v_des, V_MIN, V_MAX, 12)
    kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12)
    kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 12)
    t_int = float_to_uint(t_ff, T_MIN, T_MAX, 12)

    data = [
        (kp_int >> 4) & 0xFF,
        ((kp_int & 0xF) << 4) | ((kd_int >> 8) & 0xF),
        kd_int & 0xFF,
        (p_int >> 8) & 0xFF,
        p_int & 0xFF,
        (v_int >> 4) & 0xFF,
        ((v_int & 0xF) << 4) | ((t_int >> 8) & 0xF),
        t_int & 0xFF
    ]

    hex_data = ''.join(f'{byte:02X}' for byte in data)
    return f'cansend can0 {controller_id:08X}#{hex_data}'

def send_command(cmd, repeat):
    print(f"Sending:\n{cmd}")
    for _ in range(repeat):
        os.system(cmd)
        time.sleep(0.8)

def position_loop(pack_func, controller_id, repeat):
    pos = float(input("Target Position (rad): "))
    kp = float(input("Kp (0-500): "))
    kd = float(input("Kd (0-5): "))
    cmd = pack_func(controller_id, pos, 0.0, kp, kd, 0.0)
    send_command(cmd, repeat)

def velocity_loop(pack_func, controller_id, repeat):
    vel = float(input("Target Velocity (rad/s): "))
    kd = float(input("Kd (0-5): "))
    cmd = pack_func(controller_id, 0.0, vel, 0.0, kd, 0.0)
    send_command(cmd, repeat)

def torque_loop(pack_func, controller_id, repeat):
    torque = float(input("Target Torque (Nm): "))
    cmd = pack_func(controller_id, 0.0, 0.0, 0.0, 1.0, torque)
    send_command(cmd, repeat)

def blended_loop(pack_func, controller_id, repeat):
    pos = float(input("Target Position (rad): "))
    vel = float(input("Target Velocity (rad/s): "))
    kp = float(input("Kp (0-500): "))
    kd = float(input("Kd (0-5): "))
    torque = float(input("Torque Feedforward (Nm): "))
    cmd = pack_func(controller_id, pos, vel, kp, kd, torque)
    send_command(cmd, repeat)

if __name__ == "__main__":
    version = input("Motor version? (v3): ").strip().lower()
    mode = input("Mode (position / velocity / torque / blended): ").strip().lower()
    raw_id = input("Motor ID number (just the number, e.g., 1, 3, 4): ").strip()

    if version != "v3":
        print("Only v3 supported in this script.")
        exit(1)

    controller_id = int("801", 16) + (int(raw_id) - 1)
    repeat = int(input("How many times to send this command?: "))
    pack_func = pack_cmd_v3

    if mode == "position":
        position_loop(pack_func, controller_id, repeat)
    elif mode == "velocity":
        velocity_loop(pack_func, controller_id, repeat)
    elif mode == "torque":
        torque_loop(pack_func, controller_id, repeat)
    elif mode == "blended":
        blended_loop(pack_func, controller_id, repeat)
    else:
        print("Invalid mode! Use 'position', 'velocity', 'torque', or 'blended'.")
