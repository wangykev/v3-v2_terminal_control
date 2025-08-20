# CubeMars V2/V3 Terminal Control (AK10-9, AK70-10, AK80-64)

Lightweight command-line tools to control and monitor CubeMars actuators over SocketCAN.

- V3 control (AK10-9): v3_control/mit_cansend.py — builds a MIT-mode frame and calls `cansend`.
- V2 control (AK70-10, AK80-64): v2_control/v2_ctrl.py — interactive MIT controller with start/exit/zero helpers.
- V2 state viewer: v2_control/v2_state.py — prints decoded position/velocity/torque, with optional unwrapped position.

---

## Requirements

- Linux with SocketCAN (e.g., Ubuntu/Raspberry Pi OS)
- Python 3.8+
- System tools: can-utils (for `cansend`)
- Python libs: python-can

Install:
```bash
sudo apt update
sudo apt install -y can-utils python3-pip
python3 -m pip install --upgrade python-can
```
Bring up your CAN interface (example: 1 Mbps on can0):
```bash
# Set bitrate and bring up
sudo ip link set can0 up type can bitrate 1000000
```

## V3 (AK10-9) — MIT Sender via cansend
Script: v3_control/mit_cansend.py
This script packs a MIT-mode frame for CubeMars V3 motors and executes a cansend command.
Ranges & Units (coded in the script)

## ID Mapping
```bash
0x801 + (motor_number - 1)
```

## Run
```bash
cd v3_control
python3 mit_cansend.py
```
Example interactive session (inputs shown after prompts):
```bash
Motor version? (v3): v3
Mode (position / velocity / torque / blended): position
Motor ID number (just the number, e.g., 1, 3, 4): 1
How many times to send this command?: 5
Target Position (rad): 1.5708
Kp (0-500): 50
Kd (0-5): 1.0
```
It will print and execute a cansend can0 00000801#... line 5 times (0.8s apart).

---

## V2 (AK70-10 / AK80-64) — Interactive Controller
Script: v2_control/v2_ctrl.py
Runs a background loop at the chosen control rate and accepts commands (set p/v/kp/kd/t, start/exit/zero, etc.).

Supported Motors & Ranges
AK70-10

P: −12.5 … +12.5 rad

V: −50.0 … +50.0 rad/s

T: −25.0 … +25.0 Nm

AK80-64

P: −12.5 … +12.5 rad

V: −8.0 … +8.0 rad/s

T: −144.0 … +144.0 Nm

Gains: Kp(0,500), Kd(0,5)

## Run
```bash
cd v2_control
python3 v2_ctrl.py --id 3 --motor AK70-10 --control-hz 200 --iface can0
```
--id is standard 11-bit CAN ID (decimal).

--control-hz is the streaming rate (Hz).
When it starts, you’ll get an interactive prompt:

Commands (type at the > prompt)
set <p> <v> <kp> <kd> <t> — set all MIT params and send once
(units: rad, rad/s, –, –, Nm)

p|v|kp|kd|t <val> — set a single parameter and send once

hz <rate> — change loop rate

start — send FF FF FF FF FF FF FF FC (enter control), resume streaming

exit — send FF FF FF FF FF FF FF FD (exit control), pause streaming

zero — send one zero MIT frame, then FF FF FF FF FF FF FF FE (set current pos = 0)

clear — set all MIT params to 0 and send once

status — print current values/rate/pause state

help — show help

quit — exit

---

## V2 State Viewer (decode position/velocity/torque)
Continuously prints the most recent state frame from the specified motor. Optional features:

--unwrap — accumulates turns so position becomes continuous across ±π wrap.


## Run
```bash
cd v2_control
python3 v2_state.py --id 3 --motor AK70-10 --print-hz 20 --unwrap --iface can0
```

Output format (examples)
```bash
STATE p= +0.123 rad  v= +0.456 rad/s  τ= +0.789 Nm  T=  34°C  err=0x00
STATE p= +0.500 rad  p_abs= +6.783 rad  v= +0.010 rad/s  τ= +0.020 Nm  T=  36°C  err=0x00
```

Where:

p is reported position (wrapped to the motor’s range).

p_abs appears when --unwrap is enabled (turns accumulated).

v is velocity (rad/s), τ is torque (Nm), T is temperature (°C), err is error byte.

---

## Tips & Troubleshooting
Tips & Troubleshooting
No frames seen / permission denied: Try sudo, or add your user to the can/netdev group and re-login.

Wrong bitrate: Ensure the actuator and can0 use the same bitrate (examples above use 1,000,000).

Bus off: Check wiring (CAN_H/CAN_H, CAN_L/CAN_L, proper GND), termination (typically 120 Ω at both ends).

Extended vs Standard IDs:

V3 script uses extended (29-bit) IDs by emitting 8 hex digits before # in cansend.

V2 scripts use standard (11-bit) IDs via python-can.

Aggressive gains: Start with low Kp/Kd, increase gradually.

Unwrap jumps: If your position appears to “jump,” enable --unwrap in v2_state.py.

---

## Quick Start Cheatsheet
```bash
# 1) Bring up CAN (example: 1 Mbps)
sudo ip link set can0 up type can bitrate 1000000

# 2) Control a V2 AK70-10 @ ID 3
cd v2_control
python3 v2_ctrl.py --id 3 --motor AK70-10 --control-hz 200
# In the prompt:
# > start
# > zero
# > set 0 0 40 1.0 0
# > exit
# > quit

# 3) Watch state of that motor
python3 v2_state.py --id 3 --motor AK70-10 --print-hz 20 --poll --unwrap

# 4) Send a single V3 AK10-9 position command via cansend (ID=1)
cd ../v3_control
python3 mit_cansend.py
# Provide: v3, position, 1, repeats=3, then enter p/kp/kd when prompted
```






