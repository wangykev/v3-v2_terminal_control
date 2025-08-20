#!/usr/bin/env python3
# Interactive MIT sender (control only) with motor presets + special commands.
#   start -> FF FF FF FF FF FF FF FC  (enter control)
#   exit  -> FF FF FF FF FF FF FF FD  (exit control, pause streaming)
#   zero  -> send MIT zeros once, then FF FF FF FF FF FF FF FE (set current pos as zero)
#
# Usage:
#   python3 v2_ctrl.py --id 3 --motor AK70-10 --control-hz 200
#   python3 v2_ctrl.py --id 3 --motor AK80-64 --control-hz 200
#
# Requires: pip install python-can ; bring up can0 at correct bitrate.

import time, threading, argparse, shlex
import can

MOTOR_PRESETS = {
    "AK70-10": {
        "P_MIN": -12.5, "P_MAX": 12.5,
        "V_MIN": -50.0, "V_MAX": 50.0,
        "T_MIN": -25.0, "T_MAX": 25.0,
        "KP_MIN": 0.0,  "KP_MAX": 500.0,
        "KD_MIN": 0.0,  "KD_MAX": 5.0,
    },
    "AK80-64": {
        "P_MIN": -12.5, "P_MAX": 12.5,
        "V_MIN": -8.00, "V_MAX": 8.00,
        "T_MIN": -144.0, "T_MAX": 144.0,
        "KP_MIN": 0.0,   "KP_MAX": 500.0,
        "KD_MIN": 0.0,   "KD_MAX": 5.0,
    },
}

def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x

def f2u(x, lo, hi, bits):
    x = clamp(x, lo, hi)
    return int((x - lo) * (((1 << bits) - 1) / (hi - lo)))

def pack_cmd(p, v, kp, kd, t, R):
    p_i  = f2u(p,  R["P_MIN"],  R["P_MAX"],  16)
    v_i  = f2u(v,  R["V_MIN"],  R["V_MAX"],  12)
    kp_i = f2u(kp, R["KP_MIN"], R["KP_MAX"], 12)
    kd_i = f2u(kd, R["KD_MIN"], R["KD_MAX"], 12)
    t_i  = f2u(t,  R["T_MIN"],  R["T_MAX"],  12)
    return bytes([
        (p_i >> 8) & 0xFF,
         p_i       & 0xFF,
        (v_i >> 4) & 0xFF,
        ((v_i & 0x0F) << 4) | ((kp_i >> 8) & 0x0F),
         kp_i      & 0xFF,
        (kd_i >> 4) & 0xFF,
        ((kd_i & 0x0F) << 4) | ((t_i >> 8) & 0x0F),
         t_i       & 0xFF,
    ])

def send_special(bus, arb, code_byte):
    data = b"\xFF\xFF\xFF\xFF\xFF\xFF\xFF" + bytes([code_byte & 0xFF])
    msg = can.Message(arbitration_id=arb, data=data, is_extended_id=False)
    bus.send(msg)

class Ctrl:
    def __init__(self, bus, arb, hz, ranges):
        self.bus = bus
        self.arb = arb
        self.hz = max(1.0, float(hz))
        self.R = ranges

        self.p = 0.0
        self.v = 0.0
        self.kp = 0.0
        self.kd = 0.0
        self.t = 0.0

        self.paused = False
        self._lock = threading.Lock()
        self._stop = False
        self._th = threading.Thread(target=self._loop, daemon=True)

    def start(self):
        self._th.start()

    def stop(self):
        self._stop = True
        self._th.join(timeout=0.3)

    def set_hz(self, hz):
        self.hz = max(1.0, float(hz))

    def set_cmd(self, **kw):
        with self._lock:
            if "p" in kw:  self.p  = clamp(float(kw["p"]),  self.R["P_MIN"],  self.R["P_MAX"])
            if "v" in kw:  self.v  = clamp(float(kw["v"]),  self.R["V_MIN"],  self.R["V_MAX"])
            if "kp" in kw: self.kp = clamp(float(kw["kp"]), self.R["KP_MIN"], self.R["KP_MAX"])
            if "kd" in kw: self.kd = clamp(float(kw["kd"]), self.R["KD_MIN"], self.R["KD_MAX"])
            if "t" in kw:  self.t  = clamp(float(kw["t"]),  self.R["T_MIN"],  self.R["T_MAX"])

    def send_now(self):
        if self.paused:
            return
        with self._lock:
            data = pack_cmd(self.p, self.v, self.kp, self.kd, self.t, self.R)
        try:
            self.bus.send(can.Message(arbitration_id=self.arb, data=data, is_extended_id=False))
        except can.CanError:
            pass

    def pause(self):
        self.paused = True

    def resume(self):
        self.paused = False

    def _loop(self):
        dt = 1.0 / self.hz
        nxt = time.time()
        while not self._stop:
            if not self.paused:
                with self._lock:
                    data = pack_cmd(self.p, self.v, self.kp, self.kd, self.t, self.R)
                try:
                    self.bus.send(can.Message(arbitration_id=self.arb, data=data, is_extended_id=False))
                except can.CanError:
                    pass
            nxt += dt
            time.sleep(max(0.0, nxt - time.time()))
            dt = 1.0 / self.hz

def help_text(ranges_name, R):
    return (
        f"Motor preset: {ranges_name}  "
        f"P[{R['P_MIN']}..{R['P_MAX']}], V[{R['V_MIN']}..{R['V_MAX']}], "
        f"T[{R['T_MIN']}..{R['T_MAX']}], Kp[0..{R['KP_MAX']}], Kd[0..{R['KD_MAX']}]\n\n"
        "Commands:\n"
        "  set <p> <v> <kp> <kd> <t>     set all MIT params (rad, rad/s, -, -, Nm)\n"
        "  p|v|kp|kd|t <val>             set individually\n"
        "  hz <rate>                     change control rate (Hz)\n"
        "  start                         send FF..FC and resume streaming\n"
        "  exit                          send FF..FD and pause streaming\n"
        "  zero                          send MIT zeros once, then FF..FE (set current pos=0)\n"
        "  clear                         set MIT params to 0 (no special frame)\n"
        "  status                        show current params/rate/pause\n"
        "  help                          show this help\n"
        "  quit                          exit\n"
    )

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--id", type=int, required=True, help="motor ID (11-bit std, decimal)")
    ap.add_argument("--motor", choices=sorted(MOTOR_PRESETS.keys()), required=True,
                    help="motor preset for range scaling")
    ap.add_argument("--iface", default="can0")
    ap.add_argument("--control-hz", type=float, default=200.0)
    args = ap.parse_args()

    R = MOTOR_PRESETS[args.motor]
    bus = can.interface.Bus(bustype="socketcan", channel=args.iface)
    arb = args.id & 0x7FF

    c = Ctrl(bus, arb, args.control_hz, R)
    c.start()

    print(help_text(args.motor, R), end="")
    print("> ", end="", flush=True)

    try:
        while True:
            line = input().strip()
            if not line:
                print("> ", end="", flush=True)
                continue
            toks = shlex.split(line)
            cmd = toks[0].lower()

            try:
                if cmd == "quit":
                    break
                elif cmd == "help":
                    print(help_text(args.motor, R), end="")
                elif cmd == "status":
                    print(f"p={c.p} v={c.v} kp={c.kp} kd={c.kd} t={c.t} | hz={c.hz} | paused={c.paused}")
                elif cmd == "hz" and len(toks) == 2:
                    c.set_hz(float(toks[1]))
                elif cmd == "set" and len(toks) == 6:
                    c.set_cmd(p=float(toks[1]), v=float(toks[2]), kp=float(toks[3]), kd=float(toks[4]), t=float(toks[5]))
                    c.send_now()
                elif cmd in ("p", "v", "kp", "kd", "t") and len(toks) == 2:
                    c.set_cmd(**{cmd: float(toks[1])})
                    c.send_now()
                elif cmd == "clear":
                    c.set_cmd(p=0, v=0, kp=0, kd=0, t=0)
                    c.send_now()
                    print("MIT params cleared to 0.")
                elif cmd == "start":
                    send_special(bus, arb, 0xFC)  # Enter control
                    c.resume()
                    print("Sent START (FF..FC), streaming resumed.")
                elif cmd == "exit":
                    send_special(bus, arb, 0xFD)  # Exit control
                    c.pause()
                    print("Sent EXIT (FF..FD), streaming paused.")
                elif cmd == "zero":
                    # Clear MIT params to 0, send once immediately, THEN send FE.
                    c.set_cmd(p=0, v=0, kp=0, kd=0, t=0)
                    c.send_now()
                    send_special(bus, arb, 0xFE)  # Set current pos = zero
                    print("Sent MIT zeros, then ZERO-POS (FF..FE).")
                else:
                    print("Unknown. Type 'help'.")
            except Exception as e:
                print(f"Error: {e}")

            print("> ", end="", flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        c.stop()

if __name__ == "__main__":
    main()
