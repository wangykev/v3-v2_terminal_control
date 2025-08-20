#!/usr/bin/env python3
import time, argparse
import can

MOTOR_PRESETS = {
    "AK70-10": {"P_MIN":-12.5,"P_MAX":12.5,"V_MIN":-50.0,"V_MAX":50.0,"T_MIN":-25.0,"T_MAX":25.0},
    "AK80-64": {"P_MIN":-12.5,"P_MAX":12.5,"V_MIN":-8.00,"V_MAX":8.00,"T_MIN":-144.0,"T_MAX":144.0},
}

def u2f(u, lo, hi, bits):
    if u < 0: u = 0
    mx = (1 << bits) - 1
    if u > mx: u = mx
    return lo + (u / mx) * (hi - lo)

def parse_reply(b, R):
    if len(b) != 8: return None
    drv   = b[0]
    p_int = (b[1] << 8) | b[2]
    v_int = (b[3] << 4) | (b[4] >> 4)
    i_int = ((b[4] & 0x0F) << 8) | b[5]
    temp  = b[6]; err = b[7]
    p = u2f(p_int, R["P_MIN"], R["P_MAX"], 16)
    v = u2f(v_int, R["V_MIN"], R["V_MAX"], 12)
    t = u2f(i_int, R["T_MIN"], R["T_MAX"], 12)
    return drv, p, v, t, temp, err

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--id", type=int, required=True, help="11-bit std CAN ID (decimal)")
    ap.add_argument("--motor", choices=sorted(MOTOR_PRESETS.keys()), required=True)
    ap.add_argument("--iface", default="can0")
    ap.add_argument("--print-hz", type=float, default=20.0)
    ap.add_argument("--poll", action="store_true", help="send FF..FC each print tick")
    ap.add_argument("--unwrap", action="store_true", help="accumulate turns for continuous position")
    args = ap.parse_args()

    R = MOTOR_PRESETS[args.motor]
    bus = can.interface.Bus(bustype="socketcan", channel=args.iface)
    arb = args.id & 0x7FF
    bus.set_filters([{"can_id": arb, "can_mask": 0x7FF}])

    # unwrap state
    span = R["P_MAX"] - R["P_MIN"]
    last_p = None
    p_unw = 0.0

    period = 1.0 / max(1.0, args.print_hz)
    print(f"Watching ID 0x{arb:03X} ({args.motor}) @ {args.print_hz:.0f} Hz; poll={'ON' if args.poll else 'OFF'}; unwrap={'ON' if args.unwrap else 'OFF'}")
    try:
        while True:
            if args.poll:
                bus.send(can.Message(arbitration_id=arb,
                                     data=b"\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFC",
                                     is_extended_id=False))
            latest = None
            end = time.time() + 0.02
            while True:
                rx = bus.recv(timeout=max(0.0, end - time.time()))
                if rx is None: break
                if rx.arbitration_id != arb or len(rx.data) != 8: continue
                s = parse_reply(rx.data, R)
                if not s: continue
                drv, p, v, t, temp, err = s
                # ignore our own poll echoes or wrong-driver frames
                if drv != (arb & 0xFF): 
                    continue
                latest = (p, v, t, temp, err)
            if latest:
                p, v, t, temp, err = latest
                if args.unwrap:
                    if last_p is None:
                        p_unw = p
                    else:
                        dp = p - last_p
                        # unwrap: if jump crosses half-span, add/subtract a full span
                        if dp >  0.5*span:  dp -= span
                        if dp < -0.5*span:  dp += span
                        p_unw += dp
                    last_p = p
                    print(f"STATE p={p:+7.3f} rad  p_abs={p_unw:+8.3f} rad  v={v:+7.3f} rad/s  τ={t:+7.3f} Nm  T={temp:3d}°C  err=0x{err:02X}")
                else:
                    print(f"STATE p={p:+7.3f} rad  v={v:+7.3f} rad/s  τ={t:+7.3f} Nm  T={temp:3d}°C  err=0x{err:02X}")
            else:
                print("STATE (no reply)")
            time.sleep(period)
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()
