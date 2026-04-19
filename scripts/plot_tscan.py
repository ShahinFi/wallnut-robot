import argparse
import math
import os
import sys
import time


def parse_tscan_line(line: str):
    # Sample line:
    # - thin: TSCAN:angleDeg,distanceCm
    # - debug: TSCAN:seq,angleDeg,distanceCm,ticksAbs,ms
    if not line.startswith("TSCAN:"):
        return None
    payload = line[len("TSCAN:") :].strip()
    if payload.startswith("BEGIN") or payload.startswith("DONE") or payload.startswith("CANCEL"):
        return ("event", payload)

    try:
        parts = payload.split(",")
        if len(parts) == 2:
            angle_deg = float(parts[0])
            dist_cm = float(parts[1])
            return ("sample", (0, angle_deg, dist_cm, 0, 0))
        if len(parts) == 5:
            seq = int(parts[0])
            angle_deg = float(parts[1])
            dist_cm = float(parts[2])
            ticks_abs = int(parts[3])
            ms = int(parts[4])
            return ("sample", (seq, angle_deg, dist_cm, ticks_abs, ms))
        return None
    except ValueError:
        return None


def main():
    ap = argparse.ArgumentParser(description="Live polar plot for Mega turret scan output (TSCAN).")
    ap.add_argument("--port", required=True, help="Serial port, e.g. COM7")
    ap.add_argument("--baud", type=int, default=115200, help="Baud rate (default: 115200)")
    ap.add_argument("--max", type=int, default=3000, help="Max points to keep (default: 3000)")
    ap.add_argument("--fps", type=float, default=15.0, help="Plot refresh rate (default: 15)")
    args = ap.parse_args()

    try:
        import serial  # type: ignore
    except Exception:
        print("Missing dependency: pyserial. Install with: pip install pyserial", file=sys.stderr)
        return 2

    try:
        import matplotlib.pyplot as plt  # type: ignore
    except Exception:
        print("Missing dependency: matplotlib. Install with: pip install matplotlib", file=sys.stderr)
        return 2

    try:
        import numpy as np  # type: ignore
    except Exception:
        print("Missing dependency: numpy (required by matplotlib).", file=sys.stderr)
        return 2

    ser = serial.Serial(args.port, args.baud, timeout=0.1)

    plt.ion()
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="polar")
    ax.set_theta_zero_location("N")  # 0 deg at North
    ax.set_theta_direction(-1)       # degrees increase clockwise
    ax.set_title("TSCAN polar plot (angle deg, distance cm)")
    plt.show(block=False)

    # Allow controlling the scan even when the plot window has focus.
    def _on_key(event):
        k = (event.key or "").lower()
        # Forward a small, explicit set of single-key commands to Mega.
        # (Keep this list tight to avoid accidental motion commands.)
        if k in ("p", "i", "x", "u", "j", "o", "l", "b", "n", "v", "r"):
            ser.write(k.encode("ascii", errors="ignore"))
            try:
                ser.flush()
            except Exception:
                pass
            print(f"[sent] {k}")
        elif k in ("c",):
            # local plot clear
            nonlocal_data["clear_requested"] = True
            print("[plot] cleared")

    fig.canvas.mpl_connect("key_press_event", _on_key)

    plus_thetas = []
    plus_rs = []
    minus_thetas = []
    minus_rs = []

    sc_plus = ax.scatter([], [], s=6, c="#e11d48", label="+ sweep")
    sc_minus = ax.scatter([], [], s=6, c="#2563eb", label="- sweep")
    ax.legend(loc="upper right")

    nonlocal_data = {
        "scan_dir": None,  # "+", "-" or None
        "clear_requested": False,
    }

    last_draw = 0.0
    draw_period = 1.0 / max(args.fps, 1.0)

    print("Listening for TSCAN... Ctrl+C to quit.")
    print("Hotkeys (sent to Mega): p=scan+, i=scan-, x=cancel, u/j=pulse +/- , o/l=1rev +/-")
    print("Mapping hotkeys (sent to Mega): b=map+scan, n=map-scan, v=print map, r=reset map")
    print("Plot hotkey: c=clear")
    print("You can also type a full line and press Enter (example: @tpr 200)")

    is_windows = os.name == "nt"
    if is_windows:
        import msvcrt  # type: ignore
        line_buf = ""
    try:
        while True:
            # --- keyboard -> serial ---
            if is_windows:
                while msvcrt.kbhit():
                    ch = msvcrt.getwch()
                    if ch in ("\r", "\n"):
                        if line_buf:
                            ser.write((line_buf + "\n").encode("utf-8", errors="ignore"))
                            try:
                                ser.flush()
                            except Exception:
                                pass
                            print(f"[sent] {line_buf}")
                            line_buf = ""
                        break
                    if ch in ("\b", "\x7f"):
                        line_buf = line_buf[:-1]
                        continue
                    if ch in ("c", "C") and not line_buf:
                        nonlocal_data["clear_requested"] = True
                        print("[plot] cleared")
                        continue
                    if ch in ("p", "P", "i", "I", "x", "X", "u", "U", "j", "J", "o", "O", "l", "L",
                              "b", "B", "n", "N", "v", "V", "r", "R") and not line_buf:
                        ser.write(ch.encode("ascii", errors="ignore"))
                        try:
                            ser.flush()
                        except Exception:
                            pass
                        print(f"[sent] {ch.lower()}")
                        continue
                    if ch.isprintable():
                        line_buf += ch
            raw = ser.readline()
            if raw:
                try:
                    line = raw.decode("utf-8", errors="ignore").strip()
                except Exception:
                    line = ""

                parsed = parse_tscan_line(line)
                if parsed:
                    kind, val = parsed
                    if kind == "event":
                        print(f"[{time.strftime('%H:%M:%S')}] {val}")
                        if val.startswith("BEGIN"):
                            # BEGIN,+ or BEGIN,-
                            if "BEGIN,+" in val:
                                nonlocal_data["scan_dir"] = "+"
                            elif "BEGIN,-" in val:
                                nonlocal_data["scan_dir"] = "-"
                        elif val.startswith("DONE") or val.startswith("CANCEL"):
                            nonlocal_data["scan_dir"] = None
                    else:
                        _, angle_deg, dist_cm, _, _ = val
                        th = math.radians(angle_deg)
                        if nonlocal_data["scan_dir"] == "-":
                            minus_thetas.append(th)
                            minus_rs.append(dist_cm)
                            if len(minus_thetas) > args.max:
                                minus_thetas[:] = minus_thetas[-args.max :]
                                minus_rs[:] = minus_rs[-args.max :]
                        else:
                            # Default to "+" bucket when unknown.
                            plus_thetas.append(th)
                            plus_rs.append(dist_cm)
                            if len(plus_thetas) > args.max:
                                plus_thetas[:] = plus_thetas[-args.max :]
                                plus_rs[:] = plus_rs[-args.max :]
                else:
                    if line:
                        print(line)

            now = time.time()
            if now - last_draw >= draw_period:
                last_draw = now
                if nonlocal_data["clear_requested"]:
                    plus_thetas.clear()
                    plus_rs.clear()
                    minus_thetas.clear()
                    minus_rs.clear()
                    nonlocal_data["clear_requested"] = False

                # Update scatter in-place (more reliable than removing/re-adding).
                if plus_thetas:
                    sc_plus.set_offsets(np.column_stack((plus_thetas, plus_rs)))
                else:
                    sc_plus.set_offsets(np.empty((0, 2)))

                if minus_thetas:
                    sc_minus.set_offsets(np.column_stack((minus_thetas, minus_rs)))
                else:
                    sc_minus.set_offsets(np.empty((0, 2)))

                # Force a sane radial limit (autoscale can be flaky on polar axes).
                rmax = 1.0
                if plus_rs:
                    rmax = max(rmax, max(plus_rs))
                if minus_rs:
                    rmax = max(rmax, max(minus_rs))
                ax.set_rlim(0.0, rmax * 1.05)
                # plt.pause processes GUI events more reliably than flush_events on Windows.
                plt.pause(0.001)

    except KeyboardInterrupt:
        return 0
    finally:
        try:
            ser.close()
        except Exception:
            pass


if __name__ == "__main__":
    raise SystemExit(main())
