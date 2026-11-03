#!/usr/bin/env python3
"""Set the ESP32 kill-switch indicator (indikator_banyu.ino) without banyu_teleop.

Same protocol banyu_teleop uses: newline-terminated lines, mapped from operational_mode
(issue #15) to the board's own numbering:
  kill   operational_mode 0 -> "its2"  EMERGENCY LED
  remote operational_mode 1 -> "its0"  MANUAL LED
  auto   operational_mode 2 -> "its1"  AUTO LED
The board has no relay: thrusters are cut by its physical kill input, during which it ignores
its0/its1 and reports its2. It reports its state every 500 ms; this script prints the reports.

Usage:
  python3 tools/scripts/debug/esp32_kill_cmd.py auto --hold      # stays running, Ctrl+C to leave
  python3 tools/scripts/debug/esp32_kill_cmd.py kill             # one shot
Run it only when banyu_teleop / esp32_trigger_node are NOT running (both would own the port).
"""
import argparse
import os
import signal
import sys
import termios
import time

MODES = {"kill": b"its2\n", "remote": b"its0\n", "auto": b"its1\n"}


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("mode", choices=sorted(MODES))
    p.add_argument("--port", default="/dev/esp32_kill")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--boot-wait", type=float, default=0.5,
                   help="seconds to wait after opening (the native-USB board does not reboot on open)")
    p.add_argument("--repeat", type=int, default=3, help="send the line this many times (0.3 s apart)")
    p.add_argument("--hold", action="store_true", help="keep the port open until Ctrl+C")
    a = p.parse_args()

    baud = getattr(termios, f"B{a.baud}", None)
    if baud is None:
        sys.exit(f"unsupported baud {a.baud}")
    fd = os.open(a.port, os.O_RDWR | os.O_NOCTTY)
    attrs = termios.tcgetattr(fd)
    iflag, oflag, cflag, lflag, ispeed, ospeed, cc = attrs
    iflag = 0
    oflag = 0
    lflag = 0
    cflag = (cflag & ~(termios.CSIZE | termios.PARENB | termios.CSTOPB | termios.HUPCL)) \
        | termios.CS8 | termios.CLOCAL | termios.CREAD
    cc[termios.VMIN], cc[termios.VTIME] = 0, 1
    termios.tcsetattr(fd, termios.TCSANOW, [iflag, oflag, cflag, lflag, baud, baud, cc])
    termios.tcflush(fd, termios.TCIOFLUSH)
    print(f"opened {a.port} at {a.baud}", flush=True)
    time.sleep(a.boot_wait)
    for _ in range(max(1, a.repeat)):
        os.write(fd, MODES[a.mode])
        time.sleep(0.3)
    print(f"sent {MODES[a.mode].decode().strip()} ({a.mode}) x{max(1, a.repeat)}", flush=True)
    time.sleep(0.6)
    reports = os.read(fd, 256).decode("ascii", errors="ignore").split()
    print(f"board reports: {reports[-1] if reports else 'nothing'}"
          + (" (hardware kill active?)" if reports and reports[-1] == "its2" and a.mode != "kill" else ""),
          flush=True)
    if a.hold:
        print("holding the port open; Ctrl+C to release (state is kept, nothing is sent on exit)", flush=True)
        signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
        try:
            while True:
                time.sleep(1.0)
        except (KeyboardInterrupt, SystemExit):
            pass
    os.close(fd)


if __name__ == "__main__":
    main()
