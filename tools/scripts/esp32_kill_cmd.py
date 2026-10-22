#!/usr/bin/env python3
"""Send an operating-mode byte to the ESP32 kill-switch relay without banyu_teleop.

Same protocol banyu_teleop uses for the TX16S SB switch (single ASCII byte, no newline;
encoding from issue #15, equal to VehicleStatus.operational_mode):
  kill   '0'  disarmed / hardware kill (cuts relay power, LED red)
  remote '1'  remote / manual (LED yellow)
  auto   '2'  autonomous (LED green)

Opening a USB-serial port usually pulses DTR and reboots the ESP32, and the board
then only keeps its state while someone holds the port open (teleop normally does).
So this script waits for the ESP32 to boot, sends the byte, and with --hold keeps the
port open until Ctrl+C. HUPCL is cleared so closing the port does not toggle DTR again.

Usage:
  python3 tools/scripts/esp32_kill_cmd.py auto --hold      # stays running, Ctrl+C to leave
  python3 tools/scripts/esp32_kill_cmd.py kill             # one shot
Run it only when banyu_teleop is NOT running (both would own the port).
"""
import argparse
import os
import signal
import sys
import termios
import time

MODES = {"kill": b"0", "remote": b"1", "auto": b"2"}


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("mode", choices=sorted(MODES))
    p.add_argument("--port", default="/dev/ttyUSB0")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--boot-wait", type=float, default=2.5,
                   help="seconds to wait after opening, for the ESP32 to finish rebooting")
    p.add_argument("--repeat", type=int, default=3, help="send the byte this many times (0.3 s apart)")
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
    print(f"opened {a.port} at {a.baud}; waiting {a.boot_wait:.1f} s for the ESP32 to boot", flush=True)
    time.sleep(a.boot_wait)
    for _ in range(max(1, a.repeat)):
        os.write(fd, MODES[a.mode])
        time.sleep(0.3)
    print(f"sent {MODES[a.mode].decode()} ({a.mode}) x{max(1, a.repeat)}", flush=True)
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
