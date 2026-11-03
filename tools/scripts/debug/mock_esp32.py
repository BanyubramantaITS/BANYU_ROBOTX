#!/usr/bin/env python3
"""
Mock of the ESP32 kill-switch indicator (firmware indikator_banyu.ino) on a pseudo-terminal.

Same behaviour as the board:
- Accepts newline-terminated lines "its0" (MANUAL LED), "its1" (AUTO LED), "its2" (EMERGENCY LED).
- A hardware kill input forces EMERGENCY; its0/its1 are ignored while it is active, and on release
  the board returns to the state it had before.
- Reports its state as "itsN\\r\\n" every 500 ms.
banyu_teleop and esp32_trigger_node map operational_mode 0/1/2 to its2/its0/its1.

Usage:
  python3 mock_esp32.py [--pty-link /tmp/ttyESP32]
  then run teleop with -p serial_port:=/tmp/ttyESP32
Keys: 'k' + Enter toggles the hardware kill, 'q' + Enter exits.
"""

import argparse
import os
import pty
import select
import sys
import termios
import threading
import time

STATE_NAMES = {0: "its0 (MANUAL)", 1: "its1 (AUTO)", 2: "its2 (EMERGENCY)"}


class MockIndicator:
    def __init__(self, pty_link="/tmp/ttyESP32"):
        self.pty_link = pty_link
        self.state = 0
        self.last_state = 0
        self.hw_kill = False
        self.running = True
        self.master_fd = None
        self.slave_fd = None
        self.lock = threading.Lock()

    def setup_pty(self):
        self.master_fd, self.slave_fd = pty.openpty()
        slave_name = os.ttyname(self.slave_fd)
        attrs = termios.tcgetattr(self.master_fd)
        attrs[3] = attrs[3] & ~termios.ECHO
        termios.tcsetattr(self.master_fd, termios.TCSANOW, attrs)
        if self.pty_link:
            try:
                if os.path.islink(self.pty_link) or os.path.exists(self.pty_link):
                    os.unlink(self.pty_link)
                os.symlink(slave_name, self.pty_link)
                print(f"[mock_esp32] {self.pty_link} -> {slave_name}")
            except OSError as e:
                print(f"[mock_esp32] Warning: could not symlink {self.pty_link}: {e}")

    def set_state(self, state):
        if state != self.state:
            print(f"[mock_esp32] LED: {STATE_NAMES[state]}")
        self.state = state

    def toggle_hw_kill(self):
        with self.lock:
            self.hw_kill = not self.hw_kill
            if self.hw_kill:
                if self.state != 2:
                    self.last_state = self.state
                self.set_state(2)
            else:
                self.set_state(self.last_state)
            print(f"[mock_esp32] hardware kill {'ACTIVE' if self.hw_kill else 'released'}")

    def process_line(self, line):
        with self.lock:
            if line == "its2":
                if self.state != 2:
                    self.last_state = self.state
                self.set_state(2)
            elif line in ("its0", "its1") and not self.hw_kill:
                self.last_state = int(line[3])
                self.set_state(self.last_state)

    def report_loop(self):
        while self.running:
            with self.lock:
                msg = f"its{self.state}\r\n".encode("ascii")
            try:
                os.write(self.master_fd, msg)
            except OSError:
                pass
            time.sleep(0.5)

    def reader_loop(self):
        buffer = b""
        while self.running:
            r, _, _ = select.select([self.master_fd], [], [], 0.1)
            if not r:
                continue
            try:
                data = os.read(self.master_fd, 64)
            except OSError:
                break
            buffer += data
            while b"\n" in buffer:
                line, buffer = buffer.split(b"\n", 1)
                self.process_line(line.strip().decode("ascii", errors="ignore"))

    def run(self):
        self.setup_pty()
        threading.Thread(target=self.report_loop, daemon=True).start()
        threading.Thread(target=self.reader_loop, daemon=True).start()
        print("[mock_esp32] 'k' toggles the hardware kill, 'q' exits")
        try:
            while self.running:
                r, _, _ = select.select([sys.stdin], [], [], 0.2)
                if r:
                    key = sys.stdin.readline().strip().lower()
                    if key == "k":
                        self.toggle_hw_kill()
                    elif key == "q":
                        break
        except KeyboardInterrupt:
            pass
        finally:
            self.running = False
            os.close(self.master_fd)
            os.close(self.slave_fd)
            if self.pty_link and os.path.islink(self.pty_link):
                os.unlink(self.pty_link)


def main():
    parser = argparse.ArgumentParser(description="Mock ESP32 kill-switch indicator")
    parser.add_argument("--pty-link", default="/tmp/ttyESP32", help="Symlink to the virtual serial port")
    MockIndicator(pty_link=parser.parse_args().pty_link).run()


if __name__ == "__main__":
    main()
