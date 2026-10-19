#!/usr/bin/env python3
"""
Mock ESP32 Safety Controller and Kill Switch Simulator.

Simulates the physical ESP32 controller over serial (real port or pseudo-terminal PTY):
- Receives heartbeat 'H\n' from esp32_bridge_node or commands '0', '1', '2' from banyu_teleop.
- Maintains 500ms heartbeat watchdog failsafe in autonomous mode (mode 2).
- Streams current operational mode ('0\n', '1\n', or '2\n') at 10 Hz.
- Provides interactive CLI commands to test mode switching and watchdog triggers.

Usage:
  python3 mock_esp32.py [--pty-link /tmp/ttyESP32] [--mode 1]
"""

import argparse
import os
import pty
import select
import sys
import termios
import threading
import time

MODE_DISARMED = 0   # State 0: Disarmed / Kill (Relay OFF, LED RED)
MODE_MANUAL = 1     # State 1: Manual Teleop (Relay ON, LED YELLOW)
MODE_AUTONOMOUS = 2 # State 2: Autonomous (Relay ON, LED GREEN)

MODE_NAMES = {
    MODE_DISARMED: "0 (DISARMED / KILL)",
    MODE_MANUAL: "1 (MANUAL / REMOTE)",
    MODE_AUTONOMOUS: "2 (AUTONOMOUS)"
}


class MockEsp32:
    def __init__(self, pty_link="/tmp/ttyESP32", initial_mode=MODE_MANUAL, watchdog_timeout_s=0.5):
        self.pty_link = pty_link
        self.current_mode = initial_mode
        self.watchdog_timeout_s = watchdog_timeout_s
        self.last_heartbeat_time = time.time()
        self.running = True
        self.master_fd = None
        self.slave_fd = None
        self.slave_name = None
        self.lock = threading.Lock()

    def setup_pty(self):
        self.master_fd, self.slave_fd = pty.openpty()
        self.slave_name = os.ttyname(self.slave_fd)

        # Set raw mode
        attrs = termios.tcgetattr(self.master_fd)
        attrs[3] = attrs[3] & ~termios.ECHO
        termios.tcsetattr(self.master_fd, termios.TCSANOW, attrs)

        # Create symlink if requested
        if self.pty_link:
            try:
                if os.path.islink(self.pty_link) or os.path.exists(self.pty_link):
                    os.unlink(self.pty_link)
                os.symlink(self.slave_name, self.pty_link)
                print(f"[mock_esp32] Created symlink {self.pty_link} -> {self.slave_name}")
            except Exception as e:
                print(f"[mock_esp32] Warning: could not symlink {self.pty_link}: {e}")

        print(f"[mock_esp32] Virtual Serial Port ready at: {self.slave_name}")
        print(f"[mock_esp32] Initial state: {MODE_NAMES[self.current_mode]}")

    def stream_loop(self):
        """Streams state at 10 Hz and checks watchdog."""
        while self.running:
            now = time.time()
            with self.lock:
                # Check watchdog in AUTONOMOUS mode
                if self.current_mode == MODE_AUTONOMOUS:
                    if now - self.last_heartbeat_time > self.watchdog_timeout_s:
                        print(f"\n[mock_esp32] WATCHDOG FAULT: No heartbeat for > {self.watchdog_timeout_s:.1f}s! Tripping failsafe to DISARMED.")
                        self.current_mode = MODE_DISARMED

                msg = f"{self.current_mode}\n".encode('ascii')

            try:
                os.write(self.master_fd, msg)
            except OSError:
                pass

            time.sleep(0.1)

    def reader_loop(self):
        """Reads incoming data from ROS."""
        buffer = b""
        while self.running:
            r, _, _ = select.select([self.master_fd], [], [], 0.1)
            if not r:
                continue

            try:
                data = os.read(self.master_fd, 64)
            except OSError:
                break

            if not data:
                continue

            buffer += data
            while b"\n" in buffer:
                line, buffer = buffer.split(b"\n", 1)
                line = line.strip().decode('ascii', errors='ignore')
                self.process_command(line)

            # Check single char commands
            if len(buffer) == 1 and buffer in [b"0", b"1", b"2"]:
                char = buffer.decode('ascii')
                buffer = b""
                self.process_command(char)

    def process_command(self, cmd):
        now = time.time()
        if cmd == "H":
            with self.lock:
                self.last_heartbeat_time = now
        elif cmd in ["0", "1", "2"]:
            new_mode = int(cmd)
            with self.lock:
                if new_mode != self.current_mode:
                    print(f"\n[mock_esp32] Mode updated from banyu_teleop: {MODE_NAMES.get(new_mode, 'UNKNOWN')}")
                    self.current_mode = new_mode
                    if new_mode == MODE_AUTONOMOUS:
                        self.last_heartbeat_time = now

    def set_mode(self, mode):
        with self.lock:
            self.current_mode = mode
            if mode == MODE_AUTONOMOUS:
                self.last_heartbeat_time = time.time()
            print(f"[mock_esp32] Manual override to: {MODE_NAMES[mode]}")

    def run(self):
        self.setup_pty()

        t_stream = threading.Thread(target=self.stream_loop, daemon=True)
        t_reader = threading.Thread(target=self.reader_loop, daemon=True)
        t_stream.start()
        t_reader.start()

        print("\n--- Mock ESP32 Controls ---")
        print("Press '0' -> Disarm / Kill")
        print("Press '1' -> Manual / Remote")
        print("Press '2' -> Autonomous")
        print("Press 'q' -> Exit")
        print("---------------------------\n")

        try:
            while self.running:
                r, _, _ = select.select([sys.stdin], [], [], 0.2)
                if r:
                    line = sys.stdin.readline().strip()
                    if line == '0':
                        self.set_mode(MODE_DISARMED)
                    elif line == '1':
                        self.set_mode(MODE_MANUAL)
                    elif line == '2':
                        self.set_mode(MODE_AUTONOMOUS)
                    elif line.lower() == 'q':
                        break
        except KeyboardInterrupt:
            pass
        finally:
            self.running = False
            if self.master_fd:
                os.close(self.master_fd)
            if self.slave_fd:
                os.close(self.slave_fd)
            if self.pty_link and os.path.islink(self.pty_link):
                try:
                    os.unlink(self.pty_link)
                except OSError:
                    pass
            print("[mock_esp32] Cleaned up and exited.")


def main():
    parser = argparse.ArgumentParser(description="Mock ESP32 Safety Controller")
    parser.add_argument("--pty-link", default="/tmp/ttyESP32", help="Path to symlink the virtual serial port")
    parser.add_argument("--mode", type=int, choices=[0, 1, 2], default=1, help="Initial operational mode")
    parser.add_argument("--watchdog", type=float, default=0.5, help="Heartbeat watchdog timeout in seconds")
    args = parser.parse_args()

    mock = MockEsp32(pty_link=args.pty_link, initial_mode=args.mode, watchdog_timeout_s=args.watchdog)
    mock.run()


if __name__ == "__main__":
    main()
