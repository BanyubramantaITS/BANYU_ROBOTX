#!/usr/bin/env python3
"""ZED-F9P (USB, UBX NAV-PVT) -> MAVLink GPS_INPUT -> MAVProxy udpin 14553 -> Pixhawk (GPS1_TYPE 14).

Started by run_gps_bridge.sh. Needs pyserial and pymavlink (~/venv-ardupilot).
--fake sends a made-up 3D fix (bench pipeline test only; reboot the Pixhawk afterwards).

Only NAV-PVT is used: NMEA GGA caps the satellite count at 12. The receiver's static hold is turned
off, otherwise its position freezes below 0.3 m/s (the whole slow surface leg) and jumps up to 10 m.
Survives USB disconnects (reopens the port and re-applies the config), reopens it when no new epoch
arrives for --stall seconds, and never re-sends an old position: if GPS data stops, nothing is sent
and the Pixhawk's GPS times out.
"""
import argparse, struct, time, datetime, sys, os, signal
import serial
from pymavlink import mavutil

GPS_EPOCH = datetime.datetime(1980, 1, 6, tzinfo=datetime.timezone.utc)
LEAP = 18


def log(msg):
    print(time.strftime("%H:%M:%S ") + msg, flush=True)


def ubx(cls, mid, payload=b""):
    body = bytes([cls, mid]) + struct.pack("<H", len(payload)) + payload
    a = b = 0
    for c in body:
        a = (a + c) & 0xFF
        b = (b + a) & 0xFF
    return b"\xb5\x62" + body + bytes([a, b])


def valset(items):
    # items: (key, size_bytes, value); RAM layer
    p = struct.pack("<BBH", 0, 1, 0)
    for key, size, val in items:
        p += struct.pack("<I", key) + val.to_bytes(size, "little")
    return ubx(0x06, 0x8A, p)


def configure(ser, rate_hz):
    meas = int(1000 / rate_hz)
    ser.write(valset([
        (0x30210001, 2, meas),   # CFG-RATE-MEAS
        (0x20910009, 1, 1),      # MSGOUT UBX-NAV-PVT on USB
        (0x20110021, 1, 5),      # NAVSPG-DYNMODEL = sea
        (0x201100A1, 1, 10),     # NAVSPG-INFIL_MINELEV = 10 deg
        (0x20250038, 1, 0),      # MOTION-GNSSSPEED_THRS = 0 -> static hold off
        (0x3025003B, 2, 0),      # MOTION-GNSSDIST_THRS = 0
    ]))


class GpsPort:
    """Serial port that reopens itself after disconnects/stalls."""

    def __init__(self, path, baud, rate, stall):
        self.path, self.baud, self.rate, self.stall = path, baud, rate, stall
        self.ser, self.buf = None, b""
        self.last_pvt = 0.0
        self.last_itow = None
        self.reconnects = 0

    def close(self):
        try:
            if self.ser:
                self.ser.close()
        except Exception:
            pass
        self.ser, self.buf = None, b""

    def open(self):
        delay = 0.5
        while True:
            try:
                if not os.path.exists(self.path):
                    raise serial.SerialException(f"{self.path} not present")
                self.ser = serial.Serial(self.path, self.baud, timeout=0.2, exclusive=True)
                self.ser.reset_input_buffer()
                configure(self.ser, self.rate)
                self.buf = b""
                self.last_pvt = time.time()
                self.last_itow = None
                log(f"GPS opened {self.path} (-> {os.path.realpath(self.path)})")
                return
            except (serial.SerialException, OSError) as e:
                self.close()
                log(f"GPS open failed: {e}; retry in {delay:.1f}s")
                time.sleep(delay)
                delay = min(delay * 2, 3.0)

    def reconnect(self, why):
        self.reconnects += 1
        log(f"GPS link lost ({why}) -> reconnecting [#{self.reconnects}]")
        self.close()
        time.sleep(0.5)
        self.open()

    def read_pvt(self):
        """Return a fresh NAV-PVT payload, or None. Handles errors internally."""
        if self.ser is None:
            self.open()
        try:
            self.buf += self.ser.read(512)
        except (serial.SerialException, OSError) as e:
            self.reconnect(str(e).split("(")[0].strip())
            return None
        if time.time() - self.last_pvt > self.stall:
            self.reconnect(f"no new PVT for {self.stall:.0f}s")
            return None
        buf = self.buf
        while True:
            i = buf.find(b"\xb5\x62")
            if i < 0:
                self.buf = buf[-1:]
                return None
            if len(buf) - i < 8:
                self.buf = buf[i:]
                return None
            ln = buf[i + 4] | buf[i + 5] << 8
            if ln > 2048:                       # corrupt length, skip header
                buf = buf[i + 2:]
                continue
            if len(buf) - i < ln + 8:
                self.buf = buf[i:]
                return None
            frame = buf[i:i + ln + 8]
            buf = buf[i + ln + 8:]
            a = b = 0
            for c in frame[2:-2]:
                a = (a + c) & 0xFF
                b = (b + a) & 0xFF
            if [a, b] != list(frame[-2:]):
                continue
            if frame[2] == 0x01 and frame[3] == 0x07 and ln >= 92:
                p = frame[6:6 + ln]
                itow = struct.unpack_from("<I", p, 0)[0]
                if itow == self.last_itow:      # duplicate epoch, ignore
                    continue
                self.last_itow = itow
                self.last_pvt = time.time()
                self.buf = buf
                return p


def main():
    ap = argparse.ArgumentParser(description="u-blox NAV-PVT -> MAVLink GPS_INPUT forwarder")
    ap.add_argument("--gps", default="/dev/gps_rtk")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--out", default="udpout:127.0.0.1:14553", help="MAVLink URL (run_mavproxy.sh udpin)")
    ap.add_argument("--rate", type=float, default=5.0)
    ap.add_argument("--stall", type=float, default=3.0, help="reopen port if no new PVT for N s")
    ap.add_argument("--fake", action="store_true", help="send a made-up fix (bench test only)")
    a = ap.parse_args()

    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
    gps = None if a.fake else GpsPort(a.gps, a.baud, a.rate, a.stall)
    if gps:
        gps.open()
    mav = mavutil.mavlink_connection(a.out, source_system=255, source_component=220)
    log(f"forwarding {'FAKE' if a.fake else a.gps} -> {a.out} @ {a.rate} Hz")

    n, send_err, last_print, last_hb = 0, 0, 0, 0
    while True:
        now = time.time()
        try:
            if now - last_hb > 1:
                mav.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                       mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
                last_hb = now
        except OSError:
            pass

        if a.fake:
            time.sleep(1 / a.rate)
            fix, nsv, lat, lon, h = 3, 12, -6.2000000, 106.8166666, 10.0
            vn = ve = vd = 0.0
            hacc, vacc, sacc, hdop, vdop = 1.5, 2.5, 0.2, 0.9, 1.2
        else:
            p = gps.read_pvt()
            if p is None:
                continue
            fix_raw, nsv = p[20], p[23]
            fix = {0: 0, 1: 2, 2: 2, 3: 3, 4: 3, 5: 0}.get(fix_raw, 0)
            if not (p[21] & 0x01):          # gnssFixOK not set -> not usable
                fix = min(fix, 1)
            carr = p[21] >> 6
            if fix == 3 and carr == 2:
                fix = 6   # RTK fixed
            elif fix == 3 and carr == 1:
                fix = 5   # RTK float
            lon, lat = struct.unpack_from("<ii", p, 24)
            lon, lat = lon * 1e-7, lat * 1e-7
            h = struct.unpack_from("<i", p, 36)[0] / 1000.0           # hMSL
            hacc, vacc = [min(x / 1000.0, 9999.0) for x in struct.unpack_from("<II", p, 40)]
            vn, ve, vd = [x / 1000.0 for x in struct.unpack_from("<iii", p, 48)]
            sacc = min(struct.unpack_from("<I", p, 68)[0] / 1000.0, 9999.0)
            pdop = struct.unpack_from("<H", p, 76)[0] * 0.01
            hdop = vdop = pdop

        t = datetime.datetime.now(datetime.timezone.utc) - GPS_EPOCH + datetime.timedelta(seconds=LEAP)
        wk = t.days // 7
        ms = int(((t.days % 7) * 86400 + t.seconds) * 1000 + t.microseconds / 1000)
        try:
            mav.mav.gps_input_send(
                int(now * 1e6), 0, 0, ms, wk, fix,
                int(lat * 1e7), int(lon * 1e7), h,
                hdop, vdop, vn, ve, vd, sacc, hacc, vacc, nsv)
            n += 1
        except OSError as e:                 # MAVProxy down / UDP error: keep running
            send_err += 1
            if send_err % 25 == 1:
                log(f"UDP send error ({e}); is MAVProxy running? errors={send_err}")

        if now - last_print > 2:
            rc = gps.reconnects if gps else 0
            log(f"fix={fix} sats={nsv} lat={lat:.7f} lon={lon:.7f} hacc={hacc:.2f}m "
                f"sent={n} reconnects={rc} send_err={send_err}")
            last_print = now


if __name__ == "__main__":
    while True:
        try:
            main()
        except (KeyboardInterrupt, SystemExit):
            sys.exit(0)
        except Exception as e:               # last-resort guard: never die, restart loop
            log(f"unexpected error: {e!r}; restarting in 2s")
            time.sleep(2)
