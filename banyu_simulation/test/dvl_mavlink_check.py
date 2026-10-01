#!/usr/bin/env python3
"""End-to-end check for banyu::DvlMavlink: fake SITL <-> plugin in a bare world.

Asserts frame map, dt, confidence and the bottom-lock dropout reset.
Run after `colcon build` and `source install/setup.bash`:
    python3 src/banyu_simulation/test/dvl_mavlink_check.py
"""
import atexit, os, socket, struct, subprocess, tempfile, time
from gz.transport13 import Node
from gz.msgs10.dvl_velocity_tracking_pb2 import DVLVelocityTracking
from gz.msgs10.dvl_tracking_target_pb2 import DVLTrackingTarget


WORLD = """<?xml version="1.0"?><sdf version="1.9"><world name="t">
  <plugin filename="BanyuDvlMavlink" name="banyu::DvlMavlink"/></world></sdf>"""
w = tempfile.NamedTemporaryFile("w", suffix=".sdf", delete=False); w.write(WORLD); w.close()
gz = subprocess.Popen(["gz", "sim", "-s", "-r", w.name], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
atexit.register(lambda: (gz.terminate(), os.unlink(w.name)))
time.sleep(3)

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); s.settimeout(2)
node = Node(); pub = node.advertise("/dvl/velocity", DVLVelocityTracking); time.sleep(1)

def dvl(sec, bottom=True):
    m = DVLVelocityTracking()
    m.header.stamp.sec = sec
    m.target.type = DVLTrackingTarget.DVL_TARGET_BOTTOM if bottom else DVLTrackingTarget.DVL_TARGET_WATER_MASS
    m.velocity.mean.x, m.velocity.mean.y, m.velocity.mean.z = 0.1, 0.2, 0.3
    m.velocity.covariance.extend([0.01,0,0, 0,0.04,0, 0,0,0.01])
    pub.publish(m)

s.sendto(b"hb", ("127.0.0.1", 14552))    # "SITL" announces itself
dvl(1); time.sleep(0.2)                  # first sample: no delta yet
dvl(2)
pkt, _ = s.recvfrom(512)
assert pkt[0] == 0xFD, "not MAVLink 2"
plen, msgid = pkt[1], int.from_bytes(pkt[7:10], "little")
assert msgid == 11011, msgid
pl = pkt[10:10+plen].ljust(44, b"\0")    # MAVLink 2 trims trailing zeros
t, dt, ax, ay, az, px, py, pz, conf = struct.unpack("<QQ7f", pl)
print(t, dt, (px, py, pz), conf)
assert t == 2_000_000 and dt == 1_000_000
assert abs(px-0.2) < 1e-6 and abs(py-0.1) < 1e-6 and abs(pz+0.3) < 1e-6   # (y, x, -z)
assert abs(conf - 60.0) < 1e-3           # sigma 0.2 of 0.5 -> 60
dvl(3, bottom=False); dvl(4)             # dropout resets: no delta across the gap
try: s.recvfrom(512); raise AssertionError("sent across dropout")
except socket.timeout: pass
print("OK")
