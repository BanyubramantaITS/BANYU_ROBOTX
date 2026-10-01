#!/usr/bin/env python3
"""
RobotX 2026 Underwater Magnetically Activated Light Controller.
Fully implements Mission Task 2 (Infrastructure Survey and Repair) from the RobotX 2026 Team Handbook.

Tiers Implemented (Handbook Section 3.3.3):
1. CORE TIER:
   - Initial state: 1 DAMAGED segment (RED), 3 FUNCTIONAL segments (GREEN).
   - Magnetic hold >= 1.5 mT for >= 250 ms: Proximity feedback (solid GREEN).
   - Magnetic hold >= 2.0 s: Repair confirmed -> Permanently Solid GREEN.

2. ADVANCED TIER:
   - Initial state: 1 DAMAGED (RED), 3 FUNCTIONAL (GREEN).
   - Magnetic repair >= 2.0 s: Post-repair confirmation (Solid GREEN for 5.0 s).
   - Optical Flash Sequence: Single color (RED, GREEN, or BLUE) designating destination circle.
     Pattern: 1.0 s ON / 1.0 s OFF repeating for 30 seconds total (15 pulses).
   - After 30 s: Settles to stable Solid GREEN.

3. DISRUPTIVE TIER:
   - Initial state: 1 DAMAGED (RED), 3 FUNCTIONAL (GREEN).
   - Magnetic repair >= 2.0 s: Post-repair confirmation (Solid GREEN for 5.0 s).
   - Two-Color Optical Sequence (repeating for 30 seconds, 6 full 5-second cycles):
     * 1st Color (1.0 s ON): Specifies WHICH Resource Tin to retrieve (RED, GREEN, or BLUE).
     * OFF / Blank (1.0 s).
     * 2nd Color (1.0 s ON): Specifies WHICH Target Circle to deliver to (RED, GREEN, or BLUE).
     * OFF / Black (2.0 s).
   - After 30 s: Settles to stable Solid GREEN.

Topic & Python Interface:
- Switch tiers: /robotx/pipeline/set_tier ("CORE", "ADVANCED", "DISRUPTIVE")
- Set damaged node: /robotx/pipeline/set_damaged_node (1..4 or "RANDOM")
- Set sequence colors: /robotx/pipeline/set_sequence (JSON: {"first_color": "RED", "second_color": "BLUE"})
- Activate / Reset: /robotx/pipeline/activate, /robotx/pipeline/reset
- Repair probe: /robotx/pipeline/repair_damaged, /robotx/light_node_{i}/trigger_magnet
- Live status: /robotx/pipeline/status and /robotx/light_node_{i}/status (Gazebo + ROS 2)
"""

import sys
import os
import signal
import time
import math
import json
import re
import random
import threading

# Gazebo Transport & Protobufs
import gz.transport13 as gz_transport
import gz.msgs10.empty_pb2 as empty_pb2
import gz.msgs10.stringmsg_pb2 as stringmsg_pb2
import gz.msgs10.light_pb2 as light_pb2
import gz.msgs10.visual_pb2 as visual_pb2
import gz.msgs10.boolean_pb2 as bool_pb2
import gz.msgs10.magnetometer_pb2 as mag_pb2
import gz.msgs10.pose_v_pb2 as pose_v_pb2

# ROS 2 Integration
HAS_ROS2 = False
try:
    import rclpy
    from rclpy.node import Node as Ros2Node
    from std_msgs.msg import String as Ros2String
    HAS_ROS2 = True
except ImportError:
    pass

# Colors (R, G, B) normalized [0.0, 1.0]
COLOR_MAP = {
    "RED": (1.0, 0.0, 0.0),
    "GREEN": (0.0, 1.0, 0.0),
    "BLUE": (0.0, 0.0, 1.0),
    "CYAN": (0.0, 1.0, 1.0),
    "MAGENTA": (1.0, 0.0, 1.0),
    "YELLOW": (1.0, 1.0, 0.0),
    "WHITE": (1.0, 1.0, 1.0),
    "OFF": (0.0, 0.0, 0.0),
    "BLACK": (0.0, 0.0, 0.0),  # Alias for OFF (unlit / 0 emission)
}

# Handbook Task 2 Specifications & Timing
TIER_CORE = "CORE"
TIER_ADVANCED = "ADVANCED"
TIER_DISRUPTIVE = "DISRUPTIVE"
VALID_TIERS = [TIER_CORE, TIER_ADVANCED, TIER_DISRUPTIVE]

MAG_THRESHOLD_MT = float(os.environ.get("ROBOTX_MAG_THRESHOLD_MT", "1.5"))
PROXIMITY_CONFIRM_S = float(os.environ.get("ROBOTX_PROXIMITY_CONFIRM_S", "0.25"))
REPAIR_HOLD_CONFIRM_S = float(os.environ.get("ROBOTX_REPAIR_HOLD_CONFIRM_S", "2.00"))
POST_REPAIR_CONFIRM_S = float(os.environ.get("ROBOTX_POST_REPAIR_CONFIRM_S", "5.00"))
ADVANCED_FLASH_TOTAL_S = float(os.environ.get("ROBOTX_ADVANCED_FLASH_TOTAL_S", "30.0"))
ADVANCED_FLASH_ON_S = float(os.environ.get("ROBOTX_ADVANCED_FLASH_ON_S", "1.0"))
DISRUPTIVE_TOTAL_S = float(os.environ.get("ROBOTX_DISRUPTIVE_TOTAL_S", "30.0"))
DISRUPTIVE_CYCLE_S = float(os.environ.get("ROBOTX_DISRUPTIVE_CYCLE_S", "5.0"))


class LightNodeState:
    def __init__(self, node_id, name):
        self.node_id = node_id
        self.name = name
        self.light_id = None
        self.visual_id = None          # led_lenses_visual
        self.halo_visual_id = None     # led_halo_visual
        self.lid_visual_id = None      # lid_visual

        # Role & State
        self.role = "FUNCTIONAL"       # 'DAMAGED' or 'FUNCTIONAL'
        self.state = "FUNCTIONAL_GREEN"
        self.trigger_source = "NONE"   # 'NONE', 'MAGNET', 'MANUAL'
        self.active_color_name = "GREEN"
        self.active_rgb = COLOR_MAP["GREEN"]
        self.last_actuated_rgb = None

        # Magnetometer tracking
        self.raw_sensor_field_mT = 0.0
        self.injected_field_mT = 0.0
        self.injected_until = 0.0
        self.magnet_above_threshold_start = None

        # Post-repair timing
        self.repair_timestamp = None
        self.post_repair_confirm_start = None
        self.sequence_start_time = None

    @property
    def total_field_mT(self):
        now = time.time()
        eff_injected = self.injected_field_mT if now < self.injected_until else 0.0
        return self.raw_sensor_field_mT + eff_injected

    @property
    def is_threshold_exceeded(self):
        return self.total_field_mT >= MAG_THRESHOLD_MT

    @property
    def current_hold_duration_ms(self):
        if self.magnet_above_threshold_start is not None:
            return max(0, int((time.time() - self.magnet_above_threshold_start) * 1000))
        return 0

    def to_status_dict(self):
        return {
            "node_id": self.node_id,
            "name": self.name,
            "role": self.role,
            "state": self.state,
            "active_color": self.active_color_name,
            "rgb": [round(c, 3) for c in self.active_rgb],
            "magnetic_field_mT": round(self.total_field_mT, 3),
            "threshold_mT": MAG_THRESHOLD_MT,
            "threshold_exceeded": self.is_threshold_exceeded,
            "hold_time_ms": self.current_hold_duration_ms,
            "trigger_source": self.trigger_source,
            "timestamp": round(time.time(), 3),
        }


class RobotXPipelineController:
    def __init__(self, world_name="pipeline_lights"):
        self.world_name = world_name
        self.gz_node = gz_transport.Node()
        self.lock = threading.RLock()
        self.running = True

        # Pipeline Configuration (Handbook Task 2)
        env_tier = os.environ.get("BANYU_TIER", os.environ.get("ROBOTX_TIER", "DISRUPTIVE")).upper().strip()
        self.tier = env_tier if env_tier in VALID_TIERS else TIER_DISRUPTIVE
        self.damaged_node_id = 2
        self.advanced_target_color = "BLUE"
        self.disruptive_first_color = "RED"       # Tin color to retrieve (1st flash)
        self.disruptive_second_color = "BLUE"     # Target circle destination (2nd flash)
        self.challenge_active = True

        env_seed = os.environ.get("BANYU_LIGHT_SEED", os.environ.get("ROBOTX_SEED", "")).strip()
        if env_seed:
            try:
                seed_val = int(env_seed)
                rng = random.Random(seed_val)
                self.damaged_node_id = rng.randint(1, 4)
                color_choices = ["RED", "GREEN", "BLUE"]
                self.advanced_target_color = rng.choice(color_choices)
                self.disruptive_first_color = rng.choice(color_choices)
                self.disruptive_second_color = rng.choice(color_choices)
            except ValueError:
                pass

        # 4 Nodes
        self.nodes = {
            i: LightNodeState(i, f"light_node_{i}") for i in range(1, 5)
        }

        # Status Publishers
        self.gz_status_pubs = {}
        for i in range(1, 5):
            self.gz_status_pubs[i] = self.gz_node.advertise(
                f"/robotx/light_node_{i}/status", stringmsg_pb2.StringMsg
            )
        self.gz_pipeline_pub = self.gz_node.advertise(
            "/robotx/pipeline/status", stringmsg_pb2.StringMsg
        )

        # ROS 2 Integration
        self.ros2_node = None
        self.ros2_status_pubs = {}
        self.ros2_pipeline_pub = None
        if HAS_ROS2:
            try:
                if not rclpy.ok():
                    rclpy.init(args=None)
                self.ros2_node = Ros2Node("robotx_pipeline_controller")
                for i in range(1, 5):
                    self.ros2_status_pubs[i] = self.ros2_node.create_publisher(
                        Ros2String, f"/robotx/light_node_{i}/status", 10
                    )
                self.ros2_pipeline_pub = self.ros2_node.create_publisher(
                    Ros2String, "/robotx/pipeline/status", 10
                )
                print("[ROS 2] Initialized ROS 2 publishers successfully.")
            except Exception as e:
                print(f"[ROS 2] Notice: rclpy initialization skipped: {e}")

    def discover_entities(self, max_retries=15):
        """Query Gazebo scene graph to dynamically discover entity IDs for all 4 nodes."""
        print(f"Discovering entity IDs for 4 light nodes in '{self.world_name}'...")
        req = empty_pb2.Empty()
        service_name = f"/world/{self.world_name}/scene/graph"

        for attempt in range(1, max_retries + 1):
            res, rep = self.gz_node.request(
                service_name, req, empty_pb2.Empty, stringmsg_pb2.StringMsg, 2000
            )
            if res and rep.data:
                dot_data = rep.data
                node_map = {}
                for m in re.finditer(r'^\s*(\d+)\s*\[label=\"([^\"]+)\"\];', dot_data, re.M):
                    node_id = int(m.group(1))
                    label = m.group(2).split(' ')[0]
                    node_map[node_id] = label
                children = {}
                for m in re.finditer(r'^\s*(\d+)\s*->\s*(\d+)', dot_data, re.M):
                    p, c = int(m.group(1)), int(m.group(2))
                    children.setdefault(p, []).append(c)

                found_count = 0
                for nid, name in node_map.items():
                    m_node = re.search(r'(?:^|::)light_node_(\d+)$', name)
                    if m_node:
                        try:
                            idx = int(m_node.group(1))
                        except ValueError:
                            continue
                        if idx in self.nodes:
                            for link_id in children.get(nid, []):
                                for comp_id in children.get(link_id, []):
                                    cname = node_map.get(comp_id, "")
                                    if cname.endswith("led_light"):
                                        self.nodes[idx].light_id = comp_id
                                    elif cname.endswith("led_lenses_visual"):
                                        self.nodes[idx].visual_id = comp_id
                                    elif cname.endswith("led_halo_visual"):
                                        self.nodes[idx].halo_visual_id = comp_id
                                    elif cname.endswith("lid_visual"):
                                        self.nodes[idx].lid_visual_id = comp_id
                            if self.nodes[idx].light_id and self.nodes[idx].visual_id:
                                found_count += 1

                if found_count == 4:
                    print("Successfully mapped all 4 light nodes with realistic emission layers:")
                    for i in range(1, 5):
                        n = self.nodes[i]
                        print(f"  Node {i} ({n.name}): light={n.light_id}, lenses={n.visual_id}, halo={n.halo_visual_id}, lid={n.lid_visual_id}")
                    return True

            print(f"Waiting for Gazebo scene graph (attempt {attempt}/{max_retries})...")
            time.sleep(1.0)

        print("WARNING: Could not discover all node entities from scene graph.")
        return False

    def setup_subscriptions(self):
        """Register subscribers for sensor topics and configuration/command topics."""
        # 1. Magnetometer Sensor Topics (supports both top-level and composite model instances)
        for i in range(1, 5):
            def make_mag_cb(node_id):
                def cb(msg):
                    mag_sq = msg.field_tesla.x**2 + msg.field_tesla.y**2 + msg.field_tesla.z**2
                    field_mT = math.sqrt(mag_sq) * 1000.0
                    with self.lock:
                        self.nodes[node_id].raw_sensor_field_mT = field_mT
                return cb
            cb = make_mag_cb(i)
            # Direct top-level light_node_{i}
            self.gz_node.subscribe(
                mag_pb2.Magnetometer,
                f"/world/{self.world_name}/model/light_node_{i}/link/base_link/sensor/magnetometer/magnetometer",
                cb,
            )
            # Nested inside composite underwater_magnetic_activated_light model
            self.gz_node.subscribe(
                mag_pb2.Magnetometer,
                f"/world/{self.world_name}/model/underwater_magnetic_activated_light/model/light_node_{i}/link/base_link/sensor/magnetometer/magnetometer",
                cb,
            )
            self.gz_node.subscribe(
                mag_pb2.Magnetometer,
                f"/world/{self.world_name}/model/underwater-magnetic-activated-light/model/light_node_{i}/link/base_link/sensor/magnetometer/magnetometer",
                cb,
            )
            # Nested inside pipeline model
            self.gz_node.subscribe(
                mag_pb2.Magnetometer,
                f"/world/{self.world_name}/model/pipeline/model/light_node_{i}/link/base_link/sensor/magnetometer/magnetometer",
                cb,
            )

        # ROV proximity tracking for magnetic probe contact
        node_positions = {}
        def pose_cb(msg):
            rov_pos = None
            for p in msg.pose:
                name = p.name
                m = re.search(r'(?:^|::)light_node_(\d+)$', name)
                if m:
                    nid = int(m.group(1))
                    node_positions[nid] = (p.position.x, p.position.y, p.position.z)
                elif "bluerov2_heavy" in name:
                    rov_pos = (p.position.x, p.position.y, p.position.z)
            if rov_pos and self.damaged_node_id in node_positions:
                target_pos = node_positions[self.damaged_node_id]
                dist = math.dist(rov_pos, target_pos)
                with self.lock:
                    node = self.nodes[self.damaged_node_id]
                    if dist < 0.40:
                        node.raw_sensor_field_mT = 3.5
                    elif node.injected_until <= time.time():
                        node.raw_sensor_field_mT = 0.0

        self.gz_node.subscribe(pose_v_pb2.Pose_V, f"/world/{self.world_name}/pose/info", pose_cb)

        # 2. Pipeline Configuration Topics
        def set_tier_cb(msg):
            self.set_tier(msg.data.strip())
        self.gz_node.subscribe(stringmsg_pb2.StringMsg, "/robotx/pipeline/set_tier", set_tier_cb)

        def set_damaged_cb(msg):
            self.set_damaged_node(msg.data.strip())
        self.gz_node.subscribe(stringmsg_pb2.StringMsg, "/robotx/pipeline/set_damaged_node", set_damaged_cb)

        def set_sequence_cb(msg):
            self.set_sequence(msg.data.strip())
        self.gz_node.subscribe(stringmsg_pb2.StringMsg, "/robotx/pipeline/set_sequence", set_sequence_cb)

        def set_first_color_cb(msg):
            self.set_first_color(msg.data.strip())
        self.gz_node.subscribe(stringmsg_pb2.StringMsg, "/robotx/pipeline/set_first_color", set_first_color_cb)

        def set_second_color_cb(msg):
            self.set_second_color(msg.data.strip())
        self.gz_node.subscribe(stringmsg_pb2.StringMsg, "/robotx/pipeline/set_second_color", set_second_color_cb)

        def activate_cb(msg):
            self.activate_challenge()
        self.gz_node.subscribe(stringmsg_pb2.StringMsg, "/robotx/pipeline/activate", activate_cb)

        def reset_cb(msg):
            self.reset_pipeline()
        self.gz_node.subscribe(stringmsg_pb2.StringMsg, "/robotx/pipeline/reset", reset_cb)
        self.gz_node.subscribe(stringmsg_pb2.StringMsg, "/robotx/reset_all", reset_cb)

        # 3. Magnetic Repair Trigger Topics
        def repair_damaged_cb(msg):
            duration_s = 2.5
            try:
                duration_s = float(msg.data.strip())
            except ValueError:
                pass
            self.trigger_repair_damaged(duration_s=duration_s)
        self.gz_node.subscribe(stringmsg_pb2.StringMsg, "/robotx/pipeline/repair_damaged", repair_damaged_cb)

        for i in range(1, 5):
            def make_trigger_cb(node_id):
                def cb(msg):
                    self.trigger_node_magnet(node_id, msg.data.strip())
                return cb
            self.gz_node.subscribe(stringmsg_pb2.StringMsg, f"/robotx/light_node_{i}/trigger_magnet", make_trigger_cb(i))

        # 4. Manual Color Override Topics
        for i in range(1, 5):
            def make_color_cb(node_id):
                def cb(msg):
                    self.set_manual_color(node_id, msg.data.strip())
                return cb
            self.gz_node.subscribe(stringmsg_pb2.StringMsg, f"/robotx/light_node_{i}/set_color", make_color_cb(i))

        def all_colors_cb(msg):
            cmd = msg.data.strip()
            for idx in range(1, 5):
                self._apply_direct_color(idx, cmd)
        self.gz_node.subscribe(stringmsg_pb2.StringMsg, "/robotx/set_all_colors", all_colors_cb)

        # 5. Mirror to ROS 2 if available
        if self.ros2_node:
            self.ros2_node.create_subscription(Ros2String, "/robotx/pipeline/set_tier", set_tier_cb, 10)
            self.ros2_node.create_subscription(Ros2String, "/robotx/pipeline/set_damaged_node", set_damaged_cb, 10)
            self.ros2_node.create_subscription(Ros2String, "/robotx/pipeline/set_sequence", set_sequence_cb, 10)
            self.ros2_node.create_subscription(Ros2String, "/robotx/pipeline/activate", activate_cb, 10)
            self.ros2_node.create_subscription(Ros2String, "/robotx/pipeline/reset", reset_cb, 10)
            self.ros2_node.create_subscription(Ros2String, "/robotx/pipeline/repair_damaged", repair_damaged_cb, 10)
            for i in range(1, 5):
                def make_r2_trigger_cb(node_id):
                    def cb(msg):
                        self.trigger_node_magnet(node_id, msg.data.strip())
                    return cb
                self.ros2_node.create_subscription(Ros2String, f"/robotx/light_node_{i}/trigger_magnet", make_r2_trigger_cb(i), 10)

                def make_r2_color_cb(node_id):
                    def cb(msg):
                        self.set_manual_color(node_id, msg.data.strip())
                    return cb
                self.ros2_node.create_subscription(Ros2String, f"/robotx/light_node_{i}/set_color", make_r2_color_cb(i), 10)

        print("Subscriptions established for pipeline configuration and light control.")

    # ==================== CONFIGURATION METHODS ====================

    def set_tier(self, tier_str):
        t = tier_str.upper().strip()
        if t in VALID_TIERS:
            with self.lock:
                self.tier = t
                print(f"[Pipeline] Switched to Tier: {self.tier}")
        else:
            print(f"[Pipeline] Invalid tier '{tier_str}'. Valid: {VALID_TIERS}")

    def set_damaged_node(self, node_str):
        val = node_str.upper().strip()
        changed = False
        with self.lock:
            if val == "RANDOM":
                self.damaged_node_id = random.randint(1, 4)
                changed = True
            else:
                try:
                    nid = int(val)
                    if 1 <= nid <= 4:
                        self.damaged_node_id = nid
                        changed = True
                    else:
                        print(f"[Pipeline] Damaged node ID must be 1-4, got: {nid}")
                        return
                except ValueError:
                    return
            print(f"[Pipeline] Damaged Segment assigned to Node {self.damaged_node_id}")
        if changed:
            self._apply_pipeline_roles()

    def set_sequence(self, payload_str):
        """Set sequence via JSON e.g. {"first_color": "RED", "second_color": "BLUE", "tier": "DISRUPTIVE"}."""
        try:
            data = json.loads(payload_str)
            with self.lock:
                if "tier" in data:
                    t = data["tier"].upper().strip()
                    if t in VALID_TIERS:
                        self.tier = t
                if "first_color" in data or "tin_color" in data:
                    c = data.get("first_color", data.get("tin_color")).upper().strip()
                    if c in COLOR_MAP:
                        self.disruptive_first_color = c
                if "second_color" in data or "circle_color" in data:
                    c = data.get("second_color", data.get("circle_color")).upper().strip()
                    if c in COLOR_MAP:
                        self.disruptive_second_color = c
                        self.advanced_target_color = c
                print(f"[Pipeline] Configured Sequence -> Tier: {self.tier}, 1st Color (Tin): {self.disruptive_first_color}, 2nd Color (Circle): {self.disruptive_second_color}")
        except Exception as e:
            # Check if format is "RED BLUE"
            parts = payload_str.upper().split()
            if len(parts) >= 2 and parts[0] in COLOR_MAP and parts[1] in COLOR_MAP:
                with self.lock:
                    self.disruptive_first_color = parts[0]
                    self.disruptive_second_color = parts[1]
                    self.advanced_target_color = parts[1]
                    print(f"[Pipeline] Configured Sequence -> 1st: {self.disruptive_first_color}, 2nd: {self.disruptive_second_color}")
            else:
                print(f"[Pipeline] Error parsing sequence payload '{payload_str}': {e}")

    def set_first_color(self, color_str):
        c = color_str.upper().strip()
        if c in COLOR_MAP:
            with self.lock:
                self.disruptive_first_color = c
                print(f"[Pipeline] Disruptive 1st Color (Resource Tin) set to: {c}")

    def set_second_color(self, color_str):
        c = color_str.upper().strip()
        if c in COLOR_MAP:
            with self.lock:
                self.disruptive_second_color = c
                self.advanced_target_color = c
                print(f"[Pipeline] 2nd Color (Destination Circle) set to: {c}")

    def activate_challenge(self):
        with self.lock:
            self.challenge_active = True
            print(f"[Pipeline] Task 2 ACTIVATED. Tier: {self.tier}, Damaged Node: {self.damaged_node_id}")
        self._apply_pipeline_roles()

    def reset_pipeline(self):
        with self.lock:
            self.challenge_active = True
            print("[Pipeline] Resetting pipeline to initial Task 2 state...")
        self._apply_pipeline_roles()

    def _apply_pipeline_roles(self):
        """Set up 1 RED damaged segment and 3 GREEN functional segments."""
        updates = []
        with self.lock:
            for i, node in self.nodes.items():
                node.injected_field_mT = 0.0
                node.injected_until = 0.0
                node.magnet_above_threshold_start = None
                node.repair_timestamp = None
                node.post_repair_confirm_start = None
                node.sequence_start_time = None

                if i == self.damaged_node_id:
                    node.role = "DAMAGED"
                    node.state = "DAMAGED_RED"
                    node.trigger_source = "NONE"
                    node.active_color_name = "RED"
                    node.active_rgb = COLOR_MAP["RED"]
                    updates.append((i, COLOR_MAP["RED"]))
                else:
                    node.role = "FUNCTIONAL"
                    node.state = "FUNCTIONAL_GREEN"
                    node.trigger_source = "NONE"
                    node.active_color_name = "GREEN"
                    node.active_rgb = COLOR_MAP["GREEN"]
                    updates.append((i, COLOR_MAP["GREEN"]))

        # Actuate outside the lock so RPCs don't block subscriptions or state machine
        for node_id, rgb in updates:
            self._actuate_gazebo_hardware(node_id, rgb)

    # ==================== MAGNET & REPAIR ====================

    def trigger_repair_damaged(self, duration_s=2.5, field_mT=3.5):
        """Simulate magnetic probe repair directly on the currently damaged node."""
        with self.lock:
            target_id = self.damaged_node_id
        self.trigger_node_magnet(target_id, json.dumps({"duration_s": duration_s, "field_mT": field_mT}))

    def trigger_node_magnet(self, node_id, payload_str):
        duration_s = 2.5
        field_mT = 3.5
        try:
            duration_s = float(payload_str)
        except ValueError:
            try:
                data = json.loads(payload_str)
                if "duration" in data or "duration_s" in data:
                    duration_s = float(data.get("duration", data.get("duration_s", 2.5)))
                if "field_mT" in data:
                    field_mT = float(data["field_mT"])
            except Exception:
                pass

        with self.lock:
            node = self.nodes.get(node_id)
            if node:
                now = time.time()
                node.injected_field_mT = field_mT
                node.injected_until = now + duration_s
                if node.state == "MANUAL_OVERRIDE":
                    node.state = "DAMAGED_RED" if node.role == "DAMAGED" else "FUNCTIONAL_GREEN"
                    node.trigger_source = "NONE"
                print(f"[Pipeline] Injected magnet probe {field_mT:.1f} mT on Node {node_id} for {duration_s:.2f} s")

    def set_manual_color(self, node_id, cmd_str):
        self._apply_direct_color(node_id, cmd_str)

    def _apply_direct_color(self, node_id, cmd_str):
        cmd_upper = cmd_str.upper()
        if cmd_upper == "RESET":
            self._apply_pipeline_roles()
            return
        if cmd_upper in COLOR_MAP:
            rgb = COLOR_MAP[cmd_upper]
            with self.lock:
                node = self.nodes.get(node_id)
                if not node:
                    return
                node.active_color_name = cmd_upper
                node.active_rgb = rgb
                node.state = "MANUAL_OVERRIDE"
                node.trigger_source = "MANUAL"
            self._actuate_gazebo_hardware(node_id, rgb)

    # ==================== HARDWARE ACTUATION ====================

    def _actuate_gazebo_hardware(self, node_id, rgb, force=False):
        """Update Gazebo point light (25x), LED lenses, volumetric halo, and polycarbonate lid."""
        node = self.nodes.get(node_id)
        if not node:
            return
        if not force and node.last_actuated_rgb == rgb:
            return
        node.last_actuated_rgb = rgb

        r, g, b = rgb
        is_off = (r == 0.0 and g == 0.0 and b == 0.0)

        # 1. Update Physical Point Light (High intensity 25.0)
        if node.light_id:
            light_msg = light_pb2.Light()
            light_msg.id = node.light_id
            light_msg.is_light_off = is_off
            light_msg.intensity = 0.0 if is_off else 1.0
            light_msg.diffuse.r = r
            light_msg.diffuse.g = g
            light_msg.diffuse.b = b
            light_msg.diffuse.a = 1.0
            light_msg.specular.r = min(1.0, r * 1.2)
            light_msg.specular.g = min(1.0, g * 1.2)
            light_msg.specular.b = min(1.0, b * 1.2)
            light_msg.specular.a = 1.0
            light_msg.range = 12.0
            light_msg.attenuation_constant = 0.4
            light_msg.attenuation_linear = 0.05
            light_msg.attenuation_quadratic = 0.015
            self.gz_node.request(
                f"/world/{self.world_name}/light_config",
                light_msg, light_pb2.Light, bool_pb2.Boolean, 100
            )

        # 2. Update WS2812 LED Lenses Visual
        if node.visual_id:
            vis_msg = visual_pb2.Visual()
            vis_msg.id = node.visual_id
            vis_msg.material.ambient.r = r
            vis_msg.material.ambient.g = g
            vis_msg.material.ambient.b = b
            vis_msg.material.ambient.a = 1.0
            vis_msg.material.diffuse.r = r
            vis_msg.material.diffuse.g = g
            vis_msg.material.diffuse.b = b
            vis_msg.material.diffuse.a = 1.0
            vis_msg.material.emissive.r = r
            vis_msg.material.emissive.g = g
            vis_msg.material.emissive.b = b
            vis_msg.material.emissive.a = 1.0
            vis_msg.material.specular.r = 1.0 if not is_off else 0.2
            vis_msg.material.specular.g = 1.0 if not is_off else 0.2
            vis_msg.material.specular.b = 1.0 if not is_off else 0.2
            vis_msg.material.specular.a = 1.0
            self.gz_node.request(
                f"/world/{self.world_name}/visual_config",
                vis_msg, visual_pb2.Visual, bool_pb2.Boolean, 100
            )

        # 3. Update Volumetric Glow Halo
        if node.halo_visual_id:
            halo_msg = visual_pb2.Visual()
            halo_msg.id = node.halo_visual_id
            if is_off:
                halo_msg.material.ambient.a = 0.0
                halo_msg.material.diffuse.a = 0.0
                halo_msg.material.emissive.a = 0.0
            else:
                halo_msg.material.ambient.r = r
                halo_msg.material.ambient.g = g
                halo_msg.material.ambient.b = b
                halo_msg.material.ambient.a = 0.65
                halo_msg.material.diffuse.r = r
                halo_msg.material.diffuse.g = g
                halo_msg.material.diffuse.b = b
                halo_msg.material.diffuse.a = 0.65
                halo_msg.material.emissive.r = r
                halo_msg.material.emissive.g = g
                halo_msg.material.emissive.b = b
                halo_msg.material.emissive.a = 1.0
            self.gz_node.request(
                f"/world/{self.world_name}/visual_config",
                halo_msg, visual_pb2.Visual, bool_pb2.Boolean, 100
            )

        # 4. Update Clear Polycarbonate Lid (Scattering glow)
        if node.lid_visual_id:
            lid_msg = visual_pb2.Visual()
            lid_msg.id = node.lid_visual_id
            if is_off:
                lid_msg.material.ambient.r = 0.88
                lid_msg.material.ambient.g = 0.94
                lid_msg.material.ambient.b = 1.0
                lid_msg.material.ambient.a = 0.25
                lid_msg.material.diffuse.r = 0.88
                lid_msg.material.diffuse.g = 0.94
                lid_msg.material.diffuse.b = 1.0
                lid_msg.material.diffuse.a = 0.25
                lid_msg.material.emissive.r = 0.0
                lid_msg.material.emissive.g = 0.0
                lid_msg.material.emissive.b = 0.0
                lid_msg.material.emissive.a = 0.0
            else:
                lid_msg.material.ambient.r = 0.7 * r + 0.1
                lid_msg.material.ambient.g = 0.7 * g + 0.1
                lid_msg.material.ambient.b = 0.7 * b + 0.1
                lid_msg.material.ambient.a = 0.35
                lid_msg.material.diffuse.r = 0.7 * r + 0.1
                lid_msg.material.diffuse.g = 0.7 * g + 0.1
                lid_msg.material.diffuse.b = 0.7 * b + 0.1
                lid_msg.material.diffuse.a = 0.35
                lid_msg.material.emissive.r = 0.35 * r
                lid_msg.material.emissive.g = 0.35 * g
                lid_msg.material.emissive.b = 0.35 * b
                lid_msg.material.emissive.a = 0.35
            self.gz_node.request(
                f"/world/{self.world_name}/visual_config",
                lid_msg, visual_pb2.Visual, bool_pb2.Boolean, 100
            )

    # ==================== STATE MACHINE ENGINE ====================

    def update_state_machine(self):
        """Run non-blocking RobotX Task 2 state machine step for all nodes."""
        now = time.time()
        to_actuate = []

        with self.lock:
            for idx, node in self.nodes.items():
                if node.state == "MANUAL_OVERRIDE":
                    continue

                exceeded = node.is_threshold_exceeded

                # Track magnetic threshold timing
                if exceeded:
                    if node.magnet_above_threshold_start is None:
                        node.magnet_above_threshold_start = now
                else:
                    node.magnet_above_threshold_start = None

                # Functional segments remain solid green throughout
                if node.role == "FUNCTIONAL":
                    if node.state != "FUNCTIONAL_GREEN":
                        node.state = "FUNCTIONAL_GREEN"
                        node.active_color_name = "GREEN"
                        node.active_rgb = COLOR_MAP["GREEN"]
                        to_actuate.append((idx, COLOR_MAP["GREEN"]))
                    continue

                # ================= DAMAGED SEGMENT STATE MACHINE =================
                if node.state == "DAMAGED_RED":
                    # Proximity Feedback: turns GREEN if held >= 250 ms
                    if node.magnet_above_threshold_start is not None:
                        hold_time = now - node.magnet_above_threshold_start
                        if hold_time >= PROXIMITY_CONFIRM_S:
                            node.state = "PROBE_HELD_GREEN"
                            node.trigger_source = "MAGNET"
                            node.active_color_name = "GREEN"
                            node.active_rgb = COLOR_MAP["GREEN"]
                            to_actuate.append((idx, COLOR_MAP["GREEN"]))

                elif node.state == "PROBE_HELD_GREEN":
                    if node.magnet_above_threshold_start is None:
                        # Probe removed before full repair hold -> revert to RED
                        node.state = "DAMAGED_RED"
                        node.trigger_source = "NONE"
                        node.active_color_name = "RED"
                        node.active_rgb = COLOR_MAP["RED"]
                        to_actuate.append((idx, COLOR_MAP["RED"]))
                    else:
                        hold_time = now - node.magnet_above_threshold_start
                        if hold_time >= REPAIR_HOLD_CONFIRM_S:
                            # REPAIR SUCCESSFUL!
                            node.repair_timestamp = now
                            node.trigger_source = "MAGNET"

                            if self.tier == TIER_CORE:
                                # Core Tier: Stays permanently solid GREEN
                                node.state = "TASK_COMPLETED_GREEN"
                                node.active_color_name = "GREEN"
                                node.active_rgb = COLOR_MAP["GREEN"]
                                to_actuate.append((idx, COLOR_MAP["GREEN"]))
                                print(f"[Pipeline] Node {idx} REPAIRED in CORE Tier -> Permanently Solid GREEN")
                            else:
                                # Advanced & Disruptive: 5.0 s Solid GREEN confirmation first
                                node.state = "POST_REPAIR_CONFIRM_GREEN"
                                node.post_repair_confirm_start = now
                                node.active_color_name = "GREEN"
                                node.active_rgb = COLOR_MAP["GREEN"]
                                to_actuate.append((idx, COLOR_MAP["GREEN"]))
                                print(f"[Pipeline] Node {idx} REPAIRED in {self.tier} Tier -> 5.0s Confirmation GREEN")

                elif node.state == "POST_REPAIR_CONFIRM_GREEN":
                    elapsed_confirm = now - node.post_repair_confirm_start
                    if elapsed_confirm >= POST_REPAIR_CONFIRM_S:
                        # 5s confirmation elapsed -> start optical sequence
                        node.sequence_start_time = now
                        if self.tier == TIER_ADVANCED:
                            node.state = "ADVANCED_FLASH"
                            node.active_color_name = self.advanced_target_color
                            node.active_rgb = COLOR_MAP[self.advanced_target_color]
                            to_actuate.append((idx, COLOR_MAP[self.advanced_target_color]))
                            print(f"[Pipeline] Node {idx} starting Advanced Flash Sequence: {self.advanced_target_color} (30s)")
                        elif self.tier == TIER_DISRUPTIVE:
                            node.state = "DISRUPTIVE_CYCLE"
                            node.active_color_name = self.disruptive_first_color
                            node.active_rgb = COLOR_MAP[self.disruptive_first_color]
                            to_actuate.append((idx, COLOR_MAP[self.disruptive_first_color]))
                            print(f"[Pipeline] Node {idx} starting Disruptive Two-Color Sequence: 1st={self.disruptive_first_color} (Tin), 2nd={self.disruptive_second_color} (Circle) (30s)")

                elif node.state == "ADVANCED_FLASH":
                    elapsed = now - node.sequence_start_time
                    if elapsed >= ADVANCED_FLASH_TOTAL_S:
                        # 30s flash complete -> return to stable repaired GREEN
                        node.state = "TASK_COMPLETED_GREEN"
                        node.active_color_name = "GREEN"
                        node.active_rgb = COLOR_MAP["GREEN"]
                        to_actuate.append((idx, COLOR_MAP["GREEN"]))
                        print(f"[Pipeline] Node {idx} Advanced Sequence completed -> Solid GREEN")
                    else:
                        # 1.0s ON, 1.0s OFF cycle (period = 2.0s)
                        phase = elapsed % 2.0
                        if phase < ADVANCED_FLASH_ON_S:
                            target_c = self.advanced_target_color
                            target_rgb = COLOR_MAP[target_c]
                        else:
                            target_c = "OFF"
                            target_rgb = COLOR_MAP["OFF"]

                        if node.active_color_name != target_c:
                            node.active_color_name = target_c
                            node.active_rgb = target_rgb
                            to_actuate.append((idx, target_rgb))

                elif node.state == "DISRUPTIVE_CYCLE":
                    elapsed = now - node.sequence_start_time
                    if elapsed >= DISRUPTIVE_TOTAL_S:
                        # 30s sequence complete (6 cycles) -> return to stable repaired GREEN
                        node.state = "TASK_COMPLETED_GREEN"
                        node.active_color_name = "GREEN"
                        node.active_rgb = COLOR_MAP["GREEN"]
                        to_actuate.append((idx, COLOR_MAP["GREEN"]))
                        print(f"[Pipeline] Node {idx} Disruptive Sequence completed -> Solid GREEN")
                    else:
                        # 5.0s Cycle:
                        # [0.0, 1.0) -> Color 1 (Tin)
                        # [1.0, 2.0) -> OFF (Blank gap)
                        # [2.0, 3.0) -> Color 2 (Target Circle)
                        # [3.0, 5.0) -> OFF (Black gap)
                        cycle_time = elapsed % DISRUPTIVE_CYCLE_S
                        if cycle_time < 1.0:
                            target_c = self.disruptive_first_color
                            target_rgb = COLOR_MAP[target_c]
                        elif cycle_time < 2.0:
                            target_c = "OFF"
                            target_rgb = COLOR_MAP["OFF"]
                        elif cycle_time < 3.0:
                            target_c = self.disruptive_second_color
                            target_rgb = COLOR_MAP[target_c]
                        else:
                            target_c = "OFF"
                            target_rgb = COLOR_MAP["OFF"]

                        if node.active_color_name != target_c:
                            node.active_color_name = target_c
                            node.active_rgb = target_rgb
                            to_actuate.append((idx, target_rgb))

                elif node.state == "TASK_COMPLETED_GREEN":
                    # Fully repaired, stable green
                    if node.active_color_name != "GREEN":
                        node.active_color_name = "GREEN"
                        node.active_rgb = COLOR_MAP["GREEN"]
                        to_actuate.append((idx, COLOR_MAP["GREEN"]))

        # Actuate outside the lock
        for idx, rgb in to_actuate:
            self._actuate_gazebo_hardware(idx, rgb)

    # ==================== BROADCASTING ====================

    def broadcast_status(self):
        """Broadcast live condition of each node and the overall pipeline."""
        now = time.time()
        pipeline_status = {
            "tier": self.tier,
            "damaged_node_id": self.damaged_node_id,
            "challenge_active": self.challenge_active,
            "sequence_config": {
                "advanced_target_color": self.advanced_target_color,
                "disruptive_first_color_tin": self.disruptive_first_color,
                "disruptive_second_color_circle": self.disruptive_second_color,
            },
            "nodes": {},
            "timestamp": round(now, 3),
        }

        with self.lock:
            # Active damaged node timing info
            d_node = self.nodes.get(self.damaged_node_id)
            if d_node:
                pipeline_status["damaged_state"] = d_node.state
                if d_node.state == "POST_REPAIR_CONFIRM_GREEN" and d_node.post_repair_confirm_start:
                    rem = POST_REPAIR_CONFIRM_S - (now - d_node.post_repair_confirm_start)
                    pipeline_status["confirm_green_remaining_s"] = max(0.0, round(rem, 1))
                elif d_node.state in ["ADVANCED_FLASH", "DISRUPTIVE_CYCLE"] and d_node.sequence_start_time:
                    rem = 30.0 - (now - d_node.sequence_start_time)
                    pipeline_status["sequence_remaining_s"] = max(0.0, round(rem, 1))
                    cycle_len = 2.0 if d_node.state == "ADVANCED_FLASH" else 5.0
                    pipeline_status["cycle_progress_s"] = round((now - d_node.sequence_start_time) % cycle_len, 2)

            for idx, node in self.nodes.items():
                s = node.to_status_dict()
                pipeline_status["nodes"][f"light_node_{idx}"] = s

                # 1. Publish per-node status
                json_str = json.dumps(s)
                msg = stringmsg_pb2.StringMsg()
                msg.data = json_str
                self.gz_status_pubs[idx].publish(msg)

                if idx in self.ros2_status_pubs and self.ros2_node and rclpy.ok():
                    try:
                        r_msg = Ros2String()
                        r_msg.data = json_str
                        self.ros2_status_pubs[idx].publish(r_msg)
                    except Exception:
                        pass

            # 2. Publish overall pipeline status
            all_json = json.dumps(pipeline_status)
            p_msg = stringmsg_pb2.StringMsg()
            p_msg.data = all_json
            self.gz_pipeline_pub.publish(p_msg)

            if self.ros2_pipeline_pub and self.ros2_node and rclpy.ok():
                try:
                    r_all = Ros2String()
                    r_all.data = all_json
                    self.ros2_pipeline_pub.publish(r_all)
                except Exception:
                    pass

    def spin(self):
        """Main loop: state machine updates at 50 Hz, status broadcasts at 10 Hz."""
        def sig_handler(signum, frame):
            print("\n[Controller] Stopping controller...")
            self.running = False

        signal.signal(signal.SIGINT, sig_handler)
        signal.signal(signal.SIGTERM, sig_handler)

        print(f"[Controller] Starting RobotX Pipeline Controller (Tier={self.tier}, DamagedNode={self.damaged_node_id})...")
        self._apply_pipeline_roles()

        last_broadcast = 0.0
        try:
            while self.running:
                if self.ros2_node and rclpy.ok():
                    try:
                        rclpy.spin_once(self.ros2_node, timeout_sec=0.001)
                    except Exception:
                        pass

                self.update_state_machine()

                now = time.time()
                if now - last_broadcast >= 0.10: # 10 Hz
                    last_broadcast = now
                    self.broadcast_status()

                time.sleep(0.02) # 50 Hz
        except KeyboardInterrupt:
            print("\n[Controller] Stopping controller...")
        finally:
            if self.ros2_node:
                try:
                    self.ros2_node.destroy_node()
                except Exception:
                    pass
                if rclpy.ok():
                    try:
                        rclpy.shutdown()
                    except Exception:
                        pass


def main():
    world_name = os.environ.get("ROBOTX_WORLD_NAME", "ocean")
    if len(sys.argv) > 1 and not sys.argv[1].startswith("-"):
        world_name = sys.argv[1]

    controller = RobotXPipelineController(world_name=world_name)

    # CLI option overrides
    args = sys.argv[1:]
    for idx, arg in enumerate(args):
        if arg == "--tier" and idx + 1 < len(args):
            controller.set_tier(args[idx + 1])
        elif arg == "--damaged-node" and idx + 1 < len(args):
            controller.set_damaged_node(args[idx + 1])

    controller.discover_entities()
    controller.setup_subscriptions()
    controller.spin()


if __name__ == "__main__":
    main()
