#!/usr/bin/env python3
"""
RobotX Pipeline API: Python client library to control the 4-node underwater pipeline simulation.

Allows switching tiers (Core, Advanced, Disruptive), setting which node is damaged,
configuring the optical color sequence (1st color tin, 2nd color target circle),
simulating magnetic repairs, and monitoring live status.
"""

import time
import json
import gz.transport13 as gz_transport
import gz.msgs10.stringmsg_pb2 as stringmsg_pb2


class RobotXPipelineAPI:
    def __init__(self, node=None):
        self.gz_node = node if node is not None else gz_transport.Node()
        self._latest_status = {}

        # Publishers
        self._pub_tier = self.gz_node.advertise("/robotx/pipeline/set_tier", stringmsg_pb2.StringMsg)
        self._pub_damaged = self.gz_node.advertise("/robotx/pipeline/set_damaged_node", stringmsg_pb2.StringMsg)
        self._pub_sequence = self.gz_node.advertise("/robotx/pipeline/set_sequence", stringmsg_pb2.StringMsg)
        self._pub_first_color = self.gz_node.advertise("/robotx/pipeline/set_first_color", stringmsg_pb2.StringMsg)
        self._pub_second_color = self.gz_node.advertise("/robotx/pipeline/set_second_color", stringmsg_pb2.StringMsg)
        self._pub_activate = self.gz_node.advertise("/robotx/pipeline/activate", stringmsg_pb2.StringMsg)
        self._pub_reset = self.gz_node.advertise("/robotx/pipeline/reset", stringmsg_pb2.StringMsg)
        self._pub_repair = self.gz_node.advertise("/robotx/pipeline/repair_damaged", stringmsg_pb2.StringMsg)

        # Pre-advertise per-node command topics
        self._pub_node_mags = {
            i: self.gz_node.advertise(f"/robotx/light_node_{i}/trigger_magnet", stringmsg_pb2.StringMsg) for i in range(1, 5)
        }
        self._pub_node_colors = {
            i: self.gz_node.advertise(f"/robotx/light_node_{i}/set_color", stringmsg_pb2.StringMsg) for i in range(1, 5)
        }

        # Status subscriber
        def _status_cb(msg):
            try:
                self._latest_status = json.loads(msg.data)
            except Exception:
                pass
        self.gz_node.subscribe(stringmsg_pb2.StringMsg, "/robotx/pipeline/status", _status_cb)

        # Allow time for initial discovery
        time.sleep(0.5)

    def set_tier(self, tier: str):
        """Switch competition tier: 'CORE', 'ADVANCED', or 'DISRUPTIVE'."""
        t = tier.upper().strip()
        if t not in ["CORE", "ADVANCED", "DISRUPTIVE"]:
            raise ValueError(f"Invalid tier '{tier}'. Must be 'CORE', 'ADVANCED', or 'DISRUPTIVE'.")
        msg = stringmsg_pb2.StringMsg()
        msg.data = t
        self._pub_tier.publish(msg)
        time.sleep(0.2)
        return self

    def set_damaged_node(self, node_id):
        """Set which pipeline segment is damaged: 1, 2, 3, 4, or 'RANDOM'."""
        msg = stringmsg_pb2.StringMsg()
        msg.data = str(node_id)
        self._pub_damaged.publish(msg)
        time.sleep(0.2)
        return self

    def set_sequence(self, first_color: str = None, second_color: str = None, tier: str = None):
        """
        Configure optical sequence:
        - first_color: WHICH Resource Tin (RED, GREEN, or BLUE).
        - second_color: WHICH Target Circle destination (RED, GREEN, or BLUE).
        """
        payload = {}
        if tier:
            payload["tier"] = tier.upper()
        if first_color:
            payload["first_color"] = first_color.upper()
        if second_color:
            payload["second_color"] = second_color.upper()
        msg = stringmsg_pb2.StringMsg()
        msg.data = json.dumps(payload)
        self._pub_sequence.publish(msg)
        time.sleep(0.2)
        return self

    def set_first_color(self, color: str):
        """Set Disruptive 1st color (Resource Tin: 'RED', 'GREEN', or 'BLUE')."""
        msg = stringmsg_pb2.StringMsg()
        msg.data = color.upper()
        self._pub_first_color.publish(msg)
        time.sleep(0.1)
        return self

    def set_second_color(self, color: str):
        """Set destination circle color ('RED', 'GREEN', or 'BLUE')."""
        msg = stringmsg_pb2.StringMsg()
        msg.data = color.upper()
        self._pub_second_color.publish(msg)
        time.sleep(0.1)
        return self

    def activate_challenge(self):
        """Activate the challenge scenario with 1 RED damaged node and 3 GREEN functional nodes."""
        msg = stringmsg_pb2.StringMsg()
        msg.data = "ACTIVATE"
        self._pub_activate.publish(msg)
        time.sleep(0.2)
        return self

    def reset_pipeline(self):
        """Reset pipeline to initial state."""
        msg = stringmsg_pb2.StringMsg()
        msg.data = "RESET"
        self._pub_reset.publish(msg)
        time.sleep(0.2)
        return self

    def repair_damaged(self, duration_s: float = 2.5, field_mT: float = 3.5):
        """Simulate applying magnetic probe to the currently damaged segment."""
        msg = stringmsg_pb2.StringMsg()
        msg.data = json.dumps({"duration_s": duration_s, "field_mT": field_mT})
        self._pub_repair.publish(msg)
        time.sleep(0.1)
        return self

    def trigger_node_magnet(self, node_id: int, duration_s: float = 2.5, field_mT: float = 3.5):
        """Simulate applying magnet to a specific node ID."""
        if node_id in self._pub_node_mags:
            msg = stringmsg_pb2.StringMsg()
            msg.data = json.dumps({"duration_s": duration_s, "field_mT": field_mT})
            self._pub_node_mags[node_id].publish(msg)
            time.sleep(0.05)
        return self

    def get_status(self, timeout: float = 2.0):
        """Retrieve latest pipeline status snapshot dictionary."""
        t0 = time.time()
        while time.time() - t0 < timeout and not self._latest_status:
            time.sleep(0.05)
        return self._latest_status
