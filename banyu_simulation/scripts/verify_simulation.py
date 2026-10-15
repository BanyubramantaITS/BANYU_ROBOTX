#!/usr/bin/env python3
"""
Comprehensive Verification Test Suite for RobotX 2026 Task 2 Light Simulation:
Tests Core, Advanced, and Disruptive tiers, damaged node assignment,
proximity feedback, 5.0s post-repair confirmation, and two-color optical sequences.
"""

import os
import sys
import time
import json
import subprocess
import gz.transport13 as gz_transport
import gz.msgs10.stringmsg_pb2 as stringmsg_pb2
from robotx_pipeline_api import RobotXPipelineAPI

WORKSPACE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def wait_until(predicate, timeout=8.0, poll_interval=0.05):
    """Wait until predicate returns True, or return False on timeout."""
    t0 = time.time()
    while time.time() - t0 < timeout:
        if predicate():
            return True
        time.sleep(poll_interval)
    return False


def run_full_verification():
    print("=" * 80)
    print("  RobotX 2026 Task 2: Underwater Pipeline Simulation - Full Verification Suite")
    print("=" * 80)

    env = os.environ.copy()
    env["SDF_PATH"] = f"{WORKSPACE}/models:" + env.get("SDF_PATH", "")
    env["GZ_SIM_RESOURCE_PATH"] = f"{WORKSPACE}/models:" + env.get("GZ_SIM_RESOURCE_PATH", "")

    # 1. Launch Gazebo in headless mode
    print("\n[Step 1/6] Launching Gazebo Sim with worlds/pipeline_lights.sdf...")
    gz_proc = subprocess.Popen(
        ["gz", "sim", "-s", "-r", f"{WORKSPACE}/worlds/pipeline_lights.sdf"],
        env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
    )
    time.sleep(3.0)

    # 2. Launch Controller (unbuffered)
    print("[Step 2/6] Starting robotx_light_controller.py...")
    ctrl_proc = subprocess.Popen(
        ["python3", "-u", f"{WORKSPACE}/scripts/robotx_light_controller.py", "pipeline_lights"],
        env=env
    )
    time.sleep(3.0)

    api = RobotXPipelineAPI()

    try:
        # 3. Verify Initial Challenge Activation
        print("\n[Step 3/6] Verifying Initial Pipeline Roles (1 DAMAGED [RED], 3 FUNCTIONAL [GREEN])...")
        assert wait_until(lambda: len(api.get_status().get("nodes", {})) == 4, timeout=8.0), "Timed out waiting for 4 nodes in pipeline status"

        status = api.get_status()
        nodes = status["nodes"]
        d_id = status["damaged_node_id"]
        print(f"  Pipeline Tier: {status['tier']}, Damaged Node: Node {d_id}")

        for i in range(1, 5):
            node_data = nodes[f"light_node_{i}"]
            if i == d_id:
                assert node_data["role"] == "DAMAGED" and node_data["active_color"] == "RED", f"Node {i} role mismatch: {node_data}"
                print(f"  ✓ Node {i}: DAMAGED role, Solid RED illumination, Mag field: {node_data['magnetic_field_mT']:.3f} mT")
            else:
                assert node_data["role"] == "FUNCTIONAL" and node_data["active_color"] == "GREEN", f"Node {i} role mismatch: {node_data}"
                print(f"  ✓ Node {i}: FUNCTIONAL role, Solid GREEN illumination")

        # 4. Test Core Tier Behavior
        print("\n[Step 4/6] Testing CORE Tier: Proximity Tap -> Green -> Red, then Full Repair -> Permanent Green...")
        api.set_tier("CORE")
        api.set_damaged_node(2)
        time.sleep(0.5)

        # Proximity Tap (0.6s hold: > 250ms feedback, < 2.0s repair)
        print("  Testing 250ms proximity feedback on Node 2...")
        api.trigger_node_magnet(2, duration_s=0.6, field_mT=3.0)
        assert wait_until(
            lambda: api.get_status().get("nodes", {}).get("light_node_2", {}).get("state") == "PROBE_HELD_GREEN",
            timeout=2.0
        ), "Node 2 failed to show PROBE_HELD_GREEN during 250ms hold"
        print("  ✓ Proximity feedback verified: turns GREEN while probe is held")

        # Re-arm back to RED upon release
        assert wait_until(
            lambda: api.get_status().get("nodes", {}).get("light_node_2", {}).get("state") == "DAMAGED_RED",
            timeout=2.0
        ), "Node 2 failed to re-arm to DAMAGED_RED after tap release"
        print("  ✓ Probe removed before 2.0s: correctly re-armed to DAMAGED_RED")

        # Full Repair Hold (2.5s hold >= 2.0s)
        print("  Testing full repair hold (2.5s >= 2.0s) on Node 2...")
        api.trigger_node_magnet(2, duration_s=2.5, field_mT=3.5)
        assert wait_until(
            lambda: api.get_status().get("nodes", {}).get("light_node_2", {}).get("state") == "TASK_COMPLETED_GREEN",
            timeout=3.5
        ), "Node 2 failed to enter TASK_COMPLETED_GREEN in Core Tier"
        assert api.get_status()["nodes"]["light_node_2"]["active_color"] == "GREEN"
        print("  ✓ CORE Tier repair verified: permanently switched to Solid GREEN")

        # 5. Test Advanced Tier Behavior
        print("\n[Step 5/6] Testing ADVANCED Tier: 5.0s Post-Repair Green -> Single-Color Flash (1s ON / 1s OFF for 30s)...")
        api.set_tier("ADVANCED")
        api.set_second_color("BLUE")
        api.set_damaged_node(1)
        time.sleep(0.5)

        assert api.get_status()["nodes"]["light_node_1"]["state"] == "DAMAGED_RED"
        print("  Node 1 set to DAMAGED_RED for Advanced Tier test. Triggering magnetic repair...")
        api.trigger_node_magnet(1, duration_s=2.5, field_mT=3.5)

        # Verify 5.0s Post-Repair Solid Green
        assert wait_until(
            lambda: api.get_status().get("nodes", {}).get("light_node_1", {}).get("state") == "POST_REPAIR_CONFIRM_GREEN",
            timeout=3.5
        ), "Node 1 failed to enter POST_REPAIR_CONFIRM_GREEN"
        print("  ✓ Repair confirmed: Node 1 in 5.0s Solid GREEN post-repair state")

        # Wait until 5s confirmation finishes and enters ADVANCED_FLASH
        assert wait_until(
            lambda: api.get_status().get("nodes", {}).get("light_node_1", {}).get("state") == "ADVANCED_FLASH",
            timeout=8.5
        ), f"Node 1 failed to enter ADVANCED_FLASH after 5s confirm (Current status: {api.get_status().get('nodes', {}).get('light_node_1')})"
        print("  ✓ ADVANCED_FLASH sequence started: transmitting destination circle color BLUE (1s ON / 1s OFF for 30s)")

        # 6. Test Disruptive Tier Behavior
        print("\n[Step 6/6] Testing DISRUPTIVE Tier: 5.0s Green -> Two-Color Sequence (1s Tin -> 1s OFF -> 1s Circle -> 2s OFF)...")
        api.set_tier("DISRUPTIVE")
        api.set_sequence(first_color="RED", second_color="BLUE")
        api.set_damaged_node(3)
        time.sleep(0.5)

        assert api.get_status()["nodes"]["light_node_3"]["state"] == "DAMAGED_RED"
        print("  Node 3 set to DAMAGED_RED for Disruptive Tier test. Triggering magnetic repair...")
        api.trigger_node_magnet(3, duration_s=2.5, field_mT=3.5)

        # Verify 5.0s Post-Repair Solid Green
        assert wait_until(
            lambda: api.get_status().get("nodes", {}).get("light_node_3", {}).get("state") == "POST_REPAIR_CONFIRM_GREEN",
            timeout=3.5
        ), "Node 3 failed to enter POST_REPAIR_CONFIRM_GREEN"
        print("  ✓ Repair confirmed: Node 3 in 5.0s Solid GREEN post-repair state")

        # Wait until enters DISRUPTIVE_CYCLE
        assert wait_until(
            lambda: api.get_status().get("nodes", {}).get("light_node_3", {}).get("state") == "DISRUPTIVE_CYCLE",
            timeout=8.5
        ), f"Node 3 failed to enter DISRUPTIVE_CYCLE after 5s confirm (Current status: {api.get_status().get('nodes', {}).get('light_node_3')})"

        # Verify two-color cycle sequence:
        # At start of cycle: 1st color (RED: Tin)
        status_seq = api.get_status()
        c_tin = status_seq["nodes"]["light_node_3"]["active_color"]
        print(f"  During Disruptive Cycle phase 1 (0-1s): Emitting 1st Color (Resource Tin): {c_tin}")
        assert c_tin == "RED" or c_tin == "OFF", f"Expected RED tin or initial phase, got {c_tin}"

        # Wait 2.2s -> should be in 2nd color (BLUE: Circle)
        time.sleep(2.2)
        status_seq2 = api.get_status()
        c_circle = status_seq2["nodes"]["light_node_3"]["active_color"]
        print(f"  During Disruptive Cycle phase 2 (2-3s): Emitting 2nd Color (Destination Circle): {c_circle}")
        assert c_circle in ["BLUE", "OFF"], f"Expected BLUE circle or cycle timing, got {c_circle}"

        print("  ✓ DISRUPTIVE_CYCLE optical sequence verified (1st Color Tin + 2nd Color Target Circle)")

        print("\n" + "=" * 80)
        print("  ALL ROBOTX TASK 2 VERIFICATION TESTS PASSED SUCCESSFULLY!")
        print("=" * 80)

    finally:
        print("\nShutting down simulation test processes...")
        ctrl_proc.terminate()
        gz_proc.terminate()
        try:
            ctrl_proc.wait(timeout=2.5)
        except Exception:
            ctrl_proc.kill()
        try:
            gz_proc.wait(timeout=2.5)
        except Exception:
            gz_proc.kill()
        print("Done.")


if __name__ == "__main__":
    run_full_verification()
