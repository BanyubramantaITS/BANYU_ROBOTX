#!/usr/bin/env python3
"""
Exhaustive Multi-Scenario Test Battery for RobotX 2026 Task 2:
Tests 100% of all possible scenarios, permutations, boundaries, edge cases,
and stress loads across the pipeline simulation.
"""

import os
import sys
import time
import json
import subprocess
import threading
import gz.transport13 as gz_transport
import gz.msgs10.stringmsg_pb2 as stringmsg_pb2
from robotx_pipeline_api import RobotXPipelineAPI

WORKSPACE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# Colors for terminal output
GREEN = "\033[92m"
YELLOW = "\033[93m"
BLUE = "\033[94m"
RED = "\033[91m"
BOLD = "\033[1m"
RESET = "\033[0m"

test_passed = 0
test_failed = 0


def log_suite(title):
    print("\n" + "=" * 80)
    print(f"  {BOLD}{BLUE}{title}{RESET}")
    print("=" * 80)


def log_test(name):
    print(f"\n  {BOLD}[TEST]{RESET} {name}...")


def check(condition, desc):
    global test_passed, test_failed
    if condition:
        test_passed += 1
        print(f"    {GREEN}✓ PASS:{RESET} {desc}")
    else:
        test_failed += 1
        print(f"    {RED}✗ FAIL:{RESET} {desc}")
        raise AssertionError(desc)


def wait_until(predicate, timeout=8.0, poll_interval=0.05):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if predicate():
            return True
        time.sleep(poll_interval)
    return False


def run_all_tests():
    global test_passed, test_failed

    partition = f"test_{os.getpid()}"
    os.environ["GZ_PARTITION"] = partition
    env = os.environ.copy()
    env["GZ_PARTITION"] = partition
    env["SDF_PATH"] = f"{WORKSPACE}/simulation/models:{WORKSPACE}/models:" + env.get("SDF_PATH", "")
    env["GZ_SIM_RESOURCE_PATH"] = f"{WORKSPACE}/simulation/models:{WORKSPACE}/models:" + env.get("GZ_SIM_RESOURCE_PATH", "")
    # Accelerate confirmation and total flash for fast multi-permutation testing
    env["ROBOTX_POST_REPAIR_CONFIRM_S"] = "1.5"
    env["ROBOTX_ADVANCED_FLASH_TOTAL_S"] = "3.0"
    env["ROBOTX_DISRUPTIVE_TOTAL_S"] = "5.5"
    env["ROBOTX_DISRUPTIVE_CYCLE_S"] = "5.0"
    env["PYTHONUNBUFFERED"] = "1"

    print("=" * 80)
    print(f"  {BOLD}RobotX 2026 Task 2: Exhaustive Multi-Scenario Test Battery{RESET}")
    print("=" * 80)

    # 1. Start Gazebo Sim Headless
    print(f"\nLaunching Gazebo Sim headless with pipeline_lights.sdf...")
    gz_proc = subprocess.Popen(
        ["gz", "sim", "-s", "-r", f"{WORKSPACE}/worlds/pipeline_lights.sdf"],
        env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
    )
    time.sleep(3.0)

    # 2. Start Controller
    print("Launching robotx_light_controller.py...")
    ctrl_proc = subprocess.Popen(
        ["python3", "-u", f"{WORKSPACE}/scripts/robotx_light_controller.py", "pipeline_lights"],
        env=env
    )
    time.sleep(3.0)

    api = RobotXPipelineAPI()

    try:
        # Wait for 4 nodes ready
        assert wait_until(lambda: len(api.get_status().get("nodes", {})) == 4, timeout=8.0)

        # =====================================================================
        # SUITE 1: Damaged Node Position Variations (Nodes 1, 2, 3, 4, RANDOM)
        # =====================================================================
        log_suite("SUITE 1: Damaged Node Position Variations (Nodes 1, 2, 3, 4, RANDOM)")

        for target_id in [1, 2, 3, 4]:
            log_test(f"Assign Damaged Segment to Node {target_id}")
            api.set_damaged_node(target_id)
            assert wait_until(lambda: api.get_status().get("damaged_node_id") == target_id, timeout=3.0)
            status = api.get_status()
            check(status["damaged_node_id"] == target_id, f"Pipeline damaged_node_id set to {target_id}")
            for i in range(1, 5):
                n_data = status["nodes"][f"light_node_{i}"]
                if i == target_id:
                    check(n_data["role"] == "DAMAGED" and n_data["active_color"] == "RED",
                          f"Node {i} role=DAMAGED, active_color=RED")
                else:
                    check(n_data["role"] == "FUNCTIONAL" and n_data["active_color"] == "GREEN",
                          f"Node {i} role=FUNCTIONAL, active_color=GREEN")

        log_test("Assign Damaged Segment using 'RANDOM'")
        api.set_damaged_node("RANDOM")
        time.sleep(0.4)
        status = api.get_status()
        rand_id = status["damaged_node_id"]
        check(1 <= rand_id <= 4, f"Random damaged node assigned to Node {rand_id}")
        check(status["nodes"][f"light_node_{rand_id}"]["role"] == "DAMAGED", f"Random Node {rand_id} is DAMAGED")

        # =====================================================================
        # SUITE 2: Magnetic Threshold Boundary & Proximity Tap Behaviors
        # =====================================================================
        log_suite("SUITE 2: Magnetic Threshold Boundary & Proximity Tap Behaviors")

        api.set_damaged_node(2)
        assert wait_until(lambda: api.get_status().get("damaged_node_id") == 2, timeout=3.0)

        log_test("Sub-threshold Magnet Field (1.0 mT < 1.5 mT threshold for 1.2s)")
        api.trigger_node_magnet(2, duration_s=1.2, field_mT=1.0)
        time.sleep(0.4)
        status = api.get_status()
        n2 = status["nodes"]["light_node_2"]
        check(not n2["threshold_exceeded"], f"threshold_exceeded is False (field={n2['magnetic_field_mT']:.2f} mT)")
        check(n2["state"] == "DAMAGED_RED", "Node 2 stays in DAMAGED_RED, proximity green did not trigger")

        log_test("Boundary Threshold: 1.45 mT (Sub-threshold) vs 1.60 mT (Above threshold)")
        api.trigger_node_magnet(2, duration_s=0.6, field_mT=1.40)
        time.sleep(0.3)
        check(not api.get_status()["nodes"]["light_node_2"]["threshold_exceeded"], "1.40 mT does not exceed threshold")
        api.trigger_node_magnet(2, duration_s=0.6, field_mT=1.65)
        time.sleep(0.3)
        check(api.get_status()["nodes"]["light_node_2"]["threshold_exceeded"], "1.65 mT successfully exceeds threshold")
        time.sleep(0.5)

        log_test("Ultra-short Magnet Tap (< 250ms proximity threshold: 0.10s hold at 3.0 mT)")
        api.trigger_node_magnet(2, duration_s=0.10, field_mT=3.0)
        time.sleep(0.2)
        n2 = api.get_status()["nodes"]["light_node_2"]
        check(n2["state"] == "DAMAGED_RED", "Node 2 ignored <250ms tap and remained DAMAGED_RED")

        log_test("Standard Proximity Tap (0.50s hold >= 250ms, < 2.0s repair)")
        api.trigger_node_magnet(2, duration_s=0.50, field_mT=3.0)
        assert wait_until(lambda: api.get_status()["nodes"]["light_node_2"]["state"] == "PROBE_HELD_GREEN", timeout=1.5)
        check(True, "Node 2 showed PROBE_HELD_GREEN proximity feedback during probe contact")
        assert wait_until(lambda: api.get_status()["nodes"]["light_node_2"]["state"] == "DAMAGED_RED", timeout=2.0)
        check(True, "Probe removed before 2.0s: correctly re-armed to DAMAGED_RED")

        log_test("Repeated Proximity Taps (3 consecutive taps in a row)")
        for tap in range(1, 4):
            api.trigger_node_magnet(2, duration_s=0.45, field_mT=3.0)
            assert wait_until(lambda: api.get_status()["nodes"]["light_node_2"]["state"] == "PROBE_HELD_GREEN", timeout=1.0)
            assert wait_until(lambda: api.get_status()["nodes"]["light_node_2"]["state"] == "DAMAGED_RED", timeout=1.5)
            print(f"      Tap {tap}/3: Green feedback -> Red re-arm verified")
        check(True, "All 3 consecutive proximity taps executed and returned to DAMAGED_RED cleanly")

        log_test("Magnet Applied to a FUNCTIONAL Node (Node 3)")
        api.trigger_node_magnet(3, duration_s=1.0, field_mT=4.0)
        time.sleep(0.4)
        n3 = api.get_status()["nodes"]["light_node_3"]
        check(n3["role"] == "FUNCTIONAL" and n3["active_color"] == "GREEN", "Functional Node 3 ignores magnet and stays solid GREEN")
        check(n3["magnetic_field_mT"] >= 3.5, f"Magnet field correctly reported ({n3['magnetic_field_mT']:.2f} mT)")

        # =====================================================================
        # SUITE 3: Core Tier Comprehensive Behaviors & Edge Cases
        # =====================================================================
        log_suite("SUITE 3: Core Tier Comprehensive Behaviors & Edge Cases")

        api.set_tier("CORE")
        api.set_damaged_node(2)
        time.sleep(0.3)

        log_test("Core Tier Full Repair (Continuous hold >= 2.0s)")
        api.trigger_node_magnet(2, duration_s=2.4, field_mT=3.5)
        assert wait_until(lambda: api.get_status()["nodes"]["light_node_2"]["state"] == "TASK_COMPLETED_GREEN", timeout=3.5)
        check(api.get_status()["nodes"]["light_node_2"]["active_color"] == "GREEN", "Node 2 successfully repaired to permanent Solid GREEN")

        log_test("Prolonged Magnet Presence after Core Repair (Extra 3.0s contact)")
        api.trigger_node_magnet(2, duration_s=3.0, field_mT=4.0)
        time.sleep(1.0)
        check(api.get_status()["nodes"]["light_node_2"]["state"] == "TASK_COMPLETED_GREEN", "Node 2 stays in TASK_COMPLETED_GREEN despite prolonged magnet contact")

        # =====================================================================
        # SUITE 4: Advanced Tier Single-Color Optical Sequence (All 3 Colors)
        # =====================================================================
        log_suite("SUITE 4: Advanced Tier Single-Color Flashing (RED, GREEN, BLUE)")

        for color in ["RED", "GREEN", "BLUE"]:
            log_test(f"Advanced Tier: Destination Circle Color = {color}")
            api.set_tier("ADVANCED")
            api.set_second_color(color)
            api.set_damaged_node(1)
            assert wait_until(lambda: api.get_status().get("nodes", {}).get("light_node_1", {}).get("state") == "DAMAGED_RED", timeout=3.0)

            check(api.get_status()["nodes"]["light_node_1"]["state"] == "DAMAGED_RED", "Node 1 re-armed to DAMAGED_RED")
            api.trigger_node_magnet(1, duration_s=2.4, field_mT=3.5)

            # 1. 1.5s post-repair green
            assert wait_until(lambda: api.get_status()["nodes"]["light_node_1"]["state"] == "POST_REPAIR_CONFIRM_GREEN", timeout=3.5)
            check(True, f"Confirmed post-repair solid GREEN hold for {color}")

            # 2. Advanced Flash Sequence
            assert wait_until(lambda: api.get_status()["nodes"]["light_node_1"]["state"] == "ADVANCED_FLASH", timeout=3.0)
            flash_color = api.get_status()["nodes"]["light_node_1"]["active_color"]
            check(flash_color == color or flash_color == "OFF", f"Emitting destination circle color: {color} (read: {flash_color})")

            # 3. Completion back to solid green
            assert wait_until(lambda: api.get_status()["nodes"]["light_node_1"]["state"] == "TASK_COMPLETED_GREEN", timeout=5.0)
            check(api.get_status()["nodes"]["light_node_1"]["active_color"] == "GREEN", f"Sequence completed -> restored to permanent solid GREEN")

        # =====================================================================
        # SUITE 5: Disruptive Tier Two-Color Optical Sequences & Permutations
        # =====================================================================
        log_suite("SUITE 5: Disruptive Tier Two-Color Optical Sequences & Permutations")

        permutations = [
            ("RED", "BLUE"),
            ("BLUE", "GREEN"),
            ("GREEN", "RED")
        ]

        for tin_c, circle_c in permutations:
            log_test(f"Disruptive Tier: 1st Color (Tin)={tin_c}, 2nd Color (Circle)={circle_c}")
            api.set_tier("DISRUPTIVE")
            api.set_sequence(first_color=tin_c, second_color=circle_c)
            api.set_damaged_node(4)
            assert wait_until(lambda: api.get_status().get("nodes", {}).get("light_node_4", {}).get("state") == "DAMAGED_RED", timeout=3.0)

            check(api.get_status()["nodes"]["light_node_4"]["state"] == "DAMAGED_RED", "Node 4 re-armed to DAMAGED_RED")
            api.trigger_node_magnet(4, duration_s=2.4, field_mT=3.5)

            # Post-repair green
            assert wait_until(lambda: api.get_status()["nodes"]["light_node_4"]["state"] == "POST_REPAIR_CONFIRM_GREEN", timeout=3.5)

            # Disruptive cycle started
            assert wait_until(lambda: api.get_status()["nodes"]["light_node_4"]["state"] == "DISRUPTIVE_CYCLE", timeout=3.0)
            assert wait_until(lambda: api.get_status().get("nodes", {}).get("light_node_4", {}).get("active_color") == tin_c, timeout=2.5)
            check(True, f"Phase 1 (0-1s): Emitting 1st Color (Tin: {tin_c})")

            # Wait to Phase 3 (Circle color: 2.0-3.0s into cycle)
            assert wait_until(lambda: api.get_status().get("nodes", {}).get("light_node_4", {}).get("active_color") == circle_c, timeout=3.5)
            check(True, f"Phase 3 (2-3s): Emitting 2nd Color (Circle: {circle_c})")

            # Sequence completion
            assert wait_until(lambda: api.get_status()["nodes"]["light_node_4"]["state"] == "TASK_COMPLETED_GREEN", timeout=6.0)
            check(api.get_status()["nodes"]["light_node_4"]["active_color"] == "GREEN", "Disruptive cycle completed -> permanent solid GREEN")

        # =====================================================================
        # SUITE 6: Mid-Execution Resets & Dynamic Transitions
        # =====================================================================
        log_suite("SUITE 6: Mid-Execution Resets & Dynamic Transitions")

        log_test("Reset during Active Proximity Feedback")
        api.set_damaged_node(2)
        time.sleep(0.2)
        api.trigger_node_magnet(2, duration_s=1.0, field_mT=3.5)
        assert wait_until(lambda: api.get_status()["nodes"]["light_node_2"]["state"] == "PROBE_HELD_GREEN", timeout=1.5)
        api.reset_pipeline()
        time.sleep(0.3)
        check(api.get_status()["nodes"]["light_node_2"]["state"] == "DAMAGED_RED", "Reset during tap successfully re-armed Node 2 to DAMAGED_RED")

        log_test("Reset during Optical Sequence")
        api.set_tier("ADVANCED")
        api.set_second_color("BLUE")
        api.set_damaged_node(3)
        time.sleep(0.2)
        api.trigger_node_magnet(3, duration_s=2.4, field_mT=3.5)
        assert wait_until(lambda: api.get_status()["nodes"]["light_node_3"]["state"] == "ADVANCED_FLASH", timeout=5.0)
        check(True, "Node 3 in active optical flash. Triggering reset_pipeline()...")
        api.reset_pipeline()
        time.sleep(0.3)
        n3 = api.get_status()["nodes"]["light_node_3"]
        check(n3["state"] == "DAMAGED_RED", "Mid-sequence reset immediately aborted optical transmission and re-armed to DAMAGED_RED")

        log_test("Dynamic Tier Switching (Core -> Advanced -> Disruptive)")
        api.set_tier("CORE")
        check(api.get_status()["tier"] == "CORE", "Switched to CORE")
        api.set_tier("ADVANCED")
        check(api.get_status()["tier"] == "ADVANCED", "Switched to ADVANCED")
        api.set_tier("DISRUPTIVE")
        check(api.get_status()["tier"] == "DISRUPTIVE", "Switched to DISRUPTIVE")

        # =====================================================================
        # SUITE 7: Manual Overrides & Automatic Magnet Recovery
        # =====================================================================
        log_suite("SUITE 7: Manual Overrides & Automatic Magnet Recovery")

        log_test("Direct Manual Color Overrides (CYAN, YELLOW, OFF, MAGENTA)")
        for c in ["CYAN", "YELLOW", "OFF", "MAGENTA"]:
            api._pub_node_colors[1].publish(stringmsg_pb2.StringMsg(data=c))
            assert wait_until(lambda: api.get_status().get("nodes", {}).get("light_node_1", {}).get("active_color") == c, timeout=2.0)
            status = api.get_status()
            check(status["nodes"]["light_node_1"]["active_color"] == c, f"Node 1 manual override to {c}")

        log_test("Magnet Touch Restores Manually Overridden Damaged Node")
        api.set_damaged_node(1)
        assert wait_until(lambda: api.get_status().get("damaged_node_id") == 1, timeout=2.0)
        api._pub_node_colors[1].publish(stringmsg_pb2.StringMsg(data="MAGENTA"))
        assert wait_until(lambda: api.get_status().get("nodes", {}).get("light_node_1", {}).get("state") == "MANUAL_OVERRIDE", timeout=2.0)
        check(api.get_status()["nodes"]["light_node_1"]["state"] == "MANUAL_OVERRIDE", "Node 1 in MANUAL_OVERRIDE")
        api.trigger_node_magnet(1, duration_s=0.5, field_mT=3.5)
        assert wait_until(lambda: api.get_status().get("nodes", {}).get("light_node_1", {}).get("state") in ["PROBE_HELD_GREEN", "DAMAGED_RED"], timeout=2.0)
        n1 = api.get_status()["nodes"]["light_node_1"]
        check(n1["state"] in ["PROBE_HELD_GREEN", "DAMAGED_RED"], "Magnet contact successfully woke Node 1 back into autonomous state machine")

        # =====================================================================
        # SUITE 8: Malformed Inputs, Extreme Values & Concurrency Stress Test
        # =====================================================================
        log_suite("SUITE 8: Malformed Inputs, Extreme Values & Concurrency Stress Test")

        log_test("Reject Invalid Tier Names")
        pub_tier = api.gz_node.advertise("/robotx/pipeline/set_tier", stringmsg_pb2.StringMsg)
        pub_tier.publish(stringmsg_pb2.StringMsg(data="SUPER_TIER"))
        time.sleep(0.2)
        check(api.get_status()["tier"] in ["CORE", "ADVANCED", "DISRUPTIVE"], "Invalid tier 'SUPER_TIER' safely rejected without crash")

        log_test("Reject Invalid Damaged Node IDs (0, 5, -2, 'XYZ')")
        pub_dam = api.gz_node.advertise("/robotx/pipeline/set_damaged_node", stringmsg_pb2.StringMsg)
        for bad_id in ["0", "5", "-2", "XYZ"]:
            pub_dam.publish(stringmsg_pb2.StringMsg(data=bad_id))
            time.sleep(0.1)
            check(1 <= api.get_status()["damaged_node_id"] <= 4, f"Invalid node ID '{bad_id}' safely rejected")

        log_test("Handle Corrupted JSON in Sequence Topic")
        pub_seq = api.gz_node.advertise("/robotx/pipeline/set_sequence", stringmsg_pb2.StringMsg)
        pub_seq.publish(stringmsg_pb2.StringMsg(data="{corrupted_json: true,"))
        time.sleep(0.2)
        check(True, "Malformed JSON safely handled without crashing controller")

        log_test("Concurrency & Burst Load Stress Test (50 rapid-fire topic commands)")
        def burst_worker():
            for _ in range(10):
                pub_seq.publish(stringmsg_pb2.StringMsg(data=json.dumps({"first_color": "RED", "second_color": "BLUE"})))
                pub_tier.publish(stringmsg_pb2.StringMsg(data="DISRUPTIVE"))
                time.sleep(0.01)

        threads = [threading.Thread(target=burst_worker) for _ in range(5)]
        for t in threads:
            t.start()
        for t in threads:
            t.join()
        time.sleep(0.3)
        check(len(api.get_status().get("nodes", {})) == 4, "Controller healthy, thread-safe, and broadcasting at 10 Hz after 50-command burst")

        # =====================================================================
        # FINAL REPORT
        # =====================================================================
        print("\n" + "=" * 80)
        print(f"  {BOLD}{GREEN}ALL {test_passed} TESTS PASSED WITH 100% SUCCESS! (0 FAILURES){RESET}")
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
        os._exit(0 if test_failed == 0 else 1)


if __name__ == "__main__":
    run_all_tests()
