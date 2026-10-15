#!/usr/bin/env python3
"""
RobotX Pipeline CLI Tester & Monitor:
Interactive tool to switch tiers (Core, Advanced, Disruptive), configure color sequences,
designate damaged segments, simulate magnetic probe repair, and monitor live conditions.
"""

import sys
import time
import json
import argparse
import gz.transport13 as gz_transport
import gz.msgs10.stringmsg_pb2 as stringmsg_pb2
from robotx_pipeline_api import RobotXPipelineAPI


def print_status_table(pipeline_data):
    """Render a clean ASCII status table with Tier, Sequence, and Node info."""
    tier = pipeline_data.get("tier", "UNKNOWN")
    d_id = pipeline_data.get("damaged_node_id", "?")
    seq = pipeline_data.get("sequence_config", {})
    t_color = seq.get("disruptive_first_color_tin", "?")
    c_color = seq.get("disruptive_second_color_circle", "?")
    adv_c = seq.get("advanced_target_color", "?")

    print("\n" + "=" * 103)
    print(f"  ROBOTX PIPELINE TASK 2 STATUS  |  TIER: {tier:<11}  |  DAMAGED SEGMENT: Node {d_id}")
    if tier == "DISRUPTIVE":
        print(f"  OPTICAL SEQUENCE: 1st Color (Tin)={t_color}  |  2nd Color (Circle)={c_color}  (1s-1s-1s-2s cycle)")
    elif tier == "ADVANCED":
        print(f"  OPTICAL SEQUENCE: Destination Circle={adv_c} (1s ON / 1s OFF for 30s)")
    else:
        print("  CORE TIER: Damaged segment repairs to permanent Solid GREEN")

    # Extra phase countdown info
    if "confirm_green_remaining_s" in pipeline_data:
        print(f"  POST-REPAIR STATUS: Confirming GREEN (Remaining: {pipeline_data['confirm_green_remaining_s']}s)")
    elif "sequence_remaining_s" in pipeline_data:
        print(f"  OPTICAL BROADCAST: Active (Remaining: {pipeline_data['sequence_remaining_s']}s, Cycle progress: {pipeline_data.get('cycle_progress_s', 0)}s)")

    print("-" * 103)
    print(f"{'Node':<17} | {'Role':<12} | {'State':<26} | {'Color':<10} | {'Field (mT)':<13} | {'Trigger':<10}")
    print("-" * 103)

    nodes = pipeline_data.get("nodes", {})
    for name in sorted(nodes.keys()):
        node = nodes[name]
        nid = node.get("node_id", "?")
        role = node.get("role", "FUNCTIONAL")
        state = node.get("state", "UNKNOWN")
        color = node.get("active_color", "UNKNOWN")
        field = node.get("magnetic_field_mT", 0.0)
        source = node.get("trigger_source", "NONE")
        exceeded = "(!)" if node.get("threshold_exceeded", False) else "   "
        role_marker = "[DAMAGED] " if role == "DAMAGED" else "          "
        print(f"Node {nid} {role_marker} | {role:<12} | {state:<26} | {color:<10} | {field:>6.3f} mT {exceeded} | {source:<10}")
    print("=" * 103)


def print_snapshot(timeout=1.5):
    """Grab a single snapshot from /robotx/pipeline/status and display table."""
    node = gz_transport.Node()
    received = {}

    def cb(msg):
        nonlocal received
        try:
            received = json.loads(msg.data)
        except Exception:
            pass

    node.subscribe(stringmsg_pb2.StringMsg, "/robotx/pipeline/status", cb)
    t0 = time.time()
    while time.time() - t0 < timeout and not received:
        time.sleep(0.05)

    if received:
        print_status_table(received)


def monitor_pipeline(duration=30.0):
    """Monitor and print live status broadcasts from /robotx/pipeline/status."""
    print(f"\nMonitoring /robotx/pipeline/status for {duration} seconds (Ctrl+C to stop)...")
    node = gz_transport.Node()
    latest_data = {}

    def cb(msg):
        nonlocal latest_data
        try:
            latest_data = json.loads(msg.data)
        except Exception:
            pass

    node.subscribe(stringmsg_pb2.StringMsg, "/robotx/pipeline/status", cb)

    start = time.time()
    try:
        while time.time() - start < duration:
            if latest_data:
                print("\033[H\033[J", end="") # Clear terminal screen
                print_status_table(latest_data)
            time.sleep(0.4)
    except KeyboardInterrupt:
        print("\nMonitoring stopped.")


def main():
    parser = argparse.ArgumentParser(description="RobotX Task 2 Pipeline Light CLI Controller & Tester")
    parser.add_argument("--tier", choices=["CORE", "ADVANCED", "DISRUPTIVE", "core", "advanced", "disruptive"], help="Set active challenge tier")
    parser.add_argument("--damaged-node", type=str, help="Set damaged node (1, 2, 3, 4, or 'RANDOM')")
    parser.add_argument("--first-color", choices=["RED", "GREEN", "BLUE", "red", "green", "blue"], help="Disruptive 1st color (Resource Tin: RED, GREEN, BLUE)")
    parser.add_argument("--second-color", choices=["RED", "GREEN", "BLUE", "red", "green", "blue"], help="Destination circle color (RED, GREEN, BLUE)")
    parser.add_argument("--repair", action="store_true", help="Simulate magnetic probe repair on the damaged segment (2.5s hold)")
    parser.add_argument("--magnet-tap", action="store_true", help="Simulate magnetic probe tap on damaged segment (0.3s hold)")
    parser.add_argument("--node", type=int, help="Target specific node ID for manual actions")
    parser.add_argument("--color", type=str, help="Manually override node color (RED, GREEN, BLUE, CYAN, BLACK, OFF, etc.)")
    parser.add_argument("--activate", action="store_true", help="Activate pipeline challenge scenario")
    parser.add_argument("--reset", action="store_true", help="Reset pipeline to initial state")
    parser.add_argument("--monitor", action="store_true", help="Stream live status monitor")
    parser.add_argument("--duration", type=float, default=30.0, help="Monitor stream duration in seconds")

    args = parser.parse_args()
    api = RobotXPipelineAPI()

    # 1. Pipeline configuration
    if args.tier:
        api.set_tier(args.tier.upper())
    if args.damaged_node:
        api.set_damaged_node(args.damaged_node.upper())
    if args.first_color:
        api.set_first_color(args.first_color.upper())
    if args.second_color:
        api.set_second_color(args.second_color.upper())
    if args.activate:
        api.activate_challenge()
    if args.reset:
        api.reset_pipeline()

    # 2. Repair / Magnet actions
    if args.repair:
        target_nid = args.node
        if target_nid is not None:
            print(f"Triggering repair (2.5s magnet hold) on Node {target_nid}...")
            api.trigger_node_magnet(target_nid, duration_s=2.5, field_mT=3.5)
        else:
            print("Triggering repair (2.5s magnet hold) on current damaged segment...")
            api.repair_damaged(duration_s=2.5, field_mT=3.5)
        time.sleep(0.5)

    if args.magnet_tap:
        target_nid = args.node
        if target_nid is not None:
            print(f"Triggering magnet tap (0.3s) on Node {target_nid}...")
            api.trigger_node_magnet(target_nid, duration_s=0.3, field_mT=3.0)
        else:
            print("Triggering magnet tap (0.3s) on current damaged segment...")
            api.repair_damaged(duration_s=0.3, field_mT=3.0)
        time.sleep(0.4)

    # 3. Direct color override
    if args.color:
        nid = args.node if args.node is not None else 1
        node = gz_transport.Node()
        pub = node.advertise(f"/robotx/light_node_{nid}/set_color", stringmsg_pb2.StringMsg)
        time.sleep(0.2)
        m = stringmsg_pb2.StringMsg()
        m.data = args.color
        pub.publish(m)
        print(f"Published set_color -> {args.color} on Node {nid}")
        time.sleep(0.2)

    # 4. Monitor or snapshot
    if args.monitor:
        monitor_pipeline(duration=args.duration)
    else:
        print_snapshot()


if __name__ == "__main__":
    main()
