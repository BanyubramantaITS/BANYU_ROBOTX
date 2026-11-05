#!/usr/bin/env python3
"""HSV slider tuner for the POR gate (yellow) and marker (orange) detector.

Live mode (default): run on the laptop while por_detection runs on the robot
or in the sim. The sliders write the detector's parameters live, and the
windows show the detector's own debug images (annotated + both masks).

    python3 tools/scripts/por/por_hsv_tuner.py            # compressed debug images
    python3 tools/scripts/por/por_hsv_tuner.py --raw      # raw images (sim / wired)

The detector's debug_rate_hz is raised to --debug-rate (default 15) while the
tuner runs and restored on exit.

Offline mode: tune on recorded footage (video file, image or image folder)
without ROS. The mask is computed here with the same rules as the detector.

    python3 tools/scripts/por/por_hsv_tuner.py --source pool_run.mp4

Keys: s = save (live: /por_detection/save_params, offline: print + write YAML)
      r = reload values from the node, space = pause (offline), q / Esc = quit
"""
import argparse
import glob
import os
import sys

import cv2
import numpy as np

GATE_KEYS = ["gate_h_min", "gate_h_max", "gate_s_min", "gate_s_max", "gate_v_min", "gate_v_max"]
MARKER_KEYS = ["marker_h_min", "marker_h_max", "marker_s_min", "marker_s_max",
               "marker_v_min", "marker_v_max"]
FILTER_KEYS = ["min_area", "morph_kernel", "buoy_aspect_max"]
MAXIMA = {"min_area": 5000, "morph_kernel": 15, "buoy_aspect_max": 300}
DEFAULTS = {
    "gate_h_min": 20, "gate_h_max": 35, "gate_s_min": 80, "gate_s_max": 255,
    "gate_v_min": 80, "gate_v_max": 255,
    "marker_h_min": 5, "marker_h_max": 18, "marker_s_min": 120, "marker_s_max": 255,
    "marker_v_min": 100, "marker_v_max": 255,
    "min_area": 300, "morph_kernel": 3, "buoy_aspect_max": 65,
}
WINDOWS = {"Gate HSV (yellow)": GATE_KEYS, "Marker HSV (orange)": MARKER_KEYS,
           "Filter": FILTER_KEYS}


def slider_max(key):
    if key in MAXIMA:
        return MAXIMA[key]
    return 179 if "_h_" in key else 255


def make_windows(values):
    for window, keys in WINDOWS.items():
        cv2.namedWindow(window, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window, 420, 70 * len(keys) + 20)  # value panel + sliders
        for key in keys:
            cv2.createTrackbar(key, window, int(values[key]), slider_max(key), lambda _v: None)


LABELS = {
    "h_min": "hue min (0-179)", "h_max": "hue max (0-179)",
    "s_min": "saturation min", "s_max": "saturation max",
    "v_min": "brightness min", "v_max": "brightness max",
    "min_area": "min blob area (px)", "morph_kernel": "noise filter kernel",
    "buoy_aspect_max": "marker max w/h x100",
}


def label(key):
    for suffix, text in LABELS.items():
        if key.endswith(suffix):
            return text
    return ""


def draw_panels(values):
    """Name and value of every slider, top to bottom (the Qt backend hides trackbar labels)."""
    for window, keys in WINDOWS.items():
        panel = np.full((28 * len(keys) + 12, 420, 3), 40, np.uint8)
        for i, key in enumerate(keys):
            y = 28 * i + 26
            cv2.putText(panel, f"{i + 1}. {key} = {values[key]}", (10, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
            cv2.putText(panel, label(key), (265, y), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                        (170, 200, 255), 1, cv2.LINE_AA)
        cv2.imshow(window, panel)


def read_sliders():
    values = {}
    for window, keys in WINDOWS.items():
        for key in keys:
            values[key] = cv2.getTrackbarPos(key, window)
    return values


def set_sliders(values):
    for window, keys in WINDOWS.items():
        for key in keys:
            if key in values:
                cv2.setTrackbarPos(key, window, int(values[key]))


def hsv_mask(hsv, h_min, h_max, s_min, s_max, v_min, v_max):
    if h_min <= h_max:
        return cv2.inRange(hsv, (h_min, s_min, v_min), (h_max, s_max, v_max))
    high = cv2.inRange(hsv, (h_min, s_min, v_min), (179, s_max, v_max))
    low = cv2.inRange(hsv, (0, s_min, v_min), (h_max, s_max, v_max))
    return cv2.bitwise_or(low, high)


def detect(mask, values, is_gate):
    """Same rule as por_detection: gate = union of parts, marker = largest pole."""
    count, _labels, stats, _centroids = cv2.connectedComponentsWithStats(mask)
    box = None
    best_area = 0
    for i in range(1, count):
        x, y, w, h, area = stats[i]
        if area < values["min_area"]:
            continue
        if is_gate:
            box = (x, y, x + w, y + h) if box is None else (
                min(box[0], x), min(box[1], y), max(box[2], x + w), max(box[3], y + h))
            continue
        touches_vert = y <= 5 or y + h >= mask.shape[0] - 5
        if not touches_vert and w / max(h, 1) > values["buoy_aspect_max"] / 100.0:
            continue
        if area > best_area:
            best_area, box = area, (x, y, x + w, y + h)
    return box


def offline_frame(frame, values):
    blurred = cv2.GaussianBlur(frame, (5, 5), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    masks = []
    annotated = frame.copy()
    for keys, is_gate, color, label in ((GATE_KEYS, True, (0, 255, 0), "gate"),
                                        (MARKER_KEYS, False, (0, 140, 255), "marker")):
        mask = hsv_mask(hsv, *[values[k] for k in keys])
        kernel = values["morph_kernel"]
        if kernel > 1:
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((kernel, kernel), np.uint8))
        box = detect(mask, values, is_gate)
        if box is not None:
            cv2.rectangle(annotated, box[:2], box[2:], color, 2)
            cv2.putText(annotated, label, (box[0], max(box[1] - 8, 12)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        masks.append(mask)
    return annotated, masks[0], masks[1]


def tile(annotated, gate_mask, marker_mask, width=480):
    def fit(image):
        if image is None:
            return np.zeros((int(width * 0.75), width, 3), np.uint8)
        if image.ndim == 2:
            image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        scale = width / image.shape[1]
        return cv2.resize(image, (width, int(image.shape[0] * scale)))
    a, g, m = fit(annotated), fit(gate_mask), fit(marker_mask)
    h = max(a.shape[0], g.shape[0], m.shape[0])
    pad = [cv2.copyMakeBorder(i, 0, h - i.shape[0], 0, 0, cv2.BORDER_CONSTANT) for i in (a, g, m)]
    for image, text in zip(pad, ("annotated", "gate mask (yellow)", "marker mask (orange)")):
        cv2.putText(image, text, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    return np.hstack(pad)


def yaml_snippet(values):
    lines = ["por_detection:", "  ros__parameters:", '    color_mode: "per_object"']
    lines += [f"    {k}: {values[k]}" for k in GATE_KEYS + MARKER_KEYS + FILTER_KEYS]
    return "\n".join(lines) + "\n"


def run_offline(source):
    if os.path.isdir(source):
        files = sorted(sum((glob.glob(os.path.join(source, p))
                            for p in ("*.png", "*.jpg", "*.jpeg")), []))
        frames = (cv2.imread(f) for f in files)
        images = [f for f in frames if f is not None]
        capture = None
    elif source.lower().endswith((".png", ".jpg", ".jpeg")):
        images, capture = [cv2.imread(source)], None
    else:
        images, capture = [], cv2.VideoCapture(source)
        if not capture.isOpened():
            sys.exit(f"cannot open {source}")
    values = dict(DEFAULTS)
    make_windows(values)
    index, paused, frame = 0, False, None
    while True:
        if not paused or frame is None:
            if capture is not None:
                ok, frame = capture.read()
                if not ok:
                    capture.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    continue
            else:
                frame = images[index % len(images)]
                index += 1
        values = read_sliders()
        draw_panels(values)
        cv2.imshow("POR HSV tuner", tile(*offline_frame(frame, values)))
        key = cv2.waitKey(30 if capture is not None else 300) & 0xFF
        if key in (ord("q"), 27):
            break
        if key == ord(" "):
            paused = not paused
        if key == ord("s"):
            snippet = yaml_snippet(values)
            path = os.path.expanduser("~/por_detection_tuned.yaml")
            with open(path, "w") as out:
                out.write(snippet)
            print(snippet + f"# saved to {path}")


def run_live(args):
    import rclpy
    from rcl_interfaces.srv import GetParameters, SetParameters
    from rclpy.parameter import Parameter
    from sensor_msgs.msg import CompressedImage, Image
    from std_srvs.srv import Trigger

    rclpy.init()
    node = rclpy.create_node("por_hsv_tuner")
    target = args.node.rstrip("/")
    get_client = node.create_client(GetParameters, f"{target}/get_parameters")
    set_client = node.create_client(SetParameters, f"{target}/set_parameters")
    save_client = node.create_client(Trigger, f"{target}/save_params")
    all_keys = GATE_KEYS + MARKER_KEYS + FILTER_KEYS

    def fetch():
        if not get_client.wait_for_service(timeout_sec=5.0):
            node.get_logger().warn(
                f"{target} not reachable, using defaults (ROS_DOMAIN_ID="
                f"{os.environ.get('ROS_DOMAIN_ID', 'unset')}: must match the robot)")
            return dict(DEFAULTS)
        future = get_client.call_async(GetParameters.Request(names=all_keys))
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
        if future.result() is None:
            return dict(DEFAULTS)
        return {k: v.integer_value for k, v in zip(all_keys, future.result().values)}

    images = {"annotated": None, "mask_gate": None, "mask_marker": None}

    def on_image(name):
        def callback(msg):
            if isinstance(msg, CompressedImage):
                data = np.frombuffer(msg.data, np.uint8)
                images[name] = cv2.imdecode(data, cv2.IMREAD_UNCHANGED)
            else:
                channels = 1 if msg.encoding == "mono8" else 3
                images[name] = np.frombuffer(msg.data, np.uint8).reshape(
                    msg.height, msg.width, channels).squeeze()
        return callback

    for name in images:
        topic = f"/banyu/detection/debug/{name}"
        if args.raw:
            node.create_subscription(Image, topic, on_image(name), 2)
        else:
            node.create_subscription(CompressedImage, topic + "/compressed", on_image(name), 2)

    def set_debug_rate(rate):
        future = set_client.call_async(SetParameters.Request(
            parameters=[Parameter("debug_rate_hz", value=float(rate)).to_parameter_msg()]))
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)

    # The mission keeps the debug stream slow (it is recorded); speed it up only while tuning.
    original_rate = None
    if args.debug_rate > 0 and get_client.wait_for_service(timeout_sec=5.0):
        future = get_client.call_async(GetParameters.Request(names=["debug_rate_hz"]))
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
        if future.result() is not None and future.result().values:
            original_rate = future.result().values[0].double_value
            set_debug_rate(args.debug_rate)
            print(f"debug_rate_hz {original_rate:g} -> {args.debug_rate:g} while tuning")
    try:
        tune_loop(args, node, fetch, set_client, save_client, images, SetParameters, Parameter,
                  Trigger)
    except KeyboardInterrupt:
        pass
    finally:
        if original_rate is not None:
            set_debug_rate(original_rate)
            print(f"debug_rate_hz restored to {original_rate:g}")
        node.destroy_node()
        rclpy.try_shutdown()  # Ctrl+C has usually shut the context down already


def tune_loop(args, node, fetch, set_client, save_client, images, SetParameters, Parameter,
              Trigger):
    import rclpy
    sent = fetch()
    make_windows(sent)
    print("Sliders write live to", args.node, "- s: save, r: reload, q: quit")
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.02)
        values = read_sliders()
        changed = [Parameter(k, value=int(v)).to_parameter_msg()
                   for k, v in values.items() if sent.get(k) != v]
        if changed and set_client.service_is_ready():
            set_client.call_async(SetParameters.Request(parameters=changed))
            sent.update(values)
        draw_panels(values)
        view = tile(images["annotated"], images["mask_gate"], images["mask_marker"])
        if images["annotated"] is None:
            kind = "" if args.raw else "/compressed"
            cv2.putText(view, f"waiting for /banyu/detection/debug/annotated{kind}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 1, cv2.LINE_AA)
            cv2.putText(view, "(sim: use --raw; is por_detection running?)", (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 1, cv2.LINE_AA)
        cv2.imshow("POR HSV tuner", view)
        key = cv2.waitKey(30) & 0xFF
        if key in (ord("q"), 27):
            break
        if key == ord("r"):
            sent = fetch()
            set_sliders(sent)
        if key == ord("s"):
            future = save_client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
            result = future.result()
            print(result.message if result else "save_params did not answer")
            print(yaml_snippet(values))


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--source", help="offline: video file, image, or image folder")
    parser.add_argument("--node", default="/por_detection", help="live: detector node name")
    parser.add_argument("--raw", action="store_true", help="live: raw instead of compressed")
    parser.add_argument("--debug-rate", type=float, default=15.0,
                        help="live: detector debug_rate_hz while tuning, restored on exit "
                        "(0 = leave unchanged)")
    args = parser.parse_args()
    if args.source:
        run_offline(args.source)
    else:
        run_live(args)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
