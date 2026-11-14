#!/usr/bin/env python3
"""Run a YOLO HEF on the Hailo-8 against the front camera (or an image) and report detections + FPS.

The HEF must be a Hailo-compiled YOLO with on-chip NMS (Hailo Model Zoo yolov8/yolov11, or the
team model compiled with the Hailo Dataflow Compiler). The frame is optionally rotated (the front
camera is mounted upright; frames from before 2026-11-10 are sideways), then letterboxed to the HEF input.

  check/yolo_test.sh                                  # ~/yolo_models/yolov8s.hef, live, 10 s
  check/yolo_test.sh --hef team.hef --labels team.txt --save /tmp/yolo.jpg
  check/yolo_test.sh --image old_frame.jpg --rotation 270

Stop anything else streaming the camera first.
"""
import argparse
import os
import time

import cv2
import numpy as np
from hailo_platform import (HEF, ConfigureParams, FormatType, HailoStreamInterface, InferVStreams,
                            InputVStreamParams, OutputVStreamParams, VDevice)

COCO = ("person bicycle car motorcycle airplane bus train truck boat traffic_light fire_hydrant "
        "stop_sign parking_meter bench bird cat dog horse sheep cow elephant bear zebra giraffe "
        "backpack umbrella handbag tie suitcase frisbee skis snowboard sports_ball kite baseball_bat "
        "baseball_glove skateboard surfboard tennis_racket bottle wine_glass cup fork knife spoon bowl "
        "banana apple sandwich orange broccoli carrot hot_dog pizza donut cake chair couch "
        "potted_plant bed dining_table toilet tv laptop mouse remote keyboard cell_phone microwave "
        "oven toaster sink refrigerator book clock vase scissors teddy_bear hair_drier toothbrush").split()
ROTATIONS = {0: None, 90: cv2.ROTATE_90_CLOCKWISE, 180: cv2.ROTATE_180,
             270: cv2.ROTATE_90_COUNTERCLOCKWISE}


def letterbox(img, size):
    h, w = img.shape[:2]
    scale = min(size / w, size / h)
    nw, nh = int(round(w * scale)), int(round(h * scale))
    pad_x, pad_y = (size - nw) // 2, (size - nh) // 2
    out = np.full((size, size, 3), 114, np.uint8)
    out[pad_y:pad_y + nh, pad_x:pad_x + nw] = cv2.resize(img, (nw, nh))
    return out, scale, pad_x, pad_y


def decode(per_class, scale, pad_x, pad_y, size, conf):
    """Hailo NMS-by-class output: per class an array of [ymin, xmin, ymax, xmax, score] (0..1)."""
    dets = []
    for cls, boxes in enumerate(per_class):
        for y0, x0, y1, x1, score in np.asarray(boxes).reshape(-1, 5):
            if score < conf:
                continue
            box = [(x0 * size - pad_x) / scale, (y0 * size - pad_y) / scale,
                   (x1 * size - pad_x) / scale, (y1 * size - pad_y) / scale]
            dets.append((cls, float(score), [int(v) for v in box]))
    return dets


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--hef", default=os.path.expanduser("~/yolo_models/yolov8s.hef"))
    p.add_argument("--labels", help="one class name per line (default: COCO)")
    p.add_argument("--device", default="/dev/front_camera")
    p.add_argument("--image", help="run on this image instead of the camera")
    p.add_argument("--rotation", type=int, default=0, choices=sorted(ROTATIONS),
                   help="clockwise rotation applied before inference (default 0: mounted upright; "
                        "270 for frames from the old sideways mount)")
    p.add_argument("--seconds", type=float, default=10.0)
    p.add_argument("--conf", type=float, default=0.4)
    p.add_argument("--save", help="write the last annotated frame here")
    a = p.parse_args()

    labels = COCO
    if a.labels:
        with open(a.labels) as f:
            labels = [line.strip() for line in f if line.strip()]

    hef = HEF(a.hef)
    in_info = hef.get_input_vstream_infos()[0]
    size = in_info.shape[0]
    cap = None
    if not a.image:
        cap = cv2.VideoCapture(a.device, cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        if not cap.isOpened():
            raise SystemExit(f"cannot open {a.device} (streaming elsewhere?)")

    with VDevice() as vdevice:
        group = vdevice.configure(hef, ConfigureParams.create_from_hef(hef, interface=HailoStreamInterface.PCIe))[0]
        in_params = InputVStreamParams.make(group, format_type=FormatType.UINT8)
        out_params = OutputVStreamParams.make(group, format_type=FormatType.FLOAT32)
        with InferVStreams(group, in_params, out_params) as pipe, group.activate(group.create_params()):
            frames, infer_s, start, frame = 0, 0.0, time.time(), None
            while True:
                if a.image:
                    frame = cv2.imread(a.image)
                else:
                    ok, frame = cap.read()
                    if not ok:
                        raise SystemExit("camera read failed")
                if ROTATIONS[a.rotation] is not None:
                    frame = cv2.rotate(frame, ROTATIONS[a.rotation])
                # Hailo Model Zoo YOLO HEFs take RGB.
                boxed, scale, pad_x, pad_y = letterbox(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB), size)
                t0 = time.time()
                out = pipe.infer({in_info.name: boxed[np.newaxis]})
                infer_s += time.time() - t0
                dets = decode(next(iter(out.values()))[0], scale, pad_x, pad_y, size, a.conf)
                frames += 1
                summary = ", ".join(f"{labels[c] if c < len(labels) else c} {s:.2f}" for c, s, _ in dets) or "-"
                print(f"frame {frames:4d} {frame.shape[1]}x{frame.shape[0]}: {summary}", flush=True)
                if a.image or time.time() - start >= a.seconds:
                    break
            elapsed = time.time() - start

    if cap is not None:
        cap.release()
    print(f"\n{frames} frames in {elapsed:.1f} s: {frames / elapsed:.1f} FPS end to end, "
          f"Hailo inference {1000 * infer_s / frames:.1f} ms/frame ({os.path.basename(a.hef)})")
    if a.save and frame is not None:
        for c, s, (x0, y0, x1, y1) in dets:
            cv2.rectangle(frame, (x0, y0), (x1, y1), (0, 255, 0), 2)
            cv2.putText(frame, f"{labels[c] if c < len(labels) else c} {s:.2f}", (x0, max(15, y0 - 5)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        cv2.imwrite(a.save, frame)
        print(f"annotated frame: {a.save}")


if __name__ == "__main__":
    main()
