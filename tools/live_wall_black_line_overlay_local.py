#!/usr/bin/env python3
import argparse
import sys
import threading
import time
import tkinter as tk
from pathlib import Path

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from wall_black_line_detector import detect_wall_black_line, draw_overlay


def depth_to_meters(msg):
    if msg.encoding == "16UC1":
        arr = np.frombuffer(msg.data, dtype=np.uint16)
        img = arr.reshape(msg.height, msg.step // 2)[:, : msg.width]
        return img.astype(np.float32) * 0.001
    if msg.encoding == "32FC1":
        arr = np.frombuffer(msg.data, dtype=np.float32)
        return arr.reshape(msg.height, msg.step // 4)[:, : msg.width].copy()
    raise ValueError("unsupported depth encoding: " + msg.encoding)


def ros_image_to_bgr(msg):
    if msg.encoding not in ("rgb8", "bgr8"):
        raise ValueError("unsupported color encoding: " + msg.encoding)
    arr = np.frombuffer(msg.data, dtype=np.uint8)
    img = arr.reshape(msg.height, msg.step)[:, : msg.width * 3]
    img = img.reshape(msg.height, msg.width, 3)
    if msg.encoding == "rgb8":
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    return img.copy()


def line_sample_points(result, width, height, step_px=6.0):
    p0 = result.get("hough_segment_p1_uv") or result.get("line_p1_uv")
    p1 = result.get("hough_segment_p2_uv") or result.get("line_p2_uv")
    if p0 is None or p1 is None:
        return np.empty((0, 2), dtype=np.int32)
    p0 = np.asarray(p0, dtype=np.float32)
    p1 = np.asarray(p1, dtype=np.float32)
    length = float(np.linalg.norm(p1 - p0))
    if length < 1.0:
        return np.empty((0, 2), dtype=np.int32)
    count = max(2, int(np.ceil(length / max(1.0, step_px))) + 1)
    ts = np.linspace(0.0, 1.0, count, dtype=np.float32)
    pts = p0[None, :] * (1.0 - ts[:, None]) + p1[None, :] * ts[:, None]
    pts = np.round(pts).astype(np.int32)
    pts[:, 0] = np.clip(pts[:, 0], 0, width - 1)
    pts[:, 1] = np.clip(pts[:, 1], 0, height - 1)
    return pts


def estimate_line_depth(depth_m, result, args):
    if depth_m is None or result is None:
        return None
    height, width = depth_m.shape[:2]
    pts = line_sample_points(result, width, height, step_px=args.depth_sample_step_px)
    if pts.size == 0:
        return None

    radius = max(0, int(args.depth_sample_radius_px))
    values = []
    valid_pixels = 0
    for x, y in pts:
        x0 = max(0, int(x) - radius)
        x1 = min(width, int(x) + radius + 1)
        y0 = max(0, int(y) - radius)
        y1 = min(height, int(y) + radius + 1)
        patch = depth_m[y0:y1, x0:x1].reshape(-1)
        valid = patch[
            np.isfinite(patch)
            & (patch >= args.min_valid_depth_m)
            & (patch <= args.max_valid_depth_m)
        ]
        if valid.size == 0:
            continue
        values.append(float(np.min(valid)))
        valid_pixels += int(valid.size)

    if len(values) < args.min_depth_samples:
        return None
    vals = np.asarray(values, dtype=np.float32)
    return {
        "min_m": float(np.min(vals)),
        "median_m": float(np.median(vals)),
        "p10_m": float(np.percentile(vals, 10)),
        "sample_count": int(len(vals)),
        "valid_pixel_count": int(valid_pixels),
    }


class LiveWallLineOverlay:
    def __init__(self, args):
        self.args = args
        self.lock = threading.Lock()
        self.latest_frame = None
        self.latest_depth = None
        self.latest_stamp = None
        self.latest_depth_stamp = None
        self.seq = 0
        self.rendered_seq = -1
        self.last_print_time = 0.0
        self.latest_path = Path(args.save_latest)
        self.latest_path.parent.mkdir(parents=True, exist_ok=True)
        self.tk_path = self.latest_path.with_suffix(".png")

        rospy.init_node("g1_wall_black_line_live_overlay_local", anonymous=True)
        self.sub = rospy.Subscriber(
            args.image_topic,
            Image,
            self.image_callback,
            queue_size=1,
            buff_size=2**24,
        )
        self.depth_sub = rospy.Subscriber(
            args.depth_topic,
            Image,
            self.depth_callback,
            queue_size=1,
            buff_size=2**24,
        )
        self.root = None
        self.image_label = None
        self.status_label = None

    def image_callback(self, msg):
        frame = ros_image_to_bgr(msg)
        with self.lock:
            self.latest_frame = frame
            self.latest_stamp = msg.header.stamp.to_sec()
            self.seq += 1

    def depth_callback(self, msg):
        depth_m = depth_to_meters(msg)
        with self.lock:
            self.latest_depth = depth_m
            self.latest_depth_stamp = msg.header.stamp.to_sec()

    def build_overlay(self, frame, depth_m, stamp, depth_stamp):
        result, debug = detect_wall_black_line(
            frame,
            roi_y1_ratio=self.args.roi_y1_ratio,
            roi_y2_ratio=self.args.roi_y2_ratio,
            dark_threshold=self.args.dark_threshold,
        )
        line_depth = estimate_line_depth(depth_m, result, self.args)
        overlay = draw_overlay(frame, result, debug)

        if result is None:
            status = "NO_LINE"
            color = (0, 0, 255)
            line = "wall_line: NO_LINE"
            depth_text = "depth=NO_LINE"
        else:
            angle = float(result["angle_deg"])
            aligned = abs(angle) <= self.args.deadband_deg
            status = "ALIGNED" if aligned else "TURN"
            color = (0, 220, 0) if aligned else (0, 200, 255)
            if line_depth is None:
                depth_text = "depth=NO_DEPTH"
            else:
                in_range = self.args.target_min_distance_m <= line_depth["min_m"] <= self.args.target_max_distance_m
                depth_status = "IN_RANGE" if in_range else ("TOO_CLOSE" if line_depth["min_m"] < self.args.target_min_distance_m else "FAR")
                depth_text = (
                    f"min={line_depth['min_m'] * 1000:.0f}mm "
                    f"med={line_depth['median_m'] * 1000:.0f}mm "
                    f"p10={line_depth['p10_m'] * 1000:.0f}mm "
                    f"{depth_status}"
                )
            line = (
                f"wall_line={angle:+.2f}deg status={status} "
                f"method={result.get('method')} pts={result.get('inliers')}"
            )

        cv2.putText(overlay, line, (12, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(overlay, line, (12, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.58, color, 1, cv2.LINE_AA)
        cv2.putText(overlay, depth_text, (12, 84), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(overlay, depth_text, (12, 84), cv2.FONT_HERSHEY_SIMPLEX, 0.58, color, 1, cv2.LINE_AA)
        target_line = (
            f"target_min_depth={self.args.target_min_distance_m * 1000:.0f}-"
            f"{self.args.target_max_distance_m * 1000:.0f}mm "
            f"depth_topic={self.args.depth_topic}"
        )
        cv2.putText(overlay, target_line, (12, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(overlay, target_line, (12, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (255, 255, 255), 1, cv2.LINE_AA)
        if stamp is not None:
            depth_age = "N/A" if depth_stamp is None else f"{abs(stamp - depth_stamp):.3f}s"
            stamp_line = f"rgb_stamp={stamp:.3f} depth_age={depth_age} topic={self.args.image_topic}"
            cv2.putText(overlay, stamp_line, (12, 136), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(overlay, stamp_line, (12, 136), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (255, 255, 255), 1, cv2.LINE_AA)
        return overlay, result, status, line_depth

    def maybe_print(self, result, status, line_depth):
        now = time.time()
        if now - self.last_print_time < self.args.print_every:
            return
        self.last_print_time = now
        if result is None:
            print("wall_line status=NO_LINE", flush=True)
            return
        if line_depth is None:
            depth_text = "depth=NO_DEPTH"
        else:
            depth_text = (
                f"min={line_depth['min_m'] * 1000:.0f}mm "
                f"med={line_depth['median_m'] * 1000:.0f}mm "
                f"p10={line_depth['p10_m'] * 1000:.0f}mm "
                f"samples={line_depth['sample_count']}"
            )
        print(
            "wall_line "
            f"angle={float(result['angle_deg']):+.2f}deg "
            f"status={status} "
            f"method={result.get('method')} "
            f"pts={result.get('inliers')} "
            f"err={result.get('mean_abs_error_px'):.1f}px "
            f"{depth_text}",
            flush=True,
        )

    def run(self):
        if self.args.display:
            self.root = tk.Tk()
            self.root.title(self.args.window_name)
            self.image_label = tk.Label(self.root)
            self.image_label.pack()
            self.status_label = tk.Label(self.root, text="waiting for /camera/color/image_raw ...")
            self.status_label.pack(fill=tk.X)
            self.root.protocol("WM_DELETE_WINDOW", self.close)
            self.root.bind("<Escape>", lambda _event: self.close())
            self.root.bind("q", lambda _event: self.close())
            self.poll_tk()
            self.root.mainloop()
            return

        while not rospy.is_shutdown():
            with self.lock:
                frame = None if self.latest_frame is None else self.latest_frame.copy()
                depth_m = None if self.latest_depth is None else self.latest_depth.copy()
                stamp = self.latest_stamp
                depth_stamp = self.latest_depth_stamp
                seq = self.seq

            if frame is None:
                time.sleep(self.args.interval)
                continue

            if seq != self.rendered_seq:
                overlay, result, status, line_depth = self.build_overlay(frame, depth_m, stamp, depth_stamp)
                cv2.imwrite(str(self.latest_path), overlay, [int(cv2.IMWRITE_JPEG_QUALITY), 92])
                self.maybe_print(result, status, line_depth)
                self.rendered_seq = seq
            time.sleep(self.args.interval)

    def poll_tk(self):
        with self.lock:
            frame = None if self.latest_frame is None else self.latest_frame.copy()
            depth_m = None if self.latest_depth is None else self.latest_depth.copy()
            stamp = self.latest_stamp
            depth_stamp = self.latest_depth_stamp
            seq = self.seq

        if frame is not None and seq != self.rendered_seq:
            overlay, result, status, line_depth = self.build_overlay(frame, depth_m, stamp, depth_stamp)
            cv2.imwrite(str(self.latest_path), overlay, [int(cv2.IMWRITE_JPEG_QUALITY), 92])
            cv2.imwrite(str(self.tk_path), overlay)
            self.maybe_print(result, status, line_depth)
            img = tk.PhotoImage(file=str(self.tk_path))
            self.image_label.configure(image=img)
            self.image_label.image = img
            if result is None:
                self.status_label.configure(text="wall_line: NO_LINE")
            else:
                if line_depth is None:
                    depth_text = "depth=NO_DEPTH"
                else:
                    depth_text = (
                        f"min={line_depth['min_m'] * 1000:.0f}mm "
                        f"med={line_depth['median_m'] * 1000:.0f}mm "
                        f"p10={line_depth['p10_m'] * 1000:.0f}mm"
                    )
                self.status_label.configure(
                    text=(
                        f"angle={float(result['angle_deg']):+.2f}deg "
                        f"status={status} method={result.get('method')} "
                        f"pts={result.get('inliers')} err={result.get('mean_abs_error_px'):.1f}px "
                        f"{depth_text}"
                    )
                )
            self.rendered_seq = seq

        if self.root is not None and not rospy.is_shutdown():
            self.root.after(max(1, int(self.args.interval * 1000)), self.poll_tk)

    def close(self):
        if self.root is not None:
            self.root.destroy()


def main():
    parser = argparse.ArgumentParser(description="Live local visual wall-line overlay from G1 RGB topic.")
    parser.add_argument("--image-topic", default="/camera/color/image_raw")
    parser.add_argument("--depth-topic", default="/camera/aligned_depth_to_color/image_raw")
    parser.add_argument(
        "--save-latest",
        default=str((SCRIPT_DIR.parent / ".runtime" / "wall_black_line_live" / "latest_overlay.jpg").resolve()),
    )
    parser.add_argument("--interval", type=float, default=0.08)
    parser.add_argument("--print-every", type=float, default=0.5)
    parser.add_argument("--deadband-deg", type=float, default=1.5)
    parser.add_argument("--roi-y1-ratio", type=float, default=0.68)
    parser.add_argument("--roi-y2-ratio", type=float, default=0.98)
    parser.add_argument("--dark-threshold", type=int, default=85)
    parser.add_argument("--target-min-distance-m", type=float, default=1.20)
    parser.add_argument("--target-max-distance-m", type=float, default=1.30)
    parser.add_argument("--min-valid-depth-m", type=float, default=0.20)
    parser.add_argument("--max-valid-depth-m", type=float, default=4.0)
    parser.add_argument("--depth-sample-radius-px", type=int, default=2)
    parser.add_argument("--depth-sample-step-px", type=float, default=6.0)
    parser.add_argument("--min-depth-samples", type=int, default=24)
    parser.set_defaults(display=True)
    parser.add_argument("--display", dest="display", action="store_true")
    parser.add_argument("--no-display", dest="display", action="store_false")
    parser.add_argument("--window-name", default="G1 wall black line live overlay")
    parser.add_argument("--window-width", type=int, default=960)
    parser.add_argument("--window-height", type=int, default=720)
    args = parser.parse_args()
    LiveWallLineOverlay(args).run()


if __name__ == "__main__":
    main()
