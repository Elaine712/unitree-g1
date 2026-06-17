#!/usr/bin/env python3
import argparse
import json
import math
from pathlib import Path

import cv2
import numpy as np


IMAGE_EXTS = {".jpg", ".jpeg", ".png", ".bmp"}


def fit_line(points):
    if len(points) < 2:
        return None
    pts = points.astype(np.float32)
    vx, vy, x0, y0 = cv2.fitLine(pts, cv2.DIST_L2, 0, 0.01, 0.01).flatten()
    direction = np.array([vx, vy], dtype=np.float32)
    norm = float(np.linalg.norm(direction))
    if norm < 1e-6:
        return None
    direction /= norm
    if direction[0] < 0:
        direction = -direction
    center = np.array([x0, y0], dtype=np.float32)
    t = (pts - center) @ direction
    p_start = center + direction * float(np.min(t))
    p_end = center + direction * float(np.max(t))
    normal = np.array([-direction[1], direction[0]], dtype=np.float32)
    residual = np.abs((pts - center) @ normal)
    return {
        "direction": direction,
        "p_start": p_start,
        "p_end": p_end,
        "count": int(len(pts)),
        "mean_abs_error_px": float(np.mean(residual)),
        "max_abs_error_px": float(np.max(residual)),
    }


def line_angle_deg(line):
    d = line["direction"]
    return float(math.degrees(math.atan2(float(d[1]), float(d[0]))))


def make_line_from_endpoints(p0, p1):
    p0 = np.asarray(p0, dtype=np.float32)
    p1 = np.asarray(p1, dtype=np.float32)
    d = p1 - p0
    norm = float(np.linalg.norm(d))
    if norm < 1e-6:
        return None
    direction = d / norm
    if direction[0] < 0:
        direction = -direction
        p0, p1 = p1, p0
    return {
        "direction": direction.astype(np.float32),
        "p_start": p0.astype(np.float32),
        "p_end": p1.astype(np.float32),
        "count": 2,
        "mean_abs_error_px": 0.0,
        "max_abs_error_px": 0.0,
    }


def sample_segment_points(p0, p1, step_px=12.0):
    p0 = np.asarray(p0, dtype=np.float32)
    p1 = np.asarray(p1, dtype=np.float32)
    length = float(np.linalg.norm(p1 - p0))
    count = max(2, int(math.ceil(length / max(step_px, 1.0))) + 1)
    ts = np.linspace(0.0, 1.0, count, dtype=np.float32)
    return p0[None, :] * (1.0 - ts[:, None]) + p1[None, :] * ts[:, None]


def endpoints_for_image(line, width, height):
    p0 = line["p_start"]
    p1 = line["p_end"]
    d = line["direction"]
    if abs(float(d[0])) < 1e-6:
        x = int(round(float(p0[0])))
        return (x, 0), (x, height - 1)

    candidates = []
    for x in (0.0, float(width - 1)):
        t = (x - float(p0[0])) / float(d[0])
        y = float(p0[1]) + t * float(d[1])
        if -height <= y <= height * 2:
            candidates.append((int(round(x)), int(round(y))))
    if len(candidates) >= 2:
        a, b = candidates[:2]
    else:
        a = tuple(np.round(p0).astype(int).tolist())
        b = tuple(np.round(p1).astype(int).tolist())
    return (
        (int(np.clip(a[0], 0, width - 1)), int(np.clip(a[1], 0, height - 1))),
        (int(np.clip(b[0], 0, width - 1)), int(np.clip(b[1], 0, height - 1))),
    )


def group_hough_candidates(segments, width):
    groups = []
    for seg in segments:
        angle = seg["angle"]
        line = seg["line"]
        center_x = width * 0.5
        p = line["p_start"]
        d = line["direction"]
        if abs(float(d[0])) < 1e-6:
            continue
        center_y = float(p[1]) + (center_x - float(p[0])) * float(d[1]) / float(d[0])

        matched = None
        for group in groups:
            if abs(angle - group["mean_angle"]) > 6.0:
                continue
            if abs(center_y - group["mean_center_y"]) > 24.0:
                continue
            matched = group
            break

        if matched is None:
            matched = {
                "segments": [],
                "mean_angle": angle,
                "mean_center_y": center_y,
            }
            groups.append(matched)

        matched["segments"].append(seg)
        n = len(matched["segments"])
        matched["mean_angle"] = matched["mean_angle"] + (angle - matched["mean_angle"]) / n
        matched["mean_center_y"] = matched["mean_center_y"] + (center_y - matched["mean_center_y"]) / n
    return groups


def line_y_at_x(line, x):
    p = line["p_start"]
    d = line["direction"]
    if abs(float(d[0])) < 1e-6:
        return None
    return float(p[1]) + (float(x) - float(p[0])) * float(d[1]) / float(d[0])


def sample_line_side_stats(frame_bgr, line, x_min, x_max, step_px=16, band_w=5, offset=16):
    height, width = frame_bgr.shape[:2]
    xs = np.arange(max(0, int(x_min)), min(width - 1, int(x_max)) + 1, step_px, dtype=np.int32)
    if len(xs) == 0:
        return None

    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    above_pixels = []
    below_pixels = []
    used_xs = []
    for x in xs:
        y = line_y_at_x(line, int(x))
        if y is None:
            continue
        ya = int(round(y - offset))
        yb = int(round(y + offset))
        if ya - band_w < 0 or yb + band_w >= height:
            continue
        x0 = max(0, int(x) - 2)
        x1 = min(width, int(x) + 3)
        above_pixels.append(hsv[ya - band_w : ya + band_w + 1, x0:x1].reshape(-1, 3))
        below_pixels.append(hsv[yb - band_w : yb + band_w + 1, x0:x1].reshape(-1, 3))
        used_xs.append(int(x))

    if len(above_pixels) < 8:
        return None

    above = np.concatenate(above_pixels, axis=0).astype(np.float32)
    below = np.concatenate(below_pixels, axis=0).astype(np.float32)
    above_h, above_s, above_v = above[:, 0], above[:, 1], above[:, 2]
    below_h, below_s, below_v = below[:, 0], below[:, 1], below[:, 2]

    above_wall_fraction = float(np.mean((above_s < 70) & (above_v > 45)))
    below_yellow_fraction = float(np.mean((below_h >= 12) & (below_h <= 45) & (below_s > 45) & (below_v > 45)))
    above_yellow_fraction = float(np.mean((above_h >= 12) & (above_h <= 45) & (above_s > 45) & (above_v > 45)))
    below_floor_fraction = float(np.mean((below_s > 38) | (below_v < above_v.mean() - 8)))
    sat_delta = float(np.mean(below_s) - np.mean(above_s))
    value_delta = float(np.mean(below_v) - np.mean(above_v))

    return {
        "sample_count": int(len(used_xs)),
        "above_wall_fraction": above_wall_fraction,
        "below_yellow_fraction": below_yellow_fraction,
        "above_yellow_fraction": above_yellow_fraction,
        "below_floor_fraction": below_floor_fraction,
        "sat_delta": sat_delta,
        "value_delta": value_delta,
        "x_min": int(min(used_xs)),
        "x_max": int(max(used_xs)),
    }


def wall_floor_semantic_ok(stats):
    if stats is None:
        return False
    yellow_gain = stats["below_yellow_fraction"] - stats["above_yellow_fraction"]
    if stats["below_yellow_fraction"] >= 0.18 and yellow_gain >= 0.10 and stats["above_wall_fraction"] >= 0.18:
        return True
    if stats["below_floor_fraction"] >= 0.45 and stats["above_wall_fraction"] >= 0.35 and stats["sat_delta"] >= 8.0:
        return True
    if stats["above_wall_fraction"] >= 0.55 and stats["below_floor_fraction"] >= 0.35 and stats["value_delta"] <= 12.0:
        return True
    return False


def detect_wall_black_line(
    frame_bgr,
    roi_x1_ratio=0.02,
    roi_x2_ratio=0.98,
    roi_y1_ratio=0.68,
    roi_y2_ratio=0.98,
    dark_threshold=85,
    min_width_ratio=0.35,
    max_component_height_ratio=0.16,
    min_aspect=4.0,
):
    height, width = frame_bgr.shape[:2]
    x1 = int(round(width * roi_x1_ratio))
    x2 = int(round(width * roi_x2_ratio))
    y1 = int(round(height * roi_y1_ratio))
    y2 = int(round(height * roi_y2_ratio))
    x1, x2 = max(0, x1), min(width - 1, x2)
    y1, y2 = max(0, y1), min(height - 1, y2)
    roi = frame_bgr[y1 : y2 + 1, x1 : x2 + 1]

    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    gray_eq = clahe.apply(gray)
    dark = (gray <= dark_threshold).astype(np.uint8) * 255

    close_kernel_w = max(25, int(width * 0.06))
    dark = cv2.morphologyEx(dark, cv2.MORPH_CLOSE, np.ones((3, close_kernel_w), np.uint8))
    dark = cv2.morphologyEx(dark, cv2.MORPH_OPEN, np.ones((3, 9), np.uint8))

    edges = cv2.bitwise_or(cv2.Canny(gray, 24, 96), cv2.Canny(gray_eq, 20, 80))
    dark_fraction = float(np.count_nonzero(dark)) / float(max(1, dark.size))
    if dark_fraction < 0.55:
        dark_edges = cv2.bitwise_or(edges, cv2.Canny(dark, 50, 150))
    else:
        dark_edges = edges
    hough_lines = cv2.HoughLinesP(
        dark_edges,
        rho=1,
        theta=np.pi / 180.0,
        threshold=24,
        minLineLength=max(90, int(width * 0.18)),
        maxLineGap=130,
    )
    hough_segments = []
    if hough_lines is not None:
        for item in hough_lines[:, 0, :]:
            hx1, hy1, hx2, hy2 = [int(v) for v in item]
            gx1, gy1 = hx1 + x1, hy1 + y1
            gx2, gy2 = hx2 + x1, hy2 + y1
            length = float(math.hypot(gx2 - gx1, gy2 - gy1))
            if length < width * 0.14:
                continue
            angle = math.degrees(math.atan2(gy2 - gy1, gx2 - gx1))
            if angle < -90:
                angle += 180
            if angle > 90:
                angle -= 180
            if abs(angle) > 35.0:
                continue
            y_mid = 0.5 * (gy1 + gy2)
            # When G1 is close to correctly aligned, the wall/floor line can
            # sit near the top of the lower ROI. Keep only a tiny top guard;
            # semantic checks below reject cabinet/text edges.
            if y_mid < y1 + 0.03 * (y2 - y1):
                continue
            line = make_line_from_endpoints([gx1, gy1], [gx2, gy2])
            if line is None:
                continue
            hough_segments.append(
                {
                    "line": line,
                    "length": length,
                    "angle": angle,
                    "points": sample_segment_points([gx1, gy1], [gx2, gy2]),
                    "bbox_xywh": [
                        int(min(gx1, gx2)),
                        int(min(gy1, gy2)),
                        int(abs(gx2 - gx1) + 1),
                        int(abs(gy2 - gy1) + 1),
                    ],
                }
            )

    hough_candidates = []
    for group in group_hough_candidates(hough_segments, width):
        segments = group["segments"]
        points = np.concatenate([seg["points"] for seg in segments], axis=0)
        line = fit_line(points)
        if line is None:
            continue
        angle = line_angle_deg(line)
        if abs(angle) > 35.0:
            continue

        endpoints = []
        for seg in segments:
            endpoints.append(seg["line"]["p_start"])
            endpoints.append(seg["line"]["p_end"])
        endpoints = np.asarray(endpoints, dtype=np.float32)
        xs = endpoints[:, 0]
        ys = endpoints[:, 1]
        coverage_x = float(np.max(xs) - np.min(xs))
        total_length = float(sum(seg["length"] for seg in segments))
        segment_count = len(segments)
        mean_y = float(np.mean(ys))
        side_stats = sample_line_side_stats(frame_bgr, line, np.min(xs), np.max(xs))

        # Reject short local edges: a valid wall line should either span a
        # meaningful part of the image or be supported by multiple collinear
        # Hough segments. This prevents false "aligned" readings from tiny
        # horizontal texture/label edges.
        if coverage_x < width * 0.34 and total_length < width * 0.52:
            continue
        if segment_count < 2 and coverage_x < width * 0.42:
            continue
        if line["mean_abs_error_px"] > 12.0:
            continue
        if not wall_floor_semantic_ok(side_stats):
            continue

        score = (
            coverage_x * 3.0
            + total_length * 1.2
            + mean_y * 0.25
            + segment_count * 35.0
            + side_stats["above_wall_fraction"] * 180.0
            + max(0.0, side_stats["below_yellow_fraction"] - side_stats["above_yellow_fraction"]) * 260.0
            - line["mean_abs_error_px"] * 12.0
        )
        hough_candidates.append(
            {
                "line": line,
                "score": score,
                "coverage_x_px": coverage_x,
                "total_length_px": total_length,
                "segment_count": segment_count,
                "side_stats": side_stats,
                "bbox_xywh": [
                    int(np.min(xs)),
                    int(np.min(ys)),
                    int(np.max(xs) - np.min(xs) + 1),
                    int(np.max(ys) - np.min(ys) + 1),
                ],
            }
        )

    if hough_candidates:
        hough_candidates.sort(key=lambda item: item["score"], reverse=True)
        best_hough = hough_candidates[0]
        line = best_hough["line"]
        p0, p1 = endpoints_for_image(line, width, height)
        result = {
            "status": "ok",
            "method": "hough_group",
            "roi_xyxy": [x1, y1, x2, y2],
            "dark_threshold": int(dark_threshold),
            "dark_fraction": dark_fraction,
            "angle_deg": line_angle_deg(line),
            "line_p1_uv": [int(p0[0]), int(p0[1])],
            "line_p2_uv": [int(p1[0]), int(p1[1])],
            "hough_segment_p1_uv": [int(line["p_start"][0]), int(line["p_start"][1])],
            "hough_segment_p2_uv": [int(line["p_end"][0]), int(line["p_end"][1])],
            "component_bbox_xywh": best_hough["bbox_xywh"],
            "component_area": 0,
            "component_aspect": 0.0,
            "inliers": int(line["count"]),
            "mean_abs_error_px": float(line["mean_abs_error_px"]),
            "max_abs_error_px": float(line["max_abs_error_px"]),
            "hough_segment_count": int(best_hough["segment_count"]),
            "hough_coverage_x_px": float(best_hough["coverage_x_px"]),
            "hough_total_length_px": float(best_hough["total_length_px"]),
            "side_stats": best_hough["side_stats"],
        }
        debug = {"roi_xyxy": [x1, y1, x2, y2], "dark_mask": dark}
        return result, debug

    labels_n, labels, stats, _centroids = cv2.connectedComponentsWithStats(dark, 8)
    candidates = []
    min_width = width * min_width_ratio
    max_component_height = height * max_component_height_ratio

    for label in range(1, labels_n):
        area = int(stats[label, cv2.CC_STAT_AREA])
        bx = int(stats[label, cv2.CC_STAT_LEFT])
        by = int(stats[label, cv2.CC_STAT_TOP])
        bw = int(stats[label, cv2.CC_STAT_WIDTH])
        bh = int(stats[label, cv2.CC_STAT_HEIGHT])
        if bw < min_width or bh <= 1 or bh > max_component_height:
            continue
        aspect = float(bw) / float(max(1, bh))
        if aspect < min_aspect:
            continue
        ys, xs = np.where(labels == label)
        points = np.stack([xs + x1, ys + y1], axis=1)
        line = fit_line(points)
        if line is None:
            continue
        angle = abs(line_angle_deg(line))
        if angle > 25.0:
            continue
        score = float(bw) * 2.0 + float(area) - abs(line_angle_deg(line)) * 8.0
        candidates.append(
            {
                "label": label,
                "area": area,
                "bbox_xywh": [bx + x1, by + y1, bw, bh],
                "aspect": aspect,
                "line": line,
                "score": score,
                "points": points,
            }
        )

    if not candidates:
        return None, {"roi_xyxy": [x1, y1, x2, y2], "dark_mask": dark}

    candidates.sort(key=lambda item: item["score"], reverse=True)
    best = candidates[0]
    line = best["line"]
    p0, p1 = endpoints_for_image(line, width, height)
    result = {
        "status": "ok",
        "method": "component",
        "roi_xyxy": [x1, y1, x2, y2],
        "dark_threshold": int(dark_threshold),
        "dark_fraction": dark_fraction,
        "angle_deg": line_angle_deg(line),
        "line_p1_uv": [int(p0[0]), int(p0[1])],
        "line_p2_uv": [int(p1[0]), int(p1[1])],
        "component_bbox_xywh": best["bbox_xywh"],
        "component_area": int(best["area"]),
        "component_aspect": float(best["aspect"]),
        "inliers": int(line["count"]),
        "mean_abs_error_px": float(line["mean_abs_error_px"]),
        "max_abs_error_px": float(line["max_abs_error_px"]),
    }
    debug = {"roi_xyxy": [x1, y1, x2, y2], "dark_mask": dark, "points": best["points"]}
    return result, debug


def draw_overlay(frame_bgr, result, debug):
    overlay = frame_bgr.copy()
    height, width = overlay.shape[:2]
    x1, y1, x2, y2 = debug["roi_xyxy"]
    cv2.rectangle(overlay, (x1, y1), (x2, y2), (255, 255, 255), 1)
    if "dark_mask" in debug:
        mask = np.zeros((height, width), dtype=np.uint8)
        mask[y1 : y2 + 1, x1 : x2 + 1] = debug["dark_mask"]
        colored = overlay.copy()
        colored[mask > 0] = (0, 180, 255)
        overlay = cv2.addWeighted(colored, 0.35, overlay, 0.65, 0)
    if result is not None:
        p0 = tuple(result["line_p1_uv"])
        p1 = tuple(result["line_p2_uv"])
        cv2.line(overlay, p0, p1, (0, 0, 255), 4)
        if result.get("method") == "hough":
            hp0 = tuple(result["hough_segment_p1_uv"])
            hp1 = tuple(result["hough_segment_p2_uv"])
            cv2.line(overlay, hp0, hp1, (0, 255, 255), 2)
        else:
            bx, by, bw, bh = result["component_bbox_xywh"]
            cv2.rectangle(overlay, (bx, by), (bx + bw - 1, by + bh - 1), (0, 255, 255), 2)
        text = (
            f"black wall line angle={result['angle_deg']:+.2f}deg "
            f"pts={result['inliers']} err={result['mean_abs_error_px']:.1f}px"
        )
    else:
        text = "black wall line: NOT FOUND"
    cv2.putText(overlay, text, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (0, 0, 0), 4, cv2.LINE_AA)
    cv2.putText(overlay, text, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 255), 1, cv2.LINE_AA)
    return overlay


def iter_images(path):
    path = Path(path)
    if path.is_file():
        return [path]
    return sorted(p for p in path.iterdir() if p.suffix.lower() in IMAGE_EXTS)


def main():
    parser = argparse.ArgumentParser(description="Detect the dark wall/floor corner line from RGB images.")
    parser.add_argument("--image", type=Path, help="Single image path.")
    parser.add_argument("--image-dir", type=Path, help="Directory of images.")
    parser.add_argument("--out-dir", type=Path, default=Path("/home/ybbb/g1_dev/mimo/wall_black_line_debug"))
    parser.add_argument("--json-out", type=Path, default=None)
    parser.add_argument("--roi-y1-ratio", type=float, default=0.68)
    parser.add_argument("--roi-y2-ratio", type=float, default=0.98)
    parser.add_argument("--dark-threshold", type=int, default=85)
    args = parser.parse_args()

    source = args.image if args.image is not None else args.image_dir
    if source is None:
        raise SystemExit("provide --image or --image-dir")

    args.out_dir.mkdir(parents=True, exist_ok=True)
    records = []
    for image_path in iter_images(source):
        frame = cv2.imread(str(image_path))
        if frame is None:
            records.append({"image": str(image_path), "status": "read_failed"})
            continue
        result, debug = detect_wall_black_line(
            frame,
            roi_y1_ratio=args.roi_y1_ratio,
            roi_y2_ratio=args.roi_y2_ratio,
            dark_threshold=args.dark_threshold,
        )
        if result is None:
            rec = {"image": str(image_path), "status": "no_line"}
        else:
            rec = {"image": str(image_path), **result}
        records.append(rec)
        overlay = draw_overlay(frame, result, debug)
        cv2.imwrite(str(args.out_dir / f"{image_path.stem}_wall_line_overlay.jpg"), overlay)
        print(json.dumps(rec, ensure_ascii=False), flush=True)

    if args.json_out is not None:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(json.dumps(records, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
