import argparse
import csv
import hashlib
import inspect
import json
import math
import time
from pathlib import Path

import cv2
import numpy as np

from src.perception.lane_detection import ROAD_LINE_TAG, ROAD_TAG, LaneDetector, fit_lanes
from scripts.lane_ground_truth import component_lane_fits

CAMERA_HEIGHT_M = 1.7
REFERENCE_WIDTH = 512


def normalized_error_px(error_px: float, image_width: int) -> float:
    """Pixel error at the same FOV, expressed at the 512-pixel reference width."""
    return error_px * REFERENCE_WIDTH / image_width


def session_complete(meta: dict) -> bool:
    requested = meta.get("requested_frames")
    return bool(meta.get("complete", True) and not meta.get("aborted") and (
        requested is None or meta.get("frames", 0) >= requested
    ))


def m1_summary(results: list[dict], min_valid_frames: int, normalized: bool = False) -> dict:
    eligible = [r for r in results if r["benchmark_eligible"] and r["gt_valid"] >= min_valid_frames]
    key = "seg_within15_ref" if normalized else "seg_within15"
    visible = sum(r["gt_valid"] for r in eligible)
    correct = sum(r[key] for r in eligible)
    return {
        "eligible_sessions": len(eligible),
        "excluded_sessions": len(results) - len(eligible),
        "gt_valid": visible,
        "within15": correct,
        "score": correct / visible if visible else None,
        "verdict": ("PASS" if all(r[key] / r["gt_valid"] >= 0.8 for r in eligible)
                    else "FAIL") if eligible else "N/A",
    }


def pixels_per_meter_at_bottom(width: int, height: int, fov_deg: float,
                               camera_height_m: float = CAMERA_HEIGHT_M) -> float:
    focal_px = width / (2.0 * math.tan(math.radians(fov_deg) / 2.0))
    ray_angle = math.atan((height / 2.0) / focal_px)
    lookahead_m = camera_height_m / math.tan(ray_angle)
    return focal_px / lookahead_m


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Evaluate lane detection against semantic seg")
    parser.add_argument("--session", required=True, nargs="+")
    parser.add_argument("--start", type=int, default=0)
    parser.add_argument("--limit", type=int, default=0)
    parser.add_argument("--save-overlay", type=int, default=0)
    parser.add_argument("--save-video", action="store_true")
    parser.add_argument("--verbose", action="store_true")
    parser.add_argument("--report", type=Path, help="write versioned JSON metrics")
    parser.add_argument("--audit-dir", type=Path,
                        help="write per-frame CSV and sampled failed/GT-excluded overlays")
    parser.add_argument("--opencv-threads", type=int, default=1,
                        help="OpenCV worker count for reproducible latency measurements")
    parser.add_argument("--include-incomplete", action="store_true",
                        help="include sessions aborted early (diagnostics only; not an M1 verdict)")
    parser.add_argument("--roi-top", type=float, default=0.55)
    parser.add_argument("--min-valid-frames", type=int, default=50,
                        help="minimum visible seg-GT frames for a lane-center verdict")
    parser.add_argument("--max-missed", type=int, default=5,
                        help="frames to keep a missing lane fit before declaring loss")
    parser.add_argument("--smooth", type=float, default=0.85,
                        help="EMA weight for previous lane fit")
    args = parser.parse_args(argv)
    if not 0 <= args.smooth < 1 or args.max_missed < 0 or args.opencv_threads < 1:
        parser.error("require 0 <= smooth < 1, max-missed >= 0, and opencv-threads >= 1")
    if not 0 <= args.roi_top < 1 or args.min_valid_frames < 1 or args.start < 0 or args.limit < 0:
        parser.error("invalid ROI, frame range, or minimum valid frame count")
    return args


def draw_fit(image: np.ndarray, fit: np.ndarray, y0: int, roi_height: int,
             scale_x: float, scale_y: float, color: tuple[int, int, int]) -> None:
    ys = np.linspace(0, roi_height - 1, 30)
    xs = np.polyval(fit, ys)
    points = np.stack([xs * scale_x, (ys + y0) * scale_y], axis=1).astype(np.int32)
    cv2.polylines(image, [points], False, color, 2)


def evaluate_session(session: str, args: argparse.Namespace) -> dict:
    session = Path(session)
    meta = json.loads((session / "meta.json").read_text())
    rgb_w = meta["rgb_width"]
    rgb_h = meta["rgb_height"]
    aux_w = meta["aux_width"]
    aux_h = meta["aux_height"]
    scale_x = rgb_w / aux_w
    scale_y = rgb_h / aux_h

    ego_rows = {}
    has_lane_gt = False
    has_lane_gt_valid = False
    has_is_junction = False
    csv_path = session / "ego.csv"
    if csv_path.exists():
        with csv_path.open() as handle:
            reader = csv.DictReader(handle)
            has_lane_gt = "lane_offset_m" in (reader.fieldnames or [])
            has_lane_gt_valid = "lane_gt_valid" in (reader.fieldnames or [])
            has_is_junction = "is_junction" in (reader.fieldnames or [])
            for row in reader:
                ego_rows[int(row["index"])] = row

    frame_paths = sorted((session / "rgb").glob("*.png"))
    frame_paths = frame_paths[args.start:]
    if args.limit:
        frame_paths = frame_paths[: args.limit]
    if not frame_paths:
        raise SystemExit(f"no frames in {session / 'rgb'}")

    detector = LaneDetector(
        roi_top=args.roi_top, smooth=args.smooth, max_missed=args.max_missed
    )
    overlay_dir = session / "eval"
    if args.save_overlay or args.save_video:
        overlay_dir.mkdir(exist_ok=True)
    video = None
    if args.save_video:
        fps = 1.0 / float(meta.get("fixed_delta", 0.05))
        video = cv2.VideoWriter(
            str(overlay_dir / "overlay.mp4"),
            cv2.VideoWriter_fourcc(*"mp4v"),
            fps,
            (rgb_w, rgb_h),
        )
        if not video.isOpened():
            raise RuntimeError(f"could not create {overlay_dir / 'overlay.mp4'}")

    y0_aux = int(aux_h * args.roi_top)
    roi_h_aux = aux_h - y0_aux
    y0_rgb = int(rgb_h * args.roi_top)
    roi_h_rgb = rgb_h - y0_rgb
    aux_scale = aux_w / 192
    aux_fit = {
        "n_windows": 6,
        "margin": max(1, round(20 * aux_scale)),
        "min_pixels": max(1, round(5 * aux_scale)),
        "min_base_pixels": max(1, round(3 * aux_scale)),
    }

    tp = fp = fn = 0
    frame_count = 0
    pred_detected = 0
    gt_valid = 0
    compared = 0
    errors: list[float] = []
    seg_within15 = 0
    seg_within15_ref = 0
    detector_ms = []
    frame_audit = []
    gt_reasons = {}
    unreadable_frames = 0
    audit_images = {}
    gt_disagreements = []
    fresh_frames = 0
    held_frames = 0
    audit_dir = args.audit_dir / session.name if args.audit_dir else None
    if audit_dir:
        audit_dir.mkdir(parents=True, exist_ok=True)
    map_errors: list[float] = []
    map_gt_frames = 0
    map_within15 = 0
    map_visible_frames = 0
    map_visible_within15 = 0
    map_visible_errors: list[float] = []
    junction_frames = 0
    map_px_per_meter = pixels_per_meter_at_bottom(
        rgb_w, rgb_h, float(meta.get("fov", 90)), CAMERA_HEIGHT_M
    )

    for i, rgb_path in enumerate(frame_paths):
        bgr = cv2.imread(str(rgb_path))
        tags = cv2.imread(str(session / "seg" / rgb_path.name), cv2.IMREAD_GRAYSCALE)
        if bgr is None or tags is None:
            unreadable_frames += 1
            frame_audit.append({"index": int(rgb_path.stem), "gt_reason": "unreadable_input"})
            continue
        if bgr.shape[:2] != (rgb_h, rgb_w) or tags.shape != (aux_h, aux_w):
            raise ValueError(f"sensor dimensions disagree with metadata: {rgb_path}")
        frame_count += 1

        road = ((tags == ROAD_TAG) | (tags == ROAD_LINE_TAG)).astype(np.uint8) * 255
        road = cv2.resize(road, (rgb_w, rgb_h), interpolation=cv2.INTER_NEAREST)
        road = cv2.dilate(road, np.ones((5, 5), np.uint8))
        detect_start = time.perf_counter()
        state = detector.detect(bgr, road_mask=road)
        detector_ms.append((time.perf_counter() - detect_start) * 1000.0)
        pred_detected += int(state.detected)
        fresh = state.detected and getattr(state, "fresh", True)
        fresh_frames += int(fresh)
        held_frames += int(state.detected and not fresh)

        if has_lane_gt:
            row = ego_rows.get(int(rgb_path.stem))
            valid = row is not None
            if valid and has_lane_gt_valid:
                valid = row.get("lane_gt_valid", "0") == "1"
            is_junction = valid and has_is_junction and row.get("is_junction", "0") == "1"
            if is_junction:
                junction_frames += 1
            if valid and not is_junction and row.get("lane_offset_m", "") not in ("", "nan"):
                map_gt_frames += 1
                gt_px = float(row["lane_offset_m"]) * map_px_per_meter
                if state.detected:
                    error = abs(state.offset_px - gt_px)
                    map_errors.append(error)
                    map_within15 += int(error < 15.0)

        pred_aux = cv2.resize(state.mask, (aux_w, aux_h), interpolation=cv2.INTER_AREA) > 64
        gt_aux = tags == ROAD_LINE_TAG
        pred_roi = pred_aux[y0_aux:]
        gt_roi = gt_aux[y0_aux:]
        tp += int(np.logical_and(pred_roi, gt_roi).sum())
        fp += int(np.logical_and(pred_roi, ~gt_roi).sum())
        fn += int(np.logical_and(~pred_roi, gt_roi).sum())

        left_gt, right_gt = fit_lanes(
            gt_aux.astype(np.uint8) * 255, args.roi_top, **aux_fit
        )
        visible_row = ego_rows.get(int(rgb_path.stem))
        is_junction = (visible_row is not None and has_is_junction
                       and visible_row.get("is_junction", "0") == "1")
        if is_junction:
            gt_reason = "junction"
        elif left_gt is None and right_gt is None:
            gt_reason = "gt_fit_missing_both"
        elif left_gt is None:
            gt_reason = "gt_fit_missing_left"
        elif right_gt is None:
            gt_reason = "gt_fit_missing_right"
        else:
            gt_reason = "valid"
        gt_reasons[gt_reason] = gt_reasons.get(gt_reason, 0) + 1
        error = None
        gt_center_rgb = None
        if left_gt is not None and right_gt is not None:
            visible_row = ego_rows.get(int(rgb_path.stem))
            is_junction = (
                visible_row is not None
                and has_is_junction
                and visible_row.get("is_junction", "0") == "1"
            )
            if not is_junction:
                gt_valid += 1
                if has_lane_gt and visible_row is not None:
                    if (visible_row.get("lane_gt_valid", "0") == "1"
                            and visible_row.get("lane_offset_m", "") not in ("", "nan")):
                        map_visible_frames += 1
                        gt_px = float(visible_row["lane_offset_m"]) * map_px_per_meter
                        if state.detected:
                            error = abs(state.offset_px - gt_px)
                            map_visible_errors.append(error)
                            map_visible_within15 += int(error < 15.0)
                gt_center = 0.5 * (
                    float(np.polyval(left_gt, roi_h_aux - 1))
                    + float(np.polyval(right_gt, roi_h_aux - 1))
                )
                gt_center_rgb = gt_center * scale_x
                if state.detected and state.left_fit is not None and state.right_fit is not None:
                    compared += 1
                    pred_center_rgb = 0.5 * (
                        float(np.polyval(state.left_fit, roi_h_rgb - 1))
                        + float(np.polyval(state.right_fit, roi_h_rgb - 1))
                    )
                    pred_center = pred_center_rgb / scale_x
                    error = abs(pred_center - gt_center) * scale_x
                    errors.append(error)
                    seg_within15 += int(error < 15.0)
                    seg_within15_ref += int(normalized_error_px(error, rgb_w) < 15.0)
                    if args.verbose:
                        print(f"  {rgb_path.stem} pred=True offset={state.offset_px:+.1f}px err={error:.1f}px")
                elif args.verbose:
                    print(f"  {rgb_path.stem} pred=False gt_valid=True")

        component_left = component_right = None
        component_center = disagreement = None
        if audit_dir:
            component_left, component_right = component_lane_fits(gt_aux, args.roi_top)
            if not is_junction and component_left is not None and component_right is not None:
                component_center = 0.5 * (
                    float(np.polyval(component_left, roi_h_aux - 1))
                    + float(np.polyval(component_right, roi_h_aux - 1))
                ) * scale_x
                if gt_center_rgb is not None:
                    disagreement = normalized_error_px(abs(component_center - gt_center_rgb), rgb_w)
                    gt_disagreements.append(disagreement)
        audit_reason = gt_reason if gt_reason != "valid" else (
            "detector_miss" if error is None else
            "center_error" if normalized_error_px(error, rgb_w) >= 15.0 else "ok"
        )
        if disagreement is not None and disagreement >= 10.0:
            audit_reason = "gt_disagreement"
        frame_audit.append({
            "index": int(rgb_path.stem), "gt_reason": gt_reason,
            "detected": state.detected, "fresh": fresh,
            "missed_frames": getattr(state, "missed_frames", None), "error_px": error,
            "error_ref_px": normalized_error_px(error, rgb_w) if error is not None else None,
            "gt_center_px": gt_center_rgb, "component_center_px": component_center,
            "gt_disagreement_ref_px": disagreement,
            "detector_ms": detector_ms[-1],
        })
        save_audit = (audit_dir is not None and audit_reason != "ok"
                      and audit_images.get(audit_reason, 0) < 3)
        if i < args.save_overlay or video is not None or save_audit:
            visual = bgr.copy()
            visual[cv2.resize(gt_aux.astype(np.uint8) * 255, (rgb_w, rgb_h),
                              interpolation=cv2.INTER_NEAREST) > 0] = (0, 0, 255)
            visual[state.mask > 0] = (0, 255, 0)
            if state.left_fit is not None:
                draw_fit(visual, state.left_fit, y0_rgb, roi_h_rgb, 1.0, 1.0, (255, 0, 0))
            if state.right_fit is not None:
                draw_fit(visual, state.right_fit, y0_rgb, roi_h_rgb, 1.0, 1.0, (255, 0, 0))
            if left_gt is not None:
                draw_fit(visual, left_gt, y0_aux, roi_h_aux, scale_x, scale_y, (255, 255, 0))
            if right_gt is not None:
                draw_fit(visual, right_gt, y0_aux, roi_h_aux, scale_x, scale_y, (255, 255, 0))
            if save_audit:
                for component_fit in (component_left, component_right):
                    if component_fit is not None:
                        draw_fit(visual, component_fit, y0_aux, roi_h_aux, scale_x, scale_y, (255, 0, 255))
            cv2.putText(visual, f"idx {rgb_path.stem} offset {state.offset_px:+.1f}px", (8, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
            if i < args.save_overlay:
                cv2.imwrite(str(overlay_dir / rgb_path.name), visual)
            if video is not None:
                video.write(visual)
            if save_audit:
                cv2.putText(visual, audit_reason, (8, 42), cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, (0, 0, 255), 1, cv2.LINE_AA)
                cv2.imwrite(str(audit_dir / f"{audit_reason}_{rgb_path.name}"), visual)
                audit_images[audit_reason] = audit_images.get(audit_reason, 0) + 1

        if (i + 1) % 50 == 0:
            print(f"{i + 1}/{len(frame_paths)} frames", flush=True)

    if video is not None:
        video.release()
    if audit_dir:
        with (audit_dir / "frames.csv").open("w", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=[
                "index", "gt_reason", "detected", "fresh", "missed_frames", "error_px",
                "error_ref_px", "gt_center_px", "component_center_px",
                "gt_disagreement_ref_px", "detector_ms",
            ])
            writer.writeheader()
            writer.writerows(frame_audit)
    if not frame_count:
        raise ValueError(f"no readable sensor pairs in {session}")

    total = tp + fp + fn
    precision = tp / (tp + fp) if tp + fp else 0.0
    recall = tp / (tp + fn) if tp + fn else 0.0
    iou = tp / total if total else 0.0
    frame_count = max(frame_count, 1)

    print(f"{session}: frames={frame_count} pred_detected={pred_detected} "
          f"({pred_detected / frame_count:.1%}) gt_fit_valid={gt_valid} "
          f"compared={compared} ({compared / frame_count:.1%})")
    print(f"mask precision={precision:.3f} recall={recall:.3f} IoU={iou:.3f}")
    verdict = "FAIL"
    if errors:
        values = np.asarray(errors)
        within = float((values < 15.0).mean() * 100.0)
        print(f"seg center error px (n={len(values)}): mean={values.mean():.1f} "
              f"median={np.median(values):.1f} p90={np.percentile(values, 90):.1f} "
              f"within15px among compared={within:.1f}%")
    else:
        print("seg center error: no frames with both predictions and ground truth")
    seg_coverage = gt_valid / frame_count
    detector_coverage = compared / gt_valid if gt_valid else 0.0
    seg_accuracy = seg_within15 / compared if compared else 0.0
    seg_score = seg_within15 / gt_valid if gt_valid else 0.0
    seg_score_ref = seg_within15_ref / gt_valid if gt_valid else 0.0
    expected = meta.get("frames", len(frame_paths))
    exclusions = []
    if not session_complete(meta):
        exclusions.append("incomplete_session")
    if meta.get("route") == "intersection":
        exclusions.append("intersection_route")
    if args.start or frame_count != expected:
        exclusions.append("partial_or_missing_frames")
    if unreadable_frames:
        exclusions.append("unreadable_sensor_pairs")
    if not has_is_junction or any(int(path.stem) not in ego_rows for path in frame_paths):
        exclusions.append("missing_junction_labels")
    benchmark_eligible = not exclusions
    print(f"seg GT visible={gt_valid}/{frame_count} ({seg_coverage:.1%}) | "
          f"detector coverage={compared}/{gt_valid} ({detector_coverage:.1%}) | "
          f"accuracy on detected={seg_within15}/{compared} ({seg_accuracy:.1%}) | "
          f"end-to-end within15={seg_within15}/{gt_valid} ({seg_score:.1%})")
    if not benchmark_eligible:
        verdict = "N/A (diagnostic route, incomplete, partial, or damaged session)"
    elif gt_valid < args.min_valid_frames:
        verdict = "N/A (insufficient visible GT frames)"
    else:
        verdict = "PASS" if seg_score >= 0.80 else "FAIL"
    map_verdict = "N/A"
    map_score = 0.0
    map_visible_score = 0.0
    if has_lane_gt:
        gt_coverage = map_gt_frames / frame_count
        detected_coverage = len(map_errors) / map_gt_frames if map_gt_frames else 0.0
        map_score = map_within15 / map_gt_frames if map_gt_frames else 0.0
        if map_errors:
            values = np.asarray(map_errors)
            within = float((values < 15.0).mean() * 100.0)
            print(f"map GT center error px (n={len(values)}): mean={values.mean():.1f} "
                  f"median={np.median(values):.1f} p90={np.percentile(values, 90):.1f} "
                  f"within15px={within:.1f}%")
        else:
            print("map GT center error: detector produced no centerline")
        print(f"map GT (non-junction) valid={map_gt_frames}/{frame_count} ({gt_coverage:.1%}) | "
              f"junction={junction_frames} | detected={len(map_errors)}/{map_gt_frames} "
              f"({detected_coverage:.1%}) | within15 incl misses={map_within15}/{map_gt_frames} "
              f"({map_score:.1%})")
        if map_visible_frames:
            map_visible_score = map_visible_within15 / map_visible_frames
            print(f"map GT on visible seg-GT frames: valid={map_visible_frames}/{gt_valid} | "
                  f"detected={len(map_visible_errors)}/{map_visible_frames} | "
                  f"within15 incl misses={map_visible_within15}/{map_visible_frames} "
                  f"({map_visible_score:.1%})")
        if not benchmark_eligible or map_gt_frames < args.min_valid_frames:
            map_verdict = "N/A (insufficient non-junction GT frames)"
        else:
            map_verdict = "PASS" if map_score >= 0.80 else "FAIL"
    print("M1 seg-GT target (end-to-end >=80% visible frames, misses fail): " + verdict)
    verdict_ref = ("PASS" if seg_score_ref >= 0.8 else "FAIL") if (
        benchmark_eligible and gt_valid >= args.min_valid_frames
    ) else "N/A"
    print(f"M1-v2 (512px reference): {seg_within15_ref}/{gt_valid} ({seg_score_ref:.1%}) | {verdict_ref}")
    print(f"GT extraction audit (not manual visibility labels): {gt_reasons}")
    print(f"Detector latency ms (includes overlay): p50={np.median(detector_ms):.1f} "
          f"p95={np.percentile(detector_ms, 95):.1f}")
    print("M1 map-GT diagnostic (secondary): " + map_verdict)
    return {
        "session": str(session),
        "frames": frame_count,
        "pred_detected": pred_detected,
        "fresh_frames": fresh_frames,
        "held_frames": held_frames,
        "gt_valid": gt_valid,
        "compared": compared,
        "seg_within15": seg_within15,
        "seg_within15_ref": seg_within15_ref,
        "center_within15_ref": seg_score_ref,
        "seg_verdict_ref": verdict_ref,
        "benchmark_eligible": benchmark_eligible,
        "benchmark_exclusions": exclusions,
        "route": meta.get("route"),
        "weather": meta.get("weather"),
        "rgb_width": rgb_w,
        "fov": meta.get("fov", 90),
        "sensor_sync": meta.get("sensor_sync", "legacy_unverified"),
        "gt_extraction_reasons": gt_reasons,
        "gt_audit_compared": len(gt_disagreements),
        "gt_audit_disagreements_ge10_ref_px": sum(value >= 10.0 for value in gt_disagreements),
        "unreadable_frames": unreadable_frames,
        "detector_ms_p50": float(np.median(detector_ms)),
        "detector_ms_p95": float(np.percentile(detector_ms, 95)),
        "tp": tp,
        "fp": fp,
        "fn": fn,
        "precision": precision,
        "recall": recall,
        "iou": iou,
        "center_error_mean": float(np.mean(errors)) if errors else None,
        "center_within15": seg_within15 / gt_valid if gt_valid else 0.0,
        "gt_coverage": gt_valid / frame_count,
        "detector_coverage": compared / gt_valid if gt_valid else 0.0,
        "seg_accuracy": seg_accuracy,
        "seg_verdict": verdict,
        "seg_errors": errors,
        "has_lane_gt": has_lane_gt,
        "map_gt_frames": map_gt_frames,
        "map_within15": map_within15,
        "map_errors": map_errors,
        "map_visible_frames": map_visible_frames,
        "map_visible_within15": map_visible_within15,
        "map_visible_errors": map_visible_errors,
        "map_visible_score": map_visible_score,
        "junction_frames": junction_frames,
        "map_detected": len(map_errors),
        "map_score": map_within15 / map_gt_frames if map_gt_frames else 0.0,
        "map_verdict": map_verdict,
    }


def main() -> None:
    args = parse_args()
    cv2.setNumThreads(args.opencv_threads)
    sessions = []
    for session in args.session:
        meta = json.loads((Path(session) / "meta.json").read_text())
        requested = meta.get("requested_frames")
        complete = session_complete(meta)
        if not complete and not args.include_incomplete:
            reason = ",".join(meta.get("incomplete_reasons", [])) or meta.get("aborted") or "incomplete"
            print(f"SKIP incomplete session (reason={reason}, "
                  f"frames={meta.get('frames')}/{requested}): {session}")
            continue
        sessions.append(session)
    if not sessions:
        raise SystemExit("no complete sessions to evaluate; record a full session or use --include-incomplete for diagnostics")

    results = [evaluate_session(session, args) for session in sessions]
    frames = sum(result["frames"] for result in results)
    tp = sum(result["tp"] for result in results)
    fp = sum(result["fp"] for result in results)
    fn = sum(result["fn"] for result in results)
    pred_detected = sum(result["pred_detected"] for result in results)
    map_errors = [error for result in results for error in result["map_errors"]]
    iou = tp / (tp + fp + fn) if tp + fp + fn else 0.0
    precision = tp / (tp + fp) if tp + fp else 0.0
    recall = tp / (tp + fn) if tp + fn else 0.0
    print(f"COMBINED sessions={len(results)} frames={frames} "
          f"pred_detected={pred_detected} ({pred_detected / max(frames, 1):.1%})")
    print(f"COMBINED mask precision={precision:.3f} recall={recall:.3f} IoU={iou:.3f}")

    gt_valid = sum(result["gt_valid"] for result in results)
    compared = sum(result["compared"] for result in results)
    within15 = sum(result["seg_within15"] for result in results)
    seg_coverage = gt_valid / max(frames, 1)
    det_coverage = compared / gt_valid if gt_valid else 0.0
    seg_accuracy = within15 / compared if compared else 0.0
    seg_score = within15 / gt_valid if gt_valid else 0.0
    print(f"COMBINED seg GT visible={gt_valid}/{frames} ({seg_coverage:.1%}) | "
          f"detector coverage={compared}/{gt_valid} ({det_coverage:.1%}) | "
          f"accuracy on detected={within15}/{compared} ({seg_accuracy:.1%}) | "
          f"end-to-end within15={within15}/{gt_valid} ({seg_score:.1%})")
    map_gt_frames = sum(result["map_gt_frames"] for result in results)
    map_within15 = sum(result["map_within15"] for result in results)
    map_detected = sum(result["map_detected"] for result in results)
    map_visible_frames = sum(result["map_visible_frames"] for result in results)
    map_visible_within15 = sum(result["map_visible_within15"] for result in results)
    junction_frames = sum(result["junction_frames"] for result in results)
    map_score = map_within15 / map_gt_frames if map_gt_frames else 0.0
    map_visible_score = map_visible_within15 / map_visible_frames if map_visible_frames else 0.0
    map_coverage = map_gt_frames / max(frames, 1)
    print(f"COMBINED map GT (non-junction) valid={map_gt_frames}/{frames} ({map_coverage:.1%}) | "
          f"junction={junction_frames} | detected={map_detected}/{map_gt_frames} "
          f"({map_detected / map_gt_frames if map_gt_frames else 0.0:.1%}) | "
          f"within15 incl misses={map_within15}/{map_gt_frames} ({map_score:.1%})")
    if map_visible_frames:
        print(f"COMBINED map GT on visible seg-GT frames: valid={map_visible_frames}/{gt_valid} | "
              f"within15 incl misses={map_visible_within15}/{map_visible_frames} "
              f"({map_visible_score:.1%})")
    if map_errors:
        values = np.asarray(map_errors)
        print(f"COMBINED map GT detected-frame error: mean={values.mean():.1f} "
              f"median={np.median(values):.1f} p90={np.percentile(values, 90):.1f} "
              f"within15={float((values < 15).mean()):.1%}")
    legacy_summary = m1_summary(results, args.min_valid_frames)
    normalized_summary = m1_summary(results, args.min_valid_frames, normalized=True)
    combined_verdict = legacy_summary["verdict"]
    map_valid_sessions = [r for r in results if r["benchmark_eligible"]
                          and r["map_gt_frames"] >= args.min_valid_frames]
    if not map_valid_sessions:
        combined_map_verdict = "N/A"
    elif not all(r["map_verdict"] == "PASS" for r in map_valid_sessions):
        combined_map_verdict = "FAIL"
    else:
        combined_map_verdict = "PASS"
    print("COMBINED M1 seg-GT target (all valid sessions end-to-end >=80%): " + combined_verdict)
    print(f"COMBINED eligible-session M1 legacy: {legacy_summary}")
    print(f"COMBINED eligible-session M1-v2 (512px reference): {normalized_summary}")
    print("All-session aggregates above are diagnostics; N/A sessions do not affect either verdict.")
    print("COMBINED map-GT diagnostic (secondary): " + combined_map_verdict)
    print("\nPrimary table (per session, seg GT visible frames)")
    print("session | visible GT | det coverage | accuracy | end-to-end | verdict | seg IoU")
    for result in results:
        name = Path(result["session"]).name
        print(f"{name} | {result['gt_valid']}/{result['frames']} | "
              f"{result['detector_coverage']:.1%} | {result['seg_accuracy']:.1%} | "
              f"{result['center_within15']:.1%} | {result['seg_verdict']} | {result['iou']:.3f}")
    print(f"ALL ({len(results)} sessions) | {gt_valid}/{frames} | {det_coverage:.1%} | "
          f"{seg_accuracy:.1%} | {seg_score:.1%} | diagnostic | {iou:.3f}")
    print("\nDiagnostic table (map GT, non-junction)")
    print("session | map all | map visible")
    for result in results:
        name = Path(result["session"]).name
        print(f"{name} | {result['map_score']:.1%} | {result['map_visible_score']:.1%}")
    print(f"ALL ({len(results)} sessions) | {map_score:.1%} | {map_visible_score:.1%}")
    if args.report:
        report = {
            "benchmark_version": "M1-v2",
            "reference_width": REFERENCE_WIDTH,
            "threshold_ref_px": 15,
            "minimum_session_score": 0.8,
            "min_valid_frames": args.min_valid_frames,
            "input_mode": "RGB + simulator road mask",
            "detector_source_sha256": hashlib.sha256(Path(inspect.getfile(LaneDetector)).read_bytes()).hexdigest(),
            "opencv_version": cv2.__version__,
            "opencv_threads": cv2.getNumThreads(),
            "settings": {key: str(value) if isinstance(value, Path) else value
                         for key, value in vars(args).items()},
            "legacy": legacy_summary,
            "normalized": normalized_summary,
            "diagnostics_all_sessions": {
                "frames": frames, "visible_gt": gt_valid,
                "detector_coverage": det_coverage,
                "end_to_end_raw": seg_score,
                "end_to_end_ref": sum(r["seg_within15_ref"] for r in results) / gt_valid if gt_valid else None,
            },
            "sessions": results,
        }
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
        print(f"JSON report: {args.report}")


if __name__ == "__main__":
    main()
