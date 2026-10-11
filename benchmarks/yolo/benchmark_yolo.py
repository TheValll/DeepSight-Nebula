#!/usr/bin/env python3
"""Benchmark ONE model on the accepted videos of a guided capture session."""

import argparse
import csv
from datetime import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import statistics
import sys
import time

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
MODELS = [f"yolo11{s}" for s in "nsm"] + [f"yolo26{s}" for s in "nsm"] + [f"yoloe-26{s}" for s in "nsm"]


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as file:
        for chunk in iter(lambda: file.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")


def read_labels(path, width=1280, height=720):
    """Explicit empty file = no ball; missing file NEVER means no ball."""
    boxes = []
    for line in path.read_text().splitlines():
        if not line.strip():
            continue
        values = line.split()
        if len(values) != 5 or values[0] != "0":
            raise ValueError(f"Expected class 0 and normalized YOLO xywh: {path}")
        x, y, w, h = map(float, values[1:])
        if not all(math.isfinite(v) for v in (x, y, w, h)) or min(w, h) <= 0:
            raise ValueError(f"Invalid box: {path}")
        box = [x-w/2, y-h/2, x+w/2, y+h/2]
        if min(box) < -1e-6 or max(box) > 1+1e-6:
            raise ValueError(f"Box outside image: {path}")
        boxes.append([box[0]*width, box[1]*height, box[2]*width, box[3]*height])
    return boxes


def iou(a, b):
    intersection = max(0, min(a[2], b[2])-max(a[0], b[0])) * max(0, min(a[3], b[3])-max(a[1], b[1]))
    union = (a[2]-a[0])*(a[3]-a[1]) + (b[2]-b[0])*(b[3]-b[1]) - intersection
    return intersection / union if union > 0 else 0


def match(predictions, truth, threshold):
    """Confidence-ordered one-to-one IoU matching; duplicates remain false positives."""
    unmatched = set(range(len(truth)))
    tp = 0
    for prediction in sorted(predictions, key=lambda p: p["confidence"], reverse=True):
        if not unmatched:
            break
        target = max(unmatched, key=lambda j: iou(prediction["xyxy"], truth[j]))
        if iou(prediction["xyxy"], truth[target]) >= threshold:
            unmatched.remove(target)
            tp += 1
    return tp, len(predictions)-tp, len(truth)-tp


def ratio(numerator, denominator, scale=1):
    return scale*numerator/denominator if denominator else None


def summary(rows):
    evaluated = [r for r in rows if r["evaluated"]]
    tp = sum(r["tp"] for r in evaluated)
    fp = sum(r["fp"] for r in evaluated)
    fn = sum(r["fn"] for r in evaluated)
    negatives = [r for r in evaluated if r["ball_color"] == "none"]
    positives = [r for r in evaluated if r["gt_count"] > 0]
    result = {
        "frames": len(rows), "evaluated_frames": len(evaluated),
        "selected_reference_frames": sum(r["selected_for_evaluation"] for r in rows),
        "reference_coverage_percent": ratio(len(evaluated), sum(r["selected_for_evaluation"] for r in rows), 100),
        "tp": tp, "fp": fp, "fn": fn,
        "precision_percent": ratio(tp, tp+fp, 100),
        "recall_percent": ratio(tp, tp+fn, 100),
        "f1_percent": ratio(2*tp, 2*tp+fp+fn, 100),
        "f0_5_percent": ratio(1.25*tp, 1.25*tp+fp+0.25*fn, 100),
        "exact_match_frames_percent": ratio(sum(r["fp"] == 0 and r["fn"] == 0 for r in evaluated), len(evaluated), 100),
        "ball_visible_frames_detected_percent": ratio(sum(r["tp"] > 0 for r in positives), len(positives), 100),
        "frames_with_false_positives_percent": ratio(sum(r["fp"] > 0 for r in evaluated), len(evaluated), 100),
        "negative_frames": len(negatives),
        "negative_frames_with_false_positives_percent": ratio(sum(r["fp"] > 0 for r in negatives), len(negatives), 100),
        "false_boxes_per_negative_frame": ratio(sum(r["fp"] for r in negatives), len(negatives)),
        "box_presence_percent_NOT_accuracy": ratio(sum(r["prediction_count"] > 0 for r in rows), len(rows), 100),
    }
    for field in ("decode_ms", "inference_ms", "total_processing_ms", "cpu_system_percent", "ram_system_mb", "ram_process_mb", "gpu_system_percent", "vram_system_mb", "torch_allocated_mb"):
        values = [r[field] for r in rows if r[field] is not None]
        result[field+"_mean"] = statistics.fmean(values) if values else None
    inference = sorted(r["inference_ms"] for r in rows)
    for percentile in (50, 95, 99):
        result[f"inference_ms_p{percentile}"] = inference[max(0, math.ceil(len(inference)*percentile/100)-1)] if inference else None
    result["processing_fps"] = ratio(1000, result["total_processing_ms_mean"]) if result["total_processing_ms_mean"] else None
    return result


def load_session(path, annotations, stride, allow_unannotated=False, export=False):
    state = json.loads((path/"session.json").read_text())
    if not state.get("tasks") or set(state["accepted"]) != {t["id"] for t in state["tasks"]}:
        raise ValueError("Finish and validate every capture sequence before benchmarking")
    videos, missing = [], []
    for task in state["tasks"]:
        video = (path/state["accepted"][task["id"]]).resolve()
        if not video.is_relative_to(path) or not video.is_file():
            raise ValueError(f"Missing/unsafe accepted video: {video}")
        meta = json.loads(video.with_suffix(".json").read_text())
        if not meta.get("accepted") or meta.get("interrupted") or meta.get("id") != task["id"]:
            raise ValueError(f"Unvalidated or partial recording: {video}")
        count = meta["frames"]
        if not isinstance(count, int) or count < 2 or meta["saved_resolution"] != [1280, 720]:
            raise ValueError(f"Invalid video metadata: {video}")
        stamps = meta["frame_timestamps_s"]
        if len(stamps) != count or not all(math.isfinite(t) and t >= 0 for t in stamps) or any(b <= a for a, b in zip(stamps, stamps[1:])):
            raise ValueError(f"Invalid timestamps: {video}")
        truth, hashes = {}, {}
        if not export:
            for index in range(0, count, stride):
                label = annotations/video.stem/f"{index:06d}.txt"
                if label.is_file():
                    truth[index] = read_labels(label)
                    hashes[str(label.relative_to(annotations))] = sha256(label)
                    if task["expected_objects"] == 0 and truth[index]:
                        raise ValueError(f"A no-ball sequence has a ball annotation: {label}")
                elif task["expected_objects"] == 0:
                    truth[index] = []  # Explicit negative capture protocol, not a missing positive label.
                else:
                    missing.append(str(label))
        videos.append({"path": video, "task": task, "metadata": meta, "truth": truth,
                       "video_sha256": sha256(video), "label_sha256": hashes})
    if missing and not allow_unannotated:
        raise ValueError(f"{len(missing)} reference labels missing. Export/annotate frames first. Example: {missing[0]}. --allow-unannotated is diagnostic only.")
    return videos, missing


def export_frames(videos, session, annotations, stride, cv2):
    output = session/("reference_frames_"+datetime.now().strftime("%Y%m%d_%H%M%S_%f"))
    output.mkdir()
    with (output/"manifest.csv").open("w", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=["video", "frame_index", "image", "label", "status"])
        writer.writeheader()
        for item in videos:
            image_dir = output/item["path"].stem
            image_dir.mkdir()
            label_dir = annotations/item["path"].stem
            label_dir.mkdir(parents=True, exist_ok=True)
            cap = cv2.VideoCapture(str(item["path"]))
            count = 0
            try:
                while True:
                    ok, frame = cap.read()
                    if not ok:
                        break
                    if count % stride == 0:
                        image = image_dir/f"{count:06d}.png"
                        label = label_dir/f"{count:06d}.txt"
                        if not cv2.imwrite(str(image), frame):
                            raise RuntimeError(f"Could not save {image}")
                        if item["task"]["expected_objects"] == 0 and not label.exists():
                            label.write_text("")
                        writer.writerow({"video": item["path"].name, "frame_index": count, "image": str(image),
                                         "label": str(label), "status": "negative_capture" if item["task"]["expected_objects"] == 0 else "needs_review"})
                    count += 1
            finally:
                cap.release()
            if count != item["metadata"]["frames"]:
                raise RuntimeError(f"Decoded {count} frames, expected {item['metadata']['frames']}: {item['path']}")
    print(f"Reference images: {output}\nLabels: {annotations}\nAnnotate only real balls. An explicit empty label means no visible ball.")


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session", type=Path, required=True)
    parser.add_argument("--model", choices=MODELS)
    parser.add_argument("--variant", choices=("pretrained", "finetuned"), default="pretrained")
    parser.add_argument("--weights", type=Path, help="Override the canonical model weights")
    parser.add_argument("--annotations", type=Path, help="Default: SESSION/annotations")
    parser.add_argument("--output-dir", type=Path, help="New directory; never appends to previous runs")
    parser.add_argument("--confidence", type=float, default=0.50, help="Same threshold for all 18 configurations")
    parser.add_argument("--iou", type=float, default=0.50, help="Prediction/ground-truth matching IoU, not NMS")
    parser.add_argument("--imgsz", type=int, default=1280)
    parser.add_argument("--eval-stride", type=int, default=1, help="Score every Nth frame; all frames are still inferred")
    parser.add_argument("--device", choices=("cuda:0", "cpu"), default="cuda:0")
    parser.add_argument("--prompt", default="ping pong ball", help="Only applied to pretrained YOLOE")
    parser.add_argument("--allow-unannotated", action="store_true", help="Diagnostic mode: never count missing references as negatives")
    parser.add_argument("--export-frames", action="store_true", help="Prepare reference frames without running a model")
    args = parser.parse_args()
    if not args.export_frames and not args.model:
        parser.error("--model is required for a benchmark")
    if not 0 <= args.confidence <= 1 or not 0 < args.iou <= 1 or args.imgsz < 32 or args.eval_stride < 1:
        parser.error("Invalid confidence/IoU/resolution/stride")
    args.session = args.session.resolve()
    args.annotations = (args.annotations or args.session/"annotations").resolve()
    return args


def main():
    args = parse_args()
    import cv2
    videos, missing = load_session(args.session, args.annotations, args.eval_stride, args.allow_unannotated, args.export_frames)
    if args.export_frames:
        export_frames(videos, args.session, args.annotations, args.eval_stride, cv2)
        return 0
    if missing:
        print(f"WARNING: {len(missing)} missing reference frames. DIAGNOSTIC ONLY: partial quality metrics must not be used to rank models.", flush=True)
    import psutil
    import torch
    import ultralytics
    if args.device.startswith("cuda") and not torch.cuda.is_available():
        raise RuntimeError("CUDA unavailable; no silent fallback to CPU")
    if args.weights:
        weights = args.weights.resolve()
    elif args.variant == "finetuned":
        weights = HERE.parent/"finetuning/ping_pong_20261010"/args.model/"weights/best.pt"
    else:
        weights = HERE/"models"/(args.model+("-seg.pt" if args.model.startswith("yoloe") else ".pt"))
    if not weights.is_file():
        raise FileNotFoundError(weights)
    output = (args.output_dir or HERE/"results"/(args.model+"_"+args.variant+"_"+datetime.now().strftime("%Y%m%d_%H%M%S_%f"))).resolve()
    output.mkdir(parents=True, exist_ok=False)
    metadata = {"status": "running", "started_at": datetime.now().astimezone().isoformat(),
                "model": args.model, "variant": args.variant, "weights": str(weights), "weights_sha256": sha256(weights),
                "session": str(args.session), "annotations": str(args.annotations),
                "confidence": args.confidence, "matching_iou": args.iou, "imgsz": args.imgsz,
                "eval_stride": args.eval_stride, "device": args.device, "missing_reference_frames": len(missing),
                "quality_complete_at_selected_stride": not missing,
                "python": platform.python_version(), "ultralytics": ultralytics.__version__, "torch": torch.__version__,
                "matching": "confidence-ordered one-to-one IoU; no custom duplicate suppression",
                "notes": ["Offline throughput is not live camera FPS", "GPU/CPU/system RAM measurements include other applications", "Missing positive labels are excluded, never treated as empty", "No-ball captures have explicitly empty ground truth", "Timing excludes visualization and result serialization", "Quality is scored only on every eval_stride-th frame"],
                "inputs": [{"video": v["path"].name, "sha256": v["video_sha256"], "task": v["task"], "labels": v["label_sha256"]} for v in videos]}
    metadata["reference_fingerprint"] = hashlib.sha256(json.dumps(metadata["inputs"], sort_keys=True).encode()).hexdigest()
    write_json(output/"metadata.json", metadata)
    nvml = handle = None
    all_rows, per_video = [], []
    try:
        constructor = ultralytics.YOLOE if args.model.startswith("yoloe") else ultralytics.YOLO
        model = constructor(str(weights))
        prompt = args.prompt if args.model.startswith("yoloe") and args.variant == "pretrained" else None
        if prompt:
            previous = Path.cwd()
            try:
                os.chdir(HERE/"models")
                model.set_classes([prompt])
            finally:
                os.chdir(previous)
        class_id = 0 if args.variant == "finetuned" or prompt else 32
        if args.variant == "finetuned" and model.names != {0: "ping_pong_ball"}:
            raise ValueError(f"Unexpected fine-tuned classes: {model.names}")
        metadata.update(prompt=prompt, class_id=class_id, task=model.task)
        if args.device.startswith("cuda"):
            metadata["gpu"] = torch.cuda.get_device_name(0)
            try:
                import pynvml
                pynvml.nvmlInit()
                nvml = pynvml
                handle = nvml.nvmlDeviceGetHandleByIndex(0)
            except Exception as error:
                metadata["gpu_monitoring_warning"] = str(error)
        write_json(output/"metadata.json", metadata)
        # Same left-frame geometry and 20 warm-ups, excluded from all measurements.
        cap = cv2.VideoCapture(str(videos[0]["path"]))
        try:
            ok, warmup = cap.read()
            if not ok:
                raise RuntimeError("Could not decode warm-up image")
        finally:
            cap.release()
        options = dict(classes=[class_id], conf=args.confidence, imgsz=args.imgsz, device=args.device, verbose=False)
        for _ in range(20):
            model.predict(warmup, **options)
        process = psutil.Process()
        psutil.cpu_percent()
        wall_start = time.perf_counter()
        with (output/"frames.csv").open("w", newline="") as csv_file, (output/"predictions.jsonl").open("w") as predictions_file:
            writer = None
            for item in videos:
                cap = cv2.VideoCapture(str(item["path"]))
                rows = []
                try:
                    for index in range(item["metadata"]["frames"]):
                        started = time.perf_counter()
                        ok, image = cap.read()
                        decoded = time.perf_counter()
                        if not ok or image.shape[:2] != (720, 1280):
                            raise RuntimeError(f"Decode failed at {item['path']} frame {index}")
                        if args.device.startswith("cuda"):
                            torch.cuda.synchronize()
                        inference_start = time.perf_counter()
                        result = model.predict(image, **options)[0]
                        if args.device.startswith("cuda"):
                            torch.cuda.synchronize()
                        inference_ms = 1000*(time.perf_counter()-inference_start)
                        predictions = [] if result.boxes is None else [
                            {"xyxy": box, "confidence": confidence}
                            for box, confidence in zip(result.boxes.xyxy.detach().cpu().tolist(), result.boxes.conf.detach().cpu().tolist())]
                        truth = item["truth"].get(index)
                        tp, fp, fn = match(predictions, truth, args.iou) if truth is not None else (None, None, None)
                        gpu = vram = None
                        if handle is not None:
                            gpu = nvml.nvmlDeviceGetUtilizationRates(handle).gpu
                            vram = nvml.nvmlDeviceGetMemoryInfo(handle).used/2**20
                        row = {"video": item["path"].name, "ball_color": item["task"]["ball_color"], "movement": item["task"]["movement"],
                               "frame_index": index, "source_timestamp_s": item["metadata"]["frame_timestamps_s"][index],
                               "prediction_count": len(predictions), "evaluated": truth is not None,
                               "selected_for_evaluation": index % args.eval_stride == 0,
                               "gt_count": len(truth) if truth is not None else None, "tp": tp, "fp": fp, "fn": fn,
                               "decode_ms": 1000*(decoded-started), "inference_ms": inference_ms,
                               "cpu_system_percent": psutil.cpu_percent(), "ram_system_mb": psutil.virtual_memory().used/2**20,
                               "ram_process_mb": process.memory_info().rss/2**20, "gpu_system_percent": gpu, "vram_system_mb": vram,
                               "torch_allocated_mb": torch.cuda.memory_allocated()/2**20 if args.device.startswith("cuda") else None}
                        row["total_processing_ms"] = 1000*(time.perf_counter()-started)
                        if writer is None:
                            writer = csv.DictWriter(csv_file, fieldnames=list(row))
                            writer.writeheader()
                        writer.writerow(row)
                        predictions_file.write(json.dumps({"video": row["video"], "frame_index": index,
                                                          "predictions": predictions, "ground_truth_xyxy": truth})+"\n")
                        rows.append(row)
                finally:
                    cap.release()
                all_rows.extend(rows)
                report = dict(video=item["path"].name, ball_color=item["task"]["ball_color"], movement=item["task"]["movement"], **summary(rows))
                per_video.append(report)
                csv_file.flush()
                predictions_file.flush()
                print(f"{len(per_video)}/{len(videos)} {item['task']['id']}: {len(rows)} frames; precision={report['precision_percent']}, recall={report['recall_percent']}", flush=True)
        report = {"overall": summary(all_rows), "by_color": {c: summary([r for r in all_rows if r["ball_color"] == c]) for c in ("orange", "white", "none")}, "videos": per_video,
                  "wall_time_s_including_serialization": time.perf_counter()-wall_start}
        write_json(output/"summary.json", report)
        with (output/"summary.csv").open("w", newline="") as file:
            writer = csv.DictWriter(file, fieldnames=list(per_video[0]))
            writer.writeheader()
            writer.writerows(per_video)
        metadata["status"] = "completed"
        print(f"Results: {output}", flush=True)
        return 0
    except BaseException as error:
        metadata.update(status="failed", error=str(error))
        raise
    finally:
        metadata["finished_at"] = datetime.now().astimezone().isoformat()
        write_json(output/"metadata.json", metadata)
        if nvml is not None:
            nvml.nvmlShutdown()


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (ValueError, RuntimeError, OSError) as error:
        print(f"Error: {error}", file=sys.stderr)
        raise SystemExit(1)
