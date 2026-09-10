#!/usr/bin/env python3
"""Benchmark an Ultralytics YOLO model using the left stereo-camera image."""

import argparse
import csv
import json
import os
import platform
import re
import statistics
import sys
import time
from datetime import datetime


def import_dependencies():
    """Import optional dependencies and print useful installation errors."""
    packages = [
        ("cv2", "opencv-python"),
        ("numpy", "numpy"),
        ("psutil", "psutil"),
        ("torch", "torch"),
        ("ultralytics", "ultralytics"),
    ]
    imported = {}

    for module_name, package_name in packages:
        try:
            imported[module_name] = __import__(module_name)
        except ImportError:
            print(
                f"Missing dependency: {package_name}. "
                f"Install it with: pip install {package_name}",
                file=sys.stderr,
            )
            sys.exit(1)

    return imported


deps = import_dependencies()
cv2 = deps["cv2"]
np = deps["numpy"]
psutil = deps["psutil"]
torch = deps["torch"]
ultralytics = deps["ultralytics"]
YOLO = ultralytics.YOLO


RAW_COLUMNS = [
    "model",
    "run",
    "movement",
    "frame_id",
    "date",
    "time",
    "datetime_iso",
    "timestamp_s",
    "detected",
    "confidence",
    "inference_ms",
    "total_processing_ms",
    "cpu_percent",
    "ram_mb",
    "gpu_percent",
    "vram_mb",
]

SUMMARY_COLUMNS = [
    "model",
    "run",
    "movement",
    "date",
    "start_time",
    "duration_s",
    "processed_frames",
    "detected_frames",
    "missed_frames",
    "fps",
    "detection_rate_percent",
    "mean_confidence",
    "inference_mean_ms",
    "inference_median_ms",
    "inference_p95_ms",
    "inference_p99_ms",
    "total_processing_mean_ms",
    "cpu_mean_percent",
    "ram_mean_mb",
    "gpu_mean_percent",
    "vram_mean_mb",
]

MOVEMENTS = [
    "static",
    "horizontal_slow",
    "horizontal_fast",
    "vertical",
    "free_depth_motion",
]


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--model",
        required=True,
        help="Ultralytics model name or path to a custom .pt model",
    )
    parser.add_argument(
        "--benchmark-name",
        help=(
            "Name stored in the CSV files and used for the metadata filename "
            "(defaults to the model filename)"
        ),
    )
    parser.add_argument(
        "--class-id",
        type=int,
        default=32,
        help="Model class ID to detect (default: 32, COCO sports ball)",
    )
    parser.add_argument("--camera", type=int, default=0, help="OpenCV camera index")
    parser.add_argument(
        "--confidence", type=float, default=0.15, help="Detection threshold"
    )
    parser.add_argument(
        "--imgsz", type=int, default=640, help="YOLO inference image size"
    )
    return parser.parse_args()


def open_csv(path, columns):
    """Open an append-only CSV file and write its header only when needed."""
    needs_header = not os.path.exists(path) or os.path.getsize(path) == 0
    file_handle = open(path, "a", newline="", encoding="utf-8")
    writer = csv.DictWriter(file_handle, fieldnames=columns)
    if needs_header:
        writer.writeheader()
        file_handle.flush()
    return file_handle, writer


def init_gpu_monitoring(device_index):
    try:
        import pynvml

        pynvml.nvmlInit()
        handle = pynvml.nvmlDeviceGetHandleByIndex(device_index)
        name = pynvml.nvmlDeviceGetName(handle)
        if isinstance(name, bytes):
            name = name.decode("utf-8")
        return pynvml, handle, name
    except Exception as error:
        print(f"NVIDIA monitoring unavailable ({error}). GPU metrics will be NaN.")
        return None, None, None


def get_gpu_metrics(pynvml, handle):
    if pynvml is None or handle is None:
        return float("nan"), float("nan")
    try:
        utilization = pynvml.nvmlDeviceGetUtilizationRates(handle).gpu
        memory_mb = pynvml.nvmlDeviceGetMemoryInfo(handle).used / (1024 * 1024)
        return float(utilization), float(memory_mb)
    except Exception:
        return float("nan"), float("nan")


def put_lines(image, lines, color=(255, 255, 255)):
    y = 35
    for line in lines:
        cv2.putText(
            image,
            line,
            (20, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            (0, 0, 0),
            4,
            cv2.LINE_AA,
        )
        cv2.putText(
            image,
            line,
            (20, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            color,
            2,
            cv2.LINE_AA,
        )
        y += 32


def read_left_frame(camera):
    ok, stereo_frame = camera.read()
    if not ok or stereo_frame is None:
        return None
    height, width = stereo_frame.shape[:2]
    if width != 2560 or height != 720:
        return None
    return stereo_frame[:, :1280]


def predict(model, image, device, confidence, image_size, class_id, use_cuda):
    if use_cuda:
        torch.cuda.synchronize()
    start = time.perf_counter()
    results = model.predict(
        image,
        classes=[class_id],
        conf=confidence,
        imgsz=image_size,
        device=device,
        verbose=False,
    )
    if use_cuda:
        torch.cuda.synchronize()
    inference_ms = (time.perf_counter() - start) * 1000.0
    return results[0], inference_ms


def show_countdown(camera, model_name, run_number, movement):
    countdown_start = time.perf_counter()
    while True:
        left = read_left_frame(camera)
        if left is None:
            print("Failed to read a camera frame during the countdown.")
            return False

        remaining = 3.0 - (time.perf_counter() - countdown_start)
        if remaining <= 0:
            return True

        preview = left.copy()
        put_lines(
            preview,
            [
                f"Model: {model_name}",
                f"Run {run_number}/5: {movement}",
                f"Starting in: {max(1, int(np.ceil(remaining)))}",
                "Press ESC to quit",
            ],
            color=(0, 255, 255),
        )
        cv2.imshow("YOLO11 Stereo Benchmark - Left Camera", preview)
        if cv2.waitKey(1) & 0xFF == 27:
            return False


def mean_or_nan(values):
    valid = [value for value in values if not np.isnan(value)]
    return statistics.fmean(valid) if valid else float("nan")


def build_summary(rows, model_name, run_number, movement, started_at, duration):
    processed = len(rows)
    detected = sum(row["detected"] for row in rows)
    confidences = [row["confidence"] for row in rows]
    inference = [row["inference_ms"] for row in rows]

    if inference:
        median = statistics.median(inference)
        p95 = float(np.percentile(inference, 95))
        p99 = float(np.percentile(inference, 99))
    else:
        median = p95 = p99 = float("nan")

    return {
        "model": model_name,
        "run": run_number,
        "movement": movement,
        "date": started_at.strftime("%Y-%m-%d"),
        "start_time": started_at.strftime("%H:%M:%S.%f")[:-3],
        "duration_s": duration,
        "processed_frames": processed,
        "detected_frames": detected,
        "missed_frames": processed - detected,
        "fps": processed / duration if duration else 0.0,
        "detection_rate_percent": detected / processed * 100 if processed else 0.0,
        "mean_confidence": mean_or_nan(confidences),
        "inference_mean_ms": mean_or_nan(inference),
        "inference_median_ms": median,
        "inference_p95_ms": p95,
        "inference_p99_ms": p99,
        "total_processing_mean_ms": mean_or_nan(
            [row["total_processing_ms"] for row in rows]
        ),
        "cpu_mean_percent": mean_or_nan([row["cpu_percent"] for row in rows]),
        "ram_mean_mb": mean_or_nan([row["ram_mb"] for row in rows]),
        "gpu_mean_percent": mean_or_nan([row["gpu_percent"] for row in rows]),
        "vram_mean_mb": mean_or_nan([row["vram_mb"] for row in rows]),
    }


def print_summary(summary):
    print(f"\nRun {summary['run']}/5 complete: {summary['movement']}")
    print(
        f"Frames: {summary['processed_frames']} processed, "
        f"{summary['detected_frames']} detected, {summary['missed_frames']} missed"
    )
    print(
        f"FPS: {summary['fps']:.2f} | "
        f"Detection rate: {summary['detection_rate_percent']:.2f}% | "
        f"Mean confidence: {summary['mean_confidence']:.3f}"
    )
    print(
        f"Inference ms: mean {summary['inference_mean_ms']:.2f}, "
        f"median {summary['inference_median_ms']:.2f}, "
        f"p95 {summary['inference_p95_ms']:.2f}, "
        f"p99 {summary['inference_p99_ms']:.2f}"
    )
    print(f"Mean total processing: {summary['total_processing_mean_ms']:.2f} ms")
    print(
        f"Mean CPU: {summary['cpu_mean_percent']:.1f}% | "
        f"RAM: {summary['ram_mean_mb']:.1f} MB | "
        f"GPU: {summary['gpu_mean_percent']:.1f}% | "
        f"VRAM: {summary['vram_mean_mb']:.1f} MB\n"
    )


def write_metadata(path, args, benchmark_name, class_name, device, gpu_name):
    # A 1280x720 frame becomes 640x384 with Ultralytics' default rectangular
    # letterboxing at imgsz=640. This is also how the ROS 2 YOLO node runs.
    inference_width = int(np.ceil(args.imgsz / 32) * 32)
    inference_height = int(
        np.ceil((720 / 1280) * inference_width / 32) * 32
    )
    metadata = {
        "benchmark_name": benchmark_name,
        "model_source": args.model,
        "camera_resolution": [2560, 720],
        "target_camera_fps": 60,
        "camera_format": "MJPG",
        "inference_image_resolution": [inference_width, inference_height],
        "confidence_threshold": args.confidence,
        "target_class_id": args.class_id,
        "target_class_name": class_name,
        "selected_device": device,
        "gpu_name": gpu_name,
        "operating_system": platform.platform(),
        "python_version": platform.python_version(),
        "ultralytics_version": ultralytics.__version__,
        "pytorch_version": torch.__version__,
    }
    with open(path, "w", encoding="utf-8") as file_handle:
        json.dump(metadata, file_handle, indent=2)
        file_handle.write("\n")


def main():
    args = parse_args()
    script_dir = os.path.dirname(os.path.abspath(__file__))
    benchmark_name = args.benchmark_name or os.path.basename(args.model)
    metadata_slug = re.sub(r"[^A-Za-z0-9_.-]+", "_", benchmark_name).strip("._")
    if not metadata_slug:
        print(
            "The benchmark name must contain at least one usable character.",
            file=sys.stderr,
        )
        return 1
    raw_path = os.path.join(script_dir, "benchmark_raw.csv")
    summary_path = os.path.join(script_dir, "benchmark_summary.csv")
    metadata_path = os.path.join(
        script_dir, f"benchmark_metadata_{metadata_slug}.json"
    )

    use_cuda = torch.cuda.is_available()
    device = "cuda:0" if use_cuda else "cpu"
    pynvml, gpu_handle, gpu_name = init_gpu_monitoring(0) if use_cuda else (None, None, None)
    print(f"Loading {args.model} on {device}...")
    try:
        model = YOLO(args.model)
    except Exception as error:
        print(f"Could not load model {args.model}: {error}", file=sys.stderr)
        return 1

    class_names = model.names
    if isinstance(class_names, dict):
        class_name = class_names.get(args.class_id)
    elif 0 <= args.class_id < len(class_names):
        class_name = class_names[args.class_id]
    else:
        class_name = None
    if class_name is None:
        print(
            f"Class ID {args.class_id} is not defined by this model. "
            f"Available classes: {class_names}",
            file=sys.stderr,
        )
        return 1

    print(f"Benchmark name: {benchmark_name}")
    print(f"Target class: {args.class_id} ({class_name})")
    write_metadata(
        metadata_path, args, benchmark_name, class_name, device, gpu_name
    )

    camera = cv2.VideoCapture(args.camera, cv2.CAP_V4L2)
    camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, 2560)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    camera.set(cv2.CAP_PROP_FPS, 60)
    camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    if not camera.isOpened():
        print(f"Could not open camera index {args.camera}.", file=sys.stderr)
        camera.release()
        return 1

    raw_file = summary_file = None
    try:
        actual_width = int(round(camera.get(cv2.CAP_PROP_FRAME_WIDTH)))
        actual_height = int(round(camera.get(cv2.CAP_PROP_FRAME_HEIGHT)))
        actual_fps = camera.get(cv2.CAP_PROP_FPS)
        fourcc_value = int(camera.get(cv2.CAP_PROP_FOURCC))
        actual_fourcc = "".join(
            chr((fourcc_value >> (8 * index)) & 0xFF) for index in range(4)
        )
        print(
            f"Camera stream: {actual_width}x{actual_height} at "
            f"{actual_fps:.1f} FPS ({actual_fourcc})"
        )

        if actual_width != 2560 or actual_height != 720:
            print(
                f"Unexpected camera resolution: {actual_width}x{actual_height}; "
                "expected 2560x720.",
                file=sys.stderr,
            )
            return 1
        if abs(actual_fps - 60.0) > 1.0:
            print(
                f"Unexpected camera FPS: {actual_fps:.1f}; expected 60 FPS. "
                "The benchmark was not started.",
                file=sys.stderr,
            )
            return 1

        first_frame = read_left_frame(camera)
        if first_frame is None:
            print(
                "Could not read a 2560x720 stereo frame from the camera. "
                "Check the camera index and supported capture mode.",
                file=sys.stderr,
            )
            return 1

        print("Performing 20 warm-up inferences...")
        for warmup_number in range(20):
            frame = first_frame if warmup_number == 0 else read_left_frame(camera)
            if frame is None:
                print("Camera read failed during warm-up.", file=sys.stderr)
                return 1
            predict(
                model,
                frame,
                device,
                args.confidence,
                args.imgsz,
                args.class_id,
                use_cuda,
            )

        actual_yolo_device = str(model.predictor.device)
        print(f"YOLO inference device: {actual_yolo_device}")
        if use_cuda and not actual_yolo_device.startswith("cuda"):
            print("YOLO did not initialize on CUDA as expected.", file=sys.stderr)
            return 1

        raw_file, raw_writer = open_csv(raw_path, RAW_COLUMNS)
        summary_file, summary_writer = open_csv(summary_path, SUMMARY_COLUMNS)
        psutil.cpu_percent(interval=None)
        run_index = 0

        while True:
            left = read_left_frame(camera)
            if left is None:
                print("Camera read failed.", file=sys.stderr)
                break

            preview = left.copy()
            if run_index < len(MOVEMENTS):
                put_lines(
                    preview,
                    [
                        f"Model: {benchmark_name}",
                        f"Run {run_index + 1}/5: {MOVEMENTS[run_index]}",
                        "Press SPACE to start",
                        "Press ESC to quit",
                    ],
                )
            else:
                put_lines(
                    preview,
                    [
                        f"Model: {benchmark_name}",
                        "All 5 runs complete",
                        "Press ESC to quit",
                    ],
                    color=(0, 255, 0),
                )

            cv2.imshow("YOLO11 Stereo Benchmark - Left Camera", preview)
            key = cv2.waitKey(1) & 0xFF
            if key == 27:
                break
            if key != 32 or run_index >= len(MOVEMENTS):
                continue

            run_number = run_index + 1
            movement = MOVEMENTS[run_index]
            if not show_countdown(camera, benchmark_name, run_number, movement):
                break

            rows = []
            run_started_at = datetime.now()
            run_start = time.perf_counter()
            frame_id = 0
            quit_requested = False

            while time.perf_counter() - run_start < 10.0:
                processing_start = time.perf_counter()
                left = read_left_frame(camera)
                if left is None:
                    print("Camera read failed during benchmark.", file=sys.stderr)
                    quit_requested = True
                    break

                frame_timestamp = time.perf_counter() - run_start
                result, inference_ms = predict(
                    model,
                    left,
                    device,
                    args.confidence,
                    args.imgsz,
                    args.class_id,
                    use_cuda,
                )

                confidences = []
                if result.boxes is not None and result.boxes.conf is not None:
                    confidences = result.boxes.conf.detach().cpu().tolist()
                detected = int(bool(confidences))
                confidence = max(confidences, default=0.0)
                annotated = result.plot()

                cpu_percent = psutil.cpu_percent(interval=None)
                ram_mb = psutil.virtual_memory().used / (1024 * 1024)
                gpu_percent, vram_mb = get_gpu_metrics(pynvml, gpu_handle)
                total_processing_ms = (time.perf_counter() - processing_start) * 1000.0

                frame_id += 1
                now = datetime.now()
                row = {
                    "model": benchmark_name,
                    "run": run_number,
                    "movement": movement,
                    "frame_id": frame_id,
                    "date": now.strftime("%Y-%m-%d"),
                    "time": now.strftime("%H:%M:%S.%f")[:-3],
                    "datetime_iso": now.isoformat(timespec="milliseconds"),
                    "timestamp_s": frame_timestamp,
                    "detected": detected,
                    "confidence": confidence,
                    "inference_ms": inference_ms,
                    "total_processing_ms": total_processing_ms,
                    "cpu_percent": cpu_percent,
                    "ram_mb": ram_mb,
                    "gpu_percent": gpu_percent,
                    "vram_mb": vram_mb,
                }
                rows.append(row)
                raw_writer.writerow(row)
                if frame_id % 30 == 0:
                    raw_file.flush()

                detected_count = sum(item["detected"] for item in rows)
                put_lines(
                    annotated,
                    [
                        f"Model: {benchmark_name} | Run {run_number}/5 | {movement}",
                        f"Elapsed: {frame_timestamp:.2f} / 10.00 s",
                        f"Frames: {frame_id} | Detected frames: {detected_count}",
                        "Press ESC to quit",
                    ],
                    color=(0, 255, 0),
                )
                cv2.imshow("YOLO11 Stereo Benchmark - Left Camera", annotated)
                if cv2.waitKey(1) & 0xFF == 27:
                    quit_requested = True
                    break

            duration = time.perf_counter() - run_start
            raw_file.flush()

            if quit_requested:
                print("Benchmark interrupted; partial raw frame data was saved.")
                break

            summary = build_summary(
                rows, benchmark_name, run_number, movement, run_started_at, duration
            )
            summary_writer.writerow(summary)
            summary_file.flush()
            print_summary(summary)
            run_index += 1

    finally:
        if raw_file is not None:
            raw_file.close()
        if summary_file is not None:
            summary_file.close()
        camera.release()
        cv2.destroyAllWindows()
        if pynvml is not None:
            try:
                pynvml.nvmlShutdown()
            except Exception:
                pass

    return 0


if __name__ == "__main__":
    sys.exit(main())
