#!/usr/bin/env python3
"""Record a clean tennis-ball dataset video from the stereo robot camera."""

import argparse
import json
import re
import sys
import time
from datetime import datetime
from pathlib import Path

try:
    import cv2
except ImportError:
    print("Missing dependency: opencv-python", file=sys.stderr)
    raise SystemExit(1)


PROJECT_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_OUTPUT_DIR = PROJECT_ROOT / "datasets" / "tennis_ball" / "videos"
WINDOW_NAME = "Tennis dataset recorder - robot left camera"


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--duration",
        type=float,
        default=10.0,
        help="recording duration in seconds (default: 10)",
    )
    parser.add_argument("--camera", type=int, default=0, help="OpenCV camera index")
    parser.add_argument("--countdown", type=float, default=3.0, help="countdown in seconds")
    parser.add_argument("--fps", type=float, default=60.0, help="requested camera FPS")
    parser.add_argument(
        "--scene",
        default="scene",
        help="short scene label included in the filename, e.g. kitchen_day",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=DEFAULT_OUTPUT_DIR,
        help=f"video destination (default: {DEFAULT_OUTPUT_DIR})",
    )
    parser.add_argument(
        "--codec",
        choices=("MJPG", "mp4v"),
        default="MJPG",
        help="MJPG preserves details better; mp4v uses less disk space",
    )
    args = parser.parse_args()
    if args.duration <= 0 or args.countdown < 0 or args.fps <= 0:
        parser.error("--duration and --fps must be positive; --countdown cannot be negative")
    return args


def safe_slug(value):
    slug = re.sub(r"[^a-zA-Z0-9_-]+", "-", value.strip()).strip("-").lower()
    return slug or "scene"


def overlay(frame, lines, color=(255, 255, 255)):
    preview = frame.copy()
    y = 36
    for line in lines:
        cv2.putText(preview, line, (20, y), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(preview, line, (20, y), cv2.FONT_HERSHEY_SIMPLEX, 0.75, color, 2, cv2.LINE_AA)
        y += 34
    return preview


def read_left(camera):
    ok, stereo = camera.read()
    if not ok or stereo is None:
        return None
    if stereo.shape[:2] != (720, 2560):
        return None
    return stereo[:, :1280]


def main():
    args = parse_args()
    camera = cv2.VideoCapture(args.camera, cv2.CAP_V4L2)
    camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, 2560)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    camera.set(cv2.CAP_PROP_FPS, args.fps)
    camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    if not camera.isOpened():
        print(f"Could not open camera index {args.camera}.", file=sys.stderr)
        return 1

    actual_width = int(round(camera.get(cv2.CAP_PROP_FRAME_WIDTH)))
    actual_height = int(round(camera.get(cv2.CAP_PROP_FRAME_HEIGHT)))
    actual_fps = camera.get(cv2.CAP_PROP_FPS)
    if (actual_width, actual_height) != (2560, 720):
        print(
            f"Unexpected camera stream {actual_width}x{actual_height}; expected 2560x720.",
            file=sys.stderr,
        )
        camera.release()
        return 1

    args.output_dir.mkdir(parents=True, exist_ok=True)
    extension = ".avi" if args.codec == "MJPG" else ".mp4"
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_path = args.output_dir / f"{timestamp}_{safe_slug(args.scene)}{extension}"
    metadata_path = output_path.with_suffix(".json")

    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    print("Press SPACE to start recording or ESC to quit.")
    try:
        while True:
            frame = read_left(camera)
            if frame is None:
                print("Could not read a valid stereo frame.", file=sys.stderr)
                return 1
            preview = overlay(
                frame,
                [
                    f"Scene: {safe_slug(args.scene)}",
                    f"Duration: {args.duration:g} s",
                    "SPACE: start | ESC: quit",
                ],
            )
            cv2.imshow(WINDOW_NAME, preview)
            key = cv2.waitKey(1) & 0xFF
            if key == 27:
                return 0
            if key == 32:
                break

        countdown_started = time.perf_counter()
        while True:
            frame = read_left(camera)
            if frame is None:
                print("Camera read failed during countdown.", file=sys.stderr)
                return 1
            remaining = args.countdown - (time.perf_counter() - countdown_started)
            if remaining <= 0:
                break
            preview = overlay(frame, [f"Recording starts in {max(1, int(remaining + 0.999))}"], (0, 255, 255))
            cv2.imshow(WINDOW_NAME, preview)
            if cv2.waitKey(1) & 0xFF == 27:
                return 0

        writer = cv2.VideoWriter(
            str(output_path),
            cv2.VideoWriter_fourcc(*args.codec),
            args.fps,
            (1280, 720),
        )
        if not writer.isOpened():
            print(f"Could not create video: {output_path}", file=sys.stderr)
            return 1

        started = time.perf_counter()
        frames_written = 0
        interrupted = False
        while True:
            elapsed = time.perf_counter() - started
            if elapsed >= args.duration:
                break
            frame = read_left(camera)
            if frame is None:
                continue
            writer.write(frame)
            frames_written += 1
            preview = overlay(
                frame,
                [f"REC {elapsed:05.1f}/{args.duration:.1f} s", "ESC: stop early"],
                (0, 0, 255),
            )
            cv2.imshow(WINDOW_NAME, preview)
            if cv2.waitKey(1) & 0xFF == 27:
                interrupted = True
                break
        recorded_duration = time.perf_counter() - started
        writer.release()

        metadata = {
            "video": output_path.name,
            "recorded_at": datetime.now().astimezone().isoformat(timespec="seconds"),
            "scene": safe_slug(args.scene),
            "camera_index": args.camera,
            "source_resolution": [actual_width, actual_height],
            "saved_resolution": [1280, 720],
            "camera_side": "left",
            "requested_fps": args.fps,
            "reported_fps": actual_fps,
            "frames": frames_written,
            "duration_s": recorded_duration,
            "codec": args.codec,
            "stopped_early": interrupted,
        }
        metadata_path.write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
        print(f"Saved {frames_written} frames to {output_path}")
        print(f"Metadata: {metadata_path}")
        return 0
    finally:
        camera.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    raise SystemExit(main())
