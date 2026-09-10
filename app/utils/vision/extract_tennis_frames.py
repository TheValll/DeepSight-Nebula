#!/usr/bin/env python3
"""Extract sampled training images from every video in a directory."""

import argparse
import csv
import re
import sys
from pathlib import Path

try:
    import cv2
except ImportError:
    print("Missing dependency: opencv-python", file=sys.stderr)
    raise SystemExit(1)


PROJECT_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_INPUT_DIR = PROJECT_ROOT / "datasets" / "tennis_ball" / "videos"
DEFAULT_OUTPUT_DIR = PROJECT_ROOT / "datasets" / "tennis_ball" / "frames"
VIDEO_EXTENSIONS = {".avi", ".m4v", ".mkv", ".mov", ".mp4", ".webm"}


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-dir", type=Path, default=DEFAULT_INPUT_DIR)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    sampling = parser.add_mutually_exclusive_group()
    sampling.add_argument(
        "--sample-fps",
        type=float,
        default=5.0,
        help="images extracted per second of video (default: 5)",
    )
    sampling.add_argument("--every-nth", type=int, help="extract every Nth source frame")
    sampling.add_argument("--all-frames", action="store_true", help="extract every frame")
    parser.add_argument("--jpeg-quality", type=int, default=95, choices=range(1, 101), metavar="1..100")
    parser.add_argument("--overwrite", action="store_true", help="replace existing image files")
    args = parser.parse_args()
    if args.sample_fps is not None and args.sample_fps <= 0:
        parser.error("--sample-fps must be positive")
    if args.every_nth is not None and args.every_nth <= 0:
        parser.error("--every-nth must be positive")
    return args


def safe_name(path, root):
    relative = path.relative_to(root).with_suffix("")
    return re.sub(r"[^a-zA-Z0-9_-]+", "_", "__".join(relative.parts)).strip("_")


def should_extract(frame_index, source_fps, args):
    if args.all_frames:
        return True
    if args.every_nth is not None:
        return frame_index % args.every_nth == 0
    # Timestamp buckets avoid drift when source_fps is not divisible by sample_fps.
    previous_bucket = int(((frame_index - 1) / source_fps) * args.sample_fps) if frame_index else -1
    current_bucket = int((frame_index / source_fps) * args.sample_fps)
    return current_bucket != previous_bucket


def main():
    args = parse_args()
    if not args.input_dir.is_dir():
        print(f"Input directory does not exist: {args.input_dir}", file=sys.stderr)
        return 1
    videos = sorted(
        path for path in args.input_dir.rglob("*") if path.is_file() and path.suffix.lower() in VIDEO_EXTENSIONS
    )
    if not videos:
        print(f"No videos found in {args.input_dir}", file=sys.stderr)
        return 1

    args.output_dir.mkdir(parents=True, exist_ok=True)
    rows = []
    total_created = 0
    total_skipped = 0
    for video in videos:
        capture = cv2.VideoCapture(str(video))
        source_fps = capture.get(cv2.CAP_PROP_FPS)
        if not capture.isOpened() or source_fps <= 0:
            print(f"Skipping unreadable video: {video}", file=sys.stderr)
            capture.release()
            continue
        prefix = safe_name(video, args.input_dir)
        source_frame = 0
        extracted = 0
        while True:
            ok, frame = capture.read()
            if not ok:
                break
            if should_extract(source_frame, source_fps, args):
                timestamp = source_frame / source_fps
                output_name = f"{prefix}_f{source_frame:07d}_t{int(round(timestamp * 1000)):010d}.jpg"
                output_path = args.output_dir / output_name
                if output_path.exists() and not args.overwrite:
                    total_skipped += 1
                else:
                    success = cv2.imwrite(
                        str(output_path),
                        frame,
                        [cv2.IMWRITE_JPEG_QUALITY, args.jpeg_quality],
                    )
                    if not success:
                        print(f"Could not write image: {output_path}", file=sys.stderr)
                        capture.release()
                        return 1
                    total_created += 1
                rows.append(
                    {
                        "image": output_name,
                        "source_video": str(video.relative_to(args.input_dir)),
                        "source_frame": source_frame,
                        "timestamp_s": f"{timestamp:.6f}",
                        "source_fps": f"{source_fps:.6f}",
                    }
                )
                extracted += 1
            source_frame += 1
        capture.release()
        print(f"{video.name}: {extracted} images selected from {source_frame} frames")

    manifest = args.output_dir / "manifest.csv"
    with manifest.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(
            handle,
            fieldnames=("image", "source_video", "source_frame", "timestamp_s", "source_fps"),
        )
        writer.writeheader()
        writer.writerows(rows)
    print(f"Created: {total_created} | already present: {total_skipped}")
    print(f"Manifest: {manifest}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
