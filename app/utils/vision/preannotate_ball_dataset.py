#!/usr/bin/env python3
"""Generate reviewable YOLO detection proposals, not approved training labels."""

import argparse
from collections import Counter
import csv
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import time

import cv2
import numpy as np
import torch
from ultralytics import YOLOE

ROOT = Path(__file__).resolve().parents[3]


def normalized_box(box, width, height):
    """Clip a pixel xyxy box and convert to YOLO class-0 normalized xywh."""
    x1, y1, x2, y2 = map(float, box)
    if not np.isfinite([x1, y1, x2, y2]).all():
        raise ValueError('Non-finite box')
    x1, x2 = np.clip([x1, x2], 0, width)
    y1, y2 = np.clip([y1, y2], 0, height)
    if x2 <= x1 or y2 <= y1:
        raise ValueError('Empty box')
    return [(x1 + x2) / (2 * width), (y1 + y2) / (2 * height),
            (x2 - x1) / width, (y2 - y1) / height]


def review_reason(confidences, expected_negative, threshold):
    if expected_negative:
        return 'possible_false_positive' if confidences else 'negative_to_verify'
    if not confidences:
        return 'missing_detection'
    reasons = []
    if len(confidences) > 1:
        reasons.append('multiple_detections')
    if min(confidences) < threshold:
        reasons.append('low_confidence')
    return '+'.join(reasons) or 'box_to_verify'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input-dir', type=Path, default=ROOT / 'datasets/ping_pong_ball/frames')
    parser.add_argument('--output-dir', type=Path)
    parser.add_argument('--model', type=Path, default=ROOT / 'benchmarks/yolo/models/yoloe-26m-seg.pt')
    parser.add_argument('--prompt', default='ping pong ball')
    parser.add_argument('--confidence', type=float, default=0.25)
    parser.add_argument('--review-confidence', type=float, default=0.50)
    parser.add_argument('--imgsz', type=int, default=1280)
    parser.add_argument('--device', default='0')
    args = parser.parse_args()
    if not 0 < args.confidence <= args.review_confidence <= 1:
        parser.error('Require 0 < confidence <= review-confidence <= 1')
    if args.imgsz <= 0:
        parser.error('imgsz must be positive')
    input_dir = args.input_dir.resolve()
    images = sorted(p for p in input_dir.iterdir() if p.suffix.lower() in {'.jpg', '.jpeg', '.png'})
    if not images:
        parser.error('No input images')
    if len({p.stem for p in images}) != len(images):
        parser.error('Duplicate image stems would overwrite labels')
    model_path = args.model.resolve(strict=True)
    if args.device != 'cpu' and not torch.cuda.is_available():
        parser.error('CUDA unavailable; explicitly choose --device cpu if wanted')
    output = args.output_dir or (ROOT / 'datasets/ping_pong_ball/annotations' /
                                datetime.now().strftime('yoloe26m_%Y%m%d_%H%M%S_%f'))
    output = output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    (output / 'labels').mkdir()
    (output / 'previews').mkdir()
    (output / 'classes.txt').write_text('ping_pong_ball\n', encoding='utf-8')
    config = dict(model=str(model_path), prompt=args.prompt, confidence=args.confidence,
                  review_confidence=args.review_confidence, imgsz=args.imgsz, device=args.device,
                  input_dir=str(input_dir), output_dir=str(output), image_count=len(images),
                  created_at=datetime.now(timezone.utc).isoformat(),
                  status='in_progress', labels_are_human_validated=False)
    summary_path = output / 'summary.json'
    summary_path.write_text(json.dumps(config, indent=2) + '\n', encoding='utf-8')
    print(f'Output: {output}', flush=True)
    # Ultralytics downloads/reads its text encoder relative to the working dir.
    # Keep these model assets in the existing ignored model directory.
    previous_directory = Path.cwd()
    try:
        os.chdir(model_path.parent)
        model = YOLOE(model_path.name)
        model.set_classes([args.prompt])
    finally:
        os.chdir(previous_directory)
    manifest_path = input_dir / 'manifest.csv'
    sources = {}
    if manifest_path.exists():
        with manifest_path.open(newline='', encoding='utf-8') as handle:
            sources = {row['image']: row for row in csv.DictReader(handle)}
    counts, scenes = Counter(), {}
    started = time.perf_counter()
    with (output / 'review.csv').open('w', newline='', encoding='utf-8') as handle:
        writer = csv.DictWriter(handle, fieldnames=[
            'image', 'source_video', 'label', 'preview', 'expected_negative',
            'detections', 'confidences', 'review_reason', 'review_status'])
        writer.writeheader()
        for index, path in enumerate(images, 1):
            image = cv2.imread(str(path))
            if image is None:
                raise RuntimeError(f'Unreadable image: {path}')
            result = model.predict(image, conf=args.confidence, imgsz=args.imgsz,
                                   device=args.device, verbose=False)[0]
            height, width = image.shape[:2]
            boxes, confidences = [], []
            if result.boxes is not None:
                for xyxy, confidence in zip(result.boxes.xyxy.cpu().numpy(),
                                            result.boxes.conf.cpu().numpy()):
                    boxes.append(normalized_box(xyxy, width, height))
                    confidences.append(float(confidence))
            label = output / 'labels' / f'{path.stem}.txt'
            label.write_text(''.join('0 ' + ' '.join(f'{v:.8f}' for v in box) + '\n'
                                     for box in boxes), encoding='utf-8')
            entry = sources.get(path.name, {})
            source = entry.get('source_video', '')
            negative_flag = entry.get('expected_negative', '').lower()
            expected_negative = (negative_flag == 'true' if negative_flag in ('true', 'false')
                                 else Path(source or path.name).stem.endswith('_none') or '_none_f' in path.name)
            reason = review_reason(confidences, expected_negative, args.review_confidence)
            preview = result.plot()
            preview = cv2.resize(preview, (960, round(height * 960 / width)))
            cv2.putText(preview, f'{index}/{len(images)} | {reason}', (12, 24),
                        cv2.FONT_HERSHEY_SIMPLEX, .55, (0, 0, 255), 2, cv2.LINE_AA)
            preview_path = output / 'previews' / f'{path.stem}.jpg'
            if not cv2.imwrite(str(preview_path), preview, [cv2.IMWRITE_JPEG_QUALITY, 80]):
                raise RuntimeError(f'Cannot save preview: {preview_path}')
            writer.writerow(dict(image=str(path), source_video=source,
                                 label=str(label.relative_to(output)),
                                 preview=str(preview_path.relative_to(output)),
                                 expected_negative=expected_negative, detections=len(boxes),
                                 confidences=json.dumps(confidences), review_reason=reason,
                                 review_status='pending'))
            counts[reason] += 1
            counts['total_boxes'] += len(boxes)
            scene = scenes.setdefault(source, Counter())
            scene['images'] += 1
            scene['images_with_detection'] += bool(boxes)
            scene['boxes'] += len(boxes)
            if index % 100 == 0 or index == len(images):
                handle.flush()
                print(f'{index}/{len(images)} | {time.perf_counter()-started:.1f}s | '
                      f'boxes={counts["total_boxes"]}', flush=True)
    config.update(status='complete_pending_review', elapsed_s=time.perf_counter()-started,
                  counts=dict(counts), by_video={k: dict(v) for k, v in scenes.items()})
    summary_path.write_text(json.dumps(config, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(config['counts'], indent=2), flush=True)
    print('All labels are proposals. Empty files are NOT automatically confirmed negatives.', flush=True)


if __name__ == '__main__':
    main()
