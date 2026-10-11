#!/usr/bin/env python3
"""Prepare validated YOLO data with whole-video train/val/test splits."""

import argparse
from collections import Counter, defaultdict
import csv
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import random
import re
import shutil

from PIL import Image

ROOT = Path(__file__).resolve().parents[3]
DEFAULT_EXPORT = Path('/home/valentin/ball-label-review/workspace/export_20261010_190357_670797')
VIDEO_PATTERN = re.compile(r'^\d{8}_\d{6}_\d{6}_(.+)_(white|orange|none)\.avi$')


def video_info(name):
    match = VIDEO_PATTERN.fullmatch(name)
    if not match:
        raise ValueError(f'Unknown video naming scheme: {name}')
    scene, color = match.groups()
    parts = scene.split('-')
    if len(parts) < 2:
        raise ValueError(f'Missing environment/light: {name}')
    return dict(environment='-'.join(parts[:2]), color=color,
                scenario='-'.join(parts[2:]) or 'background')


def validate_label(text):
    lines = [line for line in text.splitlines() if line.strip()]
    for line in lines:
        values = list(map(float, line.split()))
        if len(values) != 5 or not all(math.isfinite(v) for v in values):
            raise ValueError('Invalid YOLO label fields')
        cls, x, y, w, h = values
        if cls != 0 or not (0 <= x <= 1 and 0 <= y <= 1 and 0 < w <= 1 and 0 < h <= 1):
            raise ValueError('Invalid YOLO class or coordinates')
        if min(x-w/2, y-h/2) < -1e-7 or max(x+w/2, y+h/2) > 1+1e-7:
            raise ValueError('YOLO box extends beyond image')
    return len(lines)


def plan_splits(videos, seed):
    """One positive video per environment in val and test; alternate colors.

    All negative videos stay in train. With only two negative recordings, a
    three-way negative split would either leak frames or leave a split empty.
    """
    rng = random.Random(seed)
    buckets = defaultdict(lambda: defaultdict(list))
    assignment = {}
    for video in sorted(videos):
        info = video_info(video)
        if info['color'] == 'none':
            assignment[video] = 'train'
        else:
            buckets[info['environment']][info['color']].append(video)
    if len(buckets) < 2:
        raise ValueError('Need at least two environments for this split policy')
    for index, environment in enumerate(sorted(buckets)):
        colors = buckets[environment]
        if any(len(colors[color]) < 2 for color in ('white', 'orange')):
            raise ValueError(f'Need >=2 videos of each color in {environment}')
        val_color = 'white' if index % 2 == 0 else 'orange'
        test_color = 'orange' if val_color == 'white' else 'white'
        val_video = rng.choice(sorted(colors[val_color]))
        test_video = rng.choice(sorted(colors[test_color]))
        for color in ('white', 'orange'):
            for video in colors[color]:
                assignment[video] = ('val' if video == val_video else
                                     'test' if video == test_video else 'train')
    return assignment


def prepare(source, destination, seed=42):
    source = source.resolve(strict=True)
    destination = destination.resolve()
    if destination.exists():
        raise FileExistsError(f'Refusing to overwrite: {destination}')
    if (source / 'classes.txt').read_text().splitlines() != ['ping_pong_ball']:
        raise ValueError('Expected one class: ping_pong_ball')
    manifest = source / 'manifest.csv'
    with manifest.open(newline='', encoding='utf-8') as handle:
        rows = list(csv.DictReader(handle))
    if not rows or any(r.get('status') != 'validated' for r in rows):
        raise ValueError('Only explicitly validated exports are accepted')
    names = [r['image'] for r in rows]
    if len(set(names)) != len(names) or len({Path(n).stem for n in names}) != len(names):
        raise ValueError('Duplicate image names/stems')
    if any(Path(n).name != n for n in names):
        raise ValueError('Manifest image entries must be basenames')
    videos = {r['source_video'] for r in rows}
    assignment = plan_splits(videos, seed)
    seen_hashes = {}
    enriched = []
    for index, row in enumerate(rows, 1):
        image = source / 'images' / row['image']
        label = source / 'labels' / (Path(row['image']).stem + '.txt')
        boxes = validate_label(label.read_text(encoding='utf-8'))
        info = video_info(row['source_video'])
        if info['color'] == 'none' and boxes:
            raise ValueError(f'Background video still has a box: {row["image"]}')
        with Image.open(image) as frame:
            frame.verify()
        split = assignment[row['source_video']]
        with image.open('rb') as handle:
            digest = hashlib.file_digest(handle, 'sha256').hexdigest()
        if digest in seen_hashes and seen_hashes[digest][0] != split:
            raise ValueError(f'Exact duplicate across splits: {row["image"]} / {seen_hashes[digest][1]}')
        seen_hashes[digest] = (split, row['image'])
        enriched.append(dict(row, **info, split=split, boxes=boxes, sha256=digest))
        if index % 500 == 0 or index == len(rows):
            print(f'Checked {index}/{len(rows)} images and labels', flush=True)
    for split in ('train', 'val', 'test'):
        for kind in ('images', 'labels'):
            (destination / kind / split).mkdir(parents=True, exist_ok=False)
    summary = {}
    for split in ('train', 'val', 'test'):
        selected = [r for r in enriched if r['split'] == split]
        summary[split] = dict(images=len(selected), videos=len({r['source_video'] for r in selected}),
                              boxes=sum(r['boxes'] for r in selected),
                              empty_labels=sum(r['boxes'] == 0 for r in selected),
                              colors=dict(Counter(r['color'] for r in selected)),
                              environments=dict(Counter(r['environment'] for r in selected)))
    with (destination / 'manifest.csv').open('w', newline='', encoding='utf-8') as handle:
        writer = csv.DictWriter(handle, fieldnames=list(enriched[0]))
        writer.writeheader()
        writer.writerows(sorted(enriched, key=lambda r: (r['split'], r['source_video'], r['image'])))
    for row in enriched:
        split, name = row['split'], row['image']
        shutil.copy2(source / 'images' / name, destination / 'images' / split / name)
        label_name = Path(name).stem + '.txt'
        shutil.copy2(source / 'labels' / label_name, destination / 'labels' / split / label_name)
    # JSON quoting is valid YAML quoting, including paths with spaces.
    (destination / 'data.yaml').write_text(
        f'path: {json.dumps(str(destination))}\ntrain: images/train\nval: images/val\n'
        'test: images/test\nnames:\n  0: ping_pong_ball\n', encoding='utf-8')
    report = dict(source_export=str(source), source_manifest_sha256=hashlib.sha256(manifest.read_bytes()).hexdigest(),
                  created_at=datetime.now(timezone.utc).isoformat(), seed=seed,
                  policy='whole_video_one_per_environment_for_val_and_test_alternating_colors',
                  splits=summary, video_assignment=assignment,
                  checks=dict(all_labels_validated=True, normalized_boxes_valid=True,
                              source_images_readable=True, whole_video_disjoint=True,
                              no_exact_image_duplicates_across_splits=True),
                  limitations=['No background-only videos in val or test; add independent negative recordings.',
                               'Splits share recording sessions and environments; not an unseen-session benchmark.',
                               'Consecutive frames remain correlated within each split.'])
    (destination / 'split_report.json').write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
    (destination / 'README.md').write_text(
        '# Dataset ping-pong valide\n\n'
        'Une classe : `0 = ping_pong_ball`, blanche et orange.\n'
        'Images et annotations copiees depuis un export valide. Les originaux sont conserves.\n'
        'Chaque video appartient a un seul lot. Graine : '+str(seed)+'.\n\n'
        '| Lot | Videos | Images | Boites | Sans balle |\n|---|---:|---:|---:|---:|\n'+
        ''.join(f'| {s} | {v["videos"]} | {v["images"]} | {v["boxes"]} | {v["empty_labels"]} |\n' for s,v in summary.items())+
        '\nLes deux videos sans balle sont dans train. Ajouter de nouvelles videos sans balle pour val/test.\n'
        'Les lots partagent les memes sessions : filmer une nouvelle session pour un test plus independant.\n'
        'Ne pas utiliser test pour choisir les reglages du modele. Aucun entrainement lance ici.\n', encoding='utf-8')
    print(json.dumps(summary, indent=2), flush=True)
    print(f'Dataset: {destination / "data.yaml"}', flush=True)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-export', type=Path, default=DEFAULT_EXPORT)
    parser.add_argument('--output-dir', type=Path,
                        default=ROOT / 'datasets/ping_pong_ball/prepared/v1_20261010')
    parser.add_argument('--seed', type=int, default=42)
    args = parser.parse_args()
    prepare(args.source_export, args.output_dir, args.seed)


if __name__ == '__main__':
    main()
