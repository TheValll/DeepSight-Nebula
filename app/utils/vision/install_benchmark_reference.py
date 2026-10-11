#!/usr/bin/env python3
"""Install ONLY fully human-validated benchmark labels from the local review tool."""

import argparse
import csv
import hashlib
import importlib.util
import json
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
SPEC = importlib.util.spec_from_file_location("video_benchmark", ROOT/"benchmarks/yolo/benchmark_yolo.py")
BENCH = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(BENCH)


def digest(path):
    return BENCH.sha256(path)


def build_plan(session, export):
    state = json.loads((session/"session.json").read_text())
    if set(state["accepted"]) != {t["id"] for t in state["tasks"]}:
        raise ValueError("Capture session is incomplete")
    task_by_video = {state["accepted"][t["id"]]: t for t in state["tasks"]}
    if (export/"classes.txt").read_text().splitlines() != ["ping_pong_ball"]:
        raise ValueError("Unexpected exported classes")
    with (session/"review_input/manifest.csv").open() as file:
        expected = list(csv.DictReader(file))
    with (export/"manifest.csv").open() as file:
        reviewed = list(csv.DictReader(file))
    names = [row["image"] for row in reviewed]
    if len(names) != len(set(names)) or set(names) != {row["image"] for row in expected}:
        raise ValueError(f"Export must contain exactly all {len(expected)} reference images; currently {len(names)}. Validate/export all images first.")
    by_name = {row["image"]: row for row in reviewed}
    plan = []
    for reference in expected:
        name = reference["image"]
        row = by_name[name]
        if Path(name).name != name or row["status"] != "validated" or row["source_video"] != reference["source_video"]:
            raise ValueError(f"Unvalidated/mismatched reference: {name}")
        if reference["source_video"] not in task_by_video:
            raise ValueError(f"Video not accepted in this session: {name}")
        original = session/"review_input"/name
        if digest(export/"images"/name) != digest(original):
            raise ValueError(f"Exported image changed: {name}")
        label = export/"labels"/(Path(name).stem+".txt")
        boxes = BENCH.read_labels(label)
        if task_by_video[reference["source_video"]]["expected_objects"] == 0 and boxes:
            raise ValueError(f"Ball labelled in a no-ball capture: {name}. Fix the annotation or recording.")
        destination = Path(reference["reference_label"]).resolve()
        expected_destination = session/"annotations"/Path(reference["source_video"]).stem/f"{int(reference['frame_index']):06d}.txt"
        if destination != expected_destination:
            raise ValueError(f"Unexpected target label path: {destination}")
        if destination.exists() and destination.read_bytes() != label.read_bytes():
            raise ValueError(f"Existing reference differs; refusing to overwrite: {destination}")
        plan.append((label, destination))
    return plan


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session", type=Path, required=True)
    parser.add_argument("--review-export", type=Path, required=True)
    parser.add_argument("--check", action="store_true", help="Validate without installing labels")
    args = parser.parse_args()
    session = args.session.resolve()
    export = args.review_export.resolve()
    plan = build_plan(session, export)  # All checks happen before any reference is written.
    if not args.check:
        for source, destination in plan:
            destination.parent.mkdir(parents=True, exist_ok=True)
            if not destination.exists():
                destination.write_bytes(source.read_bytes())
        receipt = {"installed_at": datetime.now().astimezone().isoformat(), "human_validated": True,
                   "review_export": str(export), "export_manifest_sha256": digest(export/"manifest.csv"),
                   "reference_count": len(plan), "labels_sha256": {str(dst.relative_to(session)): digest(dst) for _, dst in plan}}
        (session/"annotations/review_receipt.json").write_text(json.dumps(receipt, indent=2)+"\n")
    print(f"{'Checked' if args.check else 'Installed'} {len(plan)} human-validated reference labels. No benchmark launched.")


if __name__ == "__main__":
    main()
