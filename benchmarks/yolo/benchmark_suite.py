#!/usr/bin/env python3
"""Run the 18 offline configurations sequentially and export 36 color results."""
import argparse
import csv
from datetime import datetime
import json
from pathlib import Path
import subprocess
import sys

from benchmark_yolo import HERE, MODELS


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session", type=Path, required=True)
    args = parser.parse_args()
    session = args.session.resolve()
    # Check every checkpoint before starting the lengthy run.
    for model in MODELS:
        paths = [HERE/"models"/(model+("-seg.pt" if model.startswith("yoloe") else ".pt")),
                 HERE.parent/"finetuning/ping_pong_20261010"/model/"weights/best.pt"]
        for path in paths:
            if not path.is_file():
                raise FileNotFoundError(path)
    output = HERE/"results"/("suite_"+datetime.now().strftime("%Y%m%d_%H%M%S_%f"))
    output.mkdir(parents=True, exist_ok=False)
    rows, runs, fingerprints = [], [], set()
    print(f"Suite: {output}", flush=True)
    for model in MODELS:
        for variant in ("pretrained", "finetuned"):
            destination = output/f"{model}_{variant}"
            command = [sys.executable, str(HERE/"benchmark_yolo.py"), "--session", str(session),
                       "--model", model, "--variant", variant, "--confidence", "0.50",
                       "--imgsz", "1280", "--eval-stride", "5", "--device", "cuda:0",
                       "--output-dir", str(destination)]
            print(f"START {len(runs)+1}/18 {model} {variant}", flush=True)
            with (output/f"{model}_{variant}.log").open("w") as log:
                completed = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT)
            if completed.returncode:
                raise RuntimeError(f"Benchmark failed; inspect {log.name}. Remaining runs not launched.")
            metadata = json.loads((destination/"metadata.json").read_text())
            report = json.loads((destination/"summary.json").read_text())
            if metadata["status"] != "completed" or not metadata["quality_complete_at_selected_stride"]:
                raise RuntimeError(f"Incomplete results: {destination}")
            fingerprints.add(metadata["reference_fingerprint"])
            if len(fingerprints) != 1:
                raise RuntimeError("Reference annotations/videos changed during the suite")
            runs.append({"model": model, "variant": variant, "directory": str(destination),
                         "overall": report["overall"], "negative": report["by_color"]["none"]})
            for color in ("orange", "white"):
                rows.append({"model": model, "variant": variant, "color": color,
                             **report["by_color"][color],
                             "negative_false_positive_frames_percent": report["by_color"]["none"]["negative_frames_with_false_positives_percent"]})
            (output/"suite.json").write_text(json.dumps({"status": "completed" if len(runs) == 18 else "running",
                "session": str(session), "reference_fingerprint": next(iter(fingerprints)), "runs": runs}, indent=2)+"\n")
            with (output/"comparison_36.csv").open("w", newline="") as file:
                writer = csv.DictWriter(file, fieldnames=list(rows[0]))
                writer.writeheader()
                writer.writerows(rows)
            print(f"DONE {len(runs)}/18 {model} {variant}", flush=True)
    print(f"Completed: {output/'comparison_36.csv'}", flush=True)


if __name__ == "__main__":
    main()
