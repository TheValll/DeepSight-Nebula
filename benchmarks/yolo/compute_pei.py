#!/usr/bin/env python3
"""Calculate an exploratory PEI from a completed 18-configuration suite."""
import argparse
import csv
import json
import math
from pathlib import Path
import statistics

from benchmark_yolo import iou, write_json

WEIGHTS = {"quality": .30, "latency": .10, "gpu": .05, "cpu": .10,
           "vram": .20, "ram": .20, "confidence": .05}
LIMITS = {"detection_min_percent": 87.0, "precision_min_percent": 96.0,
          "latency_p95_max_ms": 40.0, "confidence_target": .85}


def meets_requirements(row):
    """Apply hard limits to unrounded values; confidence is a soft objective."""
    return (row["detection"]*100 >= LIMITS["detection_min_percent"]
            and (row["precision_percent"] or 0) >= LIMITS["precision_min_percent"]
            and row["latency_ms_p95"] <= LIMITS["latency_p95_max_ms"])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--suite", type=Path, required=True)
    args = parser.parse_args()
    suite = json.loads((args.suite/"suite.json").read_text())
    if suite["status"] != "completed" or len(suite["runs"]) != 18:
        raise ValueError("A completed 18-configuration suite is required")
    rows = []
    for run in suite["runs"]:
        directory = Path(run["directory"])
        metadata = json.loads((directory/"metadata.json").read_text())
        if metadata["reference_fingerprint"] != suite["reference_fingerprint"]:
            raise ValueError("Reference fingerprint mismatch")
        confidences = []
        with (directory/"predictions.jsonl").open() as file:
            for line in file:
                frame = json.loads(line)
                truth = frame["ground_truth_xyxy"]
                if not truth:
                    continue
                unmatched = set(range(len(truth)))
                for prediction in sorted(frame["predictions"], key=lambda p: p["confidence"], reverse=True):
                    if not unmatched:
                        break
                    target = max(unmatched, key=lambda j: iou(prediction["xyxy"], truth[j]))
                    if iou(prediction["xyxy"], truth[target]) >= metadata["matching_iou"]:
                        unmatched.remove(target)
                        confidences.append(prediction["confidence"])
        stats = run["overall"]
        if len(confidences) != stats["tp"]:
            raise ValueError("Correct-detection confidence count differs from benchmark TP")
        with (directory/"frames.csv").open() as file:
            times = sorted(float(r["total_processing_ms"]) for r in csv.DictReader(file))
        rows.append({"model": run["model"], "variant": run["variant"],
                     "quality": (stats["f1_percent"] or 0)/100,
                     "detection": (stats["ball_visible_frames_detected_percent"] or 0)/100,
                     "latency_ms_p95": times[math.ceil(.95*len(times))-1],
                     "gpu_percent": stats["gpu_system_percent_mean"],
                     "cpu_percent": stats["cpu_system_percent_mean"],
                     "vram_mb": stats["vram_system_mb_mean"],
                     "ram_mb": stats["ram_process_mb_mean"],
                     "confidence": statistics.fmean(confidences) if confidences else 0,
                     "precision_percent": stats["precision_percent"], "recall_percent": stats["recall_percent"]})
    bounds = {k: [min(r[k] for r in rows), max(r[k] for r in rows)]
              for k in ("latency_ms_p95", "vram_mb", "ram_mb")}
    def lower_is_better(row, key):
        low, high = bounds[key]
        return (high-row[key])/(high-low) if high > low else 1.0
    for row in rows:
        components = {"quality": row["quality"], "confidence": row["confidence"],
                      "gpu": 1-row["gpu_percent"]/100, "cpu": 1-row["cpu_percent"]/100,
                      "latency": lower_is_better(row, "latency_ms_p95"),
                      "vram": lower_is_better(row, "vram_mb"), "ram": lower_is_better(row, "ram_mb")}
        row["pei"] = sum(WEIGHTS[k]*v for k, v in components.items())
        row["pei_percent"] = 100*row["pei"]
        row["meets_requirements"] = meets_requirements(row)
        row["confidence_target_met"] = row["confidence"] >= LIMITS["confidence_target"]
        row.update({"component_"+k: v for k, v in components.items()})
    rows.sort(key=lambda r: r["pei_percent"], reverse=True)
    write_json(args.suite/"pei.json", {"status": "exploratory", "quality_metric": "f1", "weights": WEIGHTS,
               "requirements": LIMITS,
               "relative_cost_bounds": bounds, "reference_fingerprint": suite["reference_fingerprint"],
               "notes": ["F1 quality includes both colors and no-ball sequences",
                         "Detection is the fraction of annotated ball-visible frames with at least one matched true positive",
                         "Detection is included through F1, not scored a second time",
                         "Hard requirements flag candidates but do not exclude any model from the score table or normalization",
                         "Confidence target is advisory, not an eligibility condition",
                         "Confidence uses matched true positives only",
                         "Latency includes decoding, detection and per-frame measurement; excludes serialization",
                         "Latency and memory use inverse min-max normalization over these 18 configurations",
                         "CPU/GPU use 1 minus utilization divided by 100", "Hardware was measured at maximum throughput, not fixed 30 FPS",
                         "VRAM and CPU/GPU are system-wide; RAM is process RSS", "Not an embedded-platform validation"], "rows": rows})
    with (args.suite/"pei.csv").open("w", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    for row in rows:
        status = "eligible" if row["meets_requirements"] else "outside requirements"
        print(f"{row['model']} {row['variant']}: {row['pei']:.3f} ({status})")


if __name__ == "__main__":
    main()
