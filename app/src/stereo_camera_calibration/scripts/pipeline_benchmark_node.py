#!/usr/bin/env python3

"""Collect per-stage pipeline timings and write reproducible CSV/JSON reports."""

import csv
from datetime import datetime, timezone
import json
from pathlib import Path
import statistics

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


class PipelineBenchmarkNode(Node):
    def __init__(self) -> None:
        super().__init__("pipeline_benchmark_node")
        self.duration_sec = float(self.declare_parameter("duration_sec", 30.0).value)
        self.warmup_sec = float(self.declare_parameter("warmup_sec", 5.0).value)
        self.label = str(self.declare_parameter("label", "baseline").value)
        self.output_dir = Path(
            str(self.declare_parameter("output_dir", "../benchmarks/pipeline").value)
        ).expanduser()

        self.started_at = self.get_clock().now()
        self.samples: dict[str, list[float]] = {}
        self.source_counts: dict[str, int] = {}
        self.finished = False

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            DiagnosticArray, "/pipeline/metrics", self.store_metrics, sensor_qos
        )
        self.create_timer(0.2, self.check_finished)
        self.get_logger().info(
            f"Warm-up {self.warmup_sec:.0f} s, then benchmark "
            f"'{self.label}' for {self.duration_sec:.0f} s"
        )

    def elapsed_seconds(self) -> float:
        return (self.get_clock().now() - self.started_at).nanoseconds / 1e9

    def store_metrics(self, message: DiagnosticArray) -> None:
        if self.elapsed_seconds() < self.warmup_sec:
            return
        for status in message.status:
            self.source_counts[status.name] = self.source_counts.get(status.name, 0) + 1
            for item in status.values:
                try:
                    value = float(item.value)
                except ValueError:
                    continue
                self.samples.setdefault(f"{status.name}.{item.key}", []).append(value)

    def check_finished(self) -> None:
        if self.finished:
            return
        if self.elapsed_seconds() < self.warmup_sec + self.duration_sec:
            return
        self.finished = True
        self.write_reports()
        rclpy.shutdown()

    @staticmethod
    def percentile(values: list[float], percentile: float) -> float:
        ordered = sorted(values)
        if len(ordered) == 1:
            return ordered[0]
        position = (len(ordered) - 1) * percentile
        lower = int(position)
        upper = min(lower + 1, len(ordered) - 1)
        fraction = position - lower
        return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction

    def summarize(self) -> list[dict[str, float | int | str]]:
        rows = []
        for full_name, values in sorted(self.samples.items()):
            source, metric = full_name.split(".", 1)
            rows.append(
                {
                    "source": source,
                    "metric": metric,
                    "count": len(values),
                    "mean": statistics.fmean(values),
                    "median": statistics.median(values),
                    "p95": self.percentile(values, 0.95),
                    "min": min(values),
                    "max": max(values),
                }
            )
        return rows

    def write_reports(self) -> None:
        self.output_dir.mkdir(parents=True, exist_ok=True)
        rows = self.summarize()
        csv_path = self.output_dir / f"{self.label}.csv"
        json_path = self.output_dir / f"{self.label}.json"

        with csv_path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(
                stream,
                fieldnames=["source", "metric", "count", "mean", "median", "p95", "min", "max"],
            )
            writer.writeheader()
            writer.writerows(rows)

        report = {
            "label": self.label,
            "created_at": datetime.now(timezone.utc).isoformat(),
            "warmup_seconds": self.warmup_sec,
            "duration_seconds": self.duration_sec,
            "source_message_counts": self.source_counts,
            "metrics": rows,
        }
        with json_path.open("w", encoding="utf-8") as stream:
            json.dump(report, stream, indent=2)
            stream.write("\n")

        self.get_logger().info(f"CSV written to {csv_path.resolve()}")
        self.get_logger().info(f"JSON written to {json_path.resolve()}")
        for row in rows:
            if row["metric"].endswith("_ms") or row["metric"] == "frame_period_ms":
                self.get_logger().info(
                    f"{row['source']}.{row['metric']}: "
                    f"median={row['median']:.2f} ms, p95={row['p95']:.2f} ms "
                    f"(n={row['count']})"
                )


def main() -> None:
    rclpy.init()
    node = PipelineBenchmarkNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
