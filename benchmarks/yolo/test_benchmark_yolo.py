import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

SPEC = importlib.util.spec_from_file_location("bench", Path(__file__).with_name("benchmark_yolo.py"))
BENCH = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(BENCH)


def prediction(box, confidence=0.9):
    return {"xyxy": box, "confidence": confidence}


def row(tp, fp, fn, color="orange", evaluated=True):
    result = {"tp": tp, "fp": fp, "fn": fn, "ball_color": color,
              "gt_count": tp+fn if evaluated else None, "evaluated": evaluated,
              "selected_for_evaluation": True, "prediction_count": tp+fp if evaluated else 1}
    for field in ("decode_ms", "inference_ms", "total_processing_ms", "cpu_system_percent", "ram_system_mb", "ram_process_mb", "gpu_system_percent", "vram_system_mb", "torch_allocated_mb"):
        result[field] = 1.0
    return result


class MetricTest(unittest.TestCase):
    def test_wrong_box_is_fp_and_missed_ball_is_fn(self):
        self.assertEqual(BENCH.match([prediction([50, 50, 60, 60])], [[0, 0, 10, 10]], .5), (0, 1, 1))

    def test_duplicate_is_not_an_extra_true_positive(self):
        self.assertEqual(BENCH.match([prediction([0, 0, 10, 10])]*2, [[0, 0, 10, 10]], .5), (1, 1, 0))

    def test_two_real_balls(self):
        boxes = [[0, 0, 10, 10], [50, 50, 60, 60]]
        self.assertEqual(BENCH.match([prediction(b) for b in boxes], boxes, .5), (2, 0, 0))

    def test_negative_video_false_positive(self):
        self.assertEqual(BENCH.match([prediction([0, 0, 10, 10])], [], .5), (0, 1, 0))

    def test_micro_precision_and_recall(self):
        result = BENCH.summary([row(1, 1, 0), row(0, 0, 1), row(0, 1, 0, "none")])
        self.assertAlmostEqual(result["precision_percent"], 100/3)
        self.assertEqual(result["recall_percent"], 50)
        self.assertEqual(result["negative_frames_with_false_positives_percent"], 100)

    def test_missing_references_never_count_as_negatives(self):
        result = BENCH.summary([row(None, None, None, evaluated=False)])
        self.assertEqual(result["evaluated_frames"], 0)
        self.assertEqual(result["reference_coverage_percent"], 0)
        self.assertIsNone(result["precision_percent"])
        self.assertIsNone(result["recall_percent"])

    def test_clean_negatives_have_undefined_precision_and_zero_false_positives(self):
        result = BENCH.summary([row(0, 0, 0, "none")])
        self.assertIsNone(result["precision_percent"])
        self.assertIsNone(result["recall_percent"])
        self.assertEqual(result["negative_frames_with_false_positives_percent"], 0)

    def test_labels(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/"label.txt"
            path.write_text("0 0.5 0.5 0.1 0.2\n")
            self.assertEqual(BENCH.read_labels(path, 100, 100), [[45, 40, 55.00000000000001, 60]])
            path.write_text("")
            self.assertEqual(BENCH.read_labels(path), [])
            path.write_text("0 0.99 0.5 0.2 0.2")
            with self.assertRaises(ValueError):
                BENCH.read_labels(path)

    def test_session_requires_positive_labels_but_accepts_explicit_negative_protocol(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            tasks = [{"id": "ball", "expected_objects": 1}, {"id": "background", "expected_objects": 0}]
            accepted = {}
            for task in tasks:
                video = root/(task["id"]+".avi")
                video.write_bytes(b"fixture; decoding is tested separately")
                accepted[task["id"]] = video.name
                video.with_suffix(".json").write_text(json.dumps(dict(task, accepted=True, interrupted=False,
                    frames=2, saved_resolution=[1280, 720], frame_timestamps_s=[0.01, 0.05])))
            (root/"session.json").write_text(json.dumps({"tasks": tasks, "accepted": accepted}))
            with self.assertRaisesRegex(ValueError, "2 reference labels missing"):
                BENCH.load_session(root, root/"annotations", 1)
            videos, missing = BENCH.load_session(root, root/"annotations", 1, allow_unannotated=True)
            self.assertEqual(len(missing), 2)
            self.assertEqual(videos[0]["truth"], {})
            self.assertEqual(videos[1]["truth"], {0: [], 1: []})

    def test_unfinished_capture_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root/"session.json").write_text(json.dumps({"tasks": [{"id": "ball"}], "accepted": {}}))
            with self.assertRaisesRegex(ValueError, "Finish and validate"):
                BENCH.load_session(root, root/"annotations", 1)


if __name__ == "__main__":
    unittest.main()
