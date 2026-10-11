import csv
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

SPEC = importlib.util.spec_from_file_location("install_reference", Path(__file__).resolve().parents[1]/"install_benchmark_reference.py")
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class ReferenceInstallTest(unittest.TestCase):
    def fixture(self, root):
        session = root/"session"
        export = root/"export"
        for path in (session/"review_input", export/"images", export/"labels"):
            path.mkdir(parents=True)
        (session/"session.json").write_text(json.dumps({"tasks": [{"id": "ball", "expected_objects": 1}], "accepted": {"ball": "video.avi"}}))
        (export/"classes.txt").write_text("ping_pong_ball\n")
        refs, rows = [], []
        for index in (0, 5):
            name = f"video_f{index:06d}.png"
            (session/"review_input"/name).write_bytes(b"unchanged test image")
            (export/"images"/name).write_bytes(b"unchanged test image")
            (export/"labels"/Path(name).with_suffix(".txt")).write_text("0 0.5 0.5 0.1 0.1\n")
            refs.append({"image": name, "source_video": "video.avi", "frame_index": index,
                         "reference_label": str(session/"annotations/video"/f"{index:06d}.txt")})
            rows.append({"image": name, "source_video": "video.avi", "status": "validated"})
        for path, data in ((session/"review_input/manifest.csv", refs), (export/"manifest.csv", rows)):
            with path.open("w", newline="") as file:
                writer = csv.DictWriter(file, fieldnames=list(data[0]))
                writer.writeheader()
                writer.writerows(data)
        return session, export

    def test_complete_review_is_checked_without_writes(self):
        with tempfile.TemporaryDirectory() as directory:
            session, export = self.fixture(Path(directory))
            plan = MODULE.build_plan(session, export)
            self.assertEqual(len(plan), 2)
            self.assertFalse((session/"annotations").exists())

    def test_partial_export_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            session, export = self.fixture(Path(directory))
            manifest = export/"manifest.csv"
            manifest.write_text("image,source_video,status\nvideo_f000000.png,video.avi,validated\n")
            with self.assertRaisesRegex(ValueError, "exactly all"):
                MODULE.build_plan(session, export)

    def test_reference_overwrite_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            session, export = self.fixture(Path(directory))
            destination = session/"annotations/video/000000.txt"
            destination.parent.mkdir(parents=True)
            destination.write_text("")
            with self.assertRaisesRegex(ValueError, "refusing to overwrite"):
                MODULE.build_plan(session, export)

    def test_changed_image_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            session, export = self.fixture(Path(directory))
            (export/"images/video_f000000.png").write_bytes(b"another image")
            with self.assertRaisesRegex(ValueError, "image changed"):
                MODULE.build_plan(session, export)


if __name__ == "__main__":
    unittest.main()
