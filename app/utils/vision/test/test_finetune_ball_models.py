import contextlib
import importlib.util
import io
import json
from pathlib import Path
import signal
import tempfile
import unittest
from unittest.mock import MagicMock, patch

SPEC = importlib.util.spec_from_file_location("finetune", Path(__file__).resolve().parents[1] / "finetune_ball_models.py")
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class QueueTest(unittest.TestCase):
    def run_queue(self, failure=None, resume=False):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            data = root / "data.yaml"
            data.write_text("names: [ball]\n")
            weights = root / "source.pt"
            weights.write_bytes(b"test weights")
            output = root / "run"
            torch = MagicMock(__version__="test")
            torch.cuda.is_available.return_value = True
            torch.cuda.get_device_name.return_value = "mock GPU"
            calls = []
            commands = []

            def launch(command, **kwargs):
                name = command[command.index("--worker-model") + 1]
                calls.append(name)
                commands.append(command)
                worker = MagicMock(pid=123)
                worker.wait.return_value = 1 if name == failure else 0
                if name != failure:
                    checkpoint = output / name / "weights/best.pt"
                    checkpoint.parent.mkdir(parents=True, exist_ok=True)
                    checkpoint.write_bytes(b"checkpoint")
                return worker

            with patch.object(MODULE.sys, "argv", ["train", "--data", str(data), "--output", str(output)]), \
                 patch.dict(MODULE.sys.modules, {"torch": torch, "ultralytics": MagicMock(__version__="test")}), \
                 patch.object(MODULE, "weights_for", return_value=weights), \
                 patch.object(MODULE.subprocess, "Popen", side_effect=launch), \
                 patch.object(signal, "signal"), contextlib.redirect_stdout(io.StringIO()):
                if failure:
                    with self.assertRaises(SystemExit):
                        MODULE.main()
                else:
                    MODULE.main()
                if resume:
                    checkpoint = output / failure / "weights/last.pt"
                    checkpoint.parent.mkdir(parents=True)
                    checkpoint.write_bytes(b"saved checkpoint")
                    failure = None
                    calls.clear()
                    commands.clear()
                    MODULE.sys.argv.append("--resume")
                    MODULE.main()
                    self.assertTrue(all("--resume" in c for c in commands))
            return calls, json.loads((output / "queue.json").read_text())

    def test_nine_models_complete_in_order(self):
        calls, state = self.run_queue()
        self.assertEqual(calls, MODULE.MODELS)
        self.assertEqual(len(calls), 9)
        self.assertEqual(state["status"], "completed")
        self.assertTrue(all(m["status"] == "completed" for m in state["models"]))

    def test_failed_model_does_not_stop_remaining_models(self):
        calls, state = self.run_queue("yolo11s")
        self.assertEqual(calls, MODULE.MODELS)
        self.assertEqual(state["status"], "completed_with_errors")
        self.assertEqual(state["models"][1]["status"], "failed")

    def test_existing_output_is_not_overwritten(self):
        with tempfile.TemporaryDirectory() as directory:
            with patch.object(MODULE.sys, "argv", ["train", "--output", directory]):
                with self.assertRaises(FileExistsError):
                    MODULE.main()

    def test_resume_skips_finished_models_and_resumes_checkpoint(self):
        calls, state = self.run_queue("yolo11s", resume=True)
        self.assertEqual(calls, ["yolo11s"])
        self.assertEqual(state["status"], "completed")


if __name__ == "__main__":
    unittest.main()
