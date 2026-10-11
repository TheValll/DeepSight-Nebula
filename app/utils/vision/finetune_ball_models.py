#!/usr/bin/env python3
"""Train the nine benchmark models sequentially, with isolated CUDA workers."""

import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
from datetime import datetime, timezone

ROOT = Path(__file__).resolve().parents[3]
MODELS = [f"yolo11{s}" for s in "nsm"] + [f"yolo26{s}" for s in "nsm"] + [f"yoloe-26{s}" for s in "nsm"]


def timestamp():
    return datetime.now(timezone.utc).isoformat()


def save_state(path, state):
    temporary = path.with_suffix(".tmp")
    temporary.write_text(json.dumps(state, indent=2) + "\n")
    temporary.replace(path)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data", type=Path, default=ROOT / "datasets/ping_pong_ball/prepared/v1_20261010/data.yaml")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--epochs", type=int, default=100)
    parser.add_argument("--patience", type=int, default=20)
    parser.add_argument("--batch", type=int, default=4)
    parser.add_argument("--imgsz", type=int, default=1280)
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--fraction", type=float, default=1.0, help="Training fraction; keep 1 for the real comparison")
    parser.add_argument("--resume", action="store_true", help="Resume an interrupted queue using its saved settings")
    parser.add_argument("--worker-model", choices=MODELS, help=argparse.SUPPRESS)
    args = parser.parse_args()
    if min(args.epochs, args.batch, args.imgsz) < 1 or min(args.patience, args.workers) < 0 or not 0 < args.fraction <= 1:
        parser.error("Use positive epochs/batch/imgsz, non-negative patience/workers and 0 < fraction <= 1")
    args.data = args.data.resolve()
    args.output = args.output.resolve()
    return args


def weights_for(name):
    suffix = "-seg.pt" if name.startswith("yoloe") else ".pt"
    return ROOT / "benchmarks/yolo/models" / (name + suffix)


def train_one(args):
    import torch
    import ultralytics
    from ultralytics import YOLO, YOLOE
    from ultralytics.models.yolo.yoloe import YOLOEPETrainer

    if not torch.cuda.is_available():
        raise RuntimeError("CUDA is required; refusing to fall back to CPU")
    # The official YOLOE trainer initializes and fuses the text embedding once.
    # Box-only annotations require its detection architecture, not segmentation.
    name = args.worker_model
    os.chdir(ROOT / "benchmarks/yolo/models")
    last = args.output / name / "weights/last.pt"
    if args.resume:
        if not last.is_file():
            raise FileNotFoundError(last)
        if name.startswith("yoloe"):
            from copy import deepcopy

            class ResumeYOLOETrainer(YOLOEPETrainer):
                def get_model(self, cfg=None, weights=None, verbose=True):
                    # Already fused at the first training: do not encode/fuse again.
                    if weights is None:
                        raise RuntimeError("Resume requires the saved model")
                    return deepcopy(weights).float()

            model = YOLOE(str(last))
            extra = {"trainer": ResumeYOLOETrainer}
        else:
            model = YOLO(str(last))
            extra = {}
    elif name.startswith("yoloe"):
        model = YOLOE(name + ".yaml").load(str(weights_for(name)))
        extra = {"trainer": YOLOEPETrainer}
    else:
        model = YOLO(str(weights_for(name)))
        extra = {}

    def require_common_batch(trainer):
        if trainer.batch_size != args.batch:
            raise RuntimeError("Automatic batch reduction would invalidate the common settings; rerun all models with a smaller batch")

    model.add_callback("on_train_batch_start", require_common_batch)
    print(f"ultralytics={ultralytics.__version__}, torch={torch.__version__}, GPU={torch.cuda.get_device_name(0)}", flush=True)
    if args.resume:
        model.train(resume=True, device=0, workers=args.workers, **extra)
        return
    model.train(
        data=str(args.data), project=str(args.output), name=name,
        epochs=args.epochs, patience=args.patience, imgsz=args.imgsz,
        batch=args.batch, workers=args.workers, seed=args.seed, device=0,
        optimizer="AdamW", lr0=0.001, lrf=0.01, deterministic=True,
        warmup_bias_lr=0.0, fraction=args.fraction,
        amp=True, cache=False, plots=True, exist_ok=False, **extra,
    )
    # Keep test videos untouched until the final comparison.


def main():
    args = parse_args()
    if args.worker_model:
        train_one(args)
        return
    state_path = args.output / "queue.json"
    previous = None
    if args.resume:
        previous = json.loads(state_path.read_text())
        old_cmd = Path(f"/proc/{previous['pid']}/cmdline")
        if old_cmd.exists() and b"finetune_ball_models.py" in old_cmd.read_bytes():
            raise RuntimeError("The original queue is still alive; stop it before resuming")
        for key, value in previous["config"].items():
            if key not in ("resume", "worker_model"):
                setattr(args, key, Path(value) if key in ("data", "output") else value)
        if previous["data_sha256"] != hashlib.sha256(args.data.read_bytes()).hexdigest():
            raise RuntimeError("Dataset configuration changed since the original training")
        if [m["name"] for m in previous["models"]] != MODELS:
            raise RuntimeError("The saved model list does not match this script")
        for item in previous["models"]:
            if hashlib.sha256(weights_for(item["name"]).read_bytes()).hexdigest() != item["source_sha256"]:
                raise RuntimeError(f"Original weights changed: {item['name']}")
            if item["status"] == "completed" and not (args.output / item["name"] / "weights/best.pt").is_file():
                raise FileNotFoundError(item["best"])
            if item["status"] != "completed" and (args.output / item["name"]).exists() and not (args.output / item["name"] / "weights/last.pt").is_file():
                raise RuntimeError(f"No checkpoint to resume {item['name']}; refusing to overwrite its folder")
    elif args.output.exists():
        raise FileExistsError(f"Refusing to overwrite an existing run: {args.output}")
    if not args.data.is_file():
        raise FileNotFoundError(args.data)
    for name in MODELS:
        if not weights_for(name).is_file():
            raise FileNotFoundError(weights_for(name))
    import torch
    import ultralytics
    if not torch.cuda.is_available():
        raise RuntimeError("CUDA unavailable")
    args.output.mkdir(parents=True, exist_ok=args.resume)
    state = {
        "started_at": timestamp(), "status": "running", "pid": os.getpid(),
        "config": {k: str(v) if isinstance(v, Path) else v for k, v in vars(args).items() if k not in ("worker_model", "resume")},
        "optimizer": "AdamW", "lr0": 0.001, "lrf": 0.01, "warmup_bias_lr": 0.0,
        "ultralytics": ultralytics.__version__, "torch": torch.__version__,
        "gpu": torch.cuda.get_device_name(0),
        "data_sha256": hashlib.sha256(args.data.read_bytes()).hexdigest(),
        "notes": ["Sequential GPU training", "YOLOE detection fine-tuning with fused class embedding; no segmentation masks", "Test set not used for training or checkpoint selection"],
        "models": [{"name": n, "status": "pending", "source": str(weights_for(n)),
                    "source_sha256": hashlib.sha256(weights_for(n).read_bytes()).hexdigest()} for n in MODELS],
    }
    if previous is not None:
        if previous["ultralytics"] != ultralytics.__version__ or previous["torch"] != torch.__version__:
            raise RuntimeError("Library versions changed; restore the original environment before resuming")
        state = previous
        state.update(status="running", pid=os.getpid(), resumed_at=timestamp())
        state.pop("finished_at", None)
    save_state(state_path, state)
    child = None

    def stop(signum, frame):
        nonlocal child
        if child is not None and child.poll() is None:
            # Includes DataLoader workers; never signal unrelated robot processes.
            os.killpg(child.pid, signal.SIGTERM)
            # Do not wait inside the signal handler: the interrupted main-thread
            # Popen.wait may already hold its waitpid lock.
        for item in state["models"]:
            if item["status"] == "running":
                item["status"] = "interrupted"
        state.update(status="interrupted", finished_at=timestamp())
        save_state(state_path, state)
        raise SystemExit(128 + signum)

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    for item in state["models"]:
        if item["status"] == "completed":
            print(f"SKIP completed {item['name']}", flush=True)
            continue
        name = item["name"]
        resume_model = args.resume and (args.output / name / "weights/last.pt").is_file()
        item.update(status="running", started_at=timestamp())
        save_state(state_path, state)
        print(f"{timestamp()} START {name}", flush=True)
        command = [sys.executable, "-u", str(Path(__file__).resolve()),
                   "--worker-model", name, "--data", str(args.data), "--output", str(args.output)]
        for key in ("epochs", "patience", "batch", "imgsz", "workers", "seed", "fraction"):
            command.extend(["--" + key, str(getattr(args, key))])
        if resume_model:
            command.append("--resume")
        with (args.output / f"{name}.log").open("a" if args.resume else "w") as log:
            child = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            item["pid"] = child.pid
            save_state(state_path, state)
            code = child.wait()
        checkpoint = args.output / name / "weights/best.pt"
        item.update(status="completed" if code == 0 and checkpoint.is_file() else "failed",
                    returncode=code, finished_at=timestamp(), best=str(checkpoint))
        child = None
        save_state(state_path, state)
        print(f"{timestamp()} {item['status'].upper()} {name}", flush=True)
    state.update(status="completed" if all(m["status"] == "completed" for m in state["models"]) else "completed_with_errors",
                 finished_at=timestamp())
    save_state(state_path, state)
    if state["status"] != "completed":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
