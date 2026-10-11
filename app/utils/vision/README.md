```bash
cd /home/valentin/DeepSight-Nebula

# SPACE: start after 3 seconds. ESC: stop.
.venv/bin/python app/utils/vision/record_ball_dataset.py \
  --duration 10 \
  --scene salon-jour-static \
  --ball-color white \
  --camera 2

# Repeat with orange, other rooms, lighting, distances and movement.
.venv/bin/python app/utils/vision/record_ball_dataset.py \
  --duration 15 --scene bureau-nuit-mouvement --ball-color orange --camera 2

# Background video without a ball.
.venv/bin/python app/utils/vision/record_ball_dataset.py \
  --duration 10 --scene salon-jour --ball-color none --camera 2

# Extract 5 frames per second from all videos.
.venv/bin/python app/utils/vision/extract_ball_frames.py \
  --sample-fps 5
```

```bash
.venv/bin/python app/utils/vision/extract_ball_frames.py \
  --all-frames
```

```bash
# Generate proposals and previews. Review them before training.
.venv/bin/python app/utils/vision/preannotate_ball_dataset.py
```

```bash
# After manual review and export: prepare separate train/val/test videos.
# Never overwrites an existing prepared dataset.
.venv/bin/python app/utils/vision/prepare_ball_dataset.py
```

```bash
# Train the 9 models, one after another on CUDA. Use a new output folder each time.
.venv/bin/python app/utils/vision/finetune_ball_models.py \
  --output benchmarks/finetuning/ping_pong_20261010 \
  --imgsz 1280 --epochs 100 --patience 20 --batch 4
```

Same dataset and settings for all models. YOLOE learns boxes, not masks.
Check progress in `benchmarks/finetuning/ping_pong_20261010/queue.json`.
Each model has a log and a `weights/best.pt`. Camera benchmarks come afterwards.

```bash
# After stopping the training and restarting the PC: resume the same queue.
# Completed models are skipped. Interrupted training resumes from last.pt.
.venv/bin/python app/utils/vision/finetune_ball_models.py \
  --output benchmarks/finetuning/ping_pong_20261010 --resume
```
