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
