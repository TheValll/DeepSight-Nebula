```bash
cd /home/valentin/DeepSight-Nebula

.venv/bin/python app/utils/vision/record_tennis_dataset.py \
  --duration 10 \
  --scene salon-jour \
  --camera 0

.venv/bin/python app/utils/vision/extract_tennis_frames.py \
  --sample-fps 5
```

```bash
.venv/bin/python app/utils/vision/extract_tennis_frames.py \
  --all-frames
```
