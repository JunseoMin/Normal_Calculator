python3 python/yoloBbox.py \
  --input_dir ./data/images \
  --weights ./data/weight/best_new_astrobee.pt \
  --obj1-classes 0 \
  --obj1-csv ./data/bboxes/1.csv \
  --obj2-classes 1 \
  --obj2-csv ./data/bboxes/2.csv \
  --conf 0.25 --iou 0.45 --imgsz 1280 \
  --device 0