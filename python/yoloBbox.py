#!/usr/bin/env python3
# Requirements:
#   pip install torch torchvision
#   pip install git+https://github.com/ultralytics/yolov5.git

import argparse
import csv
from pathlib import Path
from typing import List, Optional

import torch

IMG_EXTS = {".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff", ".webp"}

def collect_images(input_dir: str, recursive: bool = True) -> List[Path]:
    p = Path(input_dir)
    if not p.exists():
        raise FileNotFoundError(f"Input directory not found: {p}")
    pattern = "**/*" if recursive else "*"
    files = [x for x in p.glob(pattern) if x.suffix.lower() in IMG_EXTS]
    files.sort()
    return files

def parse_classes(spec: str) -> Optional[List[int]]:
    spec = (spec or "").strip()
    if not spec:
        return None
    out = []
    for t in spec.split(","):
        t = t.strip()
        if t:
            out.append(int(t))
    return out

def load_yolov5(weights: str, device: str = ""):
    """
    Load YOLOv5 model via torch.hub.
    device: ''(auto) | 'cpu' | '0' | '1' ...
    """
    if device and device.lower() != "cpu":
        # Move to selected GPU index
        torch.cuda.set_device(int(device))
        dev = f"cuda:{device}"
    elif device.lower() == "cpu":
        dev = "cpu"
    else:
        dev = "cuda:0" if torch.cuda.is_available() else "cpu"

    # Load custom weights
    model = torch.hub.load('ultralytics/yolov5', 'custom', path=weights, device=dev)
    return model, dev

def ensure_parent(path: Path):
    path.parent.mkdir(parents=True, exist_ok=True)

def write_row(writer: csv.writer, include_filename: bool, img_name: str, row_vals: List[int]):
    if include_filename:
        writer.writerow([img_name] + row_vals)
    else:
        writer.writerow(row_vals)

def main():
    ap = argparse.ArgumentParser(
        description="YOLOv5 inference on images and save two object groups to separate CSV files.\n"
                    "Each line = x1,y1,x2,y2  (or -1 if none for that image)."
    )
    ap.add_argument("--input_dir", required=True, help="Image folder")
    ap.add_argument("--weights", required=True, help="YOLOv5 weights (.pt) - trained with v5")
    ap.add_argument("--obj1-classes", type=str, default="", help="Group1 class indices (e.g., '0,2')")
    ap.add_argument("--obj2-classes", type=str, default="", help="Group2 class indices (e.g., '1,3')")
    ap.add_argument("--obj1-csv", type=str, default="obj1.csv", help="Group1 CSV output path")
    ap.add_argument("--obj2-csv", type=str, default="obj2.csv", help="Group2 CSV output path")
    ap.add_argument("--conf", type=float, default=0.25, help="Confidence threshold")
    ap.add_argument("--iou", type=float, default=0.45, help="NMS IoU threshold")
    ap.add_argument("--device", type=str, default="", help="'cpu' or GPU index (e.g. '0')")
    ap.add_argument("--include-filename", action="store_true", help="Include filename in CSV")
    ap.add_argument("--no-recursive", action="store_true", help="Do not search subfolders")
    ap.add_argument("--imgsz", type=int, default=1280, help="Inference image size (letterbox)")

    args = ap.parse_args()

    imgs = collect_images(args.input_dir, recursive=(not args.no_recursive))
    if not imgs:
        raise RuntimeError(f"No images found in: {args.input_dir}")

    # Load YOLOv5 model
    model, dev = load_yolov5(args.weights, device=args.device)

    # Inference settings
    model.conf = args.conf
    model.iou = args.iou
    model.max_det = 300

    # Parse class groups
    obj1 = parse_classes(args.obj1_classes)  # None → allow all classes
    obj2 = parse_classes(args.obj2_classes)

    csv1_path = Path(args.obj1_csv)
    csv2_path = Path(args.obj2_csv)
    ensure_parent(csv1_path)
    ensure_parent(csv2_path)

    with open(csv1_path, "w", newline="") as f1, open(csv2_path, "w", newline="") as f2:
        w1 = csv.writer(f1)
        w2 = csv.writer(f2)

        for img_path in imgs:
            img_name = img_path.name

            # NOTE: YOLOv5 resizes input to a square (letterbox),
            # but returns bounding boxes mapped back to the original image coordinates.
            results = model(str(img_path), size=args.imgsz, augment=False)

            # results.xyxy[0]: Tensor [N,6] = x1 y1 x2 y2 conf cls
            # results.names: class id -> name dict
            det = results.xyxy[0]
            if det is None or det.numel() == 0:
                # Both groups = -1
                write_row(w1, args.include_filename, img_name, [-1])
                write_row(w2, args.include_filename, img_name, [-1])
                continue

            # Tensor → numpy array
            det = det.cpu().numpy()  # N x 6
            # Filtering function
            def filter_indices(target_classes: Optional[List[int]]):
                if target_classes is None:
                    return list(range(det.shape[0]))
                idxs = []
                for i in range(det.shape[0]):
                    cls_id = int(det[i, 5])
                    if cls_id in target_classes:
                        idxs.append(i)
                return idxs

            idxs1 = filter_indices(obj1)
            idxs2 = filter_indices(obj2)

            # --- Write group 1 ---
            if len(idxs1) == 0:
                write_row(w1, args.include_filename, img_name, [-1])
            else:
                wrote = False
                for i in idxs1:
                    x1, y1, x2, y2 = det[i, :4]
                    row = [int(round(x1)), int(round(y1)), int(round(x2)), int(round(y2))]
                    write_row(w1, args.include_filename, img_name, row)
                    wrote = True
                if not wrote:
                    write_row(w1, args.include_filename, img_name, [-1])

            # --- Write group 2 ---
            if len(idxs2) == 0:
                write_row(w2, args.include_filename, img_name, [-1])
            else:
                wrote = False
                for i in idxs2:
                    x1, y1, x2, y2 = det[i, :4]
                    row = [int(round(x1)), int(round(y1)), int(round(x2)), int(round(y2))]
                    write_row(w2, args.include_filename, img_name, row)
                    wrote = True
                if not wrote:
                    write_row(w2, args.include_filename, img_name, [-1])

    print(f"Saved:\n  - {csv1_path}\n  - {csv2_path}")
    print("Done.")

if __name__ == "__main__":
    main()
