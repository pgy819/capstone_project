#!/usr/bin/env python3
"""Fine-tune YOLOv8 on the exported RCWS dataset (2 classes: phone, person) and export ONNX for Hailo.

Run on a PC/Colab with a GPU (not on the Pi):
    pip install ultralytics
    python3 train_yolo.py ds/data.yaml --model yolov8s.pt --epochs 120
Output: runs/rcws/<name>/weights/best.pt and best.onnx (opset 11) -> hailo/compile_hef.sh

Choices tied to the runtime
  - yolov8s at 640: same backbone/input as the COCO HEF now used, so latency stays comparable
    and the A/B isolates the effect of the data.
  - the dataset already contains scale-normalised ROI crops (export_dataset.py), so scale
    augmentation is moderate and mosaic is switched off for the last epochs (close_mosaic):
    the final epochs see the inference distribution, not 4-image collages.
  - no vertical flip (phone/person have a canonical up), mild hue shift (monitor/screen colour),
    stronger value shift (ROI gain control changes brightness by state).
"""
import argparse


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("data")
    ap.add_argument("--model", default="yolov8s.pt", help="start from COCO weights")
    ap.add_argument("--epochs", type=int, default=120)
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--batch", type=int, default=16)
    ap.add_argument("--name", default="rcws_2cls")
    ap.add_argument("--freeze", type=int, default=0, help="freeze first N layers (10 = backbone) for tiny datasets")
    a = ap.parse_args()

    from ultralytics import YOLO

    m = YOLO(a.model)
    m.train(
        data=a.data, imgsz=a.imgsz, epochs=a.epochs, batch=a.batch, project="runs/rcws", name=a.name,
        patience=30, cos_lr=True, freeze=a.freeze or None,
        mosaic=0.7, close_mosaic=20, mixup=0.0, copy_paste=0.0,
        degrees=5.0, translate=0.15, scale=0.4, shear=0.0, perspective=0.0,
        fliplr=0.5, flipud=0.0,
        hsv_h=0.01, hsv_s=0.4, hsv_v=0.5,
        max_det=20,
    )
    best = YOLO(f"runs/rcws/{a.name}/weights/best.pt")
    print(best.val(data=a.data, imgsz=a.imgsz, split="val"))
    path = best.export(format="onnx", imgsz=a.imgsz, opset=11, simplify=True, dynamic=False)
    print("ONNX:", path)


if __name__ == "__main__":
    main()
