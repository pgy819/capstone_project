#!/usr/bin/env bash
# Compile the fine-tuned ONNX to a Hailo-8 HEF with in-domain INT8 calibration.
# Run in the Hailo AI Software Suite (DFC + Model Zoo) on an x86 Linux PC, not on the Pi.
#   ./compile_hef.sh best.onnx ../ds/calib [yolov8s]
# Output: rcws_2cls_h8.hef -> copy to rpi/models/, run with config/rcws_custom2cls.ini
#
# Calibration images must be what the runtime feeds the NPU (letterboxed lores frames and
# scale-normalised ROI crops), which export_dataset.py --calib writes. COCO calibration
# images give INT8 ranges for a different input distribution.
#
# Check after compiling (HailoRT on the Pi):
#   hailortcli parse-hef rcws_2cls_h8.hef      -> NMS output, 2 classes, input 640x640x3
# The runtime verifies the NMS layout at start-up and prints the class count.
#
# Optional (A/B separately): 16-bit precision on the box-regression outputs reduces centre
# quantisation jitter at some throughput cost. Copy the Model Zoo's yolov8s.alls, add a
#   quantization_param([<bbox output conv names>], precision_mode=a16_w16)
# line (names from `hailo profiler` / the parsed HAR) and pass it with --model-script.
set -euo pipefail
ONNX=${1:?usage: compile_hef.sh best.onnx calib_dir [arch_name]}
CALIB=${2:?calibration image directory}
ARCH=${3:-yolov8s}
N=$(ls "$CALIB" | wc -l)
if [ "$N" -lt 64 ]; then echo "only $N calibration images (want >= 256)" >&2; fi
hailomz compile "$ARCH" --ckpt "$ONNX" --hw-arch hailo8 --calib-path "$CALIB" --classes 2 --performance \
  ${MODEL_SCRIPT:+--model-script "$MODEL_SCRIPT"}
mv -f "${ARCH}.hef" rcws_2cls_h8.hef
echo "wrote rcws_2cls_h8.hef"
