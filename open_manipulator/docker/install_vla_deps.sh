#!/bin/bash
#
# Python extras for vla_bridge (openpi-client) and the scripted demonstrations (YOLO), inside a running container.
# `./container.sh start` pulls the prebuilt ROBOTIS image (it does not build this Dockerfile) and `./container.sh stop`
# removes the container, so run this after every start:
#
#   ./container.sh enter
#   bash /root/ros2_ws/src/open_manipulator/docker/install_vla_deps.sh
#
# Same pins as the Dockerfile (tested together 2026-11-05): numpy 1.26.x (< 2, openpi-client needs it), opencv-python
# 4.10.0.84, CPU torch 2.5.1 from PyTorch's index (PyPI's default is the multi-GB CUDA build), ultralytics 8.3.40.
# Skips what is already installed. If the PyTorch download keeps timing out, download the two .whl files with a browser or
# `curl -C - --retry 20` from https://download.pytorch.org/whl/cpu/ and pass their paths: install_vla_deps.sh torch*.whl torchvision*.whl
#
set -e
PIP="pip install --break-system-packages --timeout 120 --retries 10"

if ! python3 -c "import openpi_client, typing_extensions" 2>/dev/null; then
    echo "📥 openpi-client (vla_bridge)"
    $PIP typing_extensions \
        "openpi-client @ git+https://github.com/Physical-Intelligence/openpi.git@215abfb217dbac7d5f1273282331b9b1866c0479#subdirectory=packages/openpi-client"
fi

if ! python3 -c "import torch" 2>/dev/null; then
    echo "📥 PyTorch (CPU)"
    if [ "$#" -gt 0 ]; then
        $PIP "$@" "numpy<2"
    else
        $PIP --index-url https://download.pytorch.org/whl/cpu torch==2.5.1 torchvision==0.20.1 "numpy<2"
    fi
fi

if ! python3 -c "import ultralytics" 2>/dev/null; then
    echo "📥 ultralytics + OpenCV 4.10 (object_detector)"
    $PIP --ignore-installed tqdm
    $PIP "numpy<2" "opencv-python==4.10.0.84" "ultralytics==8.3.40"
fi

python3 - <<'EOF'
import numpy, cv2, torch, ultralytics, openpi_client
ok = numpy.__version__.startswith("1.") and cv2.__version__.startswith("4.10")
print(f"{'✅' if ok else '❌'} numpy {numpy.__version__}, cv2 {cv2.__version__}, torch {torch.__version__}, ultralytics {ultralytics.__version__}")
if not ok:
    raise SystemExit("❌ numpy must be 1.x and OpenCV 4.10 (see the pins above)")
EOF
