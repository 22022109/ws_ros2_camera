#!/bin/bash

# Create directory
mkdir -p ~/.ros2_pose_models
cd ~/.ros2_pose_models

echo "Downloading OpenPose model files..."

# Download prototxt
if [ ! -f "pose_deploy_linevec.prototxt" ]; then
    echo "Downloading prototxt file..."
    wget https://raw.githubusercontent.com/CMU-Perceptual-Computing-Lab/openpose/master/models/pose/coco/pose_deploy_linevec.prototxt
fi

# Try downloading the caffemodel from CMU
if [ ! -f "pose_iter_440000.caffemodel" ] || [ $(stat -f%z "pose_iter_440000.caffemodel" 2>/dev/null || stat -c%s "pose_iter_440000.caffemodel" 2>/dev/null) -lt 200000000 ]; then
    echo "Downloading model weights (~200MB, this may take a while)..."
    
    # Try multiple sources
    wget http://posefs1.perception.cs.cmu.edu/OpenPose/models/pose/coco/pose_iter_440000.caffemodel || \
    curl -L "https://huggingface.co/onnx-community/openpose/resolve/main/pose_iter_440000.caffemodel" -o pose_iter_440000.caffemodel || \
    echo "Download failed. Please manually download from:"
    echo "  https://github.com/CMU-Perceptual-Computing-Lab/openpose/blob/master/models/getModels.sh"
    echo "  Or visit: http://posefs1.perception.cs.cmu.edu/OpenPose/models/pose/coco/pose_iter_440000.caffemodel"
fi

echo "Files in ~/.ros2_pose_models:"
ls -lh ~/.ros2_pose_models/

echo ""
echo "If the caffemodel file is less than 200MB, the download failed."
echo "You can download manually and place it in: ~/.ros2_pose_models/"
