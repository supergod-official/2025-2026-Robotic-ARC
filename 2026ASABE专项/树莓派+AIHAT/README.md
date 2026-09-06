# YOLOv8 Dataset and Model by YuminZhou

## 📖 Introduction
本项目基于 YOLOv8 用于 Origin Zero 视觉模块开发。模型在 A100 上进行训练，并针对边缘计算设备 **树莓派 AI HAT+** 进行了跨架构部署（ONNX 转 HEF）。

## 📂 Dataset Access
你可以通过 [Hugging Face](https://huggingface.co/datasets/supermin11/2026ASABE-yolov8s) 获取全部训练图片和模型文件。
*   **图片数量**: 约1200张
*   **类别**: 双珠(1黄1绿)、单株(绿)、空株
*   **格式**: [YOLO txt ]
*   **模型文件**: yolov8s.hef yolov8n.hef yolov8s.onnx yolov8n.onnx

> "Interest is all you need."