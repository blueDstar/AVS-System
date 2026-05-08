# 🚢 Skill: Docker Deployment & Performance

## 1. Build Strategy
- **Cross-Platform:** Build on Acer Nitro (x86_64) using `docker buildx` for `linux/arm64`.
- **Format:** Export YOLO11n-seg to **NCNN** or **ONNX** for CPU acceleration on RPi5.

## 2. Container Orchestration (Zero-Hardcoding)
Map stable host symlinks to generic container paths:
```yaml
devices:
  - "/dev/ai_camera:/dev/video_source"
  - "/dev/dri:/dev/dri" # Access GPU for Vulkan