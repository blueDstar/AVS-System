#!/bin/bash
# scripts/deploy_to_pi.sh

PI_USER="pi"
PI_HOST="raspi5.local"
PI_DIR="/home/pi/SimpleSysIDV"

# Allow custom IP or Hostname as first argument
if [ -n "$1" ]; then
    PI_HOST="$1"
fi

echo "=== Bắt đầu đồng bộ sang Raspberry Pi ($PI_USER@$PI_HOST) ==="

# Check ping connectivity
if ! ping -c 1 "$PI_HOST" &> /dev/null; then
    echo "WARNING: Không thể ping thấy $PI_HOST. Vui lòng kiểm tra kết nối mạng."
    read -p "Nhập địa chỉ IP trực tiếp của Pi (ví dụ: 192.168.1.15): " INPUT_IP
    if [ -n "$INPUT_IP" ]; then
        PI_HOST="$INPUT_IP"
    else
        echo "Lỗi: Không có địa chỉ IP. Huỷ bỏ."
        exit 1
    fi
fi

# Sync files using rsync
rsync -avz \
  --exclude '.venv' \
  --exclude 'build' \
  --exclude 'install' \
  --exclude 'log' \
  --exclude '.git' \
  --exclude 'ncnn-src' \
  --exclude 'ncnn' \
  --exclude 'test/' \
  --exclude '*.docx' \
  --exclude '*.mp4' \
  --exclude 'config/calibration.json' \
  /home/goln/SimpleSysIDV/ \
  "$PI_USER@$PI_HOST:$PI_DIR/"

echo "=== Đồng bộ hoàn tất! ==="
echo "Các bước tiếp theo cần thực hiện trên Raspberry Pi 5:"
echo "1. ssh $PI_USER@$PI_HOST"
echo "2. cd $PI_DIR"
echo "3. sudo docker load -i avs_perception_arm64.tar"
echo "4. Cấu hình udev camera trong /etc/udev/rules.d/99-usb-camera.rules"
echo "5. Chạy: sudo docker compose -f docker-compose.prod.yml up -d"
