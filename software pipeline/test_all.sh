#!/bin/bash

echo "========================================"
echo " MÜHTAS1 - FULL TEST PACK"
echo "========================================"

BASE_DIR=$(pwd)
BUILD_DIR="$BASE_DIR/build"
CAPTURE_DIR="$BASE_DIR/captures"

# 1) Kaynak dosyalar var mı?
echo "[INFO] Kaynak dosyalar kontrol ediliyor..."
[ -f cam2tft.c ] && echo "[PASS] cam2tft.c bulundu" || { echo "[FAIL] cam2tft.c yok"; exit 1; }
[ -f controller.py ] && echo "[PASS] controller.py bulundu" || { echo "[FAIL] controller.py yok"; exit 1; }

# 2) Kernel modülleri
echo "[INFO] SPI / I2C modülleri yükleniyor..."
sudo modprobe spi_bcm2835
sudo modprobe i2c_bcm2835
sudo modprobe i2c_dev
sleep 1

# 3) Device node kontrolü
echo "[INFO] SPI/I2C aygıtları kontrol ediliyor..."
[ -e /dev/spidev0.0 ] && [ -e /dev/spidev0.1 ] && echo "[PASS] SPI aygıtları OK" || echo "[WARN] SPI aygıtları yok"
[ -e /dev/i2c-1 ] && echo "[PASS] I2C OK" || echo "[WARN] I2C yok"

# 4) Build
echo "[INFO] Derleme başlatılıyor..."
mkdir -p "$BUILD_DIR"
gcc -std=c11 -O2 cam2tft.c -o "$BUILD_DIR/cam2tft"

if [ $? -eq 0 ]; then
  echo "[PASS] Build başarılı"
else
  echo "[FAIL] Build hatası"
  exit 1
fi

# 5) Capture klasörü
mkdir -p "$CAPTURE_DIR"
echo "[PASS] Capture klasörü hazır"

# 6) Kısa çalışma testi
echo "[INFO] cam2tft kısa test (2 sn)"
timeout 2s sudo "$BUILD_DIR/cam2tft" >/dev/null 2>&1

if [ -f "$CAPTURE_DIR/frame_0001.raw" ]; then
  echo "[PASS] RAW görüntü üretildi"
else
  echo "[WARN] RAW görüntü oluşmadı"
fi

# 7) Python GPIO testi
echo "[INFO] Python GPIO kontrol..."
python3 - <<EOF
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
GPIO.cleanup()
print("[PASS] RPi.GPIO çalışıyor")
EOF

echo "========================================"
echo " Test paketi tamamlandı"
echo "========================================"
