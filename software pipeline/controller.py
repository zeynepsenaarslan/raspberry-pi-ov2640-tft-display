#!/usr/bin/env python3
# controller.py — Buton + LED + TFT warm-up + cam2tft real-time kontrol

import RPi.GPIO as GPIO
import subprocess
import time
import os
def wait_dev(path, tries=200, delay=0.1):
    for _ in range(tries):
        if os.path.exists(path):
            return True
        time.sleep(delay)
    return False

# Pinler (BCM numaraları)
BTN_CAPTURE = 17   # pin 11
BTN_RESET   = 4    # pin 7

LED_READY   = 5    # pin 29
LED_CAPTURE = 6    # pin 31
LED_ERROR   = 13   # pin 33

# Bu yolları kendi sistemine göre ayarla:
CAM2TFT_PATH   = "/home/admin/cam2tft"
TFT_TEST_PATH  = "/home/admin/tft_ok.py"   # daha önce kullandığın "TFT OK" python dosyası
TFT_PYTHON     = "/home/admin/tftenv/bin/python"

# Arka planda çalışan cam2tft süreci
cam_proc = None


def init_gpio():
    GPIO.setmode(GPIO.BCM)

    # Butonlar: pull-up (boşta iken 1, bastığında 0)
    GPIO.setup(BTN_CAPTURE, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(BTN_RESET,   GPIO.IN, pull_up_down=GPIO.PUD_UP)

    # LED'ler: çıkış
    GPIO.setup(LED_READY,   GPIO.OUT)
    GPIO.setup(LED_CAPTURE, GPIO.OUT)
    GPIO.setup(LED_ERROR,   GPIO.OUT)

    # Başlangıçta her şeyi kapat
    GPIO.output(LED_READY,   0)
    GPIO.output(LED_CAPTURE, 0)
    GPIO.output(LED_ERROR,   0)


def tft_warmup():
    """Ekranı ayağa kaldırmak için TFT test scriptini bir kez çalıştır."""
    if not os.path.exists(TFT_TEST_PATH):
        print(f"UYARI: {TFT_TEST_PATH} bulunamadı, TFT warm-up atlanıyor.")
        return

    print("TFT warm-up: tft_ok.py çalıştırılıyor...")
    try:
        subprocess.run([TFT_PYTHON, TFT_TEST_PATH], check=True)
        print("TFT warm-up tamam.")
    except subprocess.CalledProcessError as e:
        print("UYARI: tft_ok.py hata ile bitti, return code:", e.returncode)


def start_preview():
    global cam_proc

    if cam_proc is not None and cam_proc.poll() is None:
        return

    if not wait_dev("/dev/spidev0.0") or not wait_dev("/dev/spidev0.1") or not wait_dev("/dev/i2c-1"):
        print("SPI/I2C cihazları gelmedi (PCB takılı değil olabilir).")
        GPIO.output(LED_ERROR, 1)
        return

    print("cam2tft (preview) baslatiliyor...")
    try:
        cam_proc = subprocess.Popen(["sudo", CAM2TFT_PATH])
        GPIO.output(LED_READY, 1)
        GPIO.output(LED_CAPTURE, 0)
        GPIO.output(LED_ERROR, 0)
    except Exception as e:
        print("cam2tft baslatma hatasi:", e)
        GPIO.output(LED_ERROR, 1)
        cam_proc = None



def stop_preview():
    """cam2tft sürecini durdur (son kare ekranda kalır)."""
    global cam_proc

    if cam_proc is None:
        return

    if cam_proc.poll() is None:
        print("cam2tft durduruluyor (freeze)...")
        try:
            cam_proc.terminate()
            try:
                cam_proc.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                print("cam2tft takıldı, kill gönderiliyor...")
                cam_proc.kill()
        except Exception as e:
            print("cam2tft durdurma hatası:", e)

    cam_proc = None


def main():
    init_gpio()

    # Başlangıçta TFT warm-up yap
    tft_warmup()

    # Başta canlı preview başlat
    start_preview()

    print("Sistem hazır.")
    print("Canlı önizleme açık.")
    print("Fotoğrafı 10 sn dondurmak için BTN_CAPTURE (GPIO17)'ye bas.")
    print("TFT+kamera warm-up için BTN_RESET (GPIO4)'e bas.")

    try:
        while True:
            # RESET butonu (aktif low)
            if GPIO.input(BTN_RESET) == GPIO.LOW:
                print("Reset butonuna basıldı, preview durdur + TFT warm-up...")
                GPIO.output(LED_READY,   0)
                GPIO.output(LED_CAPTURE, 0)
                GPIO.output(LED_ERROR,   0)

                # Önce cam2tft'yi durdur
                stop_preview()

                # TFT warm-up tekrar
                tft_warmup()

                # Tekrar preview başlat
                start_preview()

                # butonu bırakmasını bekle (debounce)
                while GPIO.input(BTN_RESET) == GPIO.LOW:
                    time.sleep(0.05)

            # CAPTURE butonu (aktif low)
            if GPIO.input(BTN_CAPTURE) == GPIO.LOW:
                print("Capture butonuna basıldı -> 5 sn freeze mod")

                # LED durumları
                GPIO.output(LED_READY,   0)
                GPIO.output(LED_CAPTURE, 1)
                GPIO.output(LED_ERROR,   0)

                # 1) canlı preview'i durdur -> son kare ekranda donmuş kalır
                stop_preview()

                # 2) 10 saniye boyunca görüntü aynı kalsın
                time.sleep(5)

                # 3) tekrar canlı preview'e geç
                GPIO.output(LED_CAPTURE, 0)
                start_preview()

                # Butonu bırakmasını bekle (debounce)
                while GPIO.input(BTN_CAPTURE) == GPIO.LOW:
                    time.sleep(0.05)

            time.sleep(0.02)

    finally:
        print("Çıkılıyor, cam2tft durduruluyor ve GPIO temizleniyor...")
        stop_preview()
        GPIO.cleanup()


if __name__ == "__main__":
    main()
