// cam2tft.c — OV2640 (ArduCAM) -> ST7735 1.8" v1.1 (128x160)
// Bağlantılar (Raspberry Pi 4B):
//  SCK  = GPIO11 (Pin 23), MOSI = GPIO10 (Pin 19), MISO = GPIO9 (Pin 21)
//  TFT CS = CE0  (GPIO8,  Pin 24)  -> /dev/spidev0.0
//  CAM CS = CE1  (GPIO7,  Pin 26)  -> /dev/spidev0.1
//  DC(A0) = GPIO25 (Pin 22)
//  RST    = GPIO24 (Pin 18)
//  VCC/GND ortak (ArduCAM 5V, TFT genelde 5V kabul ediyor)

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/spi/spidev.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>

// =================== TUNING MAKROLARI ===================
// ST7735 tarafı ofset
static uint64_t rough_score_rgb565(const uint8_t *p, int w, int h);
#define XOFF            2
#define YOFF            1
// Eksen/renk sırası (MADCTL). 0x68: 90° + BGR bit set (çoğu v1.1'de düzgün)
#define TFT_MADCTL      0x68
// Renk yolu (OV2640): 0x08 -> BGR, 0x00 -> RGB
#define OV2640_COLORREG 0x08
// BYTE_SWAP = 0 -> kameranın gönderdiği 16-biti aynen TFT'ye yaz
#define BYTE_SWAP       0

// SPI hızları
#define SPI_SPEED_TFT   4000000   // 4 MHz
#define SPI_SPEED_CAM   4000000  // 4 MHz

// ========================================================

#define I2C_DEV         "/dev/i2c-1"
#define OV2640_ADDR_7B  0x30

#define SPI_TFT_DEV     "/dev/spidev0.0"   // CE0 (GPIO8, Pin 24)
#define SPI_CAM_DEV     "/dev/spidev0.1"   // CE1 (GPIO7, Pin 26)

#define SPI_BITS        8
#define SPI_MODE        SPI_MODE_0

// TFT pinleri (sysfs)
#define DC_PIN  25
#define RST_PIN 24

// Çözünürlükler
#define CAM_W 320
#define CAM_H 240
#define TFT_W 128
#define TFT_H 160

// ArduCAM register ve komutlar
enum {
  ARDUCHIP_TEST1 = 0x00,
  ARDUCHIP_MODE  = 0x02,
  ARDUCHIP_FIFO  = 0x04,
  ARDUCHIP_TRIG  = 0x41,
  FIFO_CLEAR_MASK = 0x01,
  FIFO_START_MASK = 0x02,
  CAP_DONE_MASK   = 0x08,
  FIFO_SINGLE_READ= 0x3D,
  FIFO_SIZE1 = 0x42, FIFO_SIZE2 = 0x43, FIFO_SIZE3 = 0x44
};

// ----------------- util -----------------
static int wait_exists(const char *path, int tries, int usec_step){
  struct stat st;
  while(tries-- > 0){
    if (stat(path, &st) == 0) return 0;
    usleep(usec_step);
  }
  return -1;
}

// ----------------- sysfs GPIO -----------------
static int gpio_exported(int pin){
  char path[64]; snprintf(path,sizeof(path),"/sys/class/gpio/gpio%d",pin);
  return access(path,F_OK)==0;
}
static void gpio_export(int pin){
  if (gpio_exported(pin)) return;
  int fd=open("/sys/class/gpio/export",O_WRONLY);
  if(fd>=0){ dprintf(fd,"%d",pin); close(fd); usleep(10000); }
}
static void gpio_dir_out(int pin){
  char path[64]; snprintf(path,sizeof(path),"/sys/class/gpio/gpio%d/direction",pin);
  int fd=open(path,O_WRONLY); if(fd>=0){ write(fd,"out",3); close(fd); }
}
static void gpio_write(int pin, int val){
  char path[64]; snprintf(path,sizeof(path),"/sys/class/gpio/gpio%d/value",pin);
  int fd=open(path,O_WRONLY); if(fd>=0){ write(fd,val?"1":"0",1); close(fd); }
}

// ----------------- SPI helpers -----------------
static int spi_open(const char *dev, uint32_t speed){
  int fd=open(dev,O_RDWR);
  if(fd<0){ perror(dev); return -1; }
  uint8_t mode=SPI_MODE, bits=SPI_BITS; uint32_t hz=speed;
  ioctl(fd,SPI_IOC_WR_MODE,&mode);
  ioctl(fd,SPI_IOC_WR_BITS_PER_WORD,&bits);
  ioctl(fd,SPI_IOC_WR_MAX_SPEED_HZ,&hz);
  return fd;
}
static int spi_xfer(int fd, uint8_t *tx, uint8_t *rx, size_t len){
  struct spi_ioc_transfer tr={0};
  tr.tx_buf=(unsigned long)tx; tr.rx_buf=(unsigned long)rx; tr.len=len;
  tr.speed_hz=0; tr.bits_per_word=0;
  return ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
}

// ----------------- ArduCAM reg -----------------
static void ardu_wr(int cam, uint8_t reg, uint8_t val){
  uint8_t tx[2]={ (uint8_t)(0x80|reg), val }, rx[2]={0};
  if (spi_xfer(cam,tx,rx,2)<1) perror("ardu_wr");
}
static uint8_t ardu_rd(int cam, uint8_t reg){
  uint8_t tx[2]={ (uint8_t)(reg&0x7F), 0x00 }, rx[2]={0};
  if (spi_xfer(cam,tx,rx,2)<1) perror("ardu_rd");
  return rx[1];
}
static uint32_t fifo_size(int cam){
  uint32_t s1=ardu_rd(cam,FIFO_SIZE1), s2=ardu_rd(cam,FIFO_SIZE2), s3=ardu_rd(cam,FIFO_SIZE3)&7;
  return (s3<<16)|(s2<<8)|s1;
}


// YAVAŞ AMA GÜVENLİ FIFO OKUMA (SINGLE_READ, byte byte)
static void fifo_read_slow(int cam, uint8_t *buf, uint32_t len){
    for (uint32_t i = 0; i < len; ++i) {
        uint8_t tx[2] = { FIFO_SINGLE_READ, 0x00 };
        uint8_t rx[2] = {0};

        if (spi_xfer(cam, tx, rx, 2) < 1) {
            perror("FIFO_SINGLE_READ");
            exit(1);
        }
        buf[i] = rx[1];   // gelen byte
    }
}

// ----------------- OV2640 init (RGB565/QVGA, OV2640_COLORREG) -----------------
static int i2c_wr(int i2c, uint8_t reg, uint8_t val){
  uint8_t b[2]={reg,val};
  return (write(i2c,b,2)==2)?0:-1;
}
static int i2c_rd(int i2c, uint8_t reg){
  if (write(i2c,&reg,1)!=1) return -1;
  uint8_t v=0; if (read(i2c,&v,1)!=1) return -1; return v;
}
static int ov2640_init_rgb565_qvga(int i2c){
  if (i2c_wr(i2c,0xFF,0x01)) return -1;
  if (i2c_wr(i2c,0x12,0x80)) return -1;
  usleep(50000);

  if (i2c_wr(i2c,0xFF,0x00)) return -1;
  if (i2c_wr(i2c,0xE0,0x04)) return -1;
  if (i2c_wr(i2c,0xDA,0x00)) return -1; // format path
  if (i2c_wr(i2c,0xE0,0x00)) return -1;

  if (i2c_wr(i2c,0xD7,0x03)) return -1;
  if (i2c_wr(i2c,0x33,0xA0)) return -1;
  if (i2c_wr(i2c,0x3C,0x00)) return -1;
  if (i2c_wr(i2c,0xDA,OV2640_COLORREG)) return -1; // 0x08=BGR, 0x00=RGB

  if (i2c_wr(i2c,0xFF,0x00)) return -1;
  if (i2c_wr(i2c,0xE0,0x04)) return -1;
  if (i2c_wr(i2c,0xC0,0x32)) return -1;
  if (i2c_wr(i2c,0xC1,0x25)) return -1;
  if (i2c_wr(i2c,0x86,0x35)) return -1;
  if (i2c_wr(i2c,0x50,0x89)) return -1;
  if (i2c_wr(i2c,0x51,0x90)) return -1;
  if (i2c_wr(i2c,0x52,0x2C)) return -1;
  if (i2c_wr(i2c,0x53,0x00)) return -1;
  if (i2c_wr(i2c,0x54,0x00)) return -1;
  if (i2c_wr(i2c,0x55,0x00)) return -1;
  if (i2c_wr(i2c,0x57,0x00)) return -1;
  if (i2c_wr(i2c,0x5A,0x90)) return -1;
  if (i2c_wr(i2c,0x5B,0x2C)) return -1;
  if (i2c_wr(i2c,0x5C,0x00)) return -1;
  if (i2c_wr(i2c,0xE0,0x00)) return -1;

  if (i2c_wr(i2c,0xFF,0x01)) return -1;
  int pidh=i2c_rd(i2c,0x0A), pidl=i2c_rd(i2c,0x0B);
  printf("OV2640 PID=%02X%02X (RGB565/QVGA)\n", pidh,pidl);
  return 0;
}

// ----------------- ST7735 -----------------
static void tft_cmd(int tft, uint8_t c){
    gpio_write(DC_PIN,0);
    write(tft, &c, 1);
}
static void tft_data(int tft, const uint8_t *d, size_t n){
    gpio_write(DC_PIN,1);
    write(tft, d, n);
}
static void tft_set_window(int tft, uint8_t x0,uint8_t y0,uint8_t x1,uint8_t y1){
    uint8_t p[4];
    tft_cmd(tft,0x2A);  // CASET
    p[0]=0; p[1]=x0+XOFF; p[2]=0; p[3]=x1+XOFF; tft_data(tft,p,4);
    tft_cmd(tft,0x2B);  // RASET
    p[0]=0; p[1]=y0+YOFF; p[2]=0; p[3]=y1+YOFF; tft_data(tft,p,4);
    tft_cmd(tft,0x2C);  // RAMWR
}

static void tft_init(int tft){
    // TFT'nin tam init işini Python (tft_ok.py) yapacak.
    // Burada sadece DC pinini hazırlıyoruz, ekrana mod vermiyoruz.

    gpio_export(DC_PIN);
    gpio_dir_out(DC_PIN);

    // RST'e dokunmuyoruz; Python nasıl bıraktıysa öyle kalsın.
    (void)tft; // kullanılmıyor uyarısı susun diye
}

// ----------------- hizalama skoru -----------------
static uint64_t rough_score_rgb565(const uint8_t *p, int w, int h){
  uint64_t s = 0;

  for (int y = 1; y < h; y += 4) {
    for (int x = 1; x < w; x += 4) {
      int idx = (y * w + x) * 2;
      uint16_t c = ((uint16_t)p[idx+1] << 8) | p[idx];

      int r = (c >> 11) & 0x1F;
      int g = (c >> 5)  & 0x3F;
      int b =  c        & 0x1F;
      int Y = (r*10 + g*20 + b*4);

      int idxl = (y * w + (x-1)) * 2;
      uint16_t cl = ((uint16_t)p[idxl+1] << 8) | p[idxl];
      int rl = (cl >> 11) & 0x1F;
      int gl = (cl >> 5)  & 0x3F;
      int bl =  cl        & 0x1F;
      int Yl = (rl*10 + gl*20 + bl*4);

      int idxt = ((y-1) * w + x) * 2;
      uint16_t ct = ((uint16_t)p[idxt+1] << 8) | p[idxt];
      int rt = (ct >> 11) & 0x1F;
      int gt = (ct >> 5)  & 0x3F;
      int bt =  ct        & 0x1F;
      int Yt = (rt*10 + gt*20 + bt*4);

      int dx = Y - Yl; if (dx < 0) dx = -dx;
      int dy = Y - Yt; if (dy < 0) dy = -dy;

      s += (dx + dy);
    }
  }
  return s;
}

// ----------------- TEK KAREYİ OKU VE TFT'YE BAS (ESKİ SAĞLAM MANTIK) -----------------
static void capture_and_draw(int cam, int tft, uint8_t *raw, uint32_t buf_size){
    // 1) Capture başlat
    ardu_wr(cam, ARDUCHIP_MODE, 0x00);
    usleep(5000);
    ardu_wr(cam, ARDUCHIP_FIFO, FIFO_CLEAR_MASK);  usleep(5000);
    ardu_wr(cam, ARDUCHIP_FIFO, FIFO_START_MASK);  usleep(5000);
    ardu_wr(cam, ARDUCHIP_TRIG, 0x01);

    int waited = 0;
    while (!(ardu_rd(cam, ARDUCHIP_TRIG) & CAP_DONE_MASK)) {
        usleep(5000);
        waited += 5;
        if (waited > 8000) {      // ~40 ms
            puts("Timeout CAP_DONE");
            return;
        }
    }

    // 2) Tüm 320x240 RGB565 frame'i oku
    uint32_t want = CAM_W * CAM_H * 2;  // 320*240*2 = 153600
    uint32_t fsz  = fifo_size(cam);
    if (fsz > want)    fsz = want;
    if (fsz > buf_size) fsz = buf_size;

    fifo_read_slow(cam, raw, fsz);

    // 3) 1-byte hizalama için iki aday: raw ve raw+1
    const uint8_t *buf0 = raw;
    const uint8_t *buf1 = raw + 1;
    uint64_t s0 = rough_score_rgb565(buf0, CAM_W, CAM_H);
    uint64_t s1 = rough_score_rgb565(buf1, CAM_W, CAM_H);
    const uint8_t *best = (s1 > s0) ? buf1 : buf0;
     FILE *f = fopen("/home/admin/captures/frame_0001.raw", "wb");
    if (f) {
        fwrite(best, 1, CAM_W * CAM_H * 2, f);   // 320x240 RGB565
        fclose(f);
    }

    // 4) Ortadan 128x160 crop al ve TFT'ye yaz
    tft_set_window(tft, 0, 0, TFT_W - 1, TFT_H - 1);
    gpio_write(DC_PIN, 1);

    int x0 = (CAM_W - TFT_W) / 2;   // 96
    int y0 = (CAM_H - TFT_H) / 2;   // 40

    for (int y = 0; y < TFT_H; ++y) {
        int sy = y0 + y;
        if (sy < 0)        sy = 0;
        if (sy >= CAM_H)   sy = CAM_H - 1;

        for (int x = 0; x < TFT_W; ++x) {
            int sx = x0 + x;
            if (sx < 0)       sx = 0;
            if (sx >= CAM_W)  sx = CAM_W - 1;

            int idx = (sy * CAM_W + sx) * 2;
            if (idx + 1 >= (int)fsz) {
                uint8_t black[2] = {0, 0};
                write(tft, black, 2);
                continue;
            }

#if BYTE_SWAP
            uint8_t px[2] = { best[idx+1], best[idx] };
#else
            uint8_t px[2] = { best[idx], best[idx+1] };
#endif
            write(tft, px, 2);
        }
    }
}


// ================= MAIN (CONTINUOUS PREVIEW) =================
int main(void){
  system("modprobe spi_bcm2835");
  system("modprobe i2c_bcm2835");
  system("modprobe i2c_dev");
  usleep(100000);

  if (wait_exists(SPI_TFT_DEV, 40, 50000)!=0 || wait_exists(SPI_CAM_DEV, 40, 50000)!=0){
    fprintf(stderr,"SPI cihazları yok (dtparam=spi=on mu?)\n");
    return 1;
  }
  if (wait_exists(I2C_DEV, 40, 50000)!=0){
    fprintf(stderr,"I2C yok: %s (dtparam=i2c_arm=on mu?)\n", I2C_DEV);
    return 1;
  }

  int tft = spi_open(SPI_TFT_DEV, SPI_SPEED_TFT);
  if(tft<0) return 1;
  tft_init(tft);

  int i2c=open(I2C_DEV,O_RDWR);
  if(i2c<0){ perror("open i2c"); return 1; }
  if(ioctl(i2c,I2C_SLAVE,OV2640_ADDR_7B)<0){ perror("I2C_SLAVE"); return 1; }
  if(ov2640_init_rgb565_qvga(i2c)!=0){ puts("OV2640 init FAIL"); return 1; }

  int cam = spi_open(SPI_CAM_DEV, SPI_SPEED_CAM);
  if(cam<0) return 1;

  // SPI test
  ardu_wr(cam,ARDUCHIP_TEST1,0x55);
  if(ardu_rd(cam,ARDUCHIP_TEST1)!=0x55){ puts("SPI test FAIL (ArduCAM)"); return 1; }
  puts("SPI test OK (ArduCAM)");

  uint32_t buf_size = CAM_W*CAM_H*2 + 4;
  uint8_t *raw = (uint8_t*)malloc(buf_size);
  if(!raw){ perror("malloc"); return 1; }

  puts("Preview basliyor (CTRL+C veya process kill ile durur)...");

  while(1){
      capture_and_draw(cam, tft, raw, buf_size);
      // biraz nefes ver, Pi'yi boğmayalım
      usleep(30000); // ~30 ms -> ~30 fps teorik
  }

  free(raw);
  close(cam); close(tft); close(i2c);
  return 0;
}
