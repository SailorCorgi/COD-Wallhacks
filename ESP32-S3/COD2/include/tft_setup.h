#ifndef TFT_SETUP_H
#define TFT_SETUP_H

#ifndef USER_SETUP_INFO
#define USER_SETUP_INFO "ILI9488_ESP32_S3"
#endif

#ifndef ILI9488_DRIVER
#define ILI9488_DRIVER
#endif

#ifndef TFT_WIDTH
#define TFT_WIDTH  320
#endif

#ifndef TFT_HEIGHT
#define TFT_HEIGHT 480
#endif

#ifndef TFT_MISO
#define TFT_MISO  13
#endif

#ifndef TFT_MOSI
#define TFT_MOSI  11
#endif

#ifndef TFT_SCLK
#define TFT_SCLK  18
#endif

#ifndef TFT_CS
#define TFT_CS    15
#endif

#ifndef TFT_DC
#define TFT_DC     2
#endif

#ifndef TFT_RST
#define TFT_RST    4
#endif

#ifndef LOAD_GLCD
#define LOAD_GLCD
#endif

#ifndef LOAD_FONT2
#define LOAD_FONT2
#endif

#ifndef LOAD_FONT4
#define LOAD_FONT4
#endif

#ifndef LOAD_FONT6
#define LOAD_FONT6
#endif

#ifndef LOAD_FONT7
#define LOAD_FONT7
#endif

#ifndef LOAD_FONT8
#define LOAD_FONT8
#endif

#ifndef LOAD_GFXFF
#define LOAD_GFXFF
#endif

#ifndef SMOOTH_FONT
#define SMOOTH_FONT
#endif

#ifndef SPI_FREQUENCY
#define SPI_FREQUENCY       27000000
#endif

#ifndef SPI_READ_FREQUENCY
#define SPI_READ_FREQUENCY  16000000
#endif

#ifndef TFT_RGB_ORDER
#define TFT_RGB_ORDER TFT_BGR
#endif

#endif // TFT_SETUP_H