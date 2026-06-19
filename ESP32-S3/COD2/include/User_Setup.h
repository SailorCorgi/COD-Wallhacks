#define USER_SETUP_INFO "ILI9488_ESP32_S3"

#define ILI9488_DRIVER

// Physical panel resolution
#define TFT_WIDTH  320
#define TFT_HEIGHT 480

// SPI pins
#define TFT_MISO 13
#define TFT_MOSI 11
#define TFT_SCLK 18
#define TFT_CS   15
#define TFT_DC    2
#define TFT_RST   4

// Fonts
#define LOAD_GLCD

// SPI speeds
#define SPI_FREQUENCY      40000000
#define SPI_READ_FREQUENCY 16000000