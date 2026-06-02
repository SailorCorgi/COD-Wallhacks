// Project TFT_eSPI configuration for ST7796S 320x480 display
#define USER_SETUP_INFO "Project User_Setup for ST7796S"

#define ST7796_DRIVER
#define TFT_WIDTH  320
#define TFT_HEIGHT 480
#define TFT_MISO  19
#define TFT_MOSI  23
#define TFT_SCLK  18

// Option 1: Original Arduino wiring used in this sketch
#define TFT_CS    15
#define TFT_DC     2
#define TFT_RST    4

// Option 2: ESPhome-style wiring for this panel
// If your module is wired like the ESPhome test config, uncomment these
// and comment out Option 1 above.
// #define TFT_CS     5
// #define TFT_DC    17
// #define TFT_RST   16

// Many ST7796 panels use BGR pixel order
#define TFT_RGB_ORDER TFT_BGR

// A lower SPI clock is often more reliable for ST7796 panels.
#define SPI_FREQUENCY  20000000
#define SPI_READ_FREQUENCY  16000000
//#define SPI_TOUCH_FREQUENCY  2500000
