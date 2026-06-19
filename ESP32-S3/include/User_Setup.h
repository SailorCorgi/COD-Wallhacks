#define USER_SETUP_INFO "ILI9488_ESP32"

// ── Driver ───────────────────────────────────────────────────────────────────
#define ILI9488_DRIVER     // NOT ST7796 — panel is actually ILI9488

// ── Dimensions ───────────────────────────────────────────────────────────────
#define TFT_WIDTH  320
#define TFT_HEIGHT 480

// ── Pins ─────────────────────────────────────────────────────────────────────
#define TFT_MOSI  23
#define TFT_SCLK  18
// TFT_MISO left undefined — write-only panel
#define TFT_CS    15
#define TFT_DC     2
#define TFT_RST    4
// TOUCH_CS not used
// TFT_BL not defined — backlight wired to 3.3V or always-on

// ── Fonts ────────────────────────────────────────────────────────────────────
#define LOAD_GLCD
#define LOAD_FONT2
#define LOAD_FONT4
#define LOAD_FONT6
#define LOAD_FONT7
#define LOAD_FONT8
#define LOAD_GFXFF
#define SMOOTH_FONT

// ── SPI speed ────────────────────────────────────────────────────────────────
// ILI9488 pushes 18-bit pixels so SPI bandwidth matters more than ST7796.
// 40 MHz is stable on most ESP32 boards; drop to 27 if you see corruption.
#define SPI_FREQUENCY       40000000
#define SPI_READ_FREQUENCY  16000000
#define SPI_TOUCH_FREQUENCY  2500000