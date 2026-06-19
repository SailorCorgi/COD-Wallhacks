// main.cpp — LD2450 radar on ILI9488 TFT (no WiFi, no web server)
#include <Arduino.h>
#include <TFT_eSPI.h>
#include <LD2450.h>

// ── Hardware ──────────────────────────────────────────────────────────────────
LD2450   ld2450;
TFT_eSPI tft = TFT_eSPI();
TFT_eSprite spr = TFT_eSprite(&tft);

// ── Layout (portrait 320 × 480) ──────────────────────────────────────────────
static constexpr int SCR_W  = 320;
static constexpr int SCR_H  = 480;
static constexpr int RAD_H  = 300;
static constexpr int INFO_Y = RAD_H + 2;

static constexpr int   CX    = SCR_W / 2;
static constexpr int   CY    = RAD_H - 6;
static constexpr float MAX_MM = 6000.0f;
static constexpr float SCALE  = 150.0f / MAX_MM;

// ── Colour palette (RGB565) ───────────────────────────────────────────────────
// ILI9488 receives 18-bit over SPI; TFT_eSPI converts RGB565→RGB666 internally.
static const uint16_t C_BG       = 0x0000;
static const uint16_t C_RING_A   = 0x0140;
static const uint16_t C_RING_B   = 0x0360;
static const uint16_t C_SPOKE    = 0x0260;
static const uint16_t C_BASE     = 0x0340;
static const uint16_t C_SWEEP    = 0x04E0;
static const uint16_t C_ORIGIN   = 0x07E0;
static const uint16_t C_LBL_RNG  = 0x0580;
static const uint16_t C_LBL_AX   = 0x0440;
static const uint16_t C_TGT      = 0xF800;
static const uint16_t C_TGT_HI   = 0xFCF3;
static const uint16_t C_TGT_RING = 0xF8A0;
static const uint16_t C_TID      = 0xFC10;
static const uint16_t C_DIST     = 0xFFE0;
static const uint16_t C_HUD      = 0x07E8;
static const uint16_t C_HUD_DIM  = 0x0220;
static const uint16_t C_DIVIDER  = 0x0140;

// ── Target data ───────────────────────────────────────────────────────────────
struct Target { int id, x, y; };
Target targets[3];
int    targetCount = 0;

static constexpr int TRAIL_LEN = 22;
struct Pt { int16_t x, y; };
Pt  trailBuf [3][TRAIL_LEN];
int trailHead[3] = {};
int trailLen [3] = {};

// ── LD2450 sign-magnitude decode ──────────────────────────────────────────────
inline int16_t decodeCoord(int16_t raw) {
  uint16_t u = (uint16_t)raw;
  return (u & 0x8000) ? (int16_t)(u & 0x7FFF) : -(int16_t)(u & 0x7FFF);
}

// ── Sweep state ───────────────────────────────────────────────────────────────
float sweepDeg = 0.0f;

// ── Radar frame ───────────────────────────────────────────────────────────────
void drawFrame() {
  spr.fillSprite(C_BG);

  // Range rings
  for (int m = 1; m <= 6; m++) {
    int r = (int)(m * 1000 * SCALE);
    uint16_t col = (m % 2 == 0) ? C_RING_B : C_RING_A;
    spr.drawArc(CX, CY, r, r - 1, 270, 90, col, C_BG);

    char buf[6];
    sprintf(buf, "%dm", m);
    spr.setTextColor(C_LBL_RNG, C_BG);
    spr.setTextFont(1);
    spr.drawString(buf, CX + 3, CY - r + 2);
  }

  // Angle spokes
  const int spokeDegs[] = {-75,-60,-45,-30,-15, 0, 15,30,45,60,75};
  for (int deg : spokeDegs) {
    float a = deg * DEG_TO_RAD;
    spr.drawLine(CX, CY,
      CX + (int)(sinf(a) * MAX_MM * SCALE),
      CY - (int)(cosf(a) * MAX_MM * SCALE),
      C_SPOKE);
  }

  // Baseline
  int baseX = (int)(MAX_MM * SCALE);
  spr.drawLine(CX - baseX, CY, CX + baseX, CY, C_BASE);

  // X-axis labels
  spr.setTextColor(C_LBL_AX, C_BG);
  for (int m = -5; m <= 5; m++) {
    if (m == 0) continue;
    char buf[5];
    sprintf(buf, "%+dm", m);
    spr.drawString(buf, CX + (int)(m * 1000 * SCALE) - 10, CY + 3);
  }

  // Sweep line
  float radSweep = (sweepDeg - 90.0f) * DEG_TO_RAD;
  int sx = CX + (int)(sinf(radSweep) * MAX_MM * SCALE);
  int sy = CY - (int)(cosf(radSweep) * MAX_MM * SCALE);
  if (sy <= CY) {
    spr.drawLine(CX, CY, sx, sy, C_SWEEP);
    float radT = ((sweepDeg - 15.0f) - 90.0f) * DEG_TO_RAD;
    int tx = CX + (int)(sinf(radT) * MAX_MM * SCALE);
    int ty = CY - (int)(cosf(radT) * MAX_MM * SCALE);
    if (ty <= CY)
      spr.drawLine(CX, CY, tx, ty, spr.color565(0, 80, 30)); // ← was tft.color565
  }

  // Origin dot
  spr.fillCircle(CX, CY, 5, C_ORIGIN);
  spr.fillCircle(CX, CY, 2, TFT_WHITE);

  // Targets
  for (int i = 0; i < targetCount; i++) {
    float fx = (float)targets[i].x;
    float fy = (float)targets[i].y;
    int   px = CX + (int)(fx * SCALE);
    int   py = CY - (int)(fy * SCALE);

    if (px < 2 || px > SCR_W - 2 || py < 2 || py > RAD_H - 2) continue;

    trailBuf [i][trailHead[i]] = { (int16_t)px, (int16_t)py };
    trailHead[i] = (trailHead[i] + 1) % TRAIL_LEN;
    if (trailLen[i] < TRAIL_LEN) trailLen[i]++;

    int tl = trailLen[i];
    for (int j = 1; j < tl; j++) {
      int idx0 = (trailHead[i] - tl + j - 1 + TRAIL_LEN) % TRAIL_LEN;
      int idx1 = (trailHead[i] - tl + j     + TRAIL_LEN) % TRAIL_LEN;
      float alpha = (float)j / tl;
      uint8_t rv  = (uint8_t)(220 * alpha * 0.65f);
      spr.drawLine(
        trailBuf[i][idx0].x, trailBuf[i][idx0].y,
        trailBuf[i][idx1].x, trailBuf[i][idx1].y,
        spr.color565(rv, 0, 0));
    }

    spr.drawCircle(px, py, 12, C_TGT_RING);
    spr.fillCircle(px, py,  7, C_TGT);
    spr.fillCircle(px, py,  3, C_TGT_HI);

    char buf[16];
    sprintf(buf, "T%d", targets[i].id);
    spr.setTextColor(C_TID, C_BG);
    spr.setTextFont(1);
    spr.drawString(buf, px - 5, py - 26);

    float dist = sqrtf(fx * fx + fy * fy) / 1000.0f;
    sprintf(buf, "%.1fm", dist);
    spr.setTextColor(C_DIST, C_BG);
    spr.drawString(buf, px - 10, py - 16);
  }

  spr.pushSprite(0, 0);
}

// ── Info panel ────────────────────────────────────────────────────────────────
void drawInfo() {
  tft.setTextFont(1);
  tft.setTextColor(C_HUD, C_BG);
  tft.setCursor(4, INFO_Y + 4);
  tft.print("LD2450 RADAR");
  tft.setTextColor(C_HUD_DIM, C_BG);
  tft.setCursor(SCR_W - 80, INFO_Y + 4);
  tft.printf("TARGETS: %d", targetCount);

  for (int i = 0; i < 3; i++) {
    int y = INFO_Y + 22 + i * 22;
    tft.fillRect(0, y, SCR_W, 20, C_BG);

    if (i < targetCount) {
      float dist = sqrtf((float)targets[i].x * targets[i].x +
                         (float)targets[i].y * targets[i].y) / 1000.0f;
      tft.setTextColor(C_TID, C_BG);
      tft.setCursor(4, y + 4);
      tft.printf("T%d", targets[i].id);
      tft.setTextColor(C_HUD, C_BG);
      tft.printf("  X=%-6d Y=%-6d D=%.2fm", targets[i].x, targets[i].y, dist);
    } else {
      tft.setTextColor(C_HUD_DIM, C_BG);
      tft.setCursor(4, y + 4);
      tft.print("--  NO TARGET");
    }
  }
}

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Serial2.begin(256000, SERIAL_8N1, 16, 17); // LD2450 on GPIO 16/17
  ld2450.begin(Serial2, true);

  tft.init();
tft.setRotation(0);   // was: tft.setRotation(2)
  tft.fillScreen(C_BG);
  tft.drawFastHLine(0, INFO_Y - 1, SCR_W, C_DIVIDER);
spr.createSprite(SCR_W, RAD_H);
if (!spr.created()) {
  Serial.println("SPRITE FAILED - not enough heap");
  tft.drawString("SPRITE ALLOC FAILED", 10, 10);
  while(1) delay(1000);
}
  spr.setTextFont(1);

  Serial.printf("Free heap: %d\n", ESP.getFreeHeap());

  drawInfo();
}

// ── Loop ──────────────────────────────────────────────────────────────────────
static uint32_t lastRead    = 0;
static uint32_t lastInfoUpd = 0;

void loop() {
  const uint32_t now = millis();

  if (now - lastRead >= 50) {
    lastRead    = now;
    int found   = ld2450.read();
    targetCount = 0;

    for (int i = 0; i < found && i < 3; i++) {
      auto t = ld2450.getTarget(i);
      if (!t.valid) continue;
      targets[targetCount++] = {
        t.id,
        (int)decodeCoord((int16_t)t.x),
        (int)decodeCoord((int16_t)t.y)
      };
    }
  }

  static float sweepDir = 1.5f;
  sweepDeg += sweepDir;
  if (sweepDeg >= 180.0f || sweepDeg <= 0.0f) sweepDir = -sweepDir;

  drawFrame();

  if (now - lastInfoUpd >= 200) {
    lastInfoUpd = now;
    drawInfo();
  }
}