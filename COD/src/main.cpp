#include <Arduino.h>
#include <TFT_eSPI.h>
#include <LD2450.h>

LD2450   ld2450;
TFT_eSPI tft = TFT_eSPI();

static constexpr int SCR_W  = 480;
static constexpr int SCR_H  = 320;
static constexpr int RAD_H  = 220;
static constexpr int INFO_Y = RAD_H + 2;

static constexpr int   CX     = SCR_W / 2;
static constexpr int   CY     = RAD_H - 6;
static constexpr float MAX_MM = 6000.0f;
static constexpr float SCALE  = (float)(RAD_H - 10) / MAX_MM;

// LD2450 actual azimuth FOV: ±60° (120° total)
// SWEEP_MIN/MAX are in "sweepDeg" space where 90 = straight ahead
static constexpr float FOV_DEG   = 60.0f;
static constexpr float SWEEP_MIN = 90.0f - FOV_DEG;   // 30°
static constexpr float SWEEP_MAX = 90.0f + FOV_DEG;   // 150°

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
static const uint16_t C_FOV      = 0x05E0;   // bright green: FOV boundary lines

struct Target { int id, x, y; };
Target targets[3];
int    targetCount = 0;

static constexpr int TRAIL_LEN = 22;
struct Pt { int16_t x, y; };
Pt  trailBuf [3][TRAIL_LEN];
int trailHead[3] = {};
int trailLen [3] = {};

inline int16_t decodeCoord(int16_t raw) {
  uint16_t u = (uint16_t)raw;
  return (u & 0x8000) ? (int16_t)(u & 0x7FFF) : -(int16_t)(u & 0x7FFF);
}

float sweepDeg = 90.0f;         // start at center (straight ahead)
float sweepDir = 1.5f;          // file-scope so drawFrame can read it for trail direction

// FOV base width at sensor horizon (pixels)
static int fovBaseX = 0;

void drawGrid() {
  tft.fillRect(0, 0, SCR_W, RAD_H, C_BG);

  // Precompute FOV base x-extent once (used for base line and label clipping)
  fovBaseX = (int)(sinf(FOV_DEG * DEG_TO_RAD) * MAX_MM * SCALE);

  // Range rings — arc only within ±60° FOV
  // TFT_eSPI drawArc angles: 0=top, clockwise.
  // ±60° from top = 300° to 60° (clockwise through the top).
  for (int m = 1; m <= 6; m++) {
    int r = (int)(m * 1000 * SCALE);
    if (r > RAD_H - 8) break;
    uint16_t col = (m % 2 == 0) ? C_RING_B : C_RING_A;
    tft.drawArc(CX, CY, r, r - 3, 300, 60, col, C_BG);
    char buf[6];
    sprintf(buf, "%dm", m);
    tft.setTextColor(C_LBL_RNG, C_BG);
    tft.setTextFont(1);
    tft.drawString(buf, CX + 3, CY - r + 2);
  }

  // Spokes — only the ones that fall within ±60°; drop ±75° since they're outside FOV
  const int spokeDegs[] = {-45, -30, -15, 0, 15, 30, 45};
  for (int deg : spokeDegs) {
    float a = deg * DEG_TO_RAD;
    tft.drawWideLine(
      CX, CY,
      CX + (int)(sinf(a) * MAX_MM * SCALE),
      CY - (int)(cosf(a) * MAX_MM * SCALE),
      2.5,
      C_SPOKE,
      C_BG
    );
  }

  // FOV boundary lines at ±60° — drawn brighter and thicker to mark the real edge
  for (int sign : {-1, 1}) {
    float a = sign * FOV_DEG * DEG_TO_RAD;
    tft.drawWideLine(
      CX, CY,
      CX + (int)(sinf(a) * MAX_MM * SCALE),
      CY - (int)(cosf(a) * MAX_MM * SCALE),
      3.5,
      C_FOV,
      C_BG
    );
  }

  // Horizontal base line — clipped to the ±60° FOV width
  tft.drawLine(CX - fovBaseX, CY, CX + fovBaseX, CY, C_BASE);

  // X-axis labels — skip any that fall outside the ±60° FOV cone
  tft.setTextColor(C_LBL_AX, C_BG);
  tft.setTextFont(1);
  for (int m = -5; m <= 5; m++) {
    if (m == 0) continue;
    int xOff = (int)(m * 1000 * SCALE);
    if (abs(xOff) > fovBaseX) continue;   // outside FOV, skip
    char buf[6];
    sprintf(buf, "%+dm", m);
    tft.drawString(buf, CX + xOff - 10, CY + 3);
  }
}

float prevSweep = -999.0f;
float prevTrail = -999.0f;

void drawFrame() {
  // Erase previous sweep and trail lines
  if (prevSweep >= 0.0f) {
    float r1 = (prevSweep - 90.0f) * DEG_TO_RAD;
    tft.drawLine(CX, CY,
      CX + (int)(sinf(r1) * MAX_MM * SCALE),
      CY - (int)(cosf(r1) * MAX_MM * SCALE), C_BG);
  }
  if (prevTrail >= 0.0f) {
    float r2 = (prevTrail - 90.0f) * DEG_TO_RAD;
    tft.drawLine(CX, CY,
      CX + (int)(sinf(r2) * MAX_MM * SCALE),
      CY - (int)(cosf(r2) * MAX_MM * SCALE), C_BG);
  }

  // Draw new sweep line
  float radSweep = (sweepDeg - 90.0f) * DEG_TO_RAD;
  int sx = CX + (int)(sinf(radSweep) * MAX_MM * SCALE);
  int sy = CY - (int)(cosf(radSweep) * MAX_MM * SCALE);
  if (sy <= CY) {
    tft.drawLine(CX, CY, sx, sy, C_SWEEP);
    prevSweep = sweepDeg;

    // Trail: 15° behind the sweep in its current direction of travel
    float trailDeg = sweepDeg - 15.0f * (sweepDir >= 0 ? 1.0f : -1.0f);
    // Clamp to FOV so we don't draw outside the valid detection zone
    trailDeg = constrain(trailDeg, SWEEP_MIN + 0.5f, SWEEP_MAX - 0.5f);

    float radT = (trailDeg - 90.0f) * DEG_TO_RAD;
    int tx = CX + (int)(sinf(radT) * MAX_MM * SCALE);
    int ty = CY - (int)(cosf(radT) * MAX_MM * SCALE);
    if (ty <= CY) {
      tft.drawLine(CX, CY, tx, ty, tft.color565(0, 80, 30));
      prevTrail = trailDeg;
    }
  }

  tft.fillCircle(CX, CY, 5, C_ORIGIN);
  tft.fillCircle(CX, CY, 2, TFT_WHITE);

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
      tft.drawLine(
        trailBuf[i][idx0].x, trailBuf[i][idx0].y,
        trailBuf[i][idx1].x, trailBuf[i][idx1].y,
        tft.color565(rv, 0, 0));
    }

    tft.drawCircle(px, py, 12, C_TGT_RING);
    tft.fillCircle(px, py,  7, C_TGT);
    tft.fillCircle(px, py,  3, C_TGT_HI);

    char buf[16];
    sprintf(buf, "T%d", targets[i].id);
    tft.setTextColor(C_TID, C_BG);
    tft.setTextFont(1);
    tft.drawString(buf, px - 5, py - 26);

    float dist = sqrtf(fx * fx + fy * fy) / 1000.0f;
    sprintf(buf, "%.1fm", dist);
    tft.setTextColor(C_DIST, C_BG);
    tft.drawString(buf, px - 10, py - 16);
  }
}

void drawInfo() {
  tft.setTextFont(1);
  tft.setTextColor(C_HUD, C_BG);
  tft.setCursor(4, INFO_Y + 4);
  tft.print("LD2450  FOV:120\xF8");      // °  character via latin-1
  tft.setTextColor(C_HUD_DIM, C_BG);
  tft.setCursor(SCR_W - 80, INFO_Y + 4);
  tft.printf("TGTS: %d", targetCount);

  const int colW = SCR_W / 3;
  for (int i = 0; i < 3; i++) {
    int x = i * colW + 4;
    int y = INFO_Y + 18;
    tft.fillRect(i * colW, y, colW, SCR_H - y, C_BG);

    if (i < targetCount) {
      float dist = sqrtf((float)targets[i].x * targets[i].x +
                         (float)targets[i].y * targets[i].y) / 1000.0f;
      tft.setTextColor(C_TID, C_BG);
      tft.setCursor(x, y + 2);
      tft.printf("T%d", targets[i].id);
      tft.setTextColor(C_HUD, C_BG);
      tft.setCursor(x, y + 14);
      tft.printf("X=%-6d Y=%-6d", targets[i].x, targets[i].y);
      tft.setCursor(x, y + 26);
      tft.printf("D=%.2fm", dist);
    } else {
      tft.setTextColor(C_HUD_DIM, C_BG);
      tft.setCursor(x, y + 2);
      tft.print("--");
      tft.setCursor(x, y + 14);
      tft.print("NO TARGET");
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial.printf("Max alloc heap: %d\n", ESP.getMaxAllocHeap());

  Serial2.begin(256000, SERIAL_8N1, 16, 17);
  ld2450.begin(Serial2, true);

  tft.init();
  tft.setRotation(1);
  tft.fillScreen(C_BG);
  tft.drawFastHLine(0, INFO_Y - 1, SCR_W, C_DIVIDER);

  drawGrid();
  drawInfo();
}

static uint32_t lastRead    = 0;
static uint32_t lastInfoUpd = 0;
static uint32_t lastGrid    = 0;

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

  // Sweep bounces within the actual ±60° FOV (sweepDeg 30..150)
  sweepDeg += sweepDir;
  if (sweepDeg >= SWEEP_MAX || sweepDeg <= SWEEP_MIN) sweepDir = -sweepDir;

  if (now - lastGrid >= 5000) {
    lastGrid = now;
    drawGrid();
  }

  drawFrame();

  if (now - lastInfoUpd >= 200) {
    lastInfoUpd = now;
    drawInfo();
  }
}