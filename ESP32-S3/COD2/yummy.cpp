#include <Arduino.h>
#include <TFT_eSPI.h>
#include <LD2450.h>

// Dedicated sensor-polling task lives on core 0 (see notes at bottom of file)
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

LD2450   ld2450;
TFT_eSPI tft = TFT_eSPI();

// The radar grid never changes, so it's cached once into an offscreen sprite
// and just blitted back whenever we need to "wipe" the screen. On the
// original ESP32 a full 480x220 16-bit sprite (~210KB) would have eaten too
// much of the ~280KB of usable internal RAM to be worth it. On the S3's 8MB
// of PSRAM it's free, and TFT_eSPI sprites use PSRAM automatically whenever
// it's present (see setAttribute call in setup()).
TFT_eSprite gridSprite = TFT_eSprite(&tft);

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
static const uint16_t C_TGT_RED  = 0xF800;
static const uint16_t C_TGT_ORG  = 0xFA00;
static const uint16_t C_TGT_DARK = 0x7800;
static const uint16_t C_TGT_HI   = 0xFCF3;
static const uint16_t C_TGT_RING = 0xF8A0;
static const uint16_t C_TID      = 0xFC10;
static const uint16_t C_DIST     = 0xFFE0;
static const uint16_t C_HUD      = 0x07E8;
static const uint16_t C_HUD_DIM  = 0x0220;
static const uint16_t C_DIVIDER  = 0x0140;
static const uint16_t C_FOV      = 0x05E0;   // bright green: FOV boundary lines

struct Target { int id, x, y; };
struct TrackedTarget {
  Target   target;
  Target   prevTarget;
  uint32_t lastSeen;
  uint32_t prevSeen;
  bool     active;
  bool     hasPrev;
  int16_t  prevDrawPx;
  int16_t  prevDrawPy;
  bool     hasPrevDraw;
};

// ---------------------------------------------------------------------------
// Shared between the sensor task (pinned to core 0) and the render code that
// runs in loop() on core 1. The stock sketch did the UART read/parse for
// LD2450 right inside loop(), so a slow or jittery read could stall the
// screen redraw. The S3 (like the original ESP32) has two cores available -
// this moves the sensor I/O off the rendering core entirely.
// ---------------------------------------------------------------------------
static portMUX_TYPE targetMux = portMUX_INITIALIZER_UNLOCKED;
static Target       targetsShared[3];
static volatile int  targetCountShared = 0;

Target targets[3];     // latest sensor snapshot copied by the render loop
int    targetCount = 0;

TrackedTarget displayTargets[3];  // persisted on-screen targets with age fade
int           displayTargetCount = 0;

// More PSRAM/RAM headroom means we can afford a longer comet trail per target.
static constexpr int TRAIL_LEN = 60;   // was 22
struct Pt { int16_t x, y; };
Pt  trailBuf [3][TRAIL_LEN];
int trailHead[3] = {};
int trailLen [3] = {};
uint32_t trailStamp[3] = {};

inline int16_t decodeCoord(int16_t raw) {
  uint16_t u = (uint16_t)raw;
  return (u & 0x8000) ? (int16_t)(u & 0x7FFF) : -(int16_t)(u & 0x7FFF);
}

float sweepDeg = 90.0f;         // start at center (straight ahead)
float sweepDir = 1.5f;          // file-scope so drawFrame can read it for trail direction

// FOV base width at sensor horizon (pixels)
static int fovBaseX = 0;
static constexpr uint32_t INTERP_DELAY_MS = 50;   // render slightly behind sensor updates
static constexpr int      TRAIL_MIN_DELTA = 2;    // ignore tiny jitter when extending trails
static constexpr int      LABEL_CLEAR_W   = 72;   // enough to wipe "T#"/distance text
static constexpr int      LABEL_CLEAR_H   = 28;   // covers the stacked label block
static constexpr int      LABEL_CLEAR_X   = 18;   // anchor box left of the target point
static constexpr int      LABEL_CLEAR_Y   = 32;   // anchor box above the target point

static void clearTargetLabelBox(int px, int py) {
  int x = px - LABEL_CLEAR_X;
  int y = py - LABEL_CLEAR_Y;
  int w = LABEL_CLEAR_W;
  int h = LABEL_CLEAR_H;
  if (x < 0) {
    w += x;
    x = 0;
  }
  if (y < 0) {
    h += y;
    y = 0;
  }
  if (x + w > SCR_W) w = SCR_W - x;
  if (y + h > RAD_H) h = RAD_H - y;
  if (w > 0 && h > 0) {
    tft.fillRect(x, y, w, h, C_BG);
  }
}

void drawGrid() {
  // Draw into the offscreen sprite instead of straight to the TFT.
  gridSprite.fillSprite(C_BG);

  // Precompute FOV base x-extent once (used for base line and label clipping)
  fovBaseX = (int)(sinf(FOV_DEG * DEG_TO_RAD) * MAX_MM * SCALE);

  // Range rings — arc only within ±60° FOV
  // TFT_eSPI drawArc angles: 0=top, clockwise.
  // ±60° from top = 300° to 60° (clockwise through the top).
  for (int m = 1; m <= 6; m++) {
    int r = (int)(m * 1000 * SCALE);
    if (r > RAD_H - 8) break;
    uint16_t col = (m % 2 == 0) ? C_RING_B : C_RING_A;
    gridSprite.drawArc(CX, CY, r, r - 3, 300, 60, col, C_BG);
    char buf[6];
    sprintf(buf, "%dm", m);
    gridSprite.setTextColor(C_LBL_RNG, C_BG);
    gridSprite.setTextFont(1);
    gridSprite.drawString(buf, CX + 3, CY - r + 2);
  }

  // Spokes — only the ones that fall within ±60°; drop ±75° since they're outside FOV
  const int spokeDegs[] = {-45, -30, -15, 0, 15, 30, 45};
  for (int deg : spokeDegs) {
    float a = deg * DEG_TO_RAD;
    gridSprite.drawWideLine(
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
    gridSprite.drawWideLine(
      CX, CY,
      CX + (int)(sinf(a) * MAX_MM * SCALE),
      CY - (int)(cosf(a) * MAX_MM * SCALE),
      3.5,
      C_FOV,
      C_BG
    );
  }

  // Horizontal base line — clipped to the ±60° FOV width
  gridSprite.drawLine(CX - fovBaseX, CY, CX + fovBaseX, CY, C_BASE);

  // X-axis labels — skip any that fall outside the ±60° FOV cone
  gridSprite.setTextColor(C_LBL_AX, C_BG);
  gridSprite.setTextFont(1);
  for (int m = -5; m <= 5; m++) {
    if (m == 0) continue;
    int xOff = (int)(m * 1000 * SCALE);
    if (abs(xOff) > fovBaseX) continue;   // outside FOV, skip
    char buf[6];
    sprintf(buf, "%+dm", m);
    gridSprite.drawString(buf, CX + xOff - 10, CY + 3);
  }

  // Blit the finished grid straight to the screen — this is now a fast
  // memory-to-display copy instead of recomputing every arc/line/label with
  // trig, so we can afford to call it far more often than the original 5s
  // interval to wipe away any lingering target-label smear.
  gridSprite.pushSprite(0, 0);
}

float prevSweep = -999.0f;
float prevTrail = -999.0f;

uint16_t targetColorForAge(uint32_t ageMs) {
  if (ageMs < 200) return C_TGT_RED;
  if (ageMs < 500) return C_TGT_ORG;
  return C_TGT_DARK;
}

void updateDisplayTargets(uint32_t now) {
  // Reclaim stale slots first so new targets reuse dead trails instead of
  // evicting a still-visible target in the same frame.
  for (int i = 0; i < displayTargetCount;) {
    if (!displayTargets[i].active || now - displayTargets[i].lastSeen >= 1000) {
      if (displayTargets[i].hasPrevDraw) {
        clearTargetLabelBox(displayTargets[i].prevDrawPx, displayTargets[i].prevDrawPy);
        displayTargets[i].hasPrevDraw = false;
      }
      for (int j = i; j < displayTargetCount - 1; j++) {
        displayTargets[j] = displayTargets[j + 1];
        memcpy(trailBuf[j], trailBuf[j + 1], sizeof(trailBuf[j]));
        trailHead[j] = trailHead[j + 1];
        trailLen[j] = trailLen[j + 1];
        trailStamp[j] = trailStamp[j + 1];
      }
      displayTargetCount--;
      trailHead[displayTargetCount] = 0;
      trailLen[displayTargetCount] = 0;
      trailStamp[displayTargetCount] = 0;
      continue;
    }
    i++;
  }

  for (int i = 0; i < targetCount; i++) {
    int slot = -1;
    for (int j = 0; j < displayTargetCount; j++) {
      if (displayTargets[j].active && displayTargets[j].target.id == targets[i].id) {
        slot = j;
        break;
      }
    }

    if (slot < 0 && displayTargetCount < 3) {
      slot = displayTargetCount++;
      trailHead[slot] = 0;
      trailLen[slot] = 0;
      trailStamp[slot] = 0;
    } else if (slot < 0) {
      slot = 0;
      for (int j = 1; j < displayTargetCount; j++) {
        if (displayTargets[j].lastSeen < displayTargets[slot].lastSeen) slot = j;
      }
      if (displayTargets[slot].hasPrevDraw) {
        clearTargetLabelBox(displayTargets[slot].prevDrawPx, displayTargets[slot].prevDrawPy);
        displayTargets[slot].hasPrevDraw = false;
      }
      trailHead[slot] = 0;
      trailLen[slot] = 0;
      trailStamp[slot] = 0;
    }

    if (slot >= 0) {
      const bool wasActive = displayTargets[slot].active;
      if (wasActive) {
        displayTargets[slot].prevTarget = displayTargets[slot].target;
        displayTargets[slot].prevSeen = displayTargets[slot].lastSeen;
        displayTargets[slot].hasPrev = true;
      } else {
        displayTargets[slot].prevTarget = targets[i];
        displayTargets[slot].prevSeen = now;
        displayTargets[slot].hasPrev = false;
      }
      displayTargets[slot].target = targets[i];
      displayTargets[slot].lastSeen = now;
      displayTargets[slot].active = true;
    }
  }
}

static void interpolatedTargetPos(const TrackedTarget &tracked, uint32_t now, float &x, float &y) {
  if (!tracked.hasPrev || tracked.lastSeen <= tracked.prevSeen) {
    x = (float)tracked.target.x;
    y = (float)tracked.target.y;
    return;
  }

  const uint32_t renderTime = (now > INTERP_DELAY_MS) ? (now - INTERP_DELAY_MS) : 0;
  if (renderTime <= tracked.prevSeen) {
    x = (float)tracked.prevTarget.x;
    y = (float)tracked.prevTarget.y;
    return;
  }
  if (renderTime >= tracked.lastSeen) {
    x = (float)tracked.target.x;
    y = (float)tracked.target.y;
    return;
  }

  const float span = (float)(tracked.lastSeen - tracked.prevSeen);
  const float t = (float)(renderTime - tracked.prevSeen) / span;
  x = (float)tracked.prevTarget.x + ((float)tracked.target.x - (float)tracked.prevTarget.x) * t;
  y = (float)tracked.prevTarget.y + ((float)tracked.target.y - (float)tracked.prevTarget.y) * t;
}

void drawFrame(uint32_t now) {
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

  for (int i = 0; i < displayTargetCount; i++) {
    const uint32_t ageMs = now - displayTargets[i].lastSeen;
    float fx = 0.0f;
    float fy = 0.0f;
    interpolatedTargetPos(displayTargets[i], now, fx, fy);
    int   px = CX + (int)(fx * SCALE);
    int   py = CY - (int)(fy * SCALE);

    if (displayTargets[i].hasPrevDraw) {
      clearTargetLabelBox(displayTargets[i].prevDrawPx, displayTargets[i].prevDrawPy);
    }

    if (px < 2 || px > SCR_W - 2 || py < 2 || py > RAD_H - 2) {
      displayTargets[i].hasPrevDraw = false;
      continue;
    }

    const int prevIdx = (trailHead[i] - 1 + TRAIL_LEN) % TRAIL_LEN;
    const bool shouldAppendTrail = trailLen[i] == 0 ||
      abs(px - trailBuf[i][prevIdx].x) + abs(py - trailBuf[i][prevIdx].y) >= TRAIL_MIN_DELTA;

    if (shouldAppendTrail) {
      trailBuf[i][trailHead[i]] = { (int16_t)px, (int16_t)py };
      trailHead[i] = (trailHead[i] + 1) % TRAIL_LEN;
      if (trailLen[i] < TRAIL_LEN) trailLen[i]++;
      trailStamp[i] = now;
    }

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

    uint16_t targetColor = targetColorForAge(ageMs);
    tft.drawCircle(px, py, 12, targetColor);
    tft.fillCircle(px, py,  7, targetColor);
    tft.fillCircle(px, py,  3, C_TGT_HI);

    char buf[16];
    sprintf(buf, "T%d", displayTargets[i].target.id);
    tft.setTextColor(targetColor, C_BG);
    tft.setTextFont(1);
    tft.drawString(buf, px - 5, py - 26);

    float dist = sqrtf(fx * fx + fy * fy) / 1000.0f;
    sprintf(buf, "%.1fm", dist);
    tft.setTextColor(C_DIST, C_BG);
    tft.drawString(buf, px - 10, py - 16);

    displayTargets[i].prevDrawPx = (int16_t)px;
    displayTargets[i].prevDrawPy = (int16_t)py;
    displayTargets[i].hasPrevDraw = true;
  }
}

void drawInfo() {
  tft.setTextFont(1);
  tft.setTextColor(C_HUD, C_BG);
  tft.setCursor(4, INFO_Y + 4);
  tft.print("LD2450  FOV:120\xF8");      // °  character via latin-1
  tft.setTextColor(C_HUD_DIM, C_BG);
  tft.setCursor(SCR_W - 80, INFO_Y + 4);
  tft.printf("TGTS: %d", displayTargetCount);

  const int colW = SCR_W / 3;
  for (int i = 0; i < 3; i++) {
    int x = i * colW + 4;
    int y = INFO_Y + 18;
    tft.fillRect(i * colW, y, colW, SCR_H - y, C_BG);

    if (i < displayTargetCount) {
      const Target &target = displayTargets[i].target;
      uint16_t targetColor = targetColorForAge(millis() - displayTargets[i].lastSeen);
      float dist = sqrtf((float)target.x * target.x +
                         (float)target.y * target.y) / 1000.0f;
      tft.setTextColor(targetColor, C_BG);
      tft.setCursor(x, y + 2);
      tft.printf("T%d", target.id);
      tft.setTextColor(C_HUD, C_BG);
      tft.setCursor(x, y + 14);
      tft.printf("X=%-6d Y=%-6d", target.x, target.y);
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

// ---------------------------------------------------------------------------
// Runs on core 0. Polls the LD2450 over UART and parses targets completely
// independently of the render loop, then publishes a snapshot under a
// spinlock. vTaskDelay matches the LD2450's own ~50ms output cadence so this
// task mostly sleeps rather than busy-polling.
// ---------------------------------------------------------------------------
void sensorTask(void *pvParameters) {
  for (;;) {
    int found = ld2450.read();

    Target localTargets[3];
    int    localCount = 0;
    for (int i = 0; i < found && i < 3; i++) {
      auto t = ld2450.getTarget(i);
      if (!t.valid) continue;
      localTargets[localCount++] = {
        t.id,
        (int)decodeCoord((int16_t)t.x),
        (int)decodeCoord((int16_t)t.y)
      };
    }

    portENTER_CRITICAL(&targetMux);
    memcpy(targetsShared, localTargets, sizeof(localTargets));
    targetCountShared = localCount;
    portEXIT_CRITICAL(&targetMux);

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

void setup() {
  Serial.begin(115200);
  Serial.printf("Max alloc heap: %d\n", ESP.getMaxAllocHeap());
  Serial.printf("PSRAM total: %u bytes, free: %u bytes\n",
                ESP.getPsramSize(), ESP.getFreePsram());
  if (!psramFound()) {
    Serial.println("WARNING: PSRAM not detected. Check that PSRAM is "
                    "enabled for this board (Arduino IDE: Tools > PSRAM, "
                    "or board_build.psram_type/psram=enabled in "
                    "platformio.ini) - the grid sprite below will fail "
                    "to allocate without it.");
  }

  Serial2.begin(256000, SERIAL_8N1, 44, 43);
  ld2450.begin(Serial2, true);

  tft.init();
  tft.setRotation(1);
  tft.fillScreen(C_BG);
  tft.drawFastHLine(0, INFO_Y - 1, SCR_W, C_DIVIDER);

  gridSprite.setColorDepth(16);
  gridSprite.setAttribute(PSRAM_ENABLE, true);  // explicit; PSRAM is used by default when present
  if (gridSprite.createSprite(SCR_W, RAD_H) == nullptr) {
    Serial.println("ERROR: grid sprite allocation failed (out of PSRAM/RAM).");
  }

  drawGrid();
  drawInfo();

  // Sensor polling on core 0; rendering stays in loop() on core 1.
  xTaskCreatePinnedToCore(sensorTask, "LD2450_Task", 4096, NULL, 1, NULL, 0);
}

static uint32_t lastInfoUpd = 0;
static uint32_t lastGrid    = 0;

void loop() {
  const uint32_t now = millis();

  // Pull the latest snapshot published by the sensor task.
  portENTER_CRITICAL(&targetMux);
  memcpy(targets, targetsShared, sizeof(targets));
  targetCount = targetCountShared;
  portEXIT_CRITICAL(&targetMux);
  updateDisplayTargets(now);

  // Sweep bounces within the actual ±60° FOV (sweepDeg 30..150)
  sweepDeg += sweepDir;
  if (sweepDeg >= SWEEP_MAX || sweepDeg <= SWEEP_MIN) sweepDir = -sweepDir;

  // Cheap sprite blit now, so we wipe far more often than the original 5s
  // to keep ghost/smeared labels from lingering on screen.
  if (now - lastGrid >= 500) {
    lastGrid = now;
    drawGrid();
  }

  drawFrame(now);

  if (now - lastInfoUpd >= 200) {
    lastInfoUpd = now;
    drawInfo();
  }
}

/* ---------------------------------------------------------------------------
 * What changed for the ESP32-S3 (8MB PSRAM / 16MB flash) and why:
 *
 * 1. Static grid -> PSRAM sprite (gridSprite)
 *    The background grid never changes, so it's rendered once into an
 *    offscreen sprite that lives in PSRAM and is blitted back with a single
 *    fast copy. That let the "wipe" interval drop from 5000ms to 500ms
 *    without extra CPU cost, since recomputing the arcs/lines/labels with
 *    trig every time (the original approach) would be too slow to do that
 *    often. The sweep/target drawing itself is untouched - it's left as
 *    cheap, direct-to-TFT primitives (lines/circles), which stay fast and
 *    don't need a full-screen framebuffer. (A full 480x320 PSRAM sprite
 *    pushed every frame was deliberately avoided - per TFT_eSPI's own
 *    benchmarks a screen that size takes ~60-90ms to push from PSRAM, which
 *    would have made the sweep animation noticeably choppier, not smoother.)
 *
 * 2. LD2450 polling moved to its own task on core 0 (sensorTask)
 *    The original sketch read/parsed the UART data inline in loop(), so any
 *    hiccup in that read could stall the screen redraw. This is now fully
 *    decoupled across the chip's two cores - rendering on core 1 never waits
 *    on sensor I/O.
 *
 * 3. Longer per-target trails (TRAIL_LEN 22 -> 60)
 *    Trivial in terms of memory, but no longer worth rationing.
 *
 * Two things to check in your board config (not in this file):
 *  - PSRAM must be enabled for the board (Arduino IDE: Tools > PSRAM >
 *    "OPI PSRAM" or "QSPI PSRAM" depending on your module; PlatformIO:
 *    board_build.arduino.memory_type / board_build.psram_type).
 *  - To actually use the 16MB flash, set the partition scheme accordingly
 *    (Arduino IDE: Tools > Partition Scheme > a 16MB option; PlatformIO:
 *    board_upload.flash_size = 16MB plus a matching board_build.partitions
 *    csv) - by default many board defs still assume 4MB.
 * ------------------------------------------------------------------------- */
