#include <Arduino.h>
#include <TFT_eSPI.h>
#include <LD2450.h>
#include <string.h>

// Dedicated sensor-polling task lives on core 0
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

LD2450   ld2450;
TFT_eSPI tft = TFT_eSPI();
TFT_eSprite frameSprite = TFT_eSprite(&tft);

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
static constexpr int MAX_TARGETS = 3;

static constexpr int   CX     = SCR_W / 2;
static constexpr int   CY     = RAD_H - 6;
static constexpr float MAX_MM = 6000.0f;
static constexpr float SCALE  = (float)(RAD_H - 10) / MAX_MM;

// LD2450 actual azimuth FOV: ±60° (120° total)
static constexpr float FOV_DEG   = 60.0f;

static const uint16_t C_BG       = 0x0000;
static const uint16_t C_RING_A   = 0x0140;
static const uint16_t C_RING_B   = 0x0360;
static const uint16_t C_SPOKE    = 0x0260;
static const uint16_t C_BASE     = 0x0340;
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

static constexpr uint32_t TARGET_STALE_BRIGHT_MS = 500;
static constexpr uint32_t TARGET_STALE_ORANGE_MS = 1000;
static constexpr uint32_t TARGET_STALE_REMOVE_MS = 3000;
static constexpr uint32_t SENSOR_DEAD_MS = 1500;

struct Target { int id, x, y, speed; uint32_t lastSeen; };

// ---------------------------------------------------------------------------
// Shared between the sensor task (pinned to core 0) and the render code that
// runs in loop() on core 1. The stock sketch did the UART read/parse for
// LD2450 right inside loop(), so a slow or jittery read could stall the
// screen redraw. The S3 (like the original ESP32) has two cores available -
// this moves the sensor I/O off the rendering core entirely.
// ---------------------------------------------------------------------------
static portMUX_TYPE targetMux = portMUX_INITIALIZER_UNLOCKED;
static Target       targetsShared[MAX_TARGETS];
static volatile int  targetCountShared = 0;
static volatile uint32_t sensorHeartbeatShared = 0;

Target targets[MAX_TARGETS];     // local snapshot used only by the render loop
int    targetCount = 0;

// A short, obvious screen-space history for each target. It is intentionally
// simple so the trail cannot disappear because of ring-buffer or age math.
static constexpr int TRAIL_LEN = 42;
static constexpr uint32_t TRAIL_SAMPLE_MS = 65;
struct Pt { int16_t x, y; };
struct Trail {
  int id;              // target ID to track by, not array index
  Pt points[TRAIL_LEN];
  int head = 0;        // next write position in the circular buffer
  int len = 0;
  uint32_t lastSample = 0;
};
Trail trails[MAX_TARGETS];

static constexpr int ARROW_MIN_SPEED_CMS = 5;    // below this, treat as stationary (filters jitter)
static constexpr int ARROW_MAX_SPEED_CMS = 200;  // ~2 m/s, brisk walk — arrow maxes out here
static constexpr int ARROW_LEN_MIN = 12;
static constexpr int ARROW_LEN_MAX = 40;
static const uint16_t C_VEL_ARROW = 0x3D5F; // bright blue, reads well against red/orange targets

inline int16_t decodeCoord(int16_t raw) {
  uint16_t u = (uint16_t)raw;
  return (u & 0x8000) ? (int16_t)(u & 0x7FFF) : -(int16_t)(u & 0x7FFF);
}

static inline uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b) {
  return (uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
}

// FOV base width at sensor horizon (pixels)
static int fovBaseX = 0;

uint16_t targetColorForAge(uint32_t age) {
  if (age < TARGET_STALE_BRIGHT_MS) {
    return rgb565(255, 0, 0);   // bright red
  }
  if (age < TARGET_STALE_ORANGE_MS) {
    return rgb565(255, 140, 0); // orange
  }
  return rgb565(120, 0, 0);     // dark red
}

uint16_t trailColorForLife(float life) {
  life = constrain(life, 0.0f, 1.0f);
  uint8_t r = (uint8_t)(70 + 185 * life);
  uint8_t g = (uint8_t)(10 + 80 * life);
  uint8_t b = (uint8_t)(0 + 20 * life);
  return rgb565(r, g, b);
}

void clearTrail(Trail &trail) {
  trail.id = -1;
  trail.head = 0;
  trail.len = 0;
  trail.lastSample = 0;
}

// Direction from the last two trail points. Screen-space is fine here -
// the radar projection uses the same SCALE for x and y plus a y-flip,
// so angles are preserved; no need to go back to mm space.
bool getHeading(const Trail &trail, float &dx, float &dy) {
  if (trail.len < 2) return false;
  const int lastIdx = (trail.head - 1 + TRAIL_LEN) % TRAIL_LEN;
  const int prevIdx = (lastIdx - 1 + TRAIL_LEN) % TRAIL_LEN;
  const Pt &p1 = trail.points[prevIdx];
  const Pt &p2 = trail.points[lastIdx];
  dx = (float)(p2.x - p1.x);
  dy = (float)(p2.y - p1.y);
  // normalize
  const float len = sqrtf(dx*dx + dy*dy);
  if (len < 1e-3f) return false;
  dx /= len;
  dy /= len;
  return true;
}

void clearAllTrails() {
  for (int i = 0; i < MAX_TARGETS; i++) {
    clearTrail(trails[i]);
  }
}

void pushTrailPoint(Trail &trail, int px, int py) {
  const int lastIdx = (trail.head - 1 + TRAIL_LEN) % TRAIL_LEN;
  if (trail.len > 0) {
    Pt &last = trail.points[lastIdx];
    if (last.x == px && last.y == py) return;
  }

  if (trail.len < TRAIL_LEN) {
    trail.len++;
  }

  trail.points[trail.head] = { (int16_t)px, (int16_t)py };
  trail.head = (trail.head + 1) % TRAIL_LEN;
}

void sampleTrail(Trail &trail, int px, int py, uint32_t now) {
  if (trail.len == 0 || now - trail.lastSample >= TRAIL_SAMPLE_MS) {
    pushTrailPoint(trail, px, py);
    trail.lastSample = now;
  }
}

void drawThickLine(TFT_eSPI &canvas, int x0, int y0, int x1, int y1, uint16_t color, int width) {
  const float w = (float)width;
  canvas.drawWideLine(x0, y0, x1, y1, w, color, C_BG);

  const int radius = max(2, width / 2);
  canvas.fillCircle(x0, y0, radius, color);
  canvas.fillCircle(x1, y1, radius, color);
}

void drawTrail(const Trail &trail, TFT_eSPI &canvas) {
  if (trail.len < 1) return;

  if (trail.len == 1) {
    const int idx = (trail.head - 1 + TRAIL_LEN) % TRAIL_LEN;
    const int x = trail.points[idx].x;
    const int y0 = min(RAD_H - 2, trail.points[idx].y + 22);
    const int y1 = min(RAD_H - 2, trail.points[idx].y + 7);
    const uint16_t color = trailColorForLife(0.8f);
    drawThickLine(canvas, x, y0, x, y1, color, 12);
    canvas.fillCircle(x, y0, 6, trailColorForLife(0.45f));
    return;
  }

  const int oldest = (trail.head - trail.len + TRAIL_LEN) % TRAIL_LEN;
  for (int i = 1; i < trail.len; i++) {
    const float life = (float)i / (float)(trail.len - 1);
    const uint16_t color = trailColorForLife(life);
    const int width = 0 + (int)(10.0f * life);
    const int radius = 3 + (int)(6.0f * life);
    const int idx0 = (oldest + i - 1) % TRAIL_LEN;
    const int idx1 = (oldest + i) % TRAIL_LEN;

    drawThickLine(
      canvas,
      trail.points[idx0].x, trail.points[idx0].y,
      trail.points[idx1].x, trail.points[idx1].y,
      color,
      width);
    canvas.fillCircle(trail.points[idx1].x, trail.points[idx1].y, radius, color);
  }
}

void drawVelocityArrow(TFT_eSPI &canvas, int px, int py, float dx, float dy, int speedCmS) {
  int absSpeed = abs(speedCmS);
  if (absSpeed < ARROW_MIN_SPEED_CMS) return;

  // clamp speed to [min,max]
  int clamped = min(max(absSpeed, ARROW_MIN_SPEED_CMS), ARROW_MAX_SPEED_CMS);
  float t = (float)(clamped - ARROW_MIN_SPEED_CMS) / (float)(ARROW_MAX_SPEED_CMS - ARROW_MIN_SPEED_CMS);
  int len = ARROW_LEN_MIN + (int)(t * (ARROW_LEN_MAX - ARROW_LEN_MIN));

  // dx,dy is a unit vector in screen space; arrow points in that direction
  int x2 = px + (int)roundf(dx * len);
  int y2 = py + (int)roundf(dy * len);

  // shaft
  drawThickLine(canvas, px, py, x2, y2, C_VEL_ARROW, 4);

  // arrow head (two lines)
  // perpendicular for head wings
  float hx = -dy;
  float hy = dx;
  int wing = max(4, len / 4);
  int ax1 = x2 - (int)roundf(dx * wing) + (int)roundf(hx * wing);
  int ay1 = y2 - (int)roundf(dy * wing) + (int)roundf(hy * wing);
  int ax2 = x2 - (int)roundf(dx * wing) - (int)roundf(hx * wing);
  int ay2 = y2 - (int)roundf(dy * wing) - (int)roundf(hy * wing);

  drawThickLine(canvas, x2, y2, ax1, ay1, C_VEL_ARROW, 4);
  drawThickLine(canvas, x2, y2, ax2, ay2, C_VEL_ARROW, 4);
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
    snprintf(buf, sizeof(buf), "%dm", m);
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
}

void drawFrame(TFT_eSPI &canvas, uint32_t now) {
  canvas.fillCircle(CX, CY, 5, C_ORIGIN);
  canvas.fillCircle(CX, CY, 2, TFT_WHITE);

  // Find or create trail for each active target
  for (int i = 0; i < targetCount; i++) {
    int targetId = targets[i].id;
    Trail *trail = nullptr;
    
    // Find existing trail by target ID
    for (int t = 0; t < MAX_TARGETS; t++) {
      if (trails[t].id == targetId) {
        trail = &trails[t];
        break;
      }
    }
    
    // Create new trail if needed
    if (!trail) {
      for (int t = 0; t < MAX_TARGETS; t++) {
        if (trails[t].id == -1) {  // unused slot
          trail = &trails[t];
          trail->id = targetId;
          trail->len = 0;
          trail->lastSample = 0;
          break;
        }
      }
    }
    
    if (!trail) continue;  // no free trail slots
    
    uint32_t age = now - targets[i].lastSeen;
    float fx = (float)targets[i].x;
    float fy = (float)targets[i].y;
    int   px = CX + (int)(fx * SCALE);
    int   py = CY - (int)(fy * SCALE);

    if (px < 2 || px > SCR_W - 2 || py < 2 || py > RAD_H - 2) continue;

    uint16_t tgtColor = targetColorForAge(age);

    sampleTrail(*trail, px, py, now);

    drawTrail(*trail, canvas);

    canvas.drawCircle(px, py, 18, tgtColor);
    canvas.fillCircle(px, py, 11, tgtColor);
    canvas.fillCircle(px, py,  5, C_TGT_HI);

    char buf[16];
    snprintf(buf, sizeof(buf), "T%d", targets[i].id);
    canvas.setTextColor(C_TID, C_BG);
    canvas.setTextFont(1);
    canvas.drawString(buf, px - 5, py - 26);

    float dist = sqrtf(fx * fx + fy * fy) / 1000.0f;
    snprintf(buf, sizeof(buf), "%.1fm", dist);
    canvas.setTextColor(C_DIST, C_BG);
    canvas.drawString(buf, px - 10, py - 16);
    float hx, hy;
    if (getHeading(*trail, hx, hy)) {
      drawVelocityArrow(canvas, px, py, hx, hy, targets[i].speed);
    }
  }
}

void drawInfo(TFT_eSPI &canvas) {
  canvas.setTextFont(1);
  canvas.setTextColor(C_HUD, C_BG);
  canvas.setCursor(4, INFO_Y + 4);
  canvas.print("LD2450  FOV:120");
  canvas.write((char)0xF8);      // ° character via latin-1
  canvas.setTextColor(C_HUD_DIM, C_BG);
  canvas.setCursor(SCR_W - 80, INFO_Y + 4);
  canvas.printf("TGTS: %d", targetCount);

  const int colW = SCR_W / 3;
  for (int i = 0; i < 3; i++) {
    int x = i * colW + 4;
    int y = INFO_Y + 18;
    canvas.fillRect(i * colW, y, colW, SCR_H - y, C_BG);

    if (i < targetCount) {
      float dist = sqrtf((float)targets[i].x * targets[i].x +
                         (float)targets[i].y * targets[i].y) / 1000.0f;
      canvas.setTextColor(C_TID, C_BG);
      canvas.setCursor(x, y + 2);
      canvas.printf("T%d", targets[i].id);
      canvas.setTextColor(C_HUD, C_BG);
      canvas.setCursor(x, y + 14);
      canvas.printf("X=%-6d Y=%-6d", targets[i].x, targets[i].y);
      canvas.setCursor(x, y + 26);
      canvas.printf("D=%.2fm", dist);
    } else {
      canvas.setTextColor(C_HUD_DIM, C_BG);
      canvas.setCursor(x, y + 2);
      canvas.print("--");
      canvas.setCursor(x, y + 14);
      canvas.print("NO TARGET");
    }
  }
}

void drawSensorStatus(TFT_eSPI &canvas, bool sensorAlive) {
  const int statusX = 176;
  const int statusY = INFO_Y + 4;
  const int statusW = 140;
  const int statusH = 10;

  canvas.fillRect(statusX - 2, statusY - 1, statusW, statusH + 3, C_BG);
  canvas.setCursor(statusX, statusY);
  if (sensorAlive) {
    canvas.setTextColor(C_HUD_DIM, C_BG);
    canvas.print("SENSOR OK");
  } else {
    canvas.setTextColor(rgb565(255, 80, 80), C_BG);
    canvas.print("SENSOR OFFLINE");
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
    uint32_t now = millis();

    Target localTargets[MAX_TARGETS];
    int    localCount = 0;
    for (int i = 0; i < found && localCount < MAX_TARGETS; i++) {
      auto t = ld2450.getTarget(i);
      if (!t.valid) continue;
      localTargets[localCount++] = {
        t.id,
        (int)decodeCoord((int16_t)t.x),
        (int)decodeCoord((int16_t)t.y),
        (int)decodeCoord((int16_t)t.speed),   // cm/s, properly sign-decoded
        now
      };
    }

    Target merged[MAX_TARGETS];
    int mergedCount = 0;

    portENTER_CRITICAL(&targetMux);
    for (int i = 0; i < targetCountShared; i++) {
      uint32_t age = now - targetsShared[i].lastSeen;
      if (age < TARGET_STALE_REMOVE_MS) {
        merged[mergedCount++] = targetsShared[i];
      }
    }
    for (int i = 0; i < localCount; i++) {
      bool matched = false;
      for (int j = 0; j < mergedCount; j++) {
        if (merged[j].id == localTargets[i].id) {
          merged[j] = localTargets[i];
          matched = true;
          break;
        }
      }
      if (!matched && mergedCount < MAX_TARGETS) {
        merged[mergedCount++] = localTargets[i];
      }
    }
    memcpy(targetsShared, merged, mergedCount * sizeof(Target));
    targetCountShared = mergedCount;
    sensorHeartbeatShared = now;
    portEXIT_CRITICAL(&targetMux);

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void setup() {
  // Initialize trails with unused (-1) IDs
  clearAllTrails();

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

  frameSprite.setColorDepth(16);
  frameSprite.setAttribute(PSRAM_ENABLE, true);
  if (frameSprite.createSprite(SCR_W, SCR_H) == nullptr) {
    Serial.println("ERROR: frame sprite allocation failed (out of PSRAM/RAM).");
  }

  drawGrid();
  frameSprite.fillSprite(C_BG);
  gridSprite.pushToSprite(&frameSprite, 0, 0);
  drawInfo(frameSprite);
  drawSensorStatus(frameSprite, true);
  frameSprite.pushSprite(0, 0);

  // Sensor polling on core 0; rendering stays in loop() on core 1.
  xTaskCreatePinnedToCore(sensorTask, "LD2450_Task", 4096, NULL, 1, NULL, 0);
}

static bool sensorWasAlive = true;

void loop() {
  const uint32_t now = millis();
  const uint32_t sensorHeartbeat = sensorHeartbeatShared;
  const bool sensorAlive = (sensorHeartbeat != 0) &&
    (now - sensorHeartbeat < SENSOR_DEAD_MS);

  // Pull the latest snapshot published by the sensor task.
  portENTER_CRITICAL(&targetMux);
  memcpy(targets, targetsShared, sizeof(targets));
  targetCount = targetCountShared;
  portEXIT_CRITICAL(&targetMux);

  if (!sensorAlive) {
    targetCount = 0;
    if (sensorWasAlive) {
      clearAllTrails();
    }
  } else if (!sensorWasAlive) {
    clearAllTrails();
  }
  sensorWasAlive = sensorAlive;

  int filteredCount = 0;
  for (int i = 0; i < targetCount; i++) {
    if (now - targets[i].lastSeen < TARGET_STALE_REMOVE_MS) {
      targets[filteredCount++] = targets[i];
    }
  }
  targetCount = filteredCount;

  // Clear trails for targets that no longer exist
  for (int t = 0; t < MAX_TARGETS; t++) {
    if (trails[t].id == -1) continue;  // already unused
    
    bool targetExists = false;
    for (int i = 0; i < targetCount; i++) {
      if (targets[i].id == trails[t].id) {
        targetExists = true;
        break;
      }
    }
    
    if (!targetExists) {
      // Clear this trail
      clearTrail(trails[t]);
    }
  }

  // Compose the whole frame off-screen, then push a single image to the TFT.
  frameSprite.fillSprite(C_BG);
  gridSprite.pushToSprite(&frameSprite, 0, 0);
  drawFrame(frameSprite, now);
  drawInfo(frameSprite);
  drawSensorStatus(frameSprite, sensorAlive);
  frameSprite.pushSprite(0, 0);

vTaskDelay(1);
}
