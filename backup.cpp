#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <LD2450.h>

LD2450 ld2450;

const char* ssid     = "RADAR";
const char* password = "12345678";

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// ── Y added with exact same pattern as X ─────────────────────────────────────
struct Target {
  int id;
  int x;
  int y;
};
Target targets[3];
int targetCount = 0;
char jsonBuf[256];

const char* makeJSON() {
  char* p = jsonBuf;
  *p++ = '[';
  for (int i = 0; i < targetCount; i++) {
    if (i) *p++ = ',';
    p += sprintf(p,
      "{\"id\":%d,\"x\":%d,\"y\":%d}",
      targets[i].id,
      targets[i].x,
      targets[i].y
    );
  }
  *p++ = ']';
  *p   = '\0';
  return jsonBuf;
}

// ── GUI ───────────────────────────────────────────────────────────────────────
const char PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html>
<head>
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>LD2450 RADAR</title>
<style>
*{margin:0;padding:0;box-sizing:border-box}
body{background:#000;overflow:hidden;font-family:'Courier New',monospace}
#hud{
  position:fixed;top:0;left:0;right:0;
  display:flex;justify-content:space-between;align-items:center;
  padding:8px 14px;color:#00ff88;font-size:12px;letter-spacing:.08em;
  border-bottom:1px solid #002800;background:rgba(0,0,0,0.65);
  pointer-events:none;
}
.blink{animation:bl 1.2s steps(1) infinite}
@keyframes bl{0%,100%{opacity:1}50%{opacity:0}}
#term{
  position:fixed;bottom:0;right:0;width:260px;
  padding:10px 12px;
  color:#00ff88;font-size:12px;line-height:1.9;letter-spacing:.05em;
  border-top:1px solid #002800;border-left:1px solid #002800;
  background:rgba(0,8,0,0.82);
  pointer-events:none;
  z-index:10;
}
#term .lbl{color:#445544;font-size:10px;margin-bottom:2px}
#term .tx {color:#00cc66}
#term .ty {color:#44ddaa}
#term .td {color:#ffdd44}
#term .tid{color:#ff8888;font-weight:bold}
</style>
</head>
<body>
<canvas id="c"></canvas>
<div id="hud">
  <div><span class="blink" style="display:inline-block;width:7px;height:7px;border-radius:50%;background:#00ff88;margin-right:7px;vertical-align:middle"></span>LD2450 RADAR</div>
  <div id="st" style="color:#ff4444">CONNECTING…</div>
  <div>TARGETS:&nbsp;<span id="tc">0</span></div>
</div>
<div id="term">
  <div class="lbl">── TARGETS ──────────────</div>
  <div id="trows"><span style="color:#224422">NO TARGETS</span></div>
</div>

<script>
const c   = document.getElementById("c");
const ctx = c.getContext("2d");
const st  = document.getElementById("st");
const tc  = document.getElementById("tc");
const tr  = document.getElementById("trows");

let tgts = [], lastGood = [];
const hist = {};         // rolling trail per target id
const MAX  = 6000;       // mm — sensor max range

// ── WebSocket with auto-reconnect ─────────────────────────────────────────────
function connect() {
  const ws = new WebSocket("ws://" + location.hostname + "/ws");

  ws.onopen = () => {
    st.textContent = "ONLINE";
    st.style.color = "#00ff88";
  };
  ws.onclose = () => {
    st.textContent = "RECONNECTING…";
    st.style.color = "#ffaa00";
    setTimeout(connect, 1500);
  };
  ws.onerror = () => {
    st.textContent = "ERROR";
    st.style.color = "#ff4444";
  };

  ws.onmessage = e => {
    try {
      const d = JSON.parse(e.data || "[]");
      if (d.length > 0) lastGood = d;
      tgts = d.length > 0 ? d : lastGood;
    } catch { tgts = lastGood; }

    tc.textContent = tgts.length;

    // terminal panel — one row per target, X / Y / dist clearly labelled
    if (tgts.length === 0) {
      tr.innerHTML = '<span style="color:#224422">NO TARGETS</span>';
    } else {
      tr.innerHTML = tgts.map(t => {
        const dist = (Math.sqrt(t.x*t.x + t.y*t.y) / 1000).toFixed(2);
        return `<div>`
          + `<span class="tid">T${t.id}</span>  `
          + `<span class="lbl">X=</span><span class="tx">${String(t.x).padStart(6)}</span>  `
          + `<span class="lbl">Y=</span><span class="ty">${String(t.y).padStart(6)}</span>  `
          + `<span class="lbl">D=</span><span class="td">${dist}m</span>`
          + `</div>`;
      }).join("");
    }
  };
}
connect();

// ── Radar draw loop ───────────────────────────────────────────────────────────
let sweep = 0;

function draw() {
  const W = c.width  = innerWidth;
  const H = c.height = innerHeight;

  // sensor at bottom-centre; targets appear above (positive Y = forward)
  const cx = W / 2;
  const cy = H * 0.86;

  // px per mm — fit the full MAX range into the available canvas
  const S = Math.min(W * 0.46, H * 0.80) / MAX;

  // ghost-fade instead of full clear → leaves trails
  ctx.fillStyle = "rgba(0,0,0,0.30)";
  ctx.fillRect(0, 0, W, H);

  // ── rotating sweep ────────────────────────────────────────────────────────
  sweep = (sweep + 1.2) % 360;
  const rad = sweep * Math.PI / 180 - Math.PI / 2;   // 0° = straight up

  // trailing sector glow
  ctx.save();
  const a0 = (sweep - 35) * Math.PI / 180 - Math.PI / 2;
  const a1 = sweep       * Math.PI / 180 - Math.PI / 2;
  ctx.beginPath();
  ctx.moveTo(cx, cy);
  ctx.arc(cx, cy, MAX * S, a0, a1);
  ctx.closePath();
  ctx.fillStyle = "rgba(0,255,100,0.055)";
  ctx.fill();
  ctx.restore();

  // sweep line
  ctx.save();
  ctx.strokeStyle = "rgba(0,255,100,0.75)";
  ctx.lineWidth   = 1.5;
  ctx.shadowColor = "#00ff44";
  ctx.shadowBlur  = 8;
  ctx.beginPath();
  ctx.moveTo(cx, cy);
  ctx.lineTo(cx + Math.sin(rad) * MAX * S,
             cy - Math.cos(rad) * MAX * S);
  ctx.stroke();
  ctx.restore();

  // ── range rings (upper half, same arc style as old code's circle) ─────────
  ctx.font      = "10px 'Courier New'";
  ctx.textAlign = "left";
  for (let m = 1; m <= 6; m++) {
    const r = m * 1000 * S;
    ctx.strokeStyle = m % 2 === 0 ? "#0f2a0f" : "#082008";
    ctx.lineWidth   = 1;
    ctx.beginPath();
    ctx.arc(cx, cy, r, Math.PI, 0);   // upper semicircle only
    ctx.stroke();
    ctx.fillStyle = "#1a5a1a";
    ctx.fillText(m + "m", cx + 4, cy - r + 11);
  }

  // ── angle spokes ──────────────────────────────────────────────────────────
  ctx.strokeStyle = "#0a2a0a";
  ctx.lineWidth   = 1;
  for (const deg of [-75,-60,-45,-30,-15,0,15,30,45,60,75]) {
    const a = deg * Math.PI / 180;
    ctx.beginPath();
    ctx.moveTo(cx, cy);
    ctx.lineTo(cx + Math.sin(a) * MAX * S,
               cy - Math.cos(a) * MAX * S);
    ctx.stroke();
  }

  // baseline (X axis)
  ctx.strokeStyle = "#0f380f";
  ctx.lineWidth   = 1;
  ctx.beginPath();
  ctx.moveTo(cx - MAX * S, cy);
  ctx.lineTo(cx + MAX * S, cy);
  ctx.stroke();

  // ── X axis labels (same range marks as old 1-D view) ─────────────────────
  ctx.fillStyle = "#1a4a1a";
  ctx.font      = "9px 'Courier New'";
  ctx.textAlign = "center";
  for (let m = -5; m <= 5; m++) {
    if (m === 0) continue;
    const px = cx + m * 1000 * S;
    ctx.fillText((m > 0 ? "+" : "") + m + "m", px, cy + 13);
  }

  // ── sensor origin dot ─────────────────────────────────────────────────────
  ctx.shadowColor = "#00ff44";
  ctx.shadowBlur  = 22;
  ctx.fillStyle   = "#00ff44";
  ctx.beginPath();
  ctx.arc(cx, cy, 5, 0, Math.PI * 2);
  ctx.fill();
  ctx.shadowBlur  = 0;

  // ── targets ───────────────────────────────────────────────────────────────
  for (const t of tgts) {
    // same Number() guard as original x-only code, now for both axes
    const x = Number(t.x);
    const y = Number(t.y);
    if (!isFinite(x) || !isFinite(y)) continue;

    // X shifts left/right; Y shifts upward (forward depth)
    const px = cx + x * S;
    const py = cy - y * S;
    const id = t.id;

    // rolling trail
    if (!hist[id]) hist[id] = [];
    hist[id].push([px, py]);
    if (hist[id].length > 28) hist[id].shift();

    const h = hist[id];
    for (let i = 1; i < h.length; i++) {
      const a = i / h.length;
      ctx.strokeStyle = `rgba(255,50,50,${a * 0.55})`;
      ctx.lineWidth   = a * 3;
      ctx.beginPath();
      ctx.moveTo(h[i-1][0], h[i-1][1]);
      ctx.lineTo(h[i  ][0], h[i  ][1]);
      ctx.stroke();
    }

    // outer glow ring
    ctx.shadowColor = "#ff2020";
    ctx.shadowBlur  = 20;
    ctx.strokeStyle = "rgba(255,80,80,0.35)";
    ctx.lineWidth   = 1;
    ctx.beginPath();
    ctx.arc(px, py, 18, 0, Math.PI * 2);
    ctx.stroke();

    // filled blob
    ctx.fillStyle = "#ff3333";
    ctx.beginPath();
    ctx.arc(px, py, 10, 0, Math.PI * 2);
    ctx.fill();

    // bright core
    ctx.shadowBlur = 0;
    ctx.fillStyle  = "#ffcccc";
    ctx.beginPath();
    ctx.arc(px, py, 4, 0, Math.PI * 2);
    ctx.fill();

    // label: id + range
    const dist = (Math.sqrt(x*x + y*y) / 1000).toFixed(1);
    ctx.textAlign = "center";
    ctx.fillStyle = "#ff8888";
    ctx.font      = "bold 12px 'Courier New'";
    ctx.fillText("T" + id, px, py - 22);
    ctx.fillStyle = "#ffdd44";
    ctx.font      = "11px 'Courier New'";
    ctx.fillText(dist + "m", px, py - 10);
  }

  requestAnimationFrame(draw);
}
draw();
</script>
</body></html>
)rawliteral";

// ── Arduino setup / loop ──────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Serial2.begin(256000, SERIAL_8N1, 16, 17);
  ld2450.begin(Serial2, true);

  WiFi.softAP(ssid, password);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest* r) {
    r->send_P(200, "text/html", PAGE);
  });

  ws.onEvent([](AsyncWebSocket*, AsyncWebSocketClient*,
                AwsEventType, void*, uint8_t*, size_t) {});
  server.addHandler(&ws);
  server.begin();

  Serial.printf("AP IP: %s\n", WiFi.softAPIP().toString().c_str());
}

void loop() {
  int found = ld2450.read();
  targetCount = 0;

  // LD2450 uses sign-magnitude encoding, NOT two's complement.
  // Bit 15 = sign flag (1 = positive, 0 = negative), bits 0-14 = magnitude.
  // Casting raw bytes straight to int16_t misreads the sign, giving shit
  // values like Y = -32381 instead of Y = +371.
  auto decodeCoord = [](int16_t raw) -> int16_t {
    uint16_t u = (uint16_t)raw;
    int16_t  mag = (int16_t)(u & 0x7FFF);
    return (u & 0x8000) ? mag : -mag;
  };

  for (int i = 0; i < found && i < 3; i++) {
    auto t = ld2450.getTarget(i);
    if (!t.valid) continue;

    int16_t x = decodeCoord((int16_t)t.x);
    int16_t y = decodeCoord((int16_t)t.y);

    targets[targetCount++] = { t.id, x, y };
    Serial.printf("T%d  X=%6d  Y=%6d\n", t.id, x, y);
  }

  ws.textAll(makeJSON());
  ws.cleanupClients();
  delay(50);
}
