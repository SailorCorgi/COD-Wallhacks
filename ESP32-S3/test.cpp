#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <LD2450.h>

LD2450 ld2450;

const char* ssid = "RADAR";
const char* password = "12345678";

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

struct Target {
  int id;
  int x;
};

Target targets[3];
int targetCount = 0;

char jsonBuf[128];

const char* makeJSON() {
  char* p = jsonBuf;
  *p++ = '[';

  for (int i = 0; i < targetCount; i++) {
    if (i) *p++ = ',';

    p += sprintf(p,
      "{\"id\":%d,\"x\":%d}",
      targets[i].id,
      targets[i].x
    );
  }

  *p++ = ']';
  *p = '\0';

  return jsonBuf;
}

const char PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<body style="margin:0;background:black;overflow:hidden;">
<canvas id="c"></canvas>

<script>
const c = document.getElementById("c");
const ctx = c.getContext("2d");

let targets = [];
let lastGood = [];

const ws = new WebSocket("ws://" + location.hostname + "/ws");

ws.onmessage = e => {
  try {
    const data = JSON.parse(e.data || "[]");

    // NEVER lose last valid frame
    if (data.length > 0) lastGood = data;
    targets = data.length > 0 ? data : lastGood;

  } catch {
    targets = lastGood;
  }
};

function draw(){
  const W = c.width = innerWidth;
  const H = c.height = innerHeight;

  const cx = W/2;
  const cy = H/2;

  ctx.fillStyle = "rgba(0,0,0,0.25)";
  ctx.fillRect(0,0,W,H);

  // center
  ctx.fillStyle = "blue";
  ctx.beginPath();
  ctx.arc(cx,cy,6,0,Math.PI*2);
  ctx.fill();

  // BIG SCALE so you CANNOT miss it
  const scale = 1.5;

  for (const t of targets) {

    const x = Number(t.x);

    if (!isFinite(x)) continue;

    const px = cx + x * scale;

    ctx.fillStyle = "red";
    ctx.beginPath();
    ctx.arc(px, cy, 14, 0, Math.PI*2);
    ctx.fill();

    ctx.fillStyle = "white";
    ctx.fillText("T"+t.id, px+10, cy);
  }

  requestAnimationFrame(draw);
}
draw();
</script>
</body>
</html>
)rawliteral";

void setup() {
  Serial.begin(115200);

  Serial2.begin(256000, SERIAL_8N1, 16, 17);
  ld2450.begin(Serial2, true);

  WiFi.softAP(ssid, password);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest* r){
    r->send_P(200, "text/html", PAGE);
  });

  ws.onEvent([](AsyncWebSocket*, AsyncWebSocketClient*, AwsEventType, void*, uint8_t*, size_t){});
  server.addHandler(&ws);
  server.begin();
}

void loop() {
  int found = ld2450.read();

  targetCount = 0;

  for (int i = 0; i < found && i < 3; i++) {
    auto t = ld2450.getTarget(i);
    if (!t.valid) continue;

    targets[targetCount++] = { t.id, t.x };

    Serial.printf("X=%d\n", t.x);
  }

  ws.textAll(makeJSON());
  ws.cleanupClients();

  delay(50);
}