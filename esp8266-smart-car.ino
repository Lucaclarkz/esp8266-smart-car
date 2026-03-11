#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <Servo.h>

// =====================================================
// ESP8266 NodeMCU Smart Car Web Controller
// Pin map used exactly as requested by user
// =====================================================

// ---------------- WiFi AP ----------------
const char* ssid = "ESP8266_Car";
const char* password = "password123";

// ---------------- Web Server -------------
ESP8266WebServer server(80);
Servo scanServo;

// ---------------- Motor Pins -------------
const uint8_t IN1 = D8;   // GPIO15
const uint8_t IN2 = D7;   // GPIO13
const uint8_t IN3 = D4;   // GPIO2
const uint8_t IN4 = D3;   // GPIO0
const uint8_t ENA = D5;   // GPIO14
const uint8_t ENB = D6;   // GPIO12

// ---------------- Sensor Pins ------------
const uint8_t FRONT_IR = D1;   // GPIO5   (front pair combined)
const uint8_t BACK_IR  = D2;   // GPIO4   (back pair combined)
const uint8_t SERVO_PIN = D0;  // GPIO16

// Ultrasonic on UART pins (as requested)
const uint8_t TRIG_PIN = 3;    // RX = GPIO3
const uint8_t ECHO_PIN = 1;    // TX = GPIO1
// NOTE: We intentionally do NOT use Serial because TX/RX are used for HC-SR04.

// ---------------- State Variables --------
volatile int carSpeed = 180;      // 0..255
volatile bool autoMode = false;
volatile bool servoTestEnabled = false;

String currentMove = "stop";
int servoAngle = 90;
unsigned long lastAutoTick = 0;
unsigned long lastCommandTime = 0;

// ---------------- Auto Mode Parameters ---
const int SAFE_DISTANCE_CM = 22;
const unsigned long AUTO_INTERVAL = 120;

// =====================================================
// Utility Functions
// =====================================================
void setMotorRaw(bool a1, bool a2, bool b1, bool b2, int spdA, int spdB) {
  spdA = constrain(spdA, 0, 255);
  spdB = constrain(spdB, 0, 255);

  analogWrite(ENA, spdA);
  analogWrite(ENB, spdB);

  digitalWrite(IN1, a1);
  digitalWrite(IN2, a2);
  digitalWrite(IN3, b1);
  digitalWrite(IN4, b2);
}

void stopCar() {
  setMotorRaw(LOW, LOW, LOW, LOW, 0, 0);
  currentMove = "stop";
}

void moveForward() {
  setMotorRaw(HIGH, LOW, HIGH, LOW, carSpeed, carSpeed);
  currentMove = "forward";
}

void moveBackward() {
  setMotorRaw(LOW, HIGH, LOW, HIGH, carSpeed, carSpeed);
  currentMove = "backward";
}

void turnLeft() {
  setMotorRaw(LOW, HIGH, HIGH, LOW, carSpeed, carSpeed);
  currentMove = "left";
}

void turnRight() {
  setMotorRaw(HIGH, LOW, LOW, HIGH, carSpeed, carSpeed);
  currentMove = "right";
}

void forwardLeft() {
  setMotorRaw(HIGH, LOW, HIGH, LOW, carSpeed / 2, carSpeed);
  currentMove = "forward-left";
}

void forwardRight() {
  setMotorRaw(HIGH, LOW, HIGH, LOW, carSpeed, carSpeed / 2);
  currentMove = "forward-right";
}

void centerServo() {
  servoAngle = 90;
  scanServo.write(servoAngle);
}

long readDistanceCM() {
  // Clear trigger
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(3);

  // 10us pulse
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long duration = pulseIn(ECHO_PIN, HIGH, 25000UL); // timeout ~25ms
  if (duration == 0) return 999; // no echo => treat as far away

  long distance = (long)(duration * 0.0343 / 2.0);
  if (distance <= 0) distance = 999;
  return distance;
}

bool isFrontBlocked() {
  // Many IR modules output LOW on detection; some output HIGH.
  // Adjust here if your IR board logic is opposite.
  return digitalRead(FRONT_IR) == LOW;
}

bool isBackBlocked() {
  return digitalRead(BACK_IR) == LOW;
}

long scanDirection(int angle) {
  scanServo.write(angle);
  delay(260);               // allow servo to reach position
  long d = readDistanceCM();
  return d;
}

// =====================================================
// Auto Driving Logic
// =====================================================
void autoDriveStep() {
  unsigned long now = millis();
  if (now - lastAutoTick < AUTO_INTERVAL) return;
  lastAutoTick = now;

  bool frontIR = isFrontBlocked();
  bool backIR = isBackBlocked();

  // Keep servo centered during normal running
  if (!servoTestEnabled) {
    if (servoAngle != 90) {
      servoAngle = 90;
      scanServo.write(servoAngle);
      delay(150);
    }
  }

  long centerDist = readDistanceCM();

  // Front danger: obstacle or cliff detection
  if (frontIR || centerDist < SAFE_DISTANCE_CM) {
    stopCar();
    delay(80);

    // Back away unless back IR already says danger
    if (!backIR) {
      moveBackward();
      delay(280);
    } else {
      stopCar();
      delay(80);
    }

    stopCar();
    delay(80);

    // Scan left and right
    long leftDist  = scanDirection(150);
    long rightDist = scanDirection(30);
    centerServo();

    if (leftDist > rightDist && leftDist > SAFE_DISTANCE_CM) {
      turnLeft();
      delay(320);
    } else if (rightDist >= leftDist && rightDist > SAFE_DISTANCE_CM) {
      turnRight();
      delay(320);
    } else {
      // Neither side looks good, reverse more and try turning
      if (!backIR) {
        moveBackward();
        delay(350);
      }
      turnRight();
      delay(380);
    }

    stopCar();
    delay(60);
    return;
  }

  // Back IR detects edge while reversing or confusion -> avoid going backward
  if (backIR && currentMove == "backward") {
    stopCar();
    delay(50);
    moveForward();
    delay(250);
    stopCar();
    return;
  }

  // Normal go forward
  moveForward();
}

// =====================================================
// Web UI
// =====================================================
String htmlPage() {
  String page = R"====(
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>ESP8266 Smart Car</title>
<style>
:root{
  --bg:#e9e9ea;
  --panel:#171717;
  --ring1:#3b3b40;
  --ring2:#232327;
  --txt:#ffffff;
  --accent:#ff9800;
  --green:#19c37d;
  --red:#ef5350;
}
*{box-sizing:border-box;-webkit-tap-highlight-color:transparent}
body{
  margin:0;
  background:var(--bg);
  font-family:Arial,Helvetica,sans-serif;
  color:#111;
  text-align:center;
}
.wrap{
  max-width:430px;
  margin:0 auto;
  padding:18px 14px 32px;
}
h2{
  margin:6px 0 12px;
  font-size:24px;
}
.card{
  background:#fff;
  border-radius:22px;
  padding:16px;
  box-shadow:0 8px 20px rgba(0,0,0,.08);
  margin-bottom:16px;
}
.topbar{
  display:flex;
  justify-content:space-between;
  gap:8px;
  flex-wrap:wrap;
  align-items:center;
}
.badge{
  padding:8px 12px;
  border-radius:999px;
  font-size:14px;
  font-weight:bold;
  background:#f3f3f3;
}
.badge.auto-on{background:#ffe0b2;color:#7a4a00}
.badge.auto-off{background:#eceff1;color:#444}
.status{
  font-size:14px;
  color:#444;
  margin-top:10px;
}
.controller-area{
  display:flex;
  justify-content:center;
  align-items:center;
  gap:18px;
  margin-top:10px;
  flex-wrap:nowrap;
}
.remote{
  position:relative;
  width:290px;
  height:290px;
  border-radius:34px;
  background:linear-gradient(180deg,#2f2f32,#111);
  box-shadow:inset 0 1px 0 rgba(255,255,255,.08), 0 10px 25px rgba(0,0,0,.20);
}
.ring{
  position:absolute;
  inset:28px;
  border-radius:50%;
  background:radial-gradient(circle at 35% 30%, #676772, #3e3e48 65%, #32323a 100%);
  box-shadow:inset 0 2px 6px rgba(255,255,255,.08), inset 0 -5px 12px rgba(0,0,0,.25);
}
.center{
  position:absolute;
  left:50%;
  top:50%;
  width:112px;
  height:112px;
  margin-left:-56px;
  margin-top:-56px;
  border-radius:50%;
  border:none;
  color:#fff;
  font-size:22px;
  font-weight:bold;
  background:radial-gradient(circle at 35% 30%, #2c2c31, #121214);
  box-shadow:0 3px 8px rgba(0,0,0,.35), inset 0 1px 2px rgba(255,255,255,.08);
}
.center.active{outline:3px solid var(--accent)}
.dir{
  position:absolute;
  width:72px;
  height:72px;
  border:none;
  border-radius:50%;
  background:transparent;
  color:#fff;
  font-size:40px;
  font-weight:bold;
}
.dir:active{transform:scale(.97)}
.up{left:50%; top:42px; transform:translateX(-50%)}
.down{left:50%; bottom:42px; transform:translateX(-50%)}
.left{left:34px; top:50%; transform:translateY(-50%)}
.right{right:34px; top:50%; transform:translateY(-50%)}

.speed-box{
  width:78px;
  display:flex;
  flex-direction:column;
  align-items:center;
  gap:8px;
}
.vwrap{
  width:66px;
  height:250px;
  background:#fff;
  border-radius:18px;
  padding:12px 0;
  box-shadow:0 8px 20px rgba(0,0,0,.08);
  display:flex;
  flex-direction:column;
  align-items:center;
  justify-content:center;
}
.speedTitle{
  font-weight:bold;
  font-size:14px;
  color:#444;
}
#speedVal{
  font-size:18px;
  font-weight:bold;
}
input[type=range].vertical{
  writing-mode:bt-lr;
  -webkit-appearance:slider-vertical;
  width:36px;
  height:180px;
}
.section-title{
  font-size:15px;
  font-weight:bold;
  margin-bottom:10px;
}
.row{
  display:flex;
  gap:10px;
  justify-content:center;
  flex-wrap:wrap;
}
.btn{
  border:none;
  border-radius:14px;
  padding:12px 16px;
  font-size:15px;
  font-weight:bold;
  background:#242424;
  color:#fff;
}
.btn.secondary{background:#e0e0e0;color:#111}
.btn.green{background:var(--green)}
.btn.orange{background:var(--accent); color:#111}
.btn.red{background:var(--red)}
.servoSlider{
  width:100%;
  max-width:280px;
}
.small{
  font-size:13px;
  color:#555;
  margin-top:8px;
}
</style>
</head>
<body>
<div class="wrap">
  <h2>ESP8266 Smart Car</h2>

  <div class="card">
    <div class="topbar">
      <div id="modeBadge" class="badge auto-off">AUTO: OFF</div>
      <div id="moveBadge" class="badge">MOVE: STOP</div>
      <div id="distBadge" class="badge">DIST: -- cm</div>
    </div>

    <div class="controller-area">
      <div class="remote">
        <div class="ring"></div>

        <button class="dir up"    onmousedown="holdCmd('forward')" ontouchstart="holdCmd('forward')" onmouseup="sendCmd('stop')" ontouchend="sendCmd('stop')">&#708;</button>
        <button class="dir left"  onmousedown="holdCmd('left')"    ontouchstart="holdCmd('left')"    onmouseup="sendCmd('stop')" ontouchend="sendCmd('stop')">&#706;</button>
        <button id="autoBtn" class="center" onclick="toggleAuto()">AUTO</button>
        <button class="dir right" onmousedown="holdCmd('right')"   ontouchstart="holdCmd('right')"   onmouseup="sendCmd('stop')" ontouchend="sendCmd('stop')">&#707;</button>
        <button class="dir down"  onmousedown="holdCmd('backward')" ontouchstart="holdCmd('backward')" onmouseup="sendCmd('stop')" ontouchend="sendCmd('stop')">&#709;</button>
      </div>

      <div class="speed-box">
        <div class="vwrap">
          <div class="speedTitle">SPEED</div>
          <input id="speed" class="vertical" type="range" min="80" max="255" value="180" oninput="speedLive(this.value)" onchange="setSpeed(this.value)">
          <div id="speedVal">180</div>
        </div>
      </div>
    </div>

    <div class="small">Manual buttons stop automatically when you release.</div>
  </div>

  <div class="card">
    <div class="section-title">Servo Test (Manual Mode)</div>
    <div class="row">
      <button id="servoToggleBtn" class="btn secondary" onclick="toggleServoTest()">Servo Test OFF</button>
      <button class="btn" onclick="setServo(30)">Left</button>
      <button class="btn" onclick="setServo(90)">Center</button>
      <button class="btn" onclick="setServo(150)">Right</button>
    </div>
    <div style="margin-top:12px">
      <input id="servoSlider" class="servoSlider" type="range" min="0" max="180" value="90" oninput="servoLive(this.value)" onchange="setServo(this.value)">
      <div id="servoVal">Servo: 90°</div>
    </div>
    <div class="small">Servo test only works in manual mode. Auto mode uses the servo for scanning.</div>
  </div>
</div>

<script>
let holdTimer = null;

function sendCmd(cmd){
  fetch('/action?cmd=' + encodeURIComponent(cmd))
    .then(r => r.text())
    .then(() => updateState())
    .catch(()=>{});
}
function holdCmd(cmd){
  sendCmd(cmd);
}
document.body.addEventListener('mouseup', ()=>sendCmd('stop'));
document.body.addEventListener('touchend', ()=>sendCmd('stop'));

function setSpeed(v){
  fetch('/action?cmd=speed&value=' + encodeURIComponent(v))
    .then(r => r.text())
    .then(() => updateState())
    .catch(()=>{});
}
function speedLive(v){
  document.getElementById('speedVal').innerText = v;
}
function toggleAuto(){
  fetch('/action?cmd=toggle_auto')
    .then(r => r.text())
    .then(() => updateState())
    .catch(()=>{});
}
function toggleServoTest(){
  fetch('/action?cmd=toggle_servo_test')
    .then(r => r.text())
    .then(() => updateState())
    .catch(()=>{});
}
function setServo(v){
  fetch('/action?cmd=servo&value=' + encodeURIComponent(v))
    .then(r => r.text())
    .then(() => updateState())
    .catch(()=>{});
}
function servoLive(v){
  document.getElementById('servoVal').innerText = 'Servo: ' + v + '°';
}

function updateState(){
  fetch('/state')
    .then(r => r.json())
    .then(s => {
      document.getElementById('speedVal').innerText = s.speed;
      document.getElementById('speed').value = s.speed;
      document.getElementById('servoSlider').value = s.servo;
      document.getElementById('servoVal').innerText = 'Servo: ' + s.servo + '°';
      document.getElementById('moveBadge').innerText = 'MOVE: ' + s.move.toUpperCase();
      document.getElementById('distBadge').innerText = 'DIST: ' + s.distance + ' cm';

      const modeBadge = document.getElementById('modeBadge');
      const autoBtn = document.getElementById('autoBtn');
      if(s.auto){
        modeBadge.innerText = 'AUTO: ON';
        modeBadge.className = 'badge auto-on';
        autoBtn.classList.add('active');
      }else{
        modeBadge.innerText = 'AUTO: OFF';
        modeBadge.className = 'badge auto-off';
        autoBtn.classList.remove('active');
      }

      const servoToggleBtn = document.getElementById('servoToggleBtn');
      servoToggleBtn.innerText = s.servoTest ? 'Servo Test ON' : 'Servo Test OFF';
      servoToggleBtn.className = s.servoTest ? 'btn green' : 'btn secondary';
    })
    .catch(()=>{});
}
setInterval(updateState, 900);
updateState();
</script>
</body>
</html>
)====";
  return page;
}

// =====================================================
// HTTP Handlers
// =====================================================
void handleRoot() {
  server.send(200, "text/html", htmlPage());
}

void handleState() {
  long d = readDistanceCM();

  String json = "{";
  json += "\"auto\":" + String(autoMode ? "true" : "false") + ",";
  json += "\"servoTest\":" + String(servoTestEnabled ? "true" : "false") + ",";
  json += "\"speed\":" + String(carSpeed) + ",";
  json += "\"servo\":" + String(servoAngle) + ",";
  json += "\"distance\":" + String(d) + ",";
  json += "\"move\":\"" + currentMove + "\"";
  json += "}";

  server.send(200, "application/json", json);
}

void handleAction() {
  String cmd = server.arg("cmd");
  String value = server.arg("value");

  lastCommandTime = millis();

  if (cmd == "forward") {
    autoMode = false;
    moveForward();
  }
  else if (cmd == "backward") {
    autoMode = false;
    moveBackward();
  }
  else if (cmd == "left") {
    autoMode = false;
    turnLeft();
  }
  else if (cmd == "right") {
    autoMode = false;
    turnRight();
  }
  else if (cmd == "stop") {
    stopCar();
  }
  else if (cmd == "toggle_auto") {
    autoMode = !autoMode;
    if (!autoMode) stopCar();
    if (autoMode) {
      servoTestEnabled = false;
      centerServo();
    }
  }
  else if (cmd == "speed") {
    int v = value.toInt();
    carSpeed = constrain(v, 0, 255);
    if (currentMove == "forward") moveForward();
    else if (currentMove == "backward") moveBackward();
    else if (currentMove == "left") turnLeft();
    else if (currentMove == "right") turnRight();
  }
  else if (cmd == "toggle_servo_test") {
    if (!autoMode) {
      servoTestEnabled = !servoTestEnabled;
      if (!servoTestEnabled) {
        centerServo();
      }
    }
  }
  else if (cmd == "servo") {
    if (!autoMode && servoTestEnabled) {
      int ang = constrain(value.toInt(), 0, 180);
      servoAngle = ang;
      scanServo.write(servoAngle);
    }
  }

  server.send(200, "text/plain", "OK");
}

void handleNotFound() {
  server.send(404, "text/plain", "Not found");
}

// =====================================================
// Setup / Loop
// =====================================================
void setup() {
  // No Serial.begin() because TX/RX are used for ultrasonic

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  pinMode(FRONT_IR, INPUT);
  pinMode(BACK_IR, INPUT);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  analogWriteRange(255);
  analogWriteFreq(1000);

  stopCar();

  scanServo.attach(SERVO_PIN);
  centerServo();
  delay(300);

  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);

  server.on("/", handleRoot);
  server.on("/state", handleState);
  server.on("/action", handleAction);
  server.onNotFound(handleNotFound);
  server.begin();

  lastCommandTime = millis();
}

void loop() {
  server.handleClient();

  if (autoMode) {
    autoDriveStep();
  }

  // Fail-safe: if manual mode and no command for some time, stop.
  if (!autoMode) {
    if (millis() - lastCommandTime > 1200 && currentMove != "stop") {
      stopCar();
    }
  }
}
