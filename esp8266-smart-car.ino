#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <Servo.h>

// =====================================================
// ESP8266 NodeMCU Smart Car
// Manual mode  -> sensors NOT used
// Auto mode    -> Front/Back cliff IR + Ultrasonic + Servo scan
// =====================================================

// ---------------- WiFi AP ----------------
const char* ssid = "ESP8266_Car";
const char* password = "password123";

// ---------------- Server / Servo ---------
ESP8266WebServer server(80);
Servo myServo;

// ---------------- Pin Mapping ------------
const uint8_t IN1 = D8;   // GPIO15
const uint8_t IN2 = D7;   // GPIO13
const uint8_t IN3 = D4;   // GPIO2
const uint8_t IN4 = D3;   // GPIO0
const uint8_t ENA = D5;   // GPIO14
const uint8_t ENB = D6;   // GPIO12

const uint8_t FRONT_IR = D1;   // Front left+right IR combined
const uint8_t BACK_IR  = D2;   // Back left+right IR combined

const uint8_t SERVO_PIN = D0;  // GPIO16

// User requested mapping
const uint8_t TRIG_PIN = 3;    // RX / GPIO3
const uint8_t ECHO_PIN = 1;    // TX / GPIO1

// ---------------- Runtime State ----------
int carSpeed = 210;
bool autoMode = false;
bool servoTestEnabled = false;
String currentMove = "stop";
int servoAngle = 90;
long lastDistance = 999;
unsigned long lastCommandTime = 0;
unsigned long lastAutoTick = 0;

// ---------------- Tuning -----------------
const int SAFE_DISTANCE = 24;
const int SERVO_LEFT = 160;
const int SERVO_CENTER = 90;
const int SERVO_RIGHT = 20;

const int BACKUP_TIME_SHORT = 260;
const int BACKUP_TIME_LONG  = 420;
const int TURN_TIME_SHORT   = 320;
const int TURN_TIME_LONG    = 420;

// IMPORTANT:
// If your IR module outputs HIGH when edge/cliff is detected,
// change LOW to HIGH.
const int IR_ACTIVE_STATE = LOW;

// =====================================================
// Motor control
// =====================================================
void setMotor(bool a1, bool a2, bool b1, bool b2, int spdA, int spdB) {
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
  setMotor(LOW, LOW, LOW, LOW, 0, 0);
  currentMove = "stop";
}

void moveForward() {
  setMotor(HIGH, LOW, HIGH, LOW, carSpeed, carSpeed);
  currentMove = "forward";
}

void moveBackward() {
  setMotor(LOW, HIGH, LOW, HIGH, carSpeed, carSpeed);
  currentMove = "backward";
}

void turnLeft() {
  setMotor(LOW, HIGH, HIGH, LOW, carSpeed, carSpeed);
  currentMove = "left";
}

void turnRight() {
  setMotor(HIGH, LOW, LOW, HIGH, carSpeed, carSpeed);
  currentMove = "right";
}

// =====================================================
// Servo
// =====================================================
void setServoSafe(int angle) {
  angle = constrain(angle, 10, 170);
  servoAngle = angle;
  myServo.write(servoAngle);
}

void centerServo() {
  setServoSafe(SERVO_CENTER);
}

// =====================================================
// Sensors
// =====================================================
bool frontCliffDetected() {
  return digitalRead(FRONT_IR) == IR_ACTIVE_STATE;
}

bool backCliffDetected() {
  return digitalRead(BACK_IR) == IR_ACTIVE_STATE;
}

long readDistanceCM() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(3);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long duration = pulseIn(ECHO_PIN, HIGH, 26000UL);
  if (duration == 0) {
    lastDistance = 999;
    return 999;
  }

  long dist = (long)(duration * 0.0343 / 2.0);
  if (dist <= 0) dist = 999;
  lastDistance = dist;
  return dist;
}

long scanAt(int angle) {
  setServoSafe(angle);
  delay(350);  // D0 servo may need more settling time

  long d1 = readDistanceCM();
  delay(40);
  long d2 = readDistanceCM();

  if (d1 == 999 && d2 == 999) return 999;
  if (d1 == 999) return d2;
  if (d2 == 999) return d1;
  return (d1 + d2) / 2;
}

// =====================================================
// Auto Mode Logic
// IR sensors are ONLY used in auto mode
// =====================================================
void chooseTurnByScan() {
  long leftDist  = scanAt(SERVO_LEFT);
  long rightDist = scanAt(SERVO_RIGHT);
  centerServo();
  delay(60);

  if (leftDist > rightDist) {
    turnLeft();
    delay(TURN_TIME_SHORT);
  } else {
    turnRight();
    delay(TURN_TIME_SHORT);
  }

  stopCar();
  delay(60);
}

void obstacleAvoidRoutine() {
  stopCar();
  delay(80);

  // Reverse only if rear cliff is NOT detected
  if (!backCliffDetected()) {
    moveBackward();
    delay(BACKUP_TIME_SHORT);
    stopCar();
    delay(80);
  }

  long leftDist   = scanAt(SERVO_LEFT);
  long centerDist = scanAt(SERVO_CENTER);
  long rightDist  = scanAt(SERVO_RIGHT);

  centerServo();
  delay(60);

  if (leftDist > rightDist && leftDist > SAFE_DISTANCE) {
    turnLeft();
    delay(TURN_TIME_SHORT);
  } 
  else if (rightDist >= leftDist && rightDist > SAFE_DISTANCE) {
    turnRight();
    delay(TURN_TIME_SHORT);
  } 
  else if (centerDist > SAFE_DISTANCE) {
    moveForward();
    delay(180);
  } 
  else {
    // Tight area
    if (!backCliffDetected()) {
      moveBackward();
      delay(BACKUP_TIME_LONG);
      stopCar();
      delay(80);
    }
    turnRight();
    delay(TURN_TIME_LONG);
  }

  stopCar();
  delay(60);
}

void frontCliffRoutine() {
  stopCar();
  delay(80);

  // If rear is safe, reverse first
  if (!backCliffDetected()) {
    moveBackward();
    delay(340);
    stopCar();
    delay(80);
  }

  chooseTurnByScan();
}

void rearCliffWhileBackingRoutine() {
  stopCar();
  delay(60);
  moveForward();
  delay(220);
  stopCar();
  delay(60);

  chooseTurnByScan();
}

void autoDriving() {
  if (millis() - lastAutoTick < 80) return;
  lastAutoTick = millis();

  bool frontCliff = frontCliffDetected();
  bool rearCliff  = backCliffDetected();
  long dist = readDistanceCM();

  // 1) Front edge danger
  if (frontCliff) {
    frontCliffRoutine();
    return;
  }

  // 2) Front obstacle by ultrasonic
  if (dist < SAFE_DISTANCE) {
    obstacleAvoidRoutine();
    return;
  }

  // 3) If somehow backing and rear cliff appears
  if (rearCliff && currentMove == "backward") {
    rearCliffWhileBackingRoutine();
    return;
  }

  // 4) Normal forward drive
  centerServo();
  moveForward();
}

// =====================================================
// Web UI
// =====================================================
String htmlPage() {
  return R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1, user-scalable=no">
<title>ESP8266 Smart Car</title>
<style>
:root{
  --bg:#ececec;
  --panel:#171717;
  --ring:#4b4b52;
  --txt:#fff;
  --orange:#f5a623;
  --green:#1db954;
}
*{box-sizing:border-box;-webkit-tap-highlight-color:transparent}
body{
  margin:0;
  background:var(--bg);
  font-family:Arial,Helvetica,sans-serif;
  text-align:center;
}
.wrap{
  max-width:430px;
  margin:0 auto;
  padding:16px 12px 28px;
}
.title{
  font-size:26px;
  font-weight:bold;
  margin:6px 0 14px;
}
.card{
  background:#fff;
  border-radius:22px;
  padding:14px;
  margin-bottom:14px;
  box-shadow:0 8px 20px rgba(0,0,0,.08);
}
.toprow{
  display:flex;
  gap:8px;
  justify-content:center;
  flex-wrap:wrap;
  margin-bottom:10px;
}
.badge{
  background:#f2f2f2;
  padding:8px 12px;
  border-radius:999px;
  font-size:13px;
  font-weight:bold;
}
.layout{
  display:flex;
  justify-content:center;
  align-items:center;
  gap:18px;
}
.remote{
  position:relative;
  width:290px;
  height:290px;
  border-radius:34px;
  background:linear-gradient(180deg,#2b2b2f,#111);
  box-shadow:0 12px 24px rgba(0,0,0,.18);
}
.ring{
  position:absolute;
  inset:26px;
  border-radius:50%;
  background:radial-gradient(circle at 30% 25%, #767682, #4d4d56 58%, #34343b 100%);
  box-shadow:inset 0 2px 6px rgba(255,255,255,.10), inset 0 -6px 12px rgba(0,0,0,.25);
}
.btnDir{
  position:absolute;
  width:68px;
  height:68px;
  border:none;
  border-radius:50%;
  background:transparent;
  color:#fff;
  font-size:42px;
  font-weight:bold;
}
.btnDir:active{transform:scale(.96)}
.up{left:50%; top:26px; transform:translateX(-50%)}
.down{left:50%; bottom:26px; transform:translateX(-50%)}
.left{left:16px; top:50%; transform:translateY(-50%)}
.right{right:16px; top:50%; transform:translateY(-50%)}

.autoBtn{
  position:absolute;
  left:50%;
  top:50%;
  width:82px;
  height:82px;
  margin-left:-41px;
  margin-top:-41px;
  border:none;
  border-radius:50%;
  background:radial-gradient(circle at 35% 30%, #2a2a2f, #121214);
  color:#fff;
  font-size:18px;
  font-weight:bold;
  box-shadow:0 2px 8px rgba(0,0,0,.35);
}
.autoBtn.on{
  outline:3px solid var(--orange);
}

.speedWrap{
  width:78px;
  background:#fff;
  border-radius:18px;
  padding:10px 0;
  box-shadow:0 8px 20px rgba(0,0,0,.08);
}
.speedTitle{font-size:13px;font-weight:bold;color:#444;margin-bottom:6px}
#speedVal{font-size:20px;font-weight:bold;margin-top:4px}
input[type=range].vertical{
  writing-mode:bt-lr;
  -webkit-appearance:slider-vertical;
  width:36px;
  height:190px;
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
  background:#202020;
  color:#fff;
  font-size:14px;
  font-weight:bold;
}
.btn.gray{background:#e0e0e0;color:#111}
.btn.green{background:var(--green)}
.slider{
  width:100%;
  max-width:280px;
}
.note{
  font-size:12px;
  color:#666;
  margin-top:8px;
}
</style>
</head>
<body>
<div class="wrap">
  <div class="title">ESP8266 Smart Car</div>

  <div class="card">
    <div class="toprow">
      <div id="modeBadge" class="badge">AUTO: OFF</div>
      <div id="moveBadge" class="badge">MOVE: STOP</div>
      <div id="distBadge" class="badge">DIST: -- cm</div>
    </div>

    <div class="layout">
      <div class="remote">
        <div class="ring"></div>

        <button class="btnDir up" onmousedown="hold('forward')" ontouchstart="hold('forward')" onmouseup="send('stop')" ontouchend="send('stop')">&#708;</button>
        <button class="btnDir left" onmousedown="hold('left')" ontouchstart="hold('left')" onmouseup="send('stop')" ontouchend="send('stop')">&#706;</button>
        <button id="autoBtn" class="autoBtn" onclick="send('toggle_auto')">AUTO</button>
        <button class="btnDir right" onmousedown="hold('right')" ontouchstart="hold('right')" onmouseup="send('stop')" ontouchend="send('stop')">&#707;</button>
        <button class="btnDir down" onmousedown="hold('backward')" ontouchstart="hold('backward')" onmouseup="send('stop')" ontouchend="send('stop')">&#709;</button>
      </div>

      <div class="speedWrap">
        <div class="speedTitle">SPEED</div>
        <input id="speed" class="vertical" type="range" min="100" max="255" value="210" oninput="speedLive(this.value)" onchange="setSpeed(this.value)">
        <div id="speedVal">210</div>
      </div>
    </div>
    <div class="note">Manual mode ignores IR and ultrasonic sensors. Auto mode uses them.</div>
  </div>

  <div class="card">
    <div style="font-weight:bold;margin-bottom:10px;">Servo Test (Manual Mode)</div>
    <div class="row">
      <button id="servoTestBtn" class="btn gray" onclick="send('toggle_servo_test')">Servo Test OFF</button>
      <button class="btn" onclick="setServo(20)">Left</button>
      <button class="btn" onclick="setServo(90)">Center</button>
      <button class="btn" onclick="setServo(160)">Right</button>
    </div>
    <div style="margin-top:12px;">
      <input id="servoSlider" class="slider" type="range" min="10" max="170" value="90" oninput="servoLive(this.value)" onchange="setServo(this.value)">
      <div id="servoVal" style="margin-top:8px;font-weight:bold;">Servo: 90°</div>
    </div>
    <div class="note">Servo test only works in manual mode. Auto mode uses servo scan.</div>
  </div>
</div>

<script>
function send(cmd){
  fetch('/action?cmd=' + encodeURIComponent(cmd))
    .then(r => r.text())
    .then(_ => updateState())
    .catch(_ => {});
}

function hold(cmd){
  send(cmd);
}

document.body.addEventListener('mouseup', ()=>send('stop'));
document.body.addEventListener('touchend', ()=>send('stop'));

function setSpeed(v){
  fetch('/action?cmd=speed&value=' + encodeURIComponent(v))
    .then(r => r.text())
    .then(_ => updateState())
    .catch(_ => {});
}
function speedLive(v){
  document.getElementById('speedVal').innerText = v;
}
function setServo(v){
  fetch('/action?cmd=servo&value=' + encodeURIComponent(v))
    .then(r => r.text())
    .then(_ => updateState())
    .catch(_ => {});
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

      let autoBtn = document.getElementById('autoBtn');
      let servoBtn = document.getElementById('servoTestBtn');
      document.getElementById('modeBadge').innerText = s.auto ? 'AUTO: ON' : 'AUTO: OFF';

      if(s.auto){ autoBtn.classList.add('on'); }
      else{ autoBtn.classList.remove('on'); }

      servoBtn.innerText = s.servoTest ? 'Servo Test ON' : 'Servo Test OFF';
      servoBtn.className = s.servoTest ? 'btn green' : 'btn gray';
    })
    .catch(_ => {});
}
setInterval(updateState, 1000);
updateState();
</script>
</body>
</html>
)rawliteral";
}

// =====================================================
// HTTP Handlers
// =====================================================
void handleRoot() {
  server.send(200, "text/html", htmlPage());
}

void handleState() {
  String json = "{";
  json += "\"auto\":" + String(autoMode ? "true" : "false") + ",";
  json += "\"servoTest\":" + String(servoTestEnabled ? "true" : "false") + ",";
  json += "\"speed\":" + String(carSpeed) + ",";
  json += "\"servo\":" + String(servoAngle) + ",";
  json += "\"distance\":" + String(lastDistance) + ",";
  json += "\"move\":\"" + currentMove + "\"";
  json += "}";
  server.send(200, "application/json", json);
}

void handleAction() {
  String cmd = server.arg("cmd");
  String value = server.arg("value");
  lastCommandTime = millis();

  // MANUAL MODE: no sensor logic here
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
    stopCar();
    servoTestEnabled = false;
    centerServo();
  }
  else if (cmd == "speed") {
    carSpeed = constrain(value.toInt(), 100, 255);
  }
  else if (cmd == "toggle_servo_test") {
    if (!autoMode) {
      servoTestEnabled = !servoTestEnabled;
      if (!servoTestEnabled) centerServo();
    }
  }
  else if (cmd == "servo") {
    if (!autoMode && servoTestEnabled) {
      setServoSafe(value.toInt());
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
  analogWriteFreq(15000);   // reduce audible motor noise

  stopCar();

  myServo.attach(SERVO_PIN, 500, 2400);
  centerServo();
  delay(500);

  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);

  server.on("/", handleRoot);
  server.on("/state", handleState);
  server.on("/action", handleAction);
  server.onNotFound(handleNotFound);
  server.begin();

  lastCommandTime = millis();
  lastDistance = readDistanceCM();
}

void loop() {
  server.handleClient();

  if (autoMode) {
    autoDriving();
  } else {
    // manual mode fail-safe only
    if (millis() - lastCommandTime > 1200 && currentMove != "stop") {
      stopCar();
    }
  }
}
