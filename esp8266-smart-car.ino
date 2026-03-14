#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <Servo.h>

// ================= WIFI =================
const char* ssid = "ESP8266_Car";
const char* password = "password123";

ESP8266WebServer server(80);
Servo scanServo;

// ================= PIN MAP =================
// Motor driver
const uint8_t IN1 = D8;
const uint8_t IN2 = D7;
const uint8_t IN3 = D4;
const uint8_t IN4 = D3;
const uint8_t ENA = D5;
const uint8_t ENB = D6;

// Sensors
const uint8_t FRONT_IR = D1;   // front cliff
const uint8_t BACK_IR  = D0;   // rear cliff

// Servo
const uint8_t SERVO_PIN = D2;

// Ultrasonic
const uint8_t TRIG_PIN = 3;    // RX
const uint8_t ECHO_PIN = 1;    // TX

// ================= SETTINGS =================
int carSpeed = 220;
bool autoMode = false;
bool servoTestEnabled = false;
String currentMove = "stop";
int servoAngle = 90;
long lastDistance = 999;
unsigned long lastAutoTick = 0;

// If cliff detect does not work correctly, change HIGH <-> LOW
const int IR_CLIFF_STATE = HIGH;

// Servo angles
const int SERVO_CENTER = 90;
const int SERVO_LEFT   = 155;
const int SERVO_RIGHT  = 25;

// Distance and movement tuning
const int SAFE_DISTANCE = 35;         // obstacle threshold
const int FORWARD_BURST = 180;
const int BACK_TIME     = 280;
const int TURN_90_TIME  = 520;        // tune for your car
const int TURN_SMALL    = 320;

// ================= MOTOR CONTROL =================
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

// LEFT / RIGHT fixed here for switch direction
void turnLeft() {
  setMotor(HIGH, LOW, LOW, HIGH, carSpeed, carSpeed);
  currentMove = "left";
}

void turnRight() {
  setMotor(LOW, HIGH, HIGH, LOW, carSpeed, carSpeed);
  currentMove = "right";
}

// softer steering if needed later
void turnLeftSmall() {
  turnLeft();
  delay(TURN_SMALL);
  stopCar();
}

void turnRightSmall() {
  turnRight();
  delay(TURN_SMALL);
  stopCar();
}

// ================= SERVO =================
void setServoSafe(int angle) {
  angle = constrain(angle, 10, 170);
  servoAngle = angle;
  scanServo.write(servoAngle);
}

void centerServo() {
  setServoSafe(SERVO_CENTER);
}

// ================= SENSORS =================
bool frontCliffDetected() {
  return digitalRead(FRONT_IR) == IR_CLIFF_STATE;
}

bool backCliffDetected() {
  return digitalRead(BACK_IR) == IR_CLIFF_STATE;
}

long readDistanceOnce() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(4);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(12);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long duration = pulseIn(ECHO_PIN, HIGH, 35000UL);
  if (duration == 0) return 999;

  long dist = (long)(duration * 0.0343 / 2.0);
  if (dist <= 0) dist = 999;
  return dist;
}

// Average multiple reads to make ultrasonic more stable / farther usable
long readDistanceCM() {
  long d1 = readDistanceOnce();
  delay(15);
  long d2 = readDistanceOnce();
  delay(15);
  long d3 = readDistanceOnce();

  // pick average of valid reads
  long sum = 0;
  int cnt = 0;

  if (d1 != 999) { sum += d1; cnt++; }
  if (d2 != 999) { sum += d2; cnt++; }
  if (d3 != 999) { sum += d3; cnt++; }

  if (cnt == 0) {
    lastDistance = 999;
    return 999;
  }

  long avg = sum / cnt;
  lastDistance = avg;
  return avg;
}

long scanAt(int angle) {
  setServoSafe(angle);
  delay(320);
  long d = readDistanceCM();
  delay(20);
  return d;
}

// ================= AUTO DRIVING =================
void turn90Left() {
  turnLeft();
  delay(TURN_90_TIME);
  stopCar();
  delay(70);
}

void turn90Right() {
  turnRight();
  delay(TURN_90_TIME);
  stopCar();
  delay(70);
}

void chooseBestTurnAndGo() {
  long leftDist  = scanAt(SERVO_LEFT);
  long centerDist = scanAt(SERVO_CENTER);
  long rightDist = scanAt(SERVO_RIGHT);

  centerServo();
  delay(50);

  if (leftDist > rightDist && leftDist > SAFE_DISTANCE) {
    turn90Left();
    moveForward();
    delay(FORWARD_BURST);
    stopCar();
    return;
  }

  if (rightDist >= leftDist && rightDist > SAFE_DISTANCE) {
    turn90Right();
    moveForward();
    delay(FORWARD_BURST);
    stopCar();
    return;
  }

  if (centerDist > SAFE_DISTANCE) {
    moveForward();
    delay(FORWARD_BURST);
    stopCar();
    return;
  }

  // all blocked -> stronger turn
  turn90Right();
  moveForward();
  delay(FORWARD_BURST);
  stopCar();
}

void handleFrontCliff() {
  stopCar();
  delay(80);

  // if rear safe, back away from cliff
  if (!backCliffDetected()) {
    moveBackward();
    delay(BACK_TIME);
    stopCar();
    delay(80);
  }

  chooseBestTurnAndGo();
}

void handleObstacle() {
  stopCar();
  delay(80);

  // if rear safe, step back a bit
  if (!backCliffDetected()) {
    moveBackward();
    delay(BACK_TIME);
    stopCar();
    delay(80);
  }

  chooseBestTurnAndGo();
}

void autoDriving() {
  if (millis() - lastAutoTick < 90) return;
  lastAutoTick = millis();

  bool frontCliff = frontCliffDetected();
  bool rearCliff  = backCliffDetected();
  long dist = readDistanceCM();

  // 1) Front cliff has highest priority
  if (frontCliff) {
    handleFrontCliff();
    return;
  }

  // 2) Front obstacle
  if (dist < SAFE_DISTANCE) {
    handleObstacle();
    return;
  }

  // 3) Never keep backing if rear cliff appears
  if (rearCliff && currentMove == "backward") {
    stopCar();
    delay(60);
    moveForward();
    delay(180);
    stopCar();
    delay(60);
    chooseBestTurnAndGo();
    return;
  }

  // 4) Straight forward in auto mode
  centerServo();
  moveForward();
}

// ================= WEB UI =================
String htmlPage() {
  return R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1, user-scalable=no">
<title>ESP8266 Smart Car</title>
<style>
:root{
  --bg:#ececec;
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
  <div class="title">ESP8266 SMART CAR</div>

  <div class="card">
    <div class="toprow">
      <div id="modeBadge" class="badge">AUTO: OFF</div>
      <div id="moveBadge" class="badge">MOVE: STOP</div>
      <div id="distBadge" class="badge">DIST: -- cm</div>
    </div>

    <div class="layout">
      <div class="remote">
        <div class="ring"></div>

        <button class="btnDir up" onmousedown="holdStart('forward')" ontouchstart="holdStart('forward')" onmouseup="holdStop()" ontouchend="holdStop()">&#9650;</button>
        <button class="btnDir left" onmousedown="holdStart('left')" ontouchstart="holdStart('left')" onmouseup="holdStop()" ontouchend="holdStop()">&#9664;</button>
        <button id="autoBtn" class="autoBtn" onclick="send('toggle_auto')">AUTO</button>
        <button class="btnDir right" onmousedown="holdStart('right')" ontouchstart="holdStart('right')" onmouseup="holdStop()" ontouchend="holdStop()">&#9654;</button>
        <button class="btnDir down" onmousedown="holdStart('backward')" ontouchstart="holdStart('backward')" onmouseup="holdStop()" ontouchend="holdStop()">&#9660;</button>
      </div>

      <div class="speedWrap">
        <div class="speedTitle">SPEED</div>
        <input id="speed" class="vertical" type="range" min="120" max="255" value="220" oninput="speedLive(this.value)" onchange="setSpeed(this.value)">
        <div id="speedVal">220</div>
      </div>
    </div>
    <div class="note">Manual mode ignores sensors. Auto mode uses IR + ultrasonic + servo.</div>
  </div>

  <div class="card">
    <div style="font-weight:bold;margin-bottom:10px;">Servo Test (Manual Mode)</div>
    <div class="row">
      <button id="servoTestBtn" class="btn gray" onclick="send('toggle_servo_test')">Servo Test OFF</button>
      <button class="btn" onclick="setServo(25)">Left</button>
      <button class="btn" onclick="setServo(90)">Center</button>
      <button class="btn" onclick="setServo(155)">Right</button>
    </div>
    <div style="margin-top:12px;">
      <input id="servoSlider" class="slider" type="range" min="10" max="170" value="90" oninput="servoLive(this.value)" onchange="setServo(this.value)">
      <div id="servoVal" style="margin-top:8px;font-weight:bold;">Servo: 90°</div>
    </div>
    <div class="note">Servo test only works in manual mode.</div>
  </div>
</div>

<script>
let holdTimer = null;
let activeCmd = '';

function send(cmd){
  fetch('/action?cmd=' + encodeURIComponent(cmd))
    .then(r => r.text())
    .then(() => updateState())
    .catch(() => {});
}

function holdStart(cmd){
  activeCmd = cmd;
  send(cmd);
  clearInterval(holdTimer);
  holdTimer = setInterval(() => {
    send(activeCmd);
  }, 250);
}

function holdStop(){
  clearInterval(holdTimer);
  holdTimer = null;
  activeCmd = '';
  send('stop');
}

document.body.addEventListener('mouseup', holdStop);
document.body.addEventListener('touchend', holdStop);
document.body.addEventListener('touchcancel', holdStop);

function setSpeed(v){
  fetch('/action?cmd=speed&value=' + encodeURIComponent(v))
    .then(r => r.text())
    .then(() => updateState())
    .catch(() => {});
}
function speedLive(v){
  document.getElementById('speedVal').innerText = v;
}
function setServo(v){
  fetch('/action?cmd=servo&value=' + encodeURIComponent(v))
    .then(r => r.text())
    .then(() => updateState())
    .catch(() => {});
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

      if(s.auto) autoBtn.classList.add('on');
      else autoBtn.classList.remove('on');

      servoBtn.innerText = s.servoTest ? 'Servo Test ON' : 'Servo Test OFF';
      servoBtn.className = s.servoTest ? 'btn green' : 'btn gray';
    })
    .catch(() => {});
}
setInterval(updateState, 1000);
updateState();
</script>
</body>
</html>
)rawliteral";
}

// ================= HTTP =================
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
    carSpeed = constrain(value.toInt(), 120, 255);
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

// ================= SETUP / LOOP =================
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
  analogWriteFreq(15000);

  stopCar();

  // Servo on D2 -> much better than D0
  scanServo.attach(SERVO_PIN, 500, 2400);
  centerServo();
  delay(500);

  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);

  server.on("/", handleRoot);
  server.on("/state", handleState);
  server.on("/action", handleAction);
  server.onNotFound(handleNotFound);
  server.begin();

  lastDistance = readDistanceCM();
}

void loop() {
  server.handleClient();

  if (autoMode) {
    autoDriving();
  }
}
