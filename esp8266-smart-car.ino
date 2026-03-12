#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <Servo.h>

// ---------------- WIFI ----------------
const char* ssid = "ESP8266_Car";
const char* password = "12345678";

ESP8266WebServer server(80);
Servo myServo;

// ---------------- MOTOR PINS ----------------
int in1 = D8;
int in2 = D7;
int in3 = D4;
int in4 = D3;

int ena = D5;
int enb = D6;

// ---------------- SENSORS ----------------
int frontIR = D1;
int backIR = D2;

int trig = 3;   // RX
int echo = 1;   // TX

int servoPin = D0;

// ---------------- VARIABLES ----------------
int carSpeed = 255;
bool autoMode = false;

const int SAFE_DISTANCE = 25;

// IR detect state (change HIGH if needed)
const int IR_ACTIVE_STATE = LOW;


// =====================================================
// MOTOR CONTROL
// =====================================================

void setMotor(bool a,bool b,bool c,bool d){

analogWrite(ena,carSpeed);
analogWrite(enb,carSpeed);

digitalWrite(in1,a);
digitalWrite(in2,b);
digitalWrite(in3,c);
digitalWrite(in4,d);

}

void stopCar(){
setMotor(0,0,0,0);
}

void forward(){
setMotor(1,0,1,0);
}

void backward(){
setMotor(0,1,0,1);
}

// swapped to fix direction
void left(){
setMotor(1,0,0,1);
}

void right(){
setMotor(0,1,1,0);
}


// =====================================================
// ULTRASONIC
// =====================================================

long readDistance(){

digitalWrite(trig,LOW);
delayMicroseconds(5);

digitalWrite(trig,HIGH);
delayMicroseconds(10);
digitalWrite(trig,LOW);

long duration=pulseIn(echo,HIGH,30000);

if(duration==0)return 999;

long dist=duration*0.034/2;

return dist;

}


// =====================================================
// AUTO MODE
// =====================================================

void autoDrive(){

bool frontCliff=digitalRead(frontIR)==IR_ACTIVE_STATE;
bool backCliff=digitalRead(backIR)==IR_ACTIVE_STATE;

long distance=readDistance();


// cliff detect
if(frontCliff){

stopCar();
delay(200);

if(!backCliff){

backward();
delay(350);

}

stopCar();
delay(100);

left();
delay(350);

stopCar();

return;

}


// obstacle detect
if(distance<SAFE_DISTANCE){

stopCar();
delay(200);

backward();
delay(300);

stopCar();
delay(200);

// scan left
myServo.write(160);
delay(350);

long leftDist=readDistance();

// scan right
myServo.write(20);
delay(350);

long rightDist=readDistance();

myServo.write(90);

if(leftDist>rightDist){

left();
delay(350);

}else{

right();
delay(350);

}

stopCar();
return;

}


// normal drive
forward();

}



// =====================================================
// WEB PAGE
// =====================================================

String webpage(){

String page = R"====(
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1">

<style>

body{
font-family:Arial;
text-align:center;
background:#eee;
}

button{
width:80px;
height:80px;
font-size:30px;
border-radius:40px;
margin:8px;
}

.auto{
background:orange;
width:100px;
height:100px;
}

</style>

</head>

<body>

<h2>ESP8266 SMART CAR</h2>

<div>

<button 
onmousedown="cmd('forward')" 
onmouseup="cmd('stop')">↑</button>

</div>

<div>

<button 
onmousedown="cmd('left')" 
onmouseup="cmd('stop')">←</button>

<button class="auto"
onclick="cmd('auto')">AUTO</button>

<button 
onmousedown="cmd('right')" 
onmouseup="cmd('stop')">→</button>

</div>

<div>

<button 
onmousedown="cmd('backward')" 
onmouseup="cmd('stop')">↓</button>

</div>

<br>

Speed

<br>

<input type="range"
min="120"
max="255"
value="255"
onchange="speed(this.value)">

<script>

function cmd(x){

fetch("/cmd?go="+x);

}

function speed(v){

fetch("/speed?val="+v);

}

</script>

</body>
</html>
)====";

return page;

}


// =====================================================
// SERVER COMMAND
// =====================================================

void handleCmd(){

String c=server.arg("go");

autoMode=false;

if(c=="forward")forward();
else if(c=="backward")backward();
else if(c=="left")left();
else if(c=="right")right();
else if(c=="stop")stopCar();

else if(c=="auto"){

autoMode=!autoMode;
stopCar();

}

server.send(200,"text/plain","OK");

}


void handleSpeed(){

carSpeed=server.arg("val").toInt();
server.send(200,"text/plain","OK");

}



// =====================================================
// SETUP
// =====================================================

void setup(){

pinMode(in1,OUTPUT);
pinMode(in2,OUTPUT);
pinMode(in3,OUTPUT);
pinMode(in4,OUTPUT);

pinMode(ena,OUTPUT);
pinMode(enb,OUTPUT);

pinMode(frontIR,INPUT);
pinMode(backIR,INPUT);

pinMode(trig,OUTPUT);
pinMode(echo,INPUT);

analogWriteRange(255);

// MOTOR NOISE FIX
analogWriteFreq(15000);


// SERVO FIX
myServo.attach(servoPin,500,2400);
myServo.write(90);


WiFi.softAP(ssid,password);


server.on("/",[](){

server.send(200,"text/html",webpage());

});

server.on("/cmd",handleCmd);
server.on("/speed",handleSpeed);

server.begin();

}



// =====================================================
// LOOP
// =====================================================

void loop(){

server.handleClient();

if(autoMode){

autoDrive();

}

}
