#include <Arduino.h>
#include <WiFi.h>

#include <WiFiClientSecure.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <cJSON.h>
#include <math.h>
#define sign(x) (x < 0 ? -1 : 1)
// network parameters
char *ssid = "WIFINAME";
char *password = "password";

String json = "{\"Test\":42}";
// Sensors
#define IR_LEFT_FRONT 34
#define IR_LEFT_BACK 35
#define IR_RIGHT_FRONT 33
#define IR_RIGHT_BACK 32
#define US_TRIG 22
#define US_ECHO 23
// min Dinstance at wich Net_speed cant be directed toward an US detected Object
#define US_minDistance 4
#define US_reduceSpeedAt 50

int IRLF_value, IRLB_value, IRRF_value, IRRB_value;
long USF_distance, USF_time;
// Motors
#define reducedMaxSpeed 512
#define ML_PIN 15
#define MR_PIN 19
#define IN1 18
#define IN2 5
#define IN3 4
#define IN4 2
#define MeasurementStatusLED 21

int ML_current_speed = 0;
int MR_current_speed = 0;
// PWN properties
const double freq = 259;
const int LMChannel = 0;
const int RMChannel = 1;
const int PWMresolution = 10;
const int PWMMaxValue = 1024;
// Timer for measurements
const unsigned long publishMs = 5000;

#define MAX_MOVES 100
typedef struct move
{
int left;
int right;
long duration;
};
move lastMoves[MAX_MOVES]={};
int lastMoveIndex = -1;

long ms_prev_move=0;
long ms_current_move;
void saveLastMove(int left, int right){
  ms_current_move=millis();

  lastMoveIndex = (lastMoveIndex+1)%MAX_MOVES;
  struct move latest_move;
  latest_move.left=left;
  latest_move.right=right;
  latest_move.duration=ms_current_move-ms_prev_move;
  lastMoves[lastMoveIndex]=latest_move;
  ms_prev_move=ms_current_move;
}

WebServer server(80);

WebSocketsServer webSocket = WebSocketsServer(81);
// Web page to be sent to clients
char webpage[] PROGMEM = R"=====(
  <!DOCTYPE html>
  <html>
  <head>
  <meta http-equiv="Content-Type" content="text/html; charset=utf-8"/> 
  <link rel="shortcut icon" href="#">
    <script>
      var Socket;
      function init(){
        Socket = new WebSocket('ws://'+window.location.hostname +':81/');
        Socket.onmessage = function(event){
          console.log(event);
          var data;
          try {
            data=JSON.parse(event.data);
          } catch (e) {
            data=event.data;
          }

          console.log(data);

        }
      }
      function sendSpeed() {
        const msg = {
          type: "message",
          data:{
            leftSpeed : Number(document.getElementById('left').value),
            rightSpeed : Number(document.getElementById('right').value)
          } ,

          send_at: Date.now()
        };
        console.log("LeftSpeed: ",msg.data.leftSpeed);
        console.log("RightSpeed: ", msg.data.rightSpeed);
        Socket.send(JSON.stringify(msg));
      }

    </script>
  </head>
  <body onload ="javascript:init()">
    <h4> Websocket client provided by the WIFI Bot!</h4>
    <h4> Send Speeds to the Bot</h4>
    <form>
      <label for="left">
        left: 
        <input id="left" type="number" min="0" max="512" step="1" value="0">
        
      </label>
      <label for="right">
        right: 
        <input id="right" type="number" min="0" max="512" step="1" value="0">
        
      </label>
      <button type="button" onClick="sendSpeed()">Send</button>
    </form>
  </body>
  </html>
)=====";
void setMotorSpeeds(int left, int right)
{

  if(ML_current_speed==left||MR_current_speed==right)
    return;
    if(ms_prev_move==0)
      ms_prev_move=millis();
  saveLastMove(ML_current_speed,MR_current_speed);
  ML_current_speed = left % PWMMaxValue;
  MR_current_speed = right % PWMMaxValue;
  Serial.printf("Left: %d\n", ML_current_speed);
  Serial.printf("Right: %d\n", MR_current_speed);
  ledcWrite(LMChannel, abs(ML_current_speed));
  ledcWrite(RMChannel, abs(MR_current_speed));
  if (ML_current_speed >= 0)
  {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  }
  else
  {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }
  if (MR_current_speed >= 0)
  {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  }
  else
  {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
  
}
bool almostEqual(int v1, int v2)
{
  return std::abs(v1 - v2) < 10 && v1 - v2;
}
bool TurnsOnSpot()
{
  return almostEqual(ML_current_speed, MR_current_speed) && almostEqual(ML_current_speed + MR_current_speed, 0);
}
bool movesForward()
{
  return ML_current_speed + MR_current_speed > 0;
}
bool movesBackward()
{
  return ML_current_speed + MR_current_speed > 0;
}
bool collisionAvoidance()
{
  // block turns which would prop cause collision
  // Idea: some diff in speeds might be okay to manouver out of the way
  //       or because same speed setting might still result in different speed

  if ((IRLF_value == 0 || IRRB_value == 0) && ML_current_speed < MR_current_speed && TurnsOnSpot())
  {
    setMotorSpeeds(0, 0);
    return true;
  }
  if ((IRRF_value == 0 || IRLB_value == 0) && ML_current_speed > MR_current_speed && TurnsOnSpot())
  {
    setMotorSpeeds(0, 0);
    return true;
  }
  if (USF_distance < US_minDistance && !TurnsOnSpot() && movesForward())
  {
    setMotorSpeeds(0, 0);
    return true;
  }
  if (USF_distance < US_reduceSpeedAt && !TurnsOnSpot() && movesForward())
  {
    // cap max speed at reduced Speeds
    setMotorSpeeds(sign(ML_current_speed) * min(abs(ML_current_speed), reducedMaxSpeed), sign(MR_current_speed) * min(abs(MR_current_speed), reducedMaxSpeed));
    return true;
  }
  // enable alongside Backwards oriented US-Sensor
  // if(USB_distance> US_minDistance&&!TurnsOnSpot()&&movesBackward()){
  //   setMotorSpeeds(0, 0);
  // }
  // if(USB_distance< US_reduceSpeedAt&&!TurnsOnSpot()&&movesBackward()){
  //   //cap max speed at reduced Speeds
  //   setMotorSpeeds(sign(ML_current_speed)*min(abs(ML_current_speed),reducedMaxSpeed), sign(MR_current_speed)*min(abs(MR_current_speed),reducedMaxSpeed));
  // }

  // set max net speed accordingly. maybe half speed if object detected
  return false;
}


void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length)
{
  Serial.print("WebsocketType: ");
  Serial.println(type);
  switch (type)
  {
  case WStype_DISCONNECTED:
    Serial.printf("[%u] Disconnected!\n", num);
    break;
  case WStype_CONNECTED:
  {
    IPAddress ip = webSocket.remoteIP(num);
    Serial.printf("[%u] Connected from %d.%d.%d.%d url: %s\n", num, ip[0], ip[1], ip[2], ip[3], payload);
    webSocket.sendTXT(num, "Connected");
    //estimate start ms of first movement
  }
  break;
  case WStype_TEXT:
  {
    Serial.printf("[%u] Message recieved: %s\n", num, payload);
    webSocket.broadcastTXT("message recieved");

    const char *tempMSG = const_cast<const char *>((char *)payload);
    cJSON *msg = cJSON_Parse(tempMSG);
    Serial.println(cJSON_Print(msg));
    cJSON *data = cJSON_GetObjectItem(msg, "data");
    cJSON *leftSpeed = cJSON_GetObjectItem(data, "leftSpeed");
    cJSON *rightSpeed = cJSON_GetObjectItem(data, "rightSpeed");
    if (cJSON_IsNumber(leftSpeed) && cJSON_IsNumber(rightSpeed))
    {
      setMotorSpeeds(leftSpeed->valueint, rightSpeed->valueint);
    }
    Serial.println(cJSON_Print(msg));
    webSocket.broadcastTXT("message processed");
  }
  break;
  default:
    break;
  }
  Serial.print("WebsocketType: ");
}
int getUSFDistance(int points)
{
  int mean=0;
  for (int i = 0; i < points; i++)
  {
    digitalWrite(US_TRIG, LOW);
    delayMicroseconds(3);
    noInterrupts();
    digitalWrite(US_TRIG, HIGH); // Trigger Impuls 10 us
    delayMicroseconds(15);
    digitalWrite(US_TRIG, LOW);
    USF_time = pulseIn(US_ECHO, HIGH,58225 ); // meassure time till echo
    interrupts();
    mean+=(USF_time<=1?58225:USF_time) * 0.017175;// time to distance in cm
  }
  mean/points==0?USF_distance=1000:USF_distance=mean/points;
  return (USF_distance);
}
void readSensors(){
  IRLF_value = digitalRead(IR_LEFT_FRONT);
  IRLB_value = digitalRead(IR_LEFT_BACK);
  IRRF_value = digitalRead(IR_RIGHT_FRONT);
  IRRB_value = digitalRead(IR_RIGHT_BACK);
  USF_distance = getUSFDistance(3);

}
bool reloadLastMove()
{
  if (lastMoveIndex == -1)
    return false;
  setMotorSpeeds(lastMoves[lastMoveIndex].left, lastMoves[lastMoveIndex].right);
  long startTime = millis();
  while (millis() - startTime < lastMoves[lastMoveIndex].duration /*&& !collisionAvoidance()*/)
  {
    delay(10);
    readSensors();
  }
  
  lastMoveIndex=(lastMoveIndex-1)%MAX_MOVES;
  return true;
}
// Take points amount of measurements and returns the average signal strength.
int getWiFiStrength(int points)
{
  long rssi = 0;
  long averageRSSI = 0;

  for (int i = 0; i < points; i++)
  {
    rssi += WiFi.RSSI();
    delay(20);
  }

  averageRSSI = rssi / points;
  return averageRSSI;
}
cJSON *getMeasurements()
{
  cJSON *result = cJSON_CreateObject();
  cJSON_AddItemToObject(result, "WiFiStrength", cJSON_CreateNumber(getWiFiStrength(3)));
  return result;
}
void sendMeasurements()
{
  digitalWrite(MeasurementStatusLED, LOW);
  String Measurements = cJSON_Print(getMeasurements());
  webSocket.broadcastTXT(Measurements.c_str(), Measurements.length());
  digitalWrite(MeasurementStatusLED, HIGH);
}

void setupPins()
{
  pinMode(MeasurementStatusLED, OUTPUT);
  pinMode(ML_PIN, OUTPUT);
  pinMode(MR_PIN, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(IR_LEFT_FRONT, INPUT);
  pinMode(IR_LEFT_BACK, INPUT);
  pinMode(IR_RIGHT_FRONT, INPUT);
  pinMode(IR_RIGHT_BACK, INPUT);
  pinMode(US_ECHO, INPUT);
  pinMode(US_TRIG, OUTPUT);
  digitalWrite(US_TRIG, HIGH);
  digitalWrite(ML_PIN, LOW);
  digitalWrite(MeasurementStatusLED, LOW);
  digitalWrite(MR_PIN, LOW);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void setup()
{
  setupPins();
  WiFi.begin(ssid, password);
  Serial.begin(115200);
  Serial.print("Connecting to WIFI, ");
  Serial.print(ssid);
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(500);
  }
  Serial.println("");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  server.on("/", []()
            { server.send_P(200, "text/html", webpage); });
  server.begin();
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  // PWM setup

  Serial.printf("ChannelLeft: %f", ledcSetup(LMChannel, freq, PWMresolution));
  Serial.printf("ChannelRight: %f", ledcSetup(RMChannel, freq, PWMresolution));
  ledcWrite(RMChannel, 0);
  ledcWrite(LMChannel, 0);

  ledcAttachPin(ML_PIN, LMChannel);
  ledcAttachPin(MR_PIN, RMChannel);
}

void loop()
{
  // put your main code here, to run repeatedly:
  webSocket.loop();
  server.handleClient();
  collisionAvoidance();
  
  readSensors();
  static unsigned long prevsMs = millis();
  unsigned long curMs = millis();
  Serial.printf("IRLF: %i, IRLB: %i, IRRF: %i, IRRB: %i, ML: %i, MR: %i, USFd: %li \n", IRLF_value, IRLB_value, IRRF_value, IRRB_value, ML_current_speed, MR_current_speed, USF_distance);
  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("Reconnecting to WIFI");
    WiFi.disconnect();
    WiFi.reconnect();
    int max_retries = 10;
    while (WiFi.status() != WL_CONNECTED&&max_retries>0)
    {
      setMotorSpeeds(0, 0);
      Serial.print(".");
      delay(500);
      max_retries--;
    }
    if(max_retries==0){
      Serial.println("Could not reconnect to WIFI");
      Serial.println("Try to relocate the robot to a better WIFI spot");

    }
    else{
      Serial.println("Reconnected to WIFI");
    }
  }
  if (curMs - prevsMs >= publishMs)
  {
    // PUBLISH DATA HERE!!!
    sendMeasurements();
    prevsMs = curMs;
  }
}