#include <SPI.h>
#include <HX711.h>
#include <DFR0534.h>
#include <SoftwareSerial.h>

#include "MotorCAN.h"

//===== 按鈕 =====
#define BUTTON_PIN 8

//===== CAN BUS =====
#define CAN0_INT 2
#define CAN_CS_PIN 9
MotorCAN motorCAN(CAN_CS_PIN);

//===== HX711 =====
#define DT A1  
#define SCK A0 
HX711 scale;

//=====
// 5 and 6 need exchange
#define SPEAKER_TX_PIN 6
#define SPEAKER_RX_PIN 5
SoftwareSerial SerialXX(SPEAKER_TX_PIN, SPEAKER_RX_PIN);
DFR0534 x_audio(SerialXX);

// === MPU sensoru Z ===
//#define OUTPUT_READABLE_ACCELGYRO
//#define OUTPUT_BINARY_ACCELGYRO
int16_t ax, ay, az;
int16_t gx, gy, gz;
bool blinkState;

// 設定
float Y = 0.0; // 初始負重
float X = 0.0; // 模式B負重
const float tolerance = 0.2; // 容忍範圍
float servoPosition = 90; // 初始伺服位置 (90度)
const int minServoAngle = 0;   // 伺服馬達最小角度
const int maxServoAngle = 135; // 伺服馬達最大角度
const float step = 1; // 每次移動角度
bool modeBActive = false; // 是否處於模式B
const int underWeight = 10000;  

long previousMillis = 0;
const long interval = 100; // Update interval in milliseconds

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  HX711_setup();
  motor_setup();
  music_setup();
  button_setup();
}

void loop() {
  bool buttonState = button_loop();
  motorCAN.update(0x94);
  float data = weight_loop(buttonState);
}

void HX711_setup(){
  scale.begin(DT, SCK);
  scale.set_scale();      // 設定比例，根據實際情況校準
  scale.tare();           // 設定零點
}

void motor_setup(){
  // Initialize the CAN bus (e.g., using 500 kbps and a 16MHz clock)
  if (motorCAN.begin(CAN_500KBPS, MCP_8MHZ)) {
    Serial.println("CAN BUS Shield Initialized");
  } else {
    Serial.println("CAN BUS Initialization Failed");
    Serial.print("Error Code: 0x");
    Serial.println(motorCAN.getError(), HEX);
  }
  motorCAN.can.setMode(MCP_NORMAL);
  pinMode(CAN0_INT, INPUT);
  motorCAN.sendIncrementalPositionControl1(1, 100*36);
  motorCAN.sendReadSingleTurnAngle(1);
}


void moveServoLeft(float _weight, bool _btn) {
  int adjustedWeight = ceil(abs(_weight) / 10000.0);  // Use ceil function and convert to int
  servoPosition = motorCAN.convertScaledToRaw((int)(motorCAN.getCurrentAngle()/100)) / 36;
  if (servoPosition > minServoAngle) {
    servoPosition = max(servoPosition - step*adjustedWeight, minServoAngle); // 限制最小角度
    motorCAN.sendSingleTurnPositionControl2(1, 0x01, motorCAN.convertRawToScaled(servoPosition*36)*100, 36*100);
    Serial.print("Moving servo left to: ");
    Serial.println((servoPosition*36));
  }
}

void moveServoRight(float _weight, bool _btn) {
  int adjustedWeight = ceil(abs(_weight) / 10000.0);  // Use ceil function and convert to int
  servoPosition = motorCAN.convertScaledToRaw((int)(motorCAN.getCurrentAngle()/100)) / 36;
  if (servoPosition < maxServoAngle) {
    servoPosition = min(servoPosition + step*adjustedWeight, maxServoAngle); // 限制最大角度
    motorCAN.sendSingleTurnPositionControl2(1, 0x00, motorCAN.convertRawToScaled(servoPosition*36) * 100, 36*100);
    Serial.print("Moving servo right to: ");
    Serial.println((servoPosition*36));
  }
}

long millis_loop() {
  unsigned long currentMillis = millis();
  previousMillis = currentMillis;
  return currentMillis;
}

float weight_loop(bool buttonState) {
  float weight = scale.get_units(); // 讀取當前重量
  if(motorCAN.getCurrentAngle() < 0) {return 0;};
  Serial.println(weight);
  if (weight > underWeight && buttonState) {
    moveServoLeft(weight, buttonState);
  }else if (weight < -1*underWeight && buttonState){
    moveServoRight(weight, buttonState);
  }
  return weight;
}

bool button_loop() {
  bool buttonState = digitalRead(BUTTON_PIN); // 讀取按鈕狀態
  // check if the pushbutton is pressed. If it is, the buttonState is HIGH
  Serial.println(buttonState);
  return buttonState;
}

void button_setup() {
  // 初始化按鈕
  pinMode(BUTTON_PIN, INPUT_PULLUP);
}

void music_setup() {
//Serial.begin(115200);
 SerialXX.begin(9600);
 x_audio.setVolume(30);
// delay(2000);
//  x_audio.playFileByName("/M_F    MP3");
//  delay(2000);
 x_audio.playFileByNumber(0x01);
//  delay(2000);
//  x_audio.playFileByNumber(0x02);
//  delay(2000);
//  x_audio.playFileByNumber(0x03);
//  delay(2000);
//  x_audio.playFileByNumber(0x04);
}