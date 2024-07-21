#include <QTRSensors.h>

int aphase = 9;
int aenlb = 6;
int bphase = 5;
int benbl = 3;
int mode = 8;

int P;
int I;
int D;

float Kp = 0.05;
float Ki = 0.00001;
float Kd = 0.8;

int lastError = 0;

int button_calibration = A3;
int button_start = 2;

QTRSensors qtr;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

void setup() {
  // put your setup code here, to run once:
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){10, 11, 12, A0, A1, A2, A4, A5}, SensorCount);
  qtr.setEmitterPin(7);

  pinMode(aphase, OUTPUT);
  pinMode(aenlb, OUTPUT);
  pinMode(bphase, OUTPUT);
  pinMode(benbl, OUTPUT);
  pinMode(mode, OUTPUT);

  pinMode(button_calibration, INPUT);
  pinMode(button_start, INPUT); 

  digitalWrite(mode, HIGH);

  while(digitalRead(button_calibration) == LOW) {}
  //10 seconds
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }

  while(digitalRead(button_start) == LOW) {}
}

void loop() {
  // put your main code here, to run repeatedly:
  PID_control();
}

void PID_control() {
  uint16_t positionLine = qtr.readLineBlack(sensorValues);
  int error = 3500 - positionLine;

  P = error;
  I = error + I;
  D = error - lastError;
  lastError = error; 

  int motorSpeedChange = P*Kp + I*Ki + D*Kd;

  int motorSpeedA = 100 + motorSpeedChange;
  int motorSpeedB = 100 - motorSpeedChange;

  if (motorSpeedA > 125) {
    motorSpeedA = 125;
  }
  if (motorSpeedB > 125) {
    motorSpeedB = 125;
  }
  if (motorSpeedA < -75) {
    motorSpeedA = -75;
  }
  if (motorSpeedB < -75) {
    motorSpeedB = -75;
  }
  forward_movement(motorSpeedA, motorSpeedB);
}

void forward_movement(int speedA, int speedB) {
  if (speedA < 0) {
    speedA = 0 - speedA;
    digitalWrite(aphase, LOW);
  }
  else {
    digitalWrite(aphase, HIGH);
  }
  if (speedB < 0) {
    speedB = 0 - speedB;
    digitalWrite(bphase, HIGH);
  }
  else {
    digitalWrite(bphase, LOW);
  }
  analogWrite(aenlb, speedA);
  analogWrite(benbl, speedB);
}