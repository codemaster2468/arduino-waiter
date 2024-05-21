#include "Arduino.h"
#include <SoftwareSerial.h>

#define trigpin 5
#define echopin 4

const int bluetoothTx = 12;
const int bluetoothRx = 11;

SoftwareSerial bluetooth(bluetoothTx, bluetoothRx);

int line_Sensor1 = 24;
int line_Sensor2 = 22;

int motor1speed = 3;
int motor2speed = 2;
int motor1pin1 = 9;
int motor1pin2 = 10;
int motor2pin1 = 7;
int motor2pin2 = 8;

int state = 0;
int key = 0;

void setup() {
  pinMode(motor1speed, OUTPUT);
  pinMode(motor2speed, OUTPUT);
  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);
  pinMode(motor2pin1, OUTPUT);
  pinMode(motor2pin2, OUTPUT);
  pinMode(line_Sensor1, INPUT);
  pinMode(line_Sensor2, INPUT);
 
  // Initialize Serial communication
  Serial.begin(38400);
  bluetooth.begin(38400);
 
  // Initialize ultrasonic sensor pins
  pinMode(trigpin, OUTPUT);
  pinMode(echopin, INPUT);
}

void loop() {
  digitalWrite(trigpin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigpin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigpin, LOW);
  long zman = pulseIn(echopin, HIGH);
  int distance = zman / 29 / 2;
  delay(0);
  int val1 = digitalRead(line_Sensor1);
  int val2 = digitalRead(line_Sensor2);

  if (bluetooth.available() > 0) {
    state = bluetooth.read();
    Serial.print("state=");
    Serial.println(state);
  } else {
    state = 0; // Reset state if no new data is available
  }

  if (state == '1') {
    key = 1;
  }

  if (key == 1) {
    // Check ultrasonic sensor distance
    if (getDistance() < 15) {
      motorStop();
      Serial.println("Obstacle detected, stopping");
    } else {
      if (val1 == 1 && val2 == 1) {
        motorForward();
      } else if (val1 == 0 && val2 == 1) {
        motorRight();
      } else if (val1 == 1 && val2 == 0) {
        motorLeft();
      } else {
        motorBackward();
      }
    }
  } else {
    motorStop();
    Serial.println("motor stop");
  }

  Serial.print("pas1= ");
  Serial.println(val1);
  Serial.print("pas2= ");
  Serial.println(val2);
}

void motorForward() {
  digitalWrite(motor1pin2, HIGH);
  digitalWrite(motor2pin2, HIGH);
  analogWrite(motor1speed, 150);
  digitalWrite(motor1pin1, LOW);
  analogWrite(motor2speed, 150);
  digitalWrite(motor2pin1, LOW);
}

void motorRight() {
  digitalWrite(motor1pin2, HIGH);
  digitalWrite(motor2pin2, LOW);
  analogWrite(motor1speed, 200);
  digitalWrite(motor1pin1, LOW);
  analogWrite(motor2speed, 150);
  digitalWrite(motor2pin1, LOW);
}

void motorLeft() {
  digitalWrite(motor1pin2, LOW);
  digitalWrite(motor2pin2, HIGH);
  analogWrite(motor1speed, 150);
  digitalWrite(motor1pin1, LOW);
  analogWrite(motor2speed, 200);
  digitalWrite(motor2pin1, LOW);
}

void motorStop() {
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor2pin2, LOW);
  analogWrite(motor1speed, 0);
  digitalWrite(motor1pin2, LOW);
  analogWrite(motor2speed, 0);
  digitalWrite(motor2pin1, LOW);
}

void motorBackward() {
  digitalWrite(motor1pin2, LOW);
  digitalWrite(motor2pin2, LOW);
  analogWrite(motor1speed, 150);
  digitalWrite(motor1pin1, HIGH);
  analogWrite(motor2speed, 150);
  digitalWrite(motor2pin1, HIGH);
}

long getDistance() {
  digitalWrite(trigpin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigpin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigpin, LOW);
  long duration = pulseIn(echopin, HIGH);
  // Calculate distance in cm
  long distance = duration * 0.034 / 2;
  return distance;
}