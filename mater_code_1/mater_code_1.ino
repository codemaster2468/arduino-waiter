#include "Arduino.h"
#include <SoftwareSerial.h>

const int bluetoothTx = 12; // HC-05 TX pin connected to Arduino digital pin 2
const int bluetoothRx = 11; // HC-05 RX pin connected to Arduino digital pin 3

SoftwareSerial bluetooth(bluetoothTx, bluetoothRx);
int line_Sensor1 = 3;
int line_Sensor2 = 2;
int motor1speed = 10;
int motor2speed = 9;
int motor1pin1 = 5;
int motor1pin2 = 4;
int motor2pin1 = 7;
int motor2pin2 = 6;

int state = 0;

void setup() {
  // Set the motor control pins as outputs
  pinMode(motor1speed, OUTPUT);
  pinMode(motor2speed, OUTPUT);
  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);
  pinMode(motor2pin1, OUTPUT);
  pinMode(motor2pin2, OUTPUT);
  pinMode(line_Sensor1, INPUT);
  pinMode(line_Sensor2, INPUT);
  Serial.begin(38400); // Start serial communication with the Serial Monitor
  bluetooth.begin(38400); // Default communication rate of the Bluetooth module
}

void loop() {
  if (bluetooth.available() > 0) { // Checks whether data is coming from the serial port
    state = bluetooth.read(); // Reads the data from the serial port
    Serial.print("state=");
    Serial.println(state);
  }

  if (state == '1') {
    int pas1 = digitalRead(line_Sensor1);
    int pas2 = digitalRead(line_Sensor2);

    if (pas1 && pas2) {
      motorForward();
    } else if (pas1 == 0 && pas2 == 1) {
      motorRight();
    } else if (pas1 == 1 && pas2 == 0) {
      motorLeft();
    } else {
      motorStop();
    }
  } 
  else {
    motorStop();
    Serial.println("motor stop");
  }

  int pas1 = digitalRead(line_Sensor1);
  int pas2 = digitalRead(line_Sensor2);
  Serial.print("pas1= ");
  Serial.println(pas1);
  Serial.print("pas2= ");
  Serial.println(pas2);
}


void motorForward() {
  digitalWrite(motor1pin2, HIGH);
  digitalWrite(motor2pin2, HIGH);
  analogWrite(motor1speed, 130); // Motor A full speed
  digitalWrite(motor1pin1, LOW);
  analogWrite(motor2speed, 130); // Motor B full speed
  digitalWrite(motor2pin1, LOW);
}

void motorRight() {
  digitalWrite(motor1pin2, HIGH);
  digitalWrite(motor2pin2, LOW);
  analogWrite(motor1speed, 130); // Motor A full speed
  digitalWrite(motor1pin1, LOW);
  analogWrite(motor2speed, 130); // Motor B full speed
  digitalWrite(motor2pin1, LOW);
}

void motorLeft() {
  digitalWrite(motor1pin2, LOW);
  digitalWrite(motor2pin2, HIGH);
  analogWrite(motor1speed, 130); // Motor A full speed
  digitalWrite(motor1pin1, LOW);
  analogWrite(motor2speed, 130); // Motor B full speed
  digitalWrite(motor2pin1, LOW);
}

void motorStop() {
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor2pin2, LOW);
  analogWrite(motor1speed, 0); // Motor A stopped
  digitalWrite(motor1pin2, LOW);
  analogWrite(motor2speed, 0); // Motor B stopped
  digitalWrite(motor2pin1, LOW);
}