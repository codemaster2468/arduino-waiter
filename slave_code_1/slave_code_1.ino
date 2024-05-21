/*
 * How to configure and pair two HC-05 Bluetooth Modules
 * by Dejan Nedelkovski, www.HowToMechatronics.com
 * 
 *                 == SLAVE CODE ==
 */
#include <SoftwareSerial.h>
#include <Servo.h>
#define button 4
const int bluetoothTx = 12; // HC-05 TX pin connected to Arduino digital pin 2
const int bluetoothRx = 11; // HC-05 RX pin connected to Arduino digital pin 3

SoftwareSerial bluetooth(bluetoothTx, bluetoothRx);

Servo myServo;
int state = 20;
int buttonState = 0;

void setup() {
  pinMode(button, INPUT);
  myServo.attach(9);
  Serial.begin(38400); // Default communication rate of the Bluetooth module
  bluetooth.begin(38400); // Default communication rate of the Bluetooth module
}

void loop() {

  int x = bluetooth.available();
  Serial.print("x=");
  Serial.println(x);

 if(bluetooth.available() > 0){ // Checks whether data is comming from the serial port
    state = bluetooth.read(); // Reads the data from the serial port
    Serial.print("state=");
    Serial.println(state);
 }
 
 // Controlling the servo motor
  
 myServo.write(state);
 Serial.print("state=");
 Serial.println(state);
 delay(10);
 
 // Reading the button
 buttonState = digitalRead(button);
 if (buttonState == HIGH) {
   bluetooth.write('1'); // Sends '1' to the master to turn on LED
   Serial.println("high");
 }
 else {
   bluetooth.write('0');
   Serial.println("low");
 }  
 delay(250);
}