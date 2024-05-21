/*
 * How to configure and pair two HC-05 Bluetooth Modules
 * by Dejan Nedelkovski, www.HowToMechatronics.com
 * 
 *                 == MASTER CODE ==
 */

#include <SoftwareSerial.h>
#define ledPin 9
const int bluetoothTx = 12; // HC-05 TX pin connected to Arduino digital pin 2
const int bluetoothRx = 11; // HC-05 RX pin connected to Arduino digital pin 3

SoftwareSerial bluetooth(bluetoothTx, bluetoothRx);


int state = 0;
int potValue = 0;

void setup() {
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);
  Serial.begin(38400); // Start serial communication with the Serial Monitor
  bluetooth.begin(38400); // Default communication rate of the Bluetooth module
}

void loop() {


  /*int x = bluetooth.available();
  Serial.print("x=");
  Serial.println(x);

   char receivedChar = bluetooth.read(); 
    Serial.print("receivedChar=");
    Serial.println(receivedChar);*/

  

 if(bluetooth.available() > 0){ // Checks whether data is comming from the serial port
    state = bluetooth.read(); // Reads the data from the serial port
    Serial.print("state=");
    Serial.println(state);
 }
 // Controlling the LED
 if (state == '1') {
  digitalWrite(ledPin, HIGH); // LED ON
  state = 0;
  Serial.println("high");
 }
 else if (state == '0') {
  digitalWrite(ledPin, LOW); // LED ON
  state = 0;
  Serial.println("low");
 }
 // Reading the potentiometer
 potValue = analogRead(A0);
 int potValueMapped = map(potValue, 0, 1023, 0, 255);
 Serial.print("potValueMapped=");
 Serial.println(potValueMapped);

 bluetooth.write(potValueMapped); // Sends potValue to servo motor
 delay(250);
}
