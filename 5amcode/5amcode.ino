#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <QTRSensors.h>
#include <Servo.h>


#include <Keypad.h>

#define BASE_SERVO_PIN 43
#define MIDDLE_SERVO_PIN 47
#define END_SERVO_PIN 45
#define mosfetPin 22

Servo baseServo;
Servo middleServo;
Servo endServo;

int baseAngle = 135;  // 90 degrees (middle position)
int middleAngle = 90;
int endAngle = 0;

#define table_trigpin 5
#define table_echopin 4
#define S0  18
#define S1  19
#define S2  16
#define S3  17
#define sensorOut 15  


const int bluetoothTx = 12; // HC-05 TX pin connected to Arduino digital pin 2
const int bluetoothRx = 11; // HC-05 RX pin connected to Arduino digital pin 3
int line_Sensor1 = 24;
int line_Sensor2 = 22;
int motor1speed = 3;
int motor2speed = 2;
int motor1pin1 = 9;
int motor1pin2 = 10;
int motor2pin1 = 7;
int motor2pin2 = 8;

const byte ROWS = 4; // four rows
const byte COLS = 3; // three columns
char key;

int Color_State = 0;
int Hamount = 0; // amount of hamburger's
int Pamount = 0; // amount of pizza's

long table_distance = 0;
long counter_distance =0;
int Turnning_state =0;
int state = 0;
int Rfrequency = 0;
int Gfrequency = 0;
int Bfrequency = 0;

const int Waiting_For_order = 0;
const int Going_To_costumer = 1;
const int Getting_order = 2;
const int going_to_counter = 3;
const int Getting_food = 4;

const int Hamburger = 1;
const int Pizza = 2;

const int Red_Color = 1;
const int Blue_Color = 2;



const int Left = 1;
const int Right = 2;
QTRSensors qtr;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
int blue_state = 0; // bluetooth state

SoftwareSerial bluetooth(bluetoothTx, bluetoothRx);
LiquidCrystal_I2C lcd(0x27, 16, 2);

char keys[ROWS][COLS] = {
    {'1', '2', '3'},
    {'4', '5', '6'},
    {'7', '8', '9'},
    {'*', '0', '#'}};

byte rowPins[ROWS] = {38, 36, 34, 32}; // connect to the row pinouts of the keypad
byte colPins[COLS] = {30, 28, 26};     // connect to the column pinouts of the keypad

// Create an object of keypad
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

void motorForward();
void motorRight();
void motorLeft();
void motorStop();
void motorBackward();
void Print_Lcd_Message(char *line1, char *line2, int delay1);
void Intro();
void OrderConfirmation();
void deletingOrder();
void BringingOrder();

void Getting_Order();

void setup()
{
  Serial.begin(9600);
  lcd.init();
  lcd.backlight();

pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(sensorOut, INPUT);
  
  // Setting frequency-scaling to 20%
  digitalWrite(S0,HIGH);
  digitalWrite(S1,LOW);

  pinMode(motor1speed, OUTPUT);
  pinMode(motor2speed, OUTPUT);
  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);
  pinMode(motor2pin1, OUTPUT);
  pinMode(motor2pin2, OUTPUT);

 // configure the sensors
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A8, A9, A10, A11, A12, A13, A14, A15}, SensorCount);
  qtr.setEmitterPin(2);

  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode

  // analogRead() takes about 0.1 ms on an AVR.
  // 0.1 ms per sensor * 4 samples per sensor read (default) * 6 sensors
  // * 10 reads per calibrate() call = ~24 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration

  // print the calibration minimum values measured when emitters were on

  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(1000);

  // Initialize Serial communication
  
  bluetooth.begin(38400);

  // Initialize ultrasonic sensor pins
  pinMode(table_trigpin, OUTPUT);
  pinMode(table_echopin, INPUT);
 
 // Attach servos to pins
    pinMode(mosfetPin, OUTPUT);

  baseServo.attach(BASE_SERVO_PIN);
  middleServo.attach(MIDDLE_SERVO_PIN);
  endServo.attach(END_SERVO_PIN);

  // Move crane to initial position
  moveCrane(baseAngle, middleAngle, endAngle);

}

int x = 0;


long prevTime = 0;
void loop()
{
  
  /*
  if (prevTime > 0)
  {
    Serial.print(millis() - prevTime)
        prevTime =
  }
*/

  switch (state)
  {
  case Waiting_For_order:
    lcd.clear();
    lcd.print(state);
    Serial.println(blue_state);
    Waiting_For_Order();
    break;

  case Going_To_costumer:
    lcd.clear();
    Serial.println(blue_state);
    lcd.print(table_distance);

    Going_To_Costumer();
    break;

  case Getting_order:
    lcd.clear();
    lcd.print(state);

    Getting_Order();
    break;

  case going_to_counter:
    lcd.clear();
    lcd.print(state);
    
    Going_To_Counter();
    takeOrder();
    break;
  }
  
}
void Waiting_For_Order()
{
  motorStop();
  if (bluetooth.available() > 0)
  {
    blue_state = bluetooth.read();
    Serial.print("state=");
    Serial.println(state);
  }
  else
  {
    state = 0; // Reset state if no new data is available
  }

  if (blue_state == '1')
  {
    state = Going_To_costumer;
  }
}
void Going_To_Costumer()
{
 
  uint16_t position = qtr.readLineBlack(sensorValues);

  // Control the direction of the robot based on sensor readings
  bool onLine = (sensorValues[2] >= 500 && sensorValues[3] >= 500 && sensorValues[4] >= 500 && sensorValues[5] >= 500) ||
                (sensorValues[3] >= 500 && sensorValues[4] >= 500 && sensorValues[5] >= 500);

  bool leftBlack = (sensorValues[0] >= 500 || sensorValues[1] >= 500 || sensorValues[2] >= 500);
  bool rightBlack = (sensorValues[5] >= 500 || sensorValues[6] >= 500 || sensorValues[7] >= 500);
  
  if (getDistance_table() < 13)
  {
    motorStop();
    state = Getting_order;
    Serial.println("Obstacle detected, stopping");
  }
  else
  {
    if (onLine) {
    Serial.println("Moving forward");
    motorForward();
  } else {
    if (leftBlack && !rightBlack) {
      Serial.println("Turning right");
      motorRight();
      Turnning_state =Right;
      
    } else if (!leftBlack && rightBlack) {
      Serial.println("Turning left");
      motorLeft();
      Turnning_state =Left;
    } else if (leftBlack && rightBlack) {
      // If both sides see black, it could indicate a split or error. Consider U-turn or stop.
      Serial.println("Line split or lost");
      motorStop(); // or another appropriate action
    } else {
      if(Turnning_state ==Left){
        motorLeft();
      }
      if(Turnning_state ==Right){
        motorRight();
      }
    }
  }
  Serial.println("not seeing anything");
  }
}
void Print_Lcd_Message(char *line1, char *line2, int delay1)
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(line1);
  lcd.setCursor(0, 1);
  lcd.print(line2);
  delay(delay1);
}

void deletingOrder()
{
  if (key == '*')
  {
    Print_Lcd_Message("deleting the", "order", 0);
    Hamount = 0;
    Pamount = 0;
  }
}

void BringingOrder()
{
  if (key == '#')
  {
    Print_Lcd_Message("bringing the ", "order", 0);
    state = going_to_counter;
  }
}

void Intro()
{
  Print_Lcd_Message("welcome to our ", "restaurant", 1500);
  Print_Lcd_Message("Exclusive dining", "1 dish per table.", 3000);
  Print_Lcd_Message("1 for hamburger", "2 for pizza", 3000);
}
void Getting_Order()
{
  delay(500);
  Intro();
  key = keypad.waitForKey(); // Read the key
  delay(350);
  // Print if key pressed
  if (key)
  {
    if (key == '1')
    {
      Print_Lcd_Message("you Chose 1 Hamburger. Confirm: #,Deny: *", "", 0);
      for (int i = 0; i < 28; ++i)
      {
        delay(350);
        lcd.scrollDisplayLeft();
      }
      key = keypad.waitForKey();

      if (key == '*')
      {
        Print_Lcd_Message("Hamburger order declined", "", 0);
        Hamount = 0;
        for (int i = 0; i < 20; ++i)
        {
          delay(350);
          lcd.scrollDisplayLeft();
        }
      }

      if (key == '#')
      { // have accepted the order of
        Hamount = 1;
        Pamount = 0;
        Print_Lcd_Message("confirm the  :", "order of 1", 2500);
        Print_Lcd_Message("hamburger with #", " or deny with *", 0);
        key = keypad.waitForKey();
        BringingOrder();
        deletingOrder();
      }
    }

    if (key == '2')
    { // if you chose pizza
      Print_Lcd_Message(" you Chose 1 pizza. Confirm: #,Deny: *", "", 0);
      for (int i = 0; i < 28; ++i)
      {
        delay(350);
        lcd.scrollDisplayLeft();
      }
      key = keypad.waitForKey();

      if (key == '*')
      {
        Hamount = 0;
        Pamount = 0;
        Print_Lcd_Message("pizza order declined", "", 0);
        for (int i = 0; i < 20; ++i)
        {
          delay(350);
          lcd.scrollDisplayLeft();
        }
      }

      if (key == '#')
      { // have accepted the order of
        Pamount = 1;
        Hamount = 0;
        Print_Lcd_Message("confirm the", "order of 1 ", 0);
        delay(2500);
        Print_Lcd_Message("pizza with #", " or deny with *", 0);
        BringingOrder();
        deletingOrder();
        // Your main code after the order confirmation goes here
      }
    }
  }
}

void Going_To_Counter()
{
   delay(1000);
  // Setting red filtered photodiodes to be read
  digitalWrite(S2,LOW);
  digitalWrite(S3,LOW);
  // Reading the output frequency
  Rfrequency = pulseIn(sensorOut, LOW);
  //Remaping the value of the frequency to the RGB Model of 0 to 255
  Rfrequency = map(Rfrequency, 250,10,255,0);
  // Printing the value on the serial monitor
 
  

  // Setting Green filtered photodiodes to be read
  digitalWrite(S2,HIGH);
  digitalWrite(S3,HIGH);
  // Reading the output frequency
  Gfrequency = pulseIn(sensorOut, LOW);
  //Remaping the value of the frequency to the RGB Model of 0 to 255
  Gfrequency = map(Gfrequency, 300,90,255,0);
  // Printing the value on the serial monitor

  // Setting Blue filtered photodiodes to be read
  digitalWrite(S2,LOW);
  digitalWrite(S3,HIGH);
  // Reading the output frequency
  Bfrequency = pulseIn(sensorOut, LOW);
  //Remaping the value of the frequency to the RGB Model of 0 to 255
  Bfrequency = map(Bfrequency, 250,70,255,0);
   // Printing the value on the serial monitor

  uint16_t position = qtr.readLineBlack(sensorValues);

  // Control the direction of the robot based on sensor readings
  bool onLine = (sensorValues[2] >= 500 && sensorValues[3] >= 500 && sensorValues[4] >= 500 && sensorValues[5] >= 500) ||
                (sensorValues[3] >= 500 && sensorValues[4] >= 500 && sensorValues[5] >= 500);

  bool leftBlack = (sensorValues[0] >= 500 || sensorValues[1] >= 500 || sensorValues[2] >= 500);
  bool rightBlack = (sensorValues[5] >= 500 || sensorValues[6] >= 500 || sensorValues[7] >= 500);
 
  if (Hamount == 1 && Rfrequency > 20 && Rfrequency < 100 && Gfrequency < 0 && Gfrequency > -100 && Bfrequency < 50 && Bfrequency > -70 || Pamount==1 && Rfrequency > 10 && Rfrequency < 30 && Gfrequency < 0 && Gfrequency > -100 && Bfrequency < 50 && Bfrequency > -70)
  {
    motorStop();
    state = Getting_food;
    Serial.println("spotted the food color, stopping");
  }
  else
  {
    if (onLine) {
    Serial.println("Moving forward");
    motorForward();
  } else {
    if (leftBlack && !rightBlack) {
      Serial.println("Turning right");
      motorRight();
      Turnning_state =Right;
      
    } else if (!leftBlack && rightBlack) {
      Serial.println("Turning left");
      motorLeft();
      Turnning_state =Left;
    } else if (leftBlack && rightBlack) {
      // If both sides see black, it could indicate a split or error. Consider U-turn or stop.
      Serial.println("Line split or lost");
      motorStop(); // or another appropriate action
    } else {
      if(Turnning_state ==Left){
        motorLeft();
      }
      if(Turnning_state ==Right){
        motorRight();
      }
    }
  }
  }
}

void motorForward()
{
  digitalWrite(motor1pin2, HIGH);
  digitalWrite(motor2pin2, HIGH);
  analogWrite(motor1speed, 150);
  digitalWrite(motor1pin1, LOW);
  analogWrite(motor2speed, 150);
  digitalWrite(motor2pin1, LOW);
}

void motorRight()
{
  digitalWrite(motor1pin2, HIGH);
  digitalWrite(motor2pin2, LOW);
  analogWrite(motor1speed, 200);
  digitalWrite(motor1pin1, LOW);
  analogWrite(motor2speed, 150);
  digitalWrite(motor2pin1, LOW);
}

void motorLeft()
{
  digitalWrite(motor1pin2, LOW);
  digitalWrite(motor2pin2, HIGH);
  analogWrite(motor1speed, 150);
  digitalWrite(motor1pin1, LOW);
  analogWrite(motor2speed, 200);
  digitalWrite(motor2pin1, LOW);
}

void motorStop()
{
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor2pin2, LOW);
  analogWrite(motor1speed, 0);
  digitalWrite(motor1pin2, LOW);
  analogWrite(motor2speed, 0);
  digitalWrite(motor2pin1, LOW);
}

void motorBackward()
{
  digitalWrite(motor1pin2, LOW);
  digitalWrite(motor2pin2, LOW);
  analogWrite(motor1speed, 150);
  digitalWrite(motor1pin1, HIGH);
  analogWrite(motor2speed, 150);
  digitalWrite(motor2pin1, HIGH);
}
long getDistance_table()
{
  digitalWrite(table_trigpin, LOW);
  delayMicroseconds(2);
  digitalWrite(table_trigpin, HIGH);
  delayMicroseconds(10);
  digitalWrite(table_trigpin, LOW);
  long table_duration = pulseIn(table_echopin, HIGH);
  // Calculate distance in cm
  table_distance = table_duration * 0.034 / 2;
  return table_distance;
}


void moveCraneDown() {
 for (int angle = 90; angle <= 130; angle++) {
    middleServo.write(angle); // Move the servo to the current angle
    delay(15); // Adjust this delay as needed for the desired speed of rotation
  }
  // Rotate the servo from 0 to 180 degrees
  for (int angle = 0; angle <= 180; angle++) {
    endServo.write(angle); // Move the servo to the current angle
    delay(15); // Adjust this delay as needed for the desired speed of rotation
  }
  
  
}

void moveCraneUp() {
  for (int angle = 180; angle >= 0; angle--) {
    endServo.write(angle); // Move the servo to the current angle
    delay(15); // Adjust this delay as needed for the desired speed of rotation
  }

  for (int angle = 130; angle >= 90; angle--) {
    middleServo.write(angle); // Move the servo to the current angle
    delay(15); // Adjust this delay as needed for the desired speed of rotation
  }
  // Rotate the servo from 0 to 180 degrees
}

void moveCraneRight() {
  for (int angle = 45; angle <= 180; angle++) {
    baseServo.write(angle); // Move the servo to the current angle
    delay(15); // Adjust this delay as needed for the desired speed of rotation
  }
 }

 void moveCraneLeft() {
  for (int angle = 180; angle >= 45; angle--) {
    baseServo.write(angle); // Move the servo to the current angle
    delay(15); // Adjust this delay as needed for the desired speed of rotation
  }
 }

 
 void takeOrder(){
 moveCraneLeft();
 delay(2000);
 moveCraneDown();
  digitalWrite(mosfetPin, HIGH);
 delay(2000);
 moveCraneUp();
 delay(2000);
 }

 void giveOrder(){
  moveCraneRight();
  delay(2000);
  moveCraneDown();
  delay(2000);
    digitalWrite(mosfetPin, LOW);

  moveCraneUp();
  delay(2000);

 }

 void moveCrane(int base, int middle, int end) {
  baseServo.write(base);
  middleServo.write(middle);
  endServo.write(end);
  delay(500); // Delay to allow the servos to reach the desired position
}

