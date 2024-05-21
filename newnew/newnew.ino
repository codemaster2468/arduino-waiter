#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <QTRSensors.h>


#include <Keypad.h>

#define counter_trigpin 31
#define counter_echopin 33

#define table_trigpin 5
#define table_echopin 4

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

long table_distance = 20;
long counter_distance =10;
int Turnning_state =0;
int state = 0;
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
  Serial.begin(9600);
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
  Serial.begin(38400);
  bluetooth.begin(38400);

  // Initialize ultrasonic sensor pins
  pinMode(table_trigpin, OUTPUT);
  pinMode(table_echopin, INPUT);
}

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
    lcd.print("jelolo");
    Waiting_For_Order();
    break;

  case Going_To_costumer:
    lcd.clear();
    lcd.print("going to cus");
    delay(2000);
    digitalWrite(table_trigpin, LOW);
  delayMicroseconds(2);
  digitalWrite(table_trigpin, HIGH);
  delayMicroseconds(10);
  digitalWrite(table_trigpin, LOW);
  long table_duration = pulseIn(table_echopin, HIGH);
  // Calculate distance in cm
  table_distance = table_duration * 0.034 / 2;
    lcd.clear();
    lcd.print(table_distance);

    Going_To_Costumer();
    break;

  case Getting_order:
      lcd.clear();
    lcd.print("going to the order");
    delay(2000);
    lcd.clear();
    lcd.print(state);

    Getting_Order();
    break;

  case going_to_counter:
    digitalWrite(counter_trigpin, LOW);
  delayMicroseconds(2);
  digitalWrite(counter_trigpin, HIGH);
  delayMicroseconds(10);
  digitalWrite(counter_trigpin, LOW);
  long counter_duration = pulseIn(counter_echopin, HIGH);
  // Calculate distance in cm
  long counter_distance = counter_duration * 0.034 / 2;
    lcd.clear();
    lcd.print(counter_distance);

    Going_To_Counter();
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
  
  if (table_distance < 8)
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
  uint16_t position = qtr.readLineBlack(sensorValues);

  // Control the direction of the robot based on sensor readings
  bool onLine = (sensorValues[2] >= 500 && sensorValues[3] >= 500 && sensorValues[4] >= 500 && sensorValues[5] >= 500) ||
                (sensorValues[3] >= 500 && sensorValues[4] >= 500 && sensorValues[5] >= 500);

  bool leftBlack = (sensorValues[0] >= 500 || sensorValues[1] >= 500 || sensorValues[2] >= 500);
  bool rightBlack = (sensorValues[5] >= 500 || sensorValues[6] >= 500 || sensorValues[7] >= 500);
 
  if (counter_distance < 15)
  {
    motorStop();
    state = Getting_food;
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
long getDistance_counter()
{
  digitalWrite(counter_trigpin, LOW);
  delayMicroseconds(2);
  digitalWrite(counter_trigpin, HIGH);
  delayMicroseconds(10);
  digitalWrite(counter_trigpin, LOW);
  long counter_duration = pulseIn(counter_echopin, HIGH);
  // Calculate distance in cm
  long counter_distance = counter_duration * 0.034 / 2;
  return counter_distance;
}
