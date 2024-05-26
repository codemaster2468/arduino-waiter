#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <QTRSensors.h>
#include <Servo.h>
#include <Keypad.h>

// Define servo motor pins
#define BASE_SERVO_PIN 46
#define MIDDLE_SERVO_PIN 48
#define END_SERVO_PIN 50
#define mosfetPin 52 // Pin to control MOSFET (possibly for power control)

// Create servo objects
Servo baseServo;
Servo middleServo;
Servo endServo;

// Initial servo angles
int baseAngle = 135;  // Base servo initial angle
int middleAngle = 55; // Middle servo initial angle
int endAngle = 0;     // End servo initial angle

// Define ultrasonic sensor pins
#define table_trigpin 3
#define table_echopin 4
#define counter_trigpin 33
#define counter_echopin 35

// Define color sensor pins
#define S0  51
#define S1  53
#define S2  45
#define S3  47
#define sensorOut 49  

// Bluetooth module pins
const int bluetoothTx = 12; // HC-05 TX pin
const int bluetoothRx = 11; // HC-05 RX pin

// Line sensor pins
int line_Sensor1 = 24;
int line_Sensor2 = 22;

// Motor control pins and speeds
int motor1speed = 10;
int motor2speed = 5;
int motor1pin1 = 9;
int motor1pin2 = 8;
int motor2pin1 = 7;
int motor2pin2 = 6;

// Keypad configuration
const byte ROWS = 4; // Number of rows in keypad
const byte COLS = 3; // Number of columns in keypad
char key; // Variable to store pressed key

// Various state and measurement variables
int Color_State = 0;
int Hamount = 0; // Number of hamburgers
int Pamount = 0; // Number of pizzas

long table_distance = 0;
long counter_distance =0;
int Turnning_state = 0;
int state = 0;
int Order = 0;
int y = 0;
int Rfrequency = 0; // Red color frequency from sensor
int Gfrequency = 0; // Green color frequency from sensor
int Bfrequency = 0; // Blue color frequency from sensor

// Define different states for the robot
const int waiting_For_order = 0;
const int going_To_costumer = 1;
const int getting_order = 2;
const int going_to_counter = 3;
const int identify_color = 4;
const int getting_food = 5;
const int going_Back_To_Customer = 6 ;
const int placing_Order = 7;
const int random_Place = 8;

// Define food types
const int No_food = 0;
const int Hamburger = 1;
const int Pizza = 2;

// Define color types
const int Red_Color = 1;
const int Blue_Color = 2;

// Define turning directions
const int Left = 1;
const int Right = 2;

// Create an object for QTR sensors
QTRSensors qtr;

const uint8_t SensorCount = 8; // Number of QTR sensors
uint16_t sensorValues[SensorCount]; // Array to hold sensor values
int blue_state = 0; // Bluetooth state

// Create a software serial object for Bluetooth communication
SoftwareSerial bluetooth(bluetoothTx, bluetoothRx);

// Create an object for the LCD display
LiquidCrystal_I2C lcd(0x27, 16, 2); // LCD I2C address and size

// Define the keymap for the keypad
char keys[ROWS][COLS] = {
    {'1', '2', '3'},
    {'4', '5', '6'},
    {'7', '8', '9'},
    {'*', '0', '#'}
};

// Connect to the row pinouts of the keypad
byte rowPins[ROWS] = {38, 36, 34, 32}; 

// Connect to the column pinouts of the keypad
byte colPins[COLS] = {30, 28, 26};     

// Create an object of keypad
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

// Function declarations for motor control and various operations
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
void Going_Back_To_Customer();
void Random_Place();
void Placing_Order();

void setup()
{
  // Initialize serial communication for debugging
  Serial.begin(9600);
  
  // Initialize the LCD
  lcd.init();
  lcd.backlight();

  // Initialize color sensor pins
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(sensorOut, INPUT);
  
  // Set frequency scaling to 20% for the color sensor
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);

  // Initialize motor control pins
  pinMode(motor1speed, OUTPUT);
  pinMode(motor2speed, OUTPUT);
  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);
  pinMode(motor2pin1, OUTPUT);
  pinMode(motor2pin2, OUTPUT);

  // Configure the QTR sensors
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A8, A9, A10, A11, A12, A13, A14, A15}, SensorCount);
  qtr.setEmitterPin(2);

  // Calibration process
  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // Indicate calibration mode with built-in LED

  // Calibrate sensors
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW); // Indicate end of calibration mode

  // Print calibration minimum values
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  // Print calibration maximum values
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(1000);

  // Initialize Bluetooth communication
  bluetooth.begin(38400);

  // Initialize ultrasonic sensor pins
  pinMode(table_trigpin, OUTPUT);
  pinMode(table_echopin, INPUT);
  pinMode(counter_trigpin, OUTPUT);
  pinMode(counter_echopin, INPUT);

  // Set up MOSFET pin
  pinMode(mosfetPin, OUTPUT);

  // Attach servo motors to their respective pins
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
    Serial.print(millis() - prevTime);
    prevTime = millis();
  }
  */

  // Main state machine to control the robot's actions based on the current state
  switch (state)
  {
    case waiting_For_order:
      lcd.clear();
      lcd.print(state); // Display current state on the LCD
      Serial.println(blue_state); // Print Bluetooth state to the Serial Monitor
      Waiting_For_Order(); // Call function to handle Waiting_For_Order state
      break;

    case going_To_costumer:
      lcd.clear();
      Serial.println(blue_state); // Print Bluetooth state to the Serial Monitor
      lcd.print(table_distance); // Display table distance on the LCD
      Going_To_Costumer(); // Call function to handle Going_To_Costumer state
      break;

    case getting_order:
      lcd.clear();
      lcd.print(state); // Display current state on the LCD
      Getting_Order(); // Call function to handle Getting_order state
      break;

    case going_to_counter:
      lcd.clear();
      lcd.print(counter_distance); // Display counter distance on the LCD
      Going_To_Counter(); // Call function to handle going_to_counter state
      break;

    case identify_color:
      lcd.clear();
      lcd.print(state); // Display current state on the LCD
      Color_Identify(); // Call function to handle Identify_color state
      break;

    case getting_food:
      lcd.clear();
      lcd.print(state); // Display current state on the LCD
      takeOrder();
      // Assuming there should be a function call here similar to others
      break;
      case going_Back_To_Customer:
      lcd.clear();
      lcd.print(state); // Display current state on the LCD
      Going_Back_To_Customer();
      break;

      case placing_Order:
      lcd.clear();
      lcd.print(state); // Display current state on the LCD
      Placing_Order();
      break;
      case random_Place:
      lcd.clear();
      lcd.print(state); // Display current state on the LCD
      Random_Place();
      break;

  }
}

// Function to handle the Waiting_For_Order state
void Waiting_For_Order()
{
  motorStop(); // Stop motors
  if (bluetooth.available() > 0)
  {
    blue_state = bluetooth.read(); // Read Bluetooth data
    Serial.print("state=");
    Serial.println(state);
  }
  else
  {
    state = waiting_For_order; // Reset state if no new data is available
  }

  if (blue_state == '1')
  {
    state = going_To_costumer; // Change state to Going_To_costumer if '1' is received
  }
}

// Function to handle the Going_To_Costumer state
void Going_To_Costumer()
{
  uint16_t position = qtr.readLineBlack(sensorValues); // Read line sensor values

  // Determine if the robot is on the line
  bool onLine = (sensorValues[2] >= 500 && sensorValues[3] >= 500 && sensorValues[4] >= 500 && sensorValues[5] >= 500) ||
                (sensorValues[3] >= 500 && sensorValues[4] >= 500 && sensorValues[5] >= 500 );

  // Check for black line on the left and right sensors
bool leftBlack = (sensorValues[0] >= 500 || sensorValues[1] >= 500 || sensorValues[2] >= 500 || sensorValues[2] >= 500 && sensorValues[1] >= 500 && sensorValues[0] >= 500 || sensorValues[1] >= 500 && sensorValues[0] >= 500 );
bool rightBlack = (sensorValues[7] >= 500 || sensorValues[6] >= 500 || sensorValues[5] >= 500 || sensorValues[5] >= 500 && sensorValues[6] >= 500 && sensorValues[7] >= 500 || sensorValues[6] >= 500 && sensorValues[7] >= 500 );
  if (getDistance_table() < 13)
  {
    motorStop(); // Stop motors if an obstacle is detected
    state = getting_order; // Change state to Getting_order
    Serial.println("Obstacle detected, stopping");
  }
  else
  {
    if (onLine) {
      Serial.println("Moving forward");
      motorForward(); // Move forward if on the line
    } else {
      if (leftBlack && !rightBlack) {
        Serial.println("Turning right");
        motorLeft(); // Turn left if black line is detected on the left
        Turnning_state = Right;
      } else if (!leftBlack && rightBlack) {
        Serial.println("Turning left");
        motorRight(); // Turn right if black line is detected on the right
        Turnning_state = Left;
      } else if (leftBlack && rightBlack) {
        Serial.println("Line split or lost");
        motorStop(); // Stop if both sides detect black (could indicate a split in the line or an error)
      } else {
        if (Turnning_state == Left) {
          motorLeft(); // Continue turning left if the last turn was left
        }
        if (Turnning_state == Right) {
          motorRight(); // Continue turning right if the last turn was right
        }
      }
    }
    Serial.println("not seeing anything");
  }
}

// Function to display messages on the LCD
void Print_Lcd_Message(char *line1, char *line2, int delay1)
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(line1); // Print first line of message
  lcd.setCursor(0, 1);
  lcd.print(line2); // Print second line of message
  delay(delay1); // Delay for specified time
}

// Function to delete the order when '*' key is pressed
void deletingOrder()
{
  if (key == '*')
  {
    Print_Lcd_Message("deleting the", "order", 0); // Display message on LCD
    Order = No_food; // Set order to No_food
  }
}

// Function to bring the order when '#' key is pressed
void BringingOrder()
{
  if (key == '#')
  {
    Print_Lcd_Message("bringing the ", "order", 0); // Display message on LCD
    state = going_to_counter; // Change state to going_to_counter
  }
}

// Function to display introduction messages
void Intro()
{
  Print_Lcd_Message("welcome to our ", "restaurant", 1500); // Display welcome message
  Print_Lcd_Message("Exclusive dining", "1 dish per table.", 3000); // Display dining policy
}

// Function to handle the Getting_Order state
void Getting_Order()
{
  Serial.println(Order); // Print current order to Serial Monitor
  delay(500);
  if (y == 0)
  {
    Intro(); // Display introductory messages
    y = y + 1;
  }

  // Prompt for order selection
  Print_Lcd_Message("1 for hamburger", "2 for pizza", 3000);
  key = keypad.waitForKey(); // Wait for key press
  delay(350);

  // Handle order selection and confirmation
  if (key == '1')
  {
    Print_Lcd_Message("you Chose 1 Hamburger. Confirm: #,Deny: *", "Hamburger.", 2000);
    Print_Lcd_Message("Confirm with #", "Deny with *", 1500);
    key = keypad.waitForKey(); // Wait for key press

    if (key == '*')
    {
      Order = No_food; // Set order to No_food if denied
      Print_Lcd_Message("hamburger order", "declined", 1500);
    }

    if (key == '#')
    {
      Order = Hamburger; // Set order to Hamburger if confirmed
      Print_Lcd_Message("confirm the :", "order of 1", 2500);
      Print_Lcd_Message("hamburger with #", " or deny with *", 500);
      key = keypad.waitForKey(); // Wait for key press
      BringingOrder(); // Call BringingOrder function
      deletingOrder(); // Call deletingOrder function
    }
  }

  if (key == '2')
  {
    Print_Lcd_Message("you Chose 1 pizza . Confirm: #,Deny: *", "Hamburger.", 1500);
    Print_Lcd_Message("Confirm with #", "Deny with *", 1000);
    key = keypad.waitForKey(); // Wait for key press

    if (key == '*')
    {
      Order = No_food; // Set order to No_food if denied
      Print_Lcd_Message("pizza order", "declined", 1500);
    }

    if (key == '#')
    {
      Order = Pizza; // Set order to Pizza if confirmed
      Print_Lcd_Message("confirm the", "order of 1", 2500);
      Print_Lcd_Message("pizza with #", " or deny with *", 500);
      BringingOrder(); // Call BringingOrder function
      deletingOrder(); // Call deletingOrder function
    }
  }
}

// Function to handle the Going_To_Counter state
void Going_To_Counter()
{
  uint16_t position = qtr.readLineBlack(sensorValues); // Read line sensor values

  // Determine if the robot is on the line
  bool onLine = (sensorValues[2] >= 500 && sensorValues[3] >= 500 && sensorValues[4] >= 500 && sensorValues[5] >= 500) ||
                (sensorValues[3] >= 500 && sensorValues[4] >= 500 && sensorValues[5] >= 500  );

  // Check for black line on the left and right sensors
bool leftBlack = (sensorValues[0] >= 500 || sensorValues[1] >= 500 || sensorValues[2] >= 500 || sensorValues[2] >= 500 && sensorValues[1] >= 500 && sensorValues[0] >= 500 || sensorValues[1] >= 500 && sensorValues[0] >= 500 );
bool rightBlack = (sensorValues[7] >= 500 || sensorValues[6] >= 500 || sensorValues[5] >= 500 || sensorValues[5] >= 500 && sensorValues[6] >= 500 && sensorValues[7] >= 500 || sensorValues[6] >= 500 && sensorValues[7] >= 500 );

  if (getDistance_Counter() < 13)
  {
    motorStop(); // Stop motors if an obstacle is detected
    state = identify_color; // Change state to Identify_color
    Serial.println("Obstacle detected, stopping");
  }
  else
  {
    if (onLine) {
      Serial.println("Moving forward");
      motorForward(); // Move forward if on the line
    } else {
      if (leftBlack && !rightBlack) {
        Serial.println("Turning right");
        motorLeft(); // Turn left if black line is detected on the left
        Turnning_state = Right;
      } else if (!leftBlack && rightBlack) {
        Serial.println("Turning left");
        motorRight(); // Turn right if black line is detected on the right
        Turnning_state = Left;
      } else if (leftBlack && rightBlack) {
        Serial.println("Line split or lost");
        motorStop(); // Stop if both sides detect black (could indicate a split in the line or an error)
      } else {
        if (Turnning_state == Left) {
          motorLeft(); // Continue turning left if the last turn was left
        }
        if (Turnning_state == Right) {
          motorRight(); // Continue turning right if the last turn was right
        }
      }
    }
    Serial.println("not seeing anything");
  }
}

void Color_Identify()
{
  delay(1000); // Small delay before starting color identification

  // Setting red filtered photodiodes to be read
  digitalWrite(S2, LOW);
  digitalWrite(S3, LOW);
  // Reading the output frequency from the sensor
  Rfrequency = pulseIn(sensorOut, LOW);
  // Remapping the frequency value to the RGB model (0 to 255)
  Rfrequency = map(Rfrequency, 250, 10, 255, 0);

  // Setting green filtered photodiodes to be read
  digitalWrite(S2, HIGH);
  digitalWrite(S3, HIGH);
  // Reading the output frequency from the sensor
  Gfrequency = pulseIn(sensorOut, LOW);
  // Remapping the frequency value to the RGB model (0 to 255)
  Gfrequency = map(Gfrequency, 300, 90, 255, 0);

  // Setting blue filtered photodiodes to be read
  digitalWrite(S2, LOW);
  digitalWrite(S3, HIGH);
  // Reading the output frequency from the sensor
  Bfrequency = pulseIn(sensorOut, LOW);
  // Remapping the frequency value to the RGB model (0 to 255)
  Bfrequency = map(Bfrequency, 250, 70, 255, 0);

  // Reading the line sensor values
  uint16_t position = qtr.readLineBlack(sensorValues);

  // Determine if the robot is on the line
  bool onLine = (sensorValues[2] >= 500 && sensorValues[3] >= 500 && sensorValues[4] >= 500 && sensorValues[5] >= 500) ||
                (sensorValues[3] >= 500 && sensorValues[4] >= 500 && sensorValues[5] >= 500);

  // Check for black line on the left and right sensors
bool leftBlack = (sensorValues[0] >= 500 || sensorValues[1] >= 500 || sensorValues[2] >= 500 || sensorValues[2] >= 500 && sensorValues[1] >= 500 && sensorValues[0] >= 500 || sensorValues[1] >= 500 && sensorValues[0] >= 500 );
bool rightBlack = (sensorValues[7] >= 500 || sensorValues[6] >= 500 || sensorValues[5] >= 500 || sensorValues[5] >= 500 && sensorValues[6] >= 500 && sensorValues[7] >= 500 || sensorValues[6] >= 500 && sensorValues[7] >= 500 );

  // Check if the detected color matches the expected color for the order
  if ((Order == Hamburger && Rfrequency > 20 && Rfrequency < 100 && Gfrequency < 0 && Gfrequency > -100 && Bfrequency < 50 && Bfrequency > -70) || 
      (Order == Pizza && Rfrequency > 10 && Rfrequency < 30 && Gfrequency < 0 && Gfrequency > -100 && Bfrequency < 50 && Bfrequency > -70))
  {
    motorStop(); // Stop the motors if the food color is detected
    state = getting_food; // Change state to Getting_food
    Serial.println("spotted the food color, stopping");
  }
  else
  {
    if (onLine) {
      Serial.println("Moving forward");
      motorForward(); // Move forward if on the line
    } else {
      if (leftBlack && !rightBlack) {
        Serial.println("Turning right");
        motorRight(); // Turn right if black line is detected on the left
        Turnning_state = Right;
      } else if (!leftBlack && rightBlack) {
        Serial.println("Turning left");
        motorLeft(); // Turn left if black line is detected on the right
        Turnning_state = Left;
      } else if (leftBlack && rightBlack) {
        Serial.println("Line split or lost");
        motorStop(); // Stop if both sides detect black (could indicate a split in the line or an error)
      } else {
        if (Turnning_state == Left) {
          motorLeft(); // Continue turning left if the last turn was left
        }
        if (Turnning_state == Right) {
          motorRight(); // Continue turning right if the last turn was right
        }
      }
    }
    Serial.println("not seeing anything");
  }
}

void motorForward()
{
  // Set motor 1 to move forward
  digitalWrite(motor1pin2, HIGH); // Enable motor 1
  digitalWrite(motor1pin1, LOW);  // Set motor 1 direction forward
  analogWrite(motor1speed, 150);  // Set motor 1 speed

  // Set motor 2 to move forward
  digitalWrite(motor2pin2, HIGH); // Enable motor 2
  digitalWrite(motor2pin1, LOW);  // Set motor 2 direction forward
  analogWrite(motor2speed, 150);  // Set motor 2 speed
}

void motorRight()
{
  // Set motor 1 to move forward
  digitalWrite(motor1pin2, HIGH); // Enable motor 1
  digitalWrite(motor1pin1, LOW);  // Set motor 1 direction forward
  analogWrite(motor1speed, 200);  // Set motor 1 speed (higher for turning)

  // Set motor 2 to stop
  digitalWrite(motor2pin2, LOW);  // Disable motor 2
  digitalWrite(motor2pin1, LOW);  // Ensure motor 2 is stopped
  analogWrite(motor2speed, 150);  // Set motor 2 speed (lower for turning)
}

void motorLeft()
{
  // Set motor 1 to stop
  digitalWrite(motor1pin2, LOW);  // Disable motor 1
  digitalWrite(motor1pin1, LOW);  // Ensure motor 1 is stopped
  analogWrite(motor1speed, 150);  // Set motor 1 speed (lower for turning)

  // Set motor 2 to move forward
  digitalWrite(motor2pin2, HIGH); // Enable motor 2
  digitalWrite(motor2pin1, LOW);  // Set motor 2 direction forward
  analogWrite(motor2speed, 200);  // Set motor 2 speed (higher for turning)
}

void motorStop()
{
  // Stop motor 1
  digitalWrite(motor1pin1, LOW);  // Disable motor 1
  digitalWrite(motor1pin2, LOW);  // Ensure motor 1 is stopped
  analogWrite(motor1speed, 0);    // Set motor 1 speed to 0

  // Stop motor 2
  digitalWrite(motor2pin1, LOW);  // Disable motor 2
  digitalWrite(motor2pin2, LOW);  // Ensure motor 2 is stopped
  analogWrite(motor2speed, 0);    // Set motor 2 speed to 0
}

void motorBackward()
{
  // Set motor 1 to move backward
  digitalWrite(motor1pin2, LOW);  // Disable motor 1 forward direction
  digitalWrite(motor1pin1, HIGH); // Enable motor 1 backward direction
  analogWrite(motor1speed, 150);  // Set motor 1 speed

  // Set motor 2 to move backward
  digitalWrite(motor2pin2, LOW);  // Disable motor 2 forward direction
  digitalWrite(motor2pin1, HIGH); // Enable motor 2 backward direction
  analogWrite(motor2speed, 150);  // Set motor 2 speed
}

long getDistance_table()
{
  // Trigger the ultrasonic sensor
  digitalWrite(table_trigpin, LOW);
  delayMicroseconds(2);
  digitalWrite(table_trigpin, HIGH);
  delayMicroseconds(10);
  digitalWrite(table_trigpin, LOW);

  // Measure the duration of the echo
  long table_duration = pulseIn(table_echopin, HIGH);

  // Calculate the distance in cm
  table_distance = table_duration * 0.034 / 2;

  return table_distance;
}

long getDistance_Counter()
{
  // Trigger the ultrasonic sensor
  digitalWrite(counter_trigpin, LOW);
  delayMicroseconds(2);
  digitalWrite(counter_trigpin, HIGH);
  delayMicroseconds(10);
  digitalWrite(counter_trigpin, LOW);

  // Measure the duration of the echo
  long counter_duration = pulseIn(counter_echopin, HIGH);

  // Calculate the distance in cm
  counter_distance = counter_duration * 0.034 / 2;

  return counter_distance;
}

void moveCraneDown() {
 for (int angle = 55; angle <= 90; angle++) {
    middleServo.write(angle); // Move the servo to the current angle
    delay(15); // Adjust this delay as needed for the desired speed of rotation
  }
  // Rotate the servo from 0 to 180 degrees
  for (int angle = 0; angle <= 360; angle++) {
    endServo.write(angle); // Move the servo to the current angle
    delay(15); // Adjust this delay as needed for the desired speed of rotation
  }
  
}

void moveCraneUp() {
  for (int angle = 360; angle >= 0; angle--) {
    endServo.write(angle); // Move the servo to the current angle
    delay(15); // Adjust this delay as needed for the desired speed of rotation
  }

  for (int angle = 90; angle >= 55; angle--) {
    middleServo.write(angle); // Move the servo to the current angle
    delay(15); // Adjust this delay as needed for the desired speed of rotation
  }
  // Rotate the servo from 0 to 180 degrees
  
}
void moveCraneRight() {
  for (int angle = 42; angle <= 180; angle++) {
    baseServo.write(angle); // Move the servo to the current angle
    delay(15); // Adjust this delay as needed for the desired speed of rotation
  }
 }


 void moveCraneLeft() {
  for (int angle = 180; angle >= 42; angle--) {
    baseServo.write(angle); // Move the servo to the current angle
    delay(15); // Adjust this delay as needed for the desired speed of rotation
  }
 }

void takeOrder(){
  // Move the crane to the left position
  moveCraneLeft();
  delay(2000);  // Delay for 2 seconds

  // Move the crane down to pick up the order
  moveCraneDown();
  digitalWrite(mosfetPin, HIGH); // Activate the MOSFET to pick up the order
  delay(2000);  // Delay for 2 seconds

  // Move the crane back up
  moveCraneUp();
  delay(2000);  // Delay for 2 seconds
  state = going_Back_To_Customer;
}



void moveCrane(int base, int middle, int end) {
  // Move the base servo to the specified angle
  baseServo.write(base);

  // Move the middle servo to the specified angle
  middleServo.write(middle);

  // Move the end servo to the specified angle
  endServo.write(end);

  delay(500);  // Delay to allow the servos to reach the desired position
}

void Going_Back_To_Customer()
{
   uint16_t position = qtr.readLineBlack(sensorValues); // Read line sensor values

  // Determine if the robot is on the line
  bool onLine = (sensorValues[2] >= 500 && sensorValues[3] >= 500 && sensorValues[4] >= 500 && sensorValues[5] >= 500) ||
                (sensorValues[3] >= 500 && sensorValues[4] >= 500 && sensorValues[5] >= 500);

  // Check for black line on the left and right sensors
bool leftBlack = (sensorValues[0] >= 500 || sensorValues[1] >= 500 || sensorValues[2] >= 500 || sensorValues[2] >= 500 && sensorValues[1] >= 500 && sensorValues[0] >= 500 || sensorValues[1] >= 500 && sensorValues[0] >= 500 );
bool rightBlack = (sensorValues[7] >= 500 || sensorValues[6] >= 500 || sensorValues[5] >= 500 || sensorValues[5] >= 500 && sensorValues[6] >= 500 && sensorValues[7] >= 500 || sensorValues[6] >= 500 && sensorValues[7] >= 500 );

  if (getDistance_table() < 13)
  {
    motorStop(); // Stop motors if an obstacle is detected
    state = placing_Order; // Change state to Getting_order
    Serial.println("Obstacle detected, stopping");
  }
  else
  {
    if (onLine) {
      Serial.println("Moving forward");
      motorForward(); // Move forward if on the line
    } else {
      if (leftBlack && !rightBlack) {
        Serial.println("Turning right");
        motorLeft(); // Turn left if black line is detected on the left
        Turnning_state = Right;
      } else if (!leftBlack && rightBlack) {
        Serial.println("Turning left");
        motorRight(); // Turn right if black line is detected on the right
        Turnning_state = Left;
      } else if (leftBlack && rightBlack) {
        Serial.println("Line split or lost");
        motorStop(); // Stop if both sides detect black (could indicate a split in the line or an error)
      } else {
        if (Turnning_state == Left) {
          motorLeft(); // Continue turning left if the last turn was left
        }
        if (Turnning_state == Right) {
          motorRight(); // Continue turning right if the last turn was right
        }
      }
    }
    Serial.println("not seeing anything");
  }


}
void Placing_Order() {
   // Move the crane to the right position
  moveCraneRight();
  delay(2000);  // Delay for 2 seconds

  // Move the crane down to place the order
  moveCraneDown();
  delay(2000);  // Delay for 2 seconds

  digitalWrite(mosfetPin, LOW);  // Deactivate the MOSFET to release the order

  // Move the crane back up
  moveCraneUp();
  delay(2000);  // Delay for 2 seconds
  state = random_Place;
}
void Random_Place()
{
   uint16_t position = qtr.readLineBlack(sensorValues); // Read line sensor values

  // Determine if the robot is on the line
  bool onLine = (sensorValues[2] >= 500 && sensorValues[3] >= 500 && sensorValues[4] >= 500 && sensorValues[5] >= 500) ||
                (sensorValues[3] >= 500 && sensorValues[4] >= 500 && sensorValues[5] >= 500);

  // Check for black line on the left and right sensors
  bool leftBlack = (sensorValues[0] >= 500 || sensorValues[1] >= 500 || sensorValues[2] >= 500 || sensorValues[2] >= 500 && sensorValues[1] >= 500 && sensorValues[0] >= 500 || sensorValues[1] >= 500 && sensorValues[0] >= 500 );
  bool rightBlack = (sensorValues[7] >= 500 || sensorValues[6] >= 500 || sensorValues[5] >= 500 || sensorValues[5] >= 500 && sensorValues[6] >= 500 && sensorValues[7] >= 500 || sensorValues[6] >= 500 && sensorValues[7] >= 500 );

  
  
   for ( int i = 0; i < 5; i++)
  {
    if (onLine) {
      Serial.println("Moving forward");
      motorForward(); // Move forward if on the line
    } else {
      if (leftBlack && !rightBlack) {
        Serial.println("Turning right");
        motorLeft(); // Turn left if black line is detected on the left
        Turnning_state = Right;
      } else if (!leftBlack && rightBlack) {
        Serial.println("Turning left");
        motorRight(); // Turn right if black line is detected on the right
        Turnning_state = Left;
      } else if (leftBlack && rightBlack) {
        Serial.println("Line split or lost");
        motorStop(); // Stop if both sides detect black (could indicate a split in the line or an error)
      } else {
        if (Turnning_state == Left) {
          motorLeft(); // Continue turning left if the last turn was left
        }
        if (Turnning_state == Right) {
          motorRight(); // Continue turning right if the last turn was right
        }
      }
    }
    Serial.println("not seeing anything");
  }
  state = waiting_For_order;
}