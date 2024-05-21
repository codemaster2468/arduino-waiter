#include <QTRSensors.h>
QTRSensors qtr;

int motor1speed = 10;
int motor2speed = 5;
int motor1pin1 = 9;
int motor1pin2 = 8;
int motor2pin1 = 7;
int motor2pin2 = 6;
int Turnning_state = 0;
const int Left = 1;
const int Right = 2;
const uint8_t SensorCount = 8; // Number of QTR sensors
uint16_t sensorValues[SensorCount]; // Array to hold sensor values
void motorForward();
void motorRight();
void motorLeft();
void motorStop();
void motorBackward();
void setup() {
  pinMode(motor1speed, OUTPUT);
  pinMode(motor2speed, OUTPUT);
  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);
  pinMode(motor2pin1, OUTPUT);
  pinMode(motor2pin2, OUTPUT);
  // put your setup code here, to run once:
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


}

void loop() {
  uint16_t position = qtr.readLineBlack(sensorValues); // Read line sensor values

  // Determine if the robot is on the line
  bool onLine = (sensorValues[2] >= 500 && sensorValues[3] >= 500 && sensorValues[4] >= 500 && sensorValues[5] >= 500) ||
                (sensorValues[3] >= 500 && sensorValues[4] >= 500 && sensorValues[5] >= 500 ) ||
                (sensorValues[3] >= 500 && sensorValues[4] >= 500) ||
                (sensorValues[4] >= 500 && sensorValues[5] >= 500) ||
                (sensorValues[0] >= 500 && sensorValues[1] >= 500 && sensorValues[2] >= 500 && sensorValues[3] >= 500 && sensorValues[4] >= 500 && sensorValues[5] >= 500 && sensorValues[6] >= 500 && sensorValues[7] >= 500) ||
                (sensorValues[0] >= 500 && sensorValues[1] >= 500 && sensorValues[2] >= 500 && sensorValues[3] >= 500 && sensorValues[4] >= 500 && sensorValues[5] >= 500 && sensorValues[6] >= 500) ||
                (sensorValues[1] >= 500 && sensorValues[2] >= 500 && sensorValues[3] >= 500 && sensorValues[4] >= 500 && sensorValues[5] >= 500 && sensorValues[6] >= 500 && sensorValues[7] >= 500);


  // Check for black line on the left and right sensors
  bool leftBlack = (sensorValues[0] >= 500 || sensorValues[1] >= 500 || sensorValues[2] >= 500);
  bool rightBlack = (sensorValues[5] >= 500 || sensorValues[6] >= 500 || sensorValues[7] >= 500);

  
  
  
    if (onLine) {
      Serial.println("Moving forward");
      motorForward(); // Move forward if on the line
    } else {
      if (leftBlack && !rightBlack) {
        Serial.println("Turning right");
        motorLeft(); // Turn left if black line is detected on the left
        Turnning_state = Left;
      } else if (!leftBlack && rightBlack) {
        Serial.println("Turning left");
        motorRight(); // Turn right if black line is detected on the right
        Turnning_state = Right;
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
