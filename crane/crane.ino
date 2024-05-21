#include <Servo.h>

// Define servo pin numbers
#define BASE_SERVO_PIN 7
#define MIDDLE_SERVO_PIN 6
#define END_SERVO_PIN 8
#define mosfetPin 22


// Define servo objects
Servo baseServo;
Servo middleServo;
Servo endServo;

// Define initial angles for servos
int baseAngle = 135;  // 90 degrees (middle position)
int middleAngle = 90;
int endAngle = 0;

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


// Define function to move the crane parts
void moveCrane(int base, int middle, int end) {
  baseServo.write(base);
  middleServo.write(middle);
  endServo.write(end);
  delay(500); // Delay to allow the servos to reach the desired position
}

void setup() {
  // Attach servos to pins
    pinMode(mosfetPin, OUTPUT);

  baseServo.attach(BASE_SERVO_PIN);
  middleServo.attach(MIDDLE_SERVO_PIN);
  endServo.attach(END_SERVO_PIN);

  // Move crane to initial position
  moveCrane(baseAngle, middleAngle, endAngle);

    Serial.begin(9600);

}

int x = 0;

void loop() {

  if (x == 0) {
    takeOrder();

    giveOrder();
    
    // Turn on the electromagnet
  

  // Turn off the electromagnet
  digitalWrite(mosfetPin, LOW);
  delay(5000); // Wait for 5 seconds

    x = 1;
  }
    
  

  // Move crane forward
 /* 
  moveCrane(180, 90, 90); // Move base to 180 degrees, middle to 90 degrees, end to 90 degrees
  delay(2000); // Wait for 2 seconds

  moveCrane(180, 135, 45); // Move base to 180 degrees, middle to 135 degrees, end to 45 degrees
  delay(2000); // Wait for 2 seconds

  // Move crane backward
  moveCrane(0, 135, 45); // Move base to 0 degrees, middle to 135 degrees, end to 45 degrees
  delay(2000); // Wait for 2 seconds

  moveCrane(0, 90, 90); // Move base to 0 degrees, middle to 90 degrees, end to 90 degrees
  delay(2000); // Wait for 2 seconds
  */
}
