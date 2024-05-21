#include <QTRSensors.h>

// Define number of sensors (8 for QTR-8A)
const int NUM_SENSORS = 8;

// Define analog pins connected to QTR-8A (modify as needed)
const int sensorPins[NUM_SENSORS] = {A0, A1, A2, A3, A4, A5, A6, A7};

// Create a QTRSensorsRC object for QTR-8A
QTRSensorsRC qtra((unsigned char[]) {A0, A1, A2, A3, A4, A5, A6, A7}, NUM_SENSORS);

void setup() {
  // Initialize serial communication
  Serial.begin(9600);

  // Calibrate the QTR-8A sensors (follow calibration procedure)
  Serial.println("Calibrating QTR-8A...");
  qtra.calibrate();
  Serial.println("Calibration complete!");
}

void loop() {
  // Declare local array to store raw readings
  unsigned int sensorValues[NUM_SENSORS];

  // Read calibrated sensor values
  qtra.read(sensorValues);

  // Print sensor readings
  Serial.print("Sensor Readings: ");
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(sensorValues[i]);
    Serial.print(" "); // Add space between readings
  }
  Serial.println(); // Add newline after all readings

  // Delay between readings (optional)
  delay(100);
}
