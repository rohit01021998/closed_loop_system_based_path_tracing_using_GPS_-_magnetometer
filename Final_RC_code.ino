#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_LIS2MDL.h>
#include <math.h>

// Pin definitions for RC car control
const int enA = 3;
const int in1 = 8;
const int in2 = 9;
const int enB = 5;
const int in3 = 7;
const int in4 = 6;
int flag=0;

// GPS setup
static const int RXPin = 10, TXPin = 11;
static const uint32_t GPSBaud = 9600;
TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);

// Magnetometer setup
Adafruit_LIS2MDL lis2mdl;
float magXmin = -50.10, magXmax = 9.75;
float magYmin = -80.85, magYmax = -22.23;
float magZmin = -37.05, magZmax = 24.75;
const float declinationAngle = 0; // Replace with your magnetic declination angle in degrees

// Destination coordinates as pointers
double destinationLatitude = 10.9030013;
double destinationLongitude = 76.8963357;

void setup() {
  // Initialize Serial for Bluetooth and Serial Monitor
  Serial1.begin(9600);
  Serial.begin(9600);

  // Initialize motor control pins
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // Set initial state of the motors
  stopMotors();

  // Initialize GPS
  ss.begin(GPSBaud);

  // Initialize magnetometer
  if (!lis2mdl.begin()) {
    Serial.println("Could not find a valid LIS2MDL sensor, check wiring!");
    while (1);
  }
  Serial.println("LIS2MDL sensor found!");

  // Allow GPS to acquire fix
  delay(2000); // Adjust as needed
}

void loop() {
  // Bluetooth control
  if (Serial1.available()) {
    char command = Serial1.read();

    // Control the motors based on the command
    if (command == 'F') {
      moveForward();
      Serial.println("Moving Forward");
    } else if (command == 'B') {
      moveBackward();
      Serial.println("Moving Backward");
    } else if (command == 'L') {
      turnLeft();
      Serial.println("Turning Left");
    } else if (command == 'R') {
      turnRight();
      Serial.println("Turning Right");
    } else if (command == 'W') {
      int count = 0;
      while(count<10){
        saveDestination();
        count++;
      }
    } else if (command == 'U') {
      while(flag==0) {
        navigateToDestination();
      }
      flag=0;
    } else {
      stopMotors();
      //Serial.println("Stopping");
    }
  }
}

void moveForward() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enA, 255); // Adjust speed with PWM
  analogWrite(enB, 255);
}

void moveBackward() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enA, 255); // Adjust speed with PWM
  analogWrite(enB, 255);
}

void turnLeft() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enA, 255); // Adjust speed with PWM
  analogWrite(enB, 255);
}

void turnRight() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enA, 255); // Adjust speed with PWM
  analogWrite(enB, 255);
}

void stopMotors() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  analogWrite(enA, 0);
  analogWrite(enB, 0);
}

void saveDestination() {
  // Override everything and stop motors
  stopMotors();

  // Attempt to read GPS data for up to 5 seconds (adjust as needed)
  unsigned long startTime = millis();
  while (millis() - startTime < 5000) { // Wait for up to 5 seconds
    while (ss.available() > 0)
      gps.encode(ss.read());

    if (gps.location.isValid()) {
      destinationLatitude = gps.location.lat();
      destinationLongitude = gps.location.lng();
      Serial.println("Success: Destination co-ordinates saved!");
      return; // Exit the function once valid data is obtained
    }
  }

  // If valid data was not obtained within 5 seconds
  Serial.println("Error: Destination co-ordinates not saved!");
}

void displayHeadingBearingAndDistance(double destinationLatitude, double destinationLongitude) {
  // Override everything and stop motors
  stopMotors();

  // Read current GPS location
  while (ss.available() > 0)
    gps.encode(ss.read());

  // Read magnetometer data
  readMagnetometer();

  if (gps.location.isValid()) {
    double currentLatitude = gps.location.lat();
    double currentLongitude = gps.location.lng();

    float headingToDestination = TinyGPSPlus::courseTo(
      currentLatitude, currentLongitude,
      destinationLatitude, destinationLongitude
    );

    float distanceToDestination = TinyGPSPlus::distanceBetween(
      currentLatitude, currentLongitude,
      destinationLatitude, destinationLongitude
    );

    Serial.print(F("Current Location: "));
    Serial.print(currentLatitude, 6);
    Serial.print(F(", "));
    Serial.println(currentLongitude, 6);

    Serial.print(F("Heading to Destination: "));
    Serial.println(headingToDestination, 6); // Print heading angle with 6 decimal places

    Serial.print(F("Distance to Destination: "));
    Serial.print(distanceToDestination / 1000.0, 6); // Print distance in kilometers
    Serial.println(F(" km"));
    Serial.print(distanceToDestination, 6); // Print distance in meters
    Serial.println(F("m"));
  } else {
    Serial.println(F("GPS location is not valid"));
  }
}

void readMagnetometer() {
  // Read magnetometer data
  sensors_event_t event;
  lis2mdl.getEvent(&event);

  // Get the raw magnetometer values
  float magX = event.magnetic.x;
  float magY = event.magnetic.y;
  float magZ = event.magnetic.z;

  // Apply calibration offsets
  magX = (magX - (magXmin + magXmax) / 2.0) / ((magXmax - magXmin) / 2.0);
  magY = (magY - (magYmin + magYmax) / 2.0) / ((magYmax - magYmin) / 2.0);
  magZ = (magZ - (magZmin + magZmax) / 2.0) / ((magZmax - magZmin) / 2.0);

  // Calculate heading (bearing) from magnetometer
  float headingMag = atan2(magY, magX);
  headingMag += declinationAngle;

  // Convert radians to degrees
  if (headingMag < 0) headingMag += 2 * PI;
  if (headingMag > 2 * PI) headingMag -= 2 * PI;
  float headingDegreesMag = (headingMag * 180 / PI);

  // Output the heading (bearing)
  Serial.print(F("Magnetometer Heading: "));
  Serial.println(headingDegreesMag);
}

void millisDelay(int outer_iterations, int inner_iterations) {
    int i, j;

    // Outer loop
    for (i = 0; i < outer_iterations; i++) {
        int k = 0;
        for (j = 0; j < inner_iterations; j++) {
            // Do nothing, just wait
            k = k + 1;
        }
    }
}

void navigateToDestination() {
  // Read current GPS location
  while (ss.available() > 0)
    gps.encode(ss.read());

  if (gps.location.isValid()) {
    double currentLatitude = gps.location.lat();
    double currentLongitude = gps.location.lng();

    float headingToDestination = TinyGPSPlus::courseTo(
      currentLatitude, currentLongitude,
      destinationLatitude, destinationLongitude
    );

    float distanceToDestination = TinyGPSPlus::distanceBetween(
      currentLatitude, currentLongitude,
      destinationLatitude, destinationLongitude
    );

    // Read magnetometer data
    readMagnetometer();

    // Calculate heading (bearing) from magnetometer
    sensors_event_t event;
    lis2mdl.getEvent(&event);

    float magX = event.magnetic.x;
    float magY = event.magnetic.y;

    // Apply calibration offsets
    magX = (magX - (magXmin + magXmax) / 2.0) / ((magXmax - magXmin) / 2.0);
    magY = (magY - (magYmin + magYmax) / 2.0) / ((magYmax - magYmin) / 2.0);

    float headingMag = atan2(magY, magX);
    headingMag += declinationAngle;

    if (headingMag < 0) headingMag += 2 * PI;
    if (headingMag > 2 * PI) headingMag -= 2 * PI;
    float headingDegreesMag = (headingMag * 180 / PI);

    // Calculate the difference between the desired heading and the current heading
    float headingDifference = headingToDestination - headingDegreesMag;

    if (headingDifference > 180) headingDifference -= 360;
    if (headingDifference < -180) headingDifference += 360;

    // Output the heading information
    Serial.print(F("Desired Heading: "));
    Serial.println(headingToDestination);
    Serial.print(F("Current Heading: "));
    Serial.println(headingDegreesMag);
    Serial.print(F("Heading Difference: "));
    Serial.println(headingDifference);

    // Decide on the direction to move
    if (distanceToDestination < 2.0) { // Check if the distance to the destination is less than 2 meters
      stopMotors();
      Serial.println("Destination Reached");
      flag = 1;
    } else if (headingDifference > 20) { // Adjust the threshold as needed
      turnRight();
      Serial.println("Right");
    } else if (headingDifference < -20) { // Adjust the threshold as needed
      turnLeft();
      Serial.println("Left");
    } else {
      moveForward();
      Serial.println("Forward");
    }
  } else {
    Serial.println(F("GPS location is not valid"));
  }
}
