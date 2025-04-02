#include <Arduino.h>
#include <Wire.h>
#include <VL53L1X.h>
#include <ESP32Servo.h>
#include <math.h>

// Pinout for ESP32 Dev Module
#define SlidestepPin 18       // STEP pin for the slider motor
#define SlidedirPin 19        // DIR pin for the slider motor
#define RotstepPin 4          // STEP pin for the rotator motor
#define RotdirPin 5           // DIR pin for the rotator motor
#define RESET_BUTTON_PIN 15   // Reset button pin
#define CLK 34                // Encoder CLK pin (S1? of Rotary Encoder, whacked both of them into any pin)
#define DT 35                 // Encoder DT pin (S2? of Rotary Encoder)
#define SlideswitchPin 26
#define RotswitchPin 25

// ToF Sensor Pins
VL53L1X sensor;

// Servo motor for vertical movement
Servo verticalServo;
const int verticalPin = 13;   // Pin for vertical servo

// Motor movement step size and delay (timing control)
const int stepDelay = 2000;   // Stepper delay for speed control
const int maxdist = 400;      // Maximum distance for the movement in steps
const int hOffset = 80;       // Horizontal offset for vertical scan
const int vOffset = 65;       // Vertical offset for scan range

// Initialize ToF sensor
bool initializeSensor() {
  Wire.begin();
  if (!sensor.init()) {
    Serial.println("ERROR: VL53L1X failed to initialize.");
    return false;
  }
  sensor.setDistanceMode(VL53L1X::Medium);  // up to 76cm in strong ambient light, 290cm in the dark
  sensor.setMeasurementTimingBudget(15000); // microseconds, duration of each measurement
  sensor.startContinuous(15);               // milliseconds between each measurement start
  delay(100);
  return true;
}

// Move the slider (platform forward/backward)
void moveSlide(int steps) {
  digitalWrite(SlidedirPin, HIGH);
  for (int i = 0; i < steps; i++) {
    digitalWrite(SlidestepPin, HIGH);
    delayMicroseconds(stepDelay);
    digitalWrite(SlidestepPin, LOW);
    delayMicroseconds(stepDelay);
  }
}

// Move the lazy susan (rotation)
void moveRotate(int steps) {
  digitalWrite(RotdirPin, HIGH);
  for (int i = 0; i < steps; i++) {
    digitalWrite(RotstepPin, HIGH);
    delayMicroseconds(stepDelay);
    digitalWrite(RotstepPin, LOW);
    delayMicroseconds(stepDelay);
  }
}

// Reset system
void resetSystem() {
  Serial.println("ESP32 RESETTING...");
  ESP.restart();
}

// Home the platform by moving back until it hits the Slide Switch
void homePlatform() {
  digitalWrite(SlidedirPin, LOW);
  while (digitalRead(SlideswitchPin) == LOW) {
    for (int i = 0; i < 150; i++) {
      digitalWrite(SlidestepPin, HIGH);
      delayMicroseconds(2000);
      digitalWrite(SlidestepPin, LOW);
      delayMicroseconds(2000);
    }
    Serial.println("moving platform back!");
  }
}

// Home the rotation by rotating back until it hits the Rotation Switch
void homeRotation() {
  digitalWrite(RotdirPin, LOW);
  while (digitalRead(RotswitchPin) == LOW) {
    for (int i = 0; i < 50; i++) {
      digitalWrite(RotstepPin, HIGH);
      delayMicroseconds(1000);
      digitalWrite(RotstepPin, LOW);
      delayMicroseconds(1000);
    }
    Serial.println("moving rotation back!");
  }
}

void setup() {
  Serial.begin(9600);

  // Set pin modes
  pinMode(SlidedirPin, OUTPUT);
  pinMode(SlidestepPin, OUTPUT);
  pinMode(RotdirPin, OUTPUT);
  pinMode(RotstepPin, OUTPUT);
  pinMode(RESET_BUTTON_PIN, INPUT_PULLUP);
  pinMode(SlideswitchPin, INPUT_PULLUP);
  pinMode(RotswitchPin, INPUT_PULLUP);

  verticalServo.attach(verticalPin);  // Attach servo for vertical movement

  // Initialize ToF sensor
  initializeSensor();

  homePlatform();  // Home the platform
  homeRotation();  // Home the rotation motor
  moveRotate(120); // Rotate to starting angle

  Serial.println("READY");
}

void loop() {
  int direction = 1;  // Variable to alternate direction of rotation, 1 is moving away from starting limit switch

  float x,y,z; // 3D space coordinates for raw data
  int verticalAngle; // angle of TOF sensor with respect to horizontal plane
  
  // one step = one degree of rotation
  int totalStep = 31; // 31 steps total: 15 steps to the left, one center, 15 steps to the right

  // iterate through platform steps
  for (int platformStep = 0; platformStep < totalStep; platformStep++) {
    Serial.println("Current Step: " + platformStep);

    moveRotate(4);  // rotate lazy susan by one step
    moveSlide(163); // move slider by one step
    delay(100);

    // For each step, rotate sensor vertically from bottom to top to scan tree trunk surface
    int minVerticalAngle = -25; // angle in degrees
    int maxVerticalAngle = 25;  
    for (verticalAngle = minVerticalAngle; verticalAngle <= maxVerticalAngle; verticalAngle++) {
      verticalServo.write(-verticalAngle + vOffset); // move motor to position, negative due to motor positioning
      delay(10); // let the motor cook to completion

      while (!sensor.dataReady()) { // checks if sensor is NOT ready
        Serial.println("Sensor not responding. Reinitializing...");
        initializeSensor();
      }
      
      int distance = sensor.read();
    
      int maxTOFDistance = 2000; // in front of TOF sensor in millimeters (mm)
      if (distance > 0 && distance < maxTOFDistance && !sensor.timeoutOccurred()) {
        // Calculate 3D coordinates based on angle and distance
        float hRad = (platformStep + 75) * PI / 180.0; // +75 degrees (HW: EXPLAIN WHY)
        float vRad = (verticalAngle) * PI / 180.0;

        // spherical coordinates (distance, hRad, vRad) -> cartesian coordinates (x,y,z)
        x = distance * cos(vRad) * cos(hRad)+ 26 * platformStep; //increase 26mm every stop
        y = distance * cos(vRad) * sin(hRad); 
        z = distance * sin(vRad); 

        // Print 3D point data
        Serial.printf("%.2f,%.2f,%.2f\n", x, y, z);
      }
    }
    delay(100);
  }
  direction *= -1;  // Alternate rotation direction after each pass

  Serial.println("SCAN_DONE");
  Serial.flush();   // Ensure data is flushed for Python reading
  homePlatform();   // Home the platform
  homeRotation();   // Home the rotation motor
  moveRotate(110);  // Rotate to Starting Angle
  delay(1000);      // Short delay before the next scan
}
