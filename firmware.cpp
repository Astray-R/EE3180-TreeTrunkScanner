#include <Arduino.h>
#include <Wire.h>
#include <VL53L1X.h>
#include <ESP32Servo.h>
#include <math.h>

// Pin Definitions
#define SlidestepPin 18  // STEP pin for the slider motor
#define SlidedirPin 19   // DIR pin for the slider motor
#define RotstepPin 4     // STEP pin for the rotator motor
#define RotdirPin 5      // DIR pin for the rotator motor
#define RESET_BUTTON_PIN 15  // Reset button pin
#define CLK 34           // Encoder CLK pin (S1? of Rotary Encoder, whacked both of them into any pin)
#define DT 35            // Encoder DT pin (S2? of Rotary Encoder)
#define SlideswitchPin 26
#define RotswitchPin 25

// ToF Sensor Pins
VL53L1X sensor;

// Servo motor for vertical movement
Servo verticalServo;
const int verticalPin = 13; // Pin for vertical servo

// Motor movement step size and delay (timing control)
const int stepDelay = 2000;  // Stepper delay for speed control
const int maxdist = 400;     // Maximum distance for the movement in steps
const int hOffset = 80;      // Horizontal offset for vertical scan
const int vOffset = 65;      // Vertical offset for scan range

// Platform Movement Variables
int currentdist = 0;
volatile int position = 0;  // Stores the encoder position

// Initialize ToF sensor
bool initializeSensor()
{
  Wire.begin();
  if (!sensor.init()) {
    Serial.println("ERROR: VL53L1X failed to initialize.");
    return false;
  }
  sensor.setDistanceMode(VL53L1X::Medium);
  sensor.setMeasurementTimingBudget(20000);
  sensor.startContinuous(20);
  delay(100);
  return true;
}

// Move the slider (platform forward/backward)
void moveSlide(int steps, bool direction)
{
  digitalWrite(SlidedirPin, direction ? HIGH : LOW);
  for (int i = 0; i < steps; i++) {
    digitalWrite(SlidestepPin, HIGH);
    delayMicroseconds(stepDelay);
    digitalWrite(SlidestepPin, LOW);
    delayMicroseconds(stepDelay);
  }
}

// Move the lazy susan (rotation)
void moveRotate(int steps, bool direction)
{
  digitalWrite(RotdirPin, direction ? HIGH : LOW);
  for (int i = 0; i < steps; i++) {
    digitalWrite(RotstepPin, HIGH);
    delayMicroseconds(stepDelay);
    digitalWrite(RotstepPin, LOW);
    delayMicroseconds(stepDelay);
  }
}

// Reset system
void resetSystem()
{
  Serial.println("ESP32 RESETTING...");
  ESP.restart();
}

// Home the platform
void homePlatform()
{
  digitalWrite(SlidedirPin, LOW);
  while (digitalRead(SlideswitchPin) == LOW) {
    moveSlide(150, false);  // Move backward until limit switch is pressed
  }
}

// Home the rotation
void homeRotation()
{
  digitalWrite(RotdirPin, LOW);
  while (digitalRead(RotswitchPin) == LOW) {
    moveRotate(50, false);  // Move backward to home position
  }
}

void setup()
{
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

  Serial.println("READY");
}

void loop()
{
  int direction = 1;  // Variable to alternate direction of rotation
  
  // Iterate through vertical angles (to scan tree surface)
  for (int verticalAngle = -25; verticalAngle <= 5; verticalAngle++) {
    verticalServo.write(verticalAngle + vOffset);
    delay(10);  // Allow the servo to reach the position
    
    // Iterate through rotation angles for Lazy Susan
    for (int rotationStep = 45; rotationStep <= 135; rotationStep += 2.25) {
      // Move the Lazy Susan (rotator)
      moveRotate(1, direction > 0);  // Move one step in the set direction
      
      // Collect distance data from the sensor
      while (!sensor.dataReady()) {
        Serial.println("Sensor not responding. Reinitializing...");
        initializeSensor();
      }
      
      int distance = sensor.read();
      if (distance > 0 && distance < 1000 && !sensor.timeoutOccurred()) {
        // Calculate 3D coordinates based on angle and distance
        float hRad = rotationStep * PI / 180.0;
        float vRad = (verticalAngle) * PI / 180.0;
        float x = distance * cos(vRad) * cos(hRad); 
        float y = distance * cos(vRad) * sin(hRad); 
        float z = distance * sin(vRad); 

        // Print 3D point data
        Serial.printf("%.2f,%.2f,%.2f\n", x, y, z);
      }

      // Move the platform forward by one step after each rotation
      moveSlide(1, true);  // Move platform forward one step
      delay(100);
    }

    direction *= -1;  // Alternate rotation direction after each pass
  }

  Serial.println("SCAN_DONE");
  Serial.flush();  // Ensure data is flushed for Python reading
  delay(1000);  // Short delay before the next scan
}
