#include <Arduino.h>
#include <math.h>

// Header for VL53L1X ToF Sensor via I2C
#include <Wire.h>
#include <VL53L1X.h>

// Header for Servo Motors
#include <ESP32Servo.h>

// Header for MQTT
#include <WiFi.h>
#include <PubSubClient.h>

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

// Wi-Fi credentials for ToF ESP32
const char* ssid = "ee014_esp32";
const char* password = "abcd1234";
const char* mqtt_server = "10.66.42.18"; // MQTT BROKER IP ADDRESS

// Not sure what this does yet, will document later
WifiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;

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

  // Initialize Wifi for MQTT
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  Serial.println("READY");
}

// copied over from https://randomnerdtutorials.com/esp32-mqtt-publish-subscribe-arduino-ide/

void setup_wifi() {
  delay(10);

  // Start by connecting to a Wi-Fi Network
  Serial.println();
  Serial.print("Connecting to " + ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;
  
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ee014")) { // REPLACE WITH CLIENT NAME
      Serial.println("connected");
      // Subscribe
      client.subscribe("Tree");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 3 seconds");
      delay(3000); // Wait 3 seconds before retrying
    }
  }
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  
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
    int minVerticalAngle = -35; // angle in degrees
    int maxVerticalAngle = 35;  
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
        Serial.println("%.2f,%.2f,%.2f", x, y, z);
        client.publish("Tree", x);
        client.publish("Tree", y);
        client.publish("Tree", z);
      }
    }
    delay(100);

    if (platformStep == 0  platformStep == 15  platformStep == 30) {
      Serial.println("Triggering Pi capture!");
      client.publish("Tree", "capture");
      delay(100);  // wait for Pi to capture
    }
  }
  direction *= -1;  // Alternate rotation direction after each pass

  Serial.println("SCAN_DONE");
  Serial.flush();   // Ensure data is flushed for Python reading
  homePlatform();   // Home the platform
  homeRotation();   // Home the rotation motor
  moveRotate(110);  // Rotate to Starting Angle
  delay(1000);      // Short delay before the next scan
}
