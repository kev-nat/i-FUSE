#include <Firebase_ESP_Client.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ESP32Servo.h>
#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>

// Provide the token generation process info
#include "addons/TokenHelper.h"
// Provide the RTDB payload printing info and other helper functions
#include "addons/RTDBHelper.h"

// Network credentials
#define WIFI_SSID "K"
#define WIFI_PASSWORD "vqfx1117"

// Firebase project API Key
#define API_KEY "DB_API_KEY"
#define USER_EMAIL "USER_EMAIL"
#define USER_PASSWORD "USER_PASS"
#define DATABASE_URL "DB_URL"

// Pin definitions
#define CO_SENSOR_PIN 34        // CO sensor on GPIO34 (ADC1_6)
#define VIBRATION_SENSOR_PIN 19 // Vibration sensor digital pin
#define SERVO_PIN 13            // Servo motor control pin

// Define Firebase objects
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

// MPU6050 object
Adafruit_MPU6050 mpu;

// Servo object
Servo fuelSaveServo;

// Variable to save USER UID
String uid;

// Variables to save database paths
String databasePath;
String elevationPath;
String fuelUsedPath;
String monoxidePath;
String vibrationPath;
String fuzzyOutputPath;

// Sensor reading and calculated values
float elevation = 0.0;          // from MPU6050 y-axis
float monoxideLevel = 0.0;      // from CO sensor
float vibrationLevel = 0.0;     // from vibration sensor
float fuelUsed = 0.0;           // calculated based on other inputs
float fuzzyOutput = 0.0;        // final output from fuzzy logic

// Timer variables (send new readings every 10 seconds)
unsigned long sendDataPrevMillis = 0;
unsigned long timerDelay = 10000;

// Fuel consumption simulation variables
unsigned long lastUpdateTime = 0;
float totalFuelUsed = 0.0;
const float BASE_FUEL_RATE = 0.05; // Base fuel consumption rate per second

// Fuzzy logic membership functions
// Input ranges
const float ELEVATION_LOW = -5.0;
const float ELEVATION_HIGH = 5.0;
const float CO_LOW = 0.0;
const float CO_HIGH = 4095.0;
const float VIBRATION_LOW = 0.0;  
const float VIBRATION_HIGH = 10000.0;

// Output ranges for servo
const int SERVO_LOW = 0;    // Low priority (minimum fuel saving)
const int SERVO_MED = 90;   // Medium priority
const int SERVO_HIGH = 180; // High priority (maximum fuel saving)

// Initialize WiFi
void initWiFi() {
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(1000);
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
}

// Initialize MPU6050
bool initMPU() {
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    return false;
  }
  Serial.println("MPU6050 Found!");
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  return true;
}

// Write float values to the database
void sendFloat(String path, float value) {
  if (Firebase.RTDB.setFloat(&fbdo, path.c_str(), value)) {
    Serial.print("Writing value: ");
    Serial.print(value);
    Serial.print(" on path: ");
    Serial.println(path);
  } else {
    Serial.println("FAILED");
    Serial.println("REASON: " + fbdo.errorReason());
  }
}

// Read vibration sensor
float readVibration() {
  long measurement = pulseIn(VIBRATION_SENSOR_PIN, HIGH);
  if (measurement > VIBRATION_HIGH) measurement = VIBRATION_HIGH;
  return measurement;
}

// Read CO sensor
float readCO() {
  int rawValue = analogRead(CO_SENSOR_PIN);
  return rawValue;
}

// Read elevation from MPU6050 (using Y-axis acceleration)
float readElevation() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  return a.acceleration.y; // Using Y-axis as elevation indicator
}

// Calculate fuel used (simulated based on inputs)
float calculateFuelUsed() {
  unsigned long currentTime = millis();
  float timeDelta = (currentTime - lastUpdateTime) / 1000.0; // Convert to seconds
  
  if (lastUpdateTime == 0) {
    lastUpdateTime = currentTime;
    return totalFuelUsed;
  }
  
  // Adjust fuel consumption based on sensor readings
  float elevationFactor = map(abs(elevation), 0, ELEVATION_HIGH, 100, 150) / 100.0;
  float vibrationFactor = map(vibrationLevel, VIBRATION_LOW, VIBRATION_HIGH, 100, 130) / 100.0;
  float coFactor = map(monoxideLevel, CO_LOW, CO_HIGH, 100, 120) / 100.0;
  
  // Calculate fuel used in this period
  float fuelIncrement = BASE_FUEL_RATE * timeDelta * elevationFactor * vibrationFactor * coFactor;
  totalFuelUsed += fuelIncrement;
  
  lastUpdateTime = currentTime;
  return totalFuelUsed;
}

// Fuzzy membership function for LOW
float membershipLow(float value, float lowLimit, float highLimit) {
  float midpoint = (lowLimit + highLimit) / 2;
  
  if (value <= lowLimit) return 1.0;
  if (value >= midpoint) return 0.0;
  
  return (midpoint - value) / (midpoint - lowLimit);
}

// Fuzzy membership function for MEDIUM
float membershipMedium(float value, float lowLimit, float highLimit) {
  float range = highLimit - lowLimit;
  float midpoint = lowLimit + range / 2;
  float quarterRange = range / 4;
  
  if (value <= midpoint - quarterRange || value >= midpoint + quarterRange) return 0.0;
  if (value == midpoint) return 1.0;
  
  if (value < midpoint) {
    return (value - (midpoint - quarterRange)) / quarterRange;
  } else {
    return ((midpoint + quarterRange) - value) / quarterRange;
  }
}

// Fuzzy membership function for HIGH
float membershipHigh(float value, float lowLimit, float highLimit) {
  float midpoint = (lowLimit + highLimit) / 2;
  
  if (value <= midpoint) return 0.0;
  if (value >= highLimit) return 1.0;
  
  return (value - midpoint) / (highLimit - midpoint);
}

// Apply fuzzy logic to determine output priority
float calculateFuzzyOutput() {
  // Normalize inputs to 0-1 range
  float elevationNorm = constrain(map(abs(elevation), ELEVATION_LOW, ELEVATION_HIGH, 0, 100) / 100.0, 0, 1);
  float coNorm = constrain(map(monoxideLevel, CO_LOW, CO_HIGH, 0, 100) / 100.0, 0, 1);
  float vibrationNorm = constrain(map(vibrationLevel, VIBRATION_LOW, VIBRATION_HIGH, 0, 100) / 100.0, 0, 1);
  
  // Calculate membership values
  float elevLow = membershipLow(elevationNorm, 0, 1);
  float elevMed = membershipMedium(elevationNorm, 0, 1);
  float elevHigh = membershipHigh(elevationNorm, 0, 1);
  
  float coLow = membershipLow(coNorm, 0, 1);
  float coMed = membershipMedium(coNorm, 0, 1);
  float coHigh = membershipHigh(coNorm, 0, 1);
  
  float vibLow = membershipLow(vibrationNorm, 0, 1);
  float vibMed = membershipMedium(vibrationNorm, 0, 1);
  float vibHigh = membershipHigh(vibrationNorm, 0, 1);
  
  // Fuzzy rules (simplified)
  // 1. If any input is HIGH, output is HIGH priority
  float ruleHigh = max(max(elevHigh, coHigh), vibHigh);
  
  // 2. If all inputs are LOW, output is LOW priority
  float ruleLow = min(min(elevLow, coLow), vibLow);
  
  // 3. Otherwise, output is MEDIUM priority
  float ruleMed = max(max(elevMed, coMed), vibMed);
  if (ruleHigh < 0.3 && ruleLow < 0.3) {
    ruleMed = max(ruleMed, 0.5);  // Strengthen medium rule if neither high nor low are strong
  }
  
  // Defuzzification (centroid method simplified)
  float weightedSum = (ruleLow * SERVO_LOW + ruleMed * SERVO_MED + ruleHigh * SERVO_HIGH);
  float sumOfWeights = (ruleLow + ruleMed + ruleHigh);
  
  if (sumOfWeights > 0) {
    return weightedSum / sumOfWeights;
  } else {
    return SERVO_MED;  // Default to medium if all rules have zero weight
  }
}

// Set servo position based on fuzzy output
void setServoPosition(float position) {
  int servoPos = constrain(position, SERVO_LOW, SERVO_HIGH);
  fuelSaveServo.write(servoPos);
  Serial.print("Setting servo to: ");
  Serial.println(servoPos);
}

void setup() {
  Serial.begin(115200);
  
  // Initialize sensors
  pinMode(VIBRATION_SENSOR_PIN, INPUT);
  
  // Initialize MPU6050
  Wire.begin();
  if (!initMPU()) {
    // If MPU initialization fails, try to restart or handle gracefully
    ESP.restart();
  }
  
  // Initialize Servo
  fuelSaveServo.attach(SERVO_PIN);
  fuelSaveServo.write(SERVO_MED); // Start at medium position
  
  // Initialize WiFi
  initWiFi();
  
  // Configure Firebase
  config.api_key = API_KEY;
  auth.user.email = USER_EMAIL;
  auth.user.password = USER_PASSWORD;
  config.database_url = DATABASE_URL;
  Firebase.reconnectWiFi(true);
  fbdo.setResponseSize(4096);
  config.token_status_callback = tokenStatusCallback; //see addons/TokenHelper.h
  config.max_token_generation_retry = 5;
  Firebase.begin(&config, &auth);
  
  // Getting the user UID might take a few seconds
  Serial.println("Getting User UID");
  while ((auth.token.uid) == "") {
    Serial.print('.');
    delay(1000);
  }
  
  // Print user UID
  uid = auth.token.uid.c_str();
  Serial.print("User UID: ");
  Serial.println(uid);
  
  // Update database paths
  databasePath = "/UsersData/" + uid;
  elevationPath = databasePath + "/elevation";
  fuelUsedPath = databasePath + "/fuelUsed";
  monoxidePath = databasePath + "/monoxideLevel";
  vibrationPath = databasePath + "/vibration";
  fuzzyOutputPath = databasePath + "/fuzzyOutput";
  
  Serial.println("System initialized!");
}

void loop() {
  // Read all sensors
  elevation = readElevation();
  monoxideLevel = readCO();
  vibrationLevel = readVibration();
  fuelUsed = calculateFuelUsed();
  
  // Apply fuzzy logic
  fuzzyOutput = calculateFuzzyOutput();
  
  // Set servo position
  setServoPosition(fuzzyOutput);
  
  // Print to serial for debugging
  Serial.println("=== Sensor Readings ===");
  Serial.print("Elevation: "); Serial.println(elevation);
  Serial.print("CO Level: "); Serial.println(monoxideLevel);
  Serial.print("Vibration: "); Serial.println(vibrationLevel);
  Serial.print("Fuel Used: "); Serial.println(fuelUsed);
  Serial.print("Fuzzy Output: "); Serial.println(fuzzyOutput);
  Serial.println("=====================");
  
  // Send data to Firebase
  if (Firebase.ready() && (millis() - sendDataPrevMillis > timerDelay || sendDataPrevMillis == 0)) {
    sendDataPrevMillis = millis();
    
    // Send readings to database
    sendFloat(elevationPath, elevation);
    sendFloat(monoxidePath, monoxideLevel);
    sendFloat(vibrationPath, vibrationLevel);
    sendFloat(fuelUsedPath, fuelUsed);
    sendFloat(fuzzyOutputPath, fuzzyOutput);
  }
  
  delay(200); // delay for stability
}
