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
String inclinationPath;  // Changed from elevation to inclination
String fuelUsedPath;
String monoxidePath;
String vibrationPath;
String fuzzyOutputPath;

// Sensor reading and calculated values
float inclination = 0.0;        // from MPU6050 sensor fusion
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

// Sensor fusion variables
unsigned long lastSensorReadTime = 0;
float complementaryFilterAlpha = 0.98;
float currentPitchAngle = 0.0;

// Vibration measurement variables
unsigned long lastVibrationTime = 0;
volatile unsigned int vibrationCount = 0;
unsigned long vibrationMeasurementPeriod = 1000; // 1 second

// Fuzzy logic membership functions
// Input ranges
const float INCLINATION_LOW = -10.0;      // Degrees
const float INCLINATION_HIGH = 10.0;      // Degrees
const float CO_LOW = 0.0;
const float CO_HIGH = 4095.0;             // ADC max value
const float VIBRATION_LOW = 0.0;  
const float VIBRATION_HIGH = 100.0;       // Count per second

// Output ranges for servo
const int SERVO_LOW = 0;    // Low priority (minimum fuel saving)
const int SERVO_MED = 90;   // Medium priority
const int SERVO_HIGH = 180; // High priority (maximum fuel saving)

// Vibration sensor interrupt handler
void IRAM_ATTR vibrationISR() {
  vibrationCount++;
}

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
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
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

// Check WiFi connection and reconnect if needed
void checkWiFiConnection() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi disconnected, reconnecting...");
    WiFi.reconnect();
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
      delay(500);
      Serial.print(".");
      attempts++;
    }
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("WiFi reconnected");
    }
  }
}

// Read vibration using interrupt-based approach
float readVibration() {
  // Calculate vibrations per second
  unsigned long currentTime = millis();
  if (currentTime - lastVibrationTime >= vibrationMeasurementPeriod) {
    float vibrationsPerSecond = (float)vibrationCount * (1000.0 / (currentTime - lastVibrationTime));
    vibrationCount = 0;  // Reset counter
    lastVibrationTime = currentTime;
    return vibrationsPerSecond;
  }
  
  // Return last value if measurement period hasn't elapsed
  return vibrationLevel;
}

// Read CO sensor
float readCO() {
  int rawValue = analogRead(CO_SENSOR_PIN);
  return rawValue;
}

// Read inclination from MPU6050 using sensor fusion
float readInclination() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastSensorReadTime) / 1000.0;
  lastSensorReadTime = currentTime;
  
  if (deltaTime <= 0) deltaTime = 0.01; // Prevent division by zero
  
  // Calculate pitch angle from accelerometer (tan^-1(x/z))
  float accelPitch = atan2(a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0 / PI;
  
  // Integrate gyro data
  // Pitch is around Y axis (using gyro Y data)
  float gyroPitch = currentPitchAngle + g.gyro.y * deltaTime * 180.0 / PI;
  
  // Complementary filter - combine accelerometer and gyroscope data
  currentPitchAngle = complementaryFilterAlpha * gyroPitch + (1 - complementaryFilterAlpha) * accelPitch;
  
  return currentPitchAngle;
}

// Calculate fuel used (simplified for proof of concept)
float calculateFuelUsed() {
  unsigned long currentTime = millis();
  float timeDelta = (currentTime - lastUpdateTime) / 1000.0; // Convert to seconds
  
  if (lastUpdateTime == 0) {
    lastUpdateTime = currentTime;
    return totalFuelUsed;
  }
  
  // Adjust fuel consumption based on sensor readings
  float inclinationFactor = map(abs(inclination), 0, INCLINATION_HIGH, 100, 150) / 100.0;
  float vibrationFactor = map(vibrationLevel, VIBRATION_LOW, VIBRATION_HIGH, 100, 130) / 100.0;
  float coFactor = map(monoxideLevel, CO_LOW, CO_HIGH, 100, 120) / 100.0;
  
  // Calculate fuel used in this period
  float fuelIncrement = BASE_FUEL_RATE * timeDelta * inclinationFactor * vibrationFactor * coFactor;
  totalFuelUsed += fuelIncrement;
  
  lastUpdateTime = currentTime;
  return totalFuelUsed;
}

// Helper function to map values between ranges
float map(float x, float in_min, float in_max, float out_min, float out_max) {
  if (x <= in_min) return out_min;
  if (x >= in_max) return out_max;
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
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
  float inclinationNorm = constrain(map(abs(inclination), INCLINATION_LOW, INCLINATION_HIGH, 0, 100) / 100.0, 0, 1);
  float coNorm = constrain(map(monoxideLevel, CO_LOW, CO_HIGH, 0, 100) / 100.0, 0, 1);
  float vibrationNorm = constrain(map(vibrationLevel, VIBRATION_LOW, VIBRATION_HIGH, 0, 100) / 100.0, 0, 1);
  
  // Calculate membership values
  float inclLow = membershipLow(inclinationNorm, 0, 1);
  float inclMed = membershipMedium(inclinationNorm, 0, 1);
  float inclHigh = membershipHigh(inclinationNorm, 0, 1);
  
  float coLow = membershipLow(coNorm, 0, 1);
  float coMed = membershipMedium(coNorm, 0, 1);
  float coHigh = membershipHigh(coNorm, 0, 1);
  
  float vibLow = membershipLow(vibrationNorm, 0, 1);
  float vibMed = membershipMedium(vibrationNorm, 0, 1);
  float vibHigh = membershipHigh(vibrationNorm, 0, 1);
  
  // Fuzzy rules (enhanced with more specific scenarios)
  // 1. If any input is HIGH, output is HIGH priority
  float ruleHigh = max(max(inclHigh, coHigh), vibHigh);
  
  // 2. If all inputs are LOW, output is LOW priority
  float ruleLow = min(min(inclLow, coLow), vibLow);
  
  // 3. Special case: Traffic jam on hill (high inclination AND high CO)
  // Need to maintain power even in traffic when on a hill
  float ruleHillTraffic = min(inclHigh, coHigh);
  ruleHigh = max(ruleHigh, ruleHillTraffic);
  
  // 4. Special case: Idle in traffic (low vibration AND high CO AND low inclination)
  // Maximum fuel saving when idling in traffic on flat road
  float ruleIdleTraffic = min(min(vibLow, coHigh), inclLow);
  ruleLow = max(ruleLow, ruleIdleTraffic);
  
  // 5. Normal driving conditions: medium priority
  float ruleMed = max(max(inclMed, coMed), vibMed);
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
  
  // Attach interrupt for vibration sensor
  attachInterrupt(digitalPinToInterrupt(VIBRATION_SENSOR_PIN), vibrationISR, RISING);
  
  // Initialize MPU6050
  Wire.begin();
  if (!initMPU()) {
    // If MPU initialization fails, try to restart
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
  inclinationPath = databasePath + "/inclination";  // Changed from elevation
  fuelUsedPath = databasePath + "/fuelUsed";
  monoxidePath = databasePath + "/monoxideLevel";
  vibrationPath = databasePath + "/vibration";
  fuzzyOutputPath = databasePath + "/fuzzyOutput";
  
  // Initialize timing variables
  lastSensorReadTime = millis();
  lastVibrationTime = millis();
  
  Serial.println("i-FUSE System initialized!");
}

void loop() {
  // Read all sensors
  inclination = readInclination();  // Using sensor fusion for inclination
  monoxideLevel = readCO();
  vibrationLevel = readVibration();  // Using interrupt-based approach
  fuelUsed = calculateFuelUsed();
  
  // Apply fuzzy logic with enhanced rules
  fuzzyOutput = calculateFuzzyOutput();
  
  // Set servo position
  setServoPosition(fuzzyOutput);
  
  // Print to serial for debugging
  Serial.println("=== Sensor Readings ===");
  Serial.print("Inclination: "); Serial.println(inclination);
  Serial.print("CO Level: "); Serial.println(monoxideLevel);
  Serial.print("Vibration: "); Serial.println(vibrationLevel);
  Serial.print("Fuel Used: "); Serial.println(fuelUsed);
  Serial.print("Fuzzy Output: "); Serial.println(fuzzyOutput);
  Serial.println("=====================");
  
  // Check WiFi status
  checkWiFiConnection();
  
  // Send data to Firebase
  if (Firebase.ready() && (millis() - sendDataPrevMillis > timerDelay || sendDataPrevMillis == 0)) {
    sendDataPrevMillis = millis();
    
    // Send readings to database
    sendFloat(inclinationPath, inclination);
    sendFloat(monoxidePath, monoxideLevel);
    sendFloat(vibrationPath, vibrationLevel);
    sendFloat(fuelUsedPath, fuelUsed);
    sendFloat(fuzzyOutputPath, fuzzyOutput);
  }
  
  delay(100); // shorter delay for more responsive readings
}