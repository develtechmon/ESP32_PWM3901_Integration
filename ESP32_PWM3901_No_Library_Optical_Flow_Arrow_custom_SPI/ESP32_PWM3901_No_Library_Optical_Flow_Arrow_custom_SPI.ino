#include <SPI.h>

// Pin connections for ESP32-S3 Super Mini - like addresses where wires go
#define CS_PIN   7   // Chip Select pin (changed from 5 to 7)

// SPI pins for ESP32-S3 Super Mini
static constexpr uint8_t PIN_SCK  = 3;
static constexpr uint8_t PIN_MISO = 6;
static constexpr uint8_t PIN_MOSI = 1;

// Sensor "addresses" - like room numbers in a hotel
#define MOTION_REG    0x02  // Room where motion info lives
#define X_LOW_REG     0x03  // X movement (first half)
#define X_HIGH_REG    0x04  // X movement (second half) 
#define Y_LOW_REG     0x05  // Y movement (first half)
#define Y_HIGH_REG    0x06  // Y movement (second half)

void setup() {
  Serial.begin(115200);
  Serial.println("🚀 Starting Optical Flow Sensor on ESP32-S3 Super Mini");
  
  // Set up the "conversation channel" with sensor
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);  // Start with "not talking"
  
  // Initialize SPI with custom pins for ESP32-S3 Super Mini
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, CS_PIN);
  SPI.setClockDivider(SPI_CLOCK_DIV64); // Talk slowly and clearly
  
  delay(500); // Let sensor wake up - like waiting for morning coffee
  Serial.println("✅ Sensor ready!");
}

// Talk to sensor - like knocking on a door and asking a question
byte askSensor(byte question) {
  digitalWrite(CS_PIN, LOW);   // "Hey sensor, listen!"
  SPI.transfer(question);      // Ask the question
  delayMicroseconds(100);      // Wait for sensor to think
  byte answer = SPI.transfer(0x00); // Get the answer
  digitalWrite(CS_PIN, HIGH);  // "Thanks, bye!"
  delayMicroseconds(50);
  return answer;
}

// Get movement direction as arrow - like a compass pointing where you went
String getDirectionArrow(int x, int y) {
  // If barely moving, show no arrow
  if (abs(x) < 3 && abs(y) < 3) {
    return "🔴"; // Red dot = no movement
  }
  
  // Figure out the main direction - like "which way did you go?"
  if (abs(x) > abs(y)) {
    // Moving more left/right than up/down
    return (x > 0) ? "➡️" : "⬅️";
  } else {
    // Moving more up/down than left/right  
    return (y > 0) ? "⬇️" : "⬆️";
  }
}

void loop() {
  // Ask sensor: "Did anything move?"
  byte motionHappened = askSensor(MOTION_REG);
  bool isMoving = (motionHappened & 0x80) != 0; // Check the "motion" flag
  
  if (isMoving) {
    // Get movement amounts - like asking "how much did you move?"
    byte xLow = askSensor(X_LOW_REG);
    byte xHigh = askSensor(X_HIGH_REG);
    byte yLow = askSensor(Y_LOW_REG);
    byte yHigh = askSensor(Y_HIGH_REG);
    
    // Put the two halves together - like combining puzzle pieces
    int deltaX = (int16_t)((xHigh << 8) | xLow);
    int deltaY = (int16_t)((yHigh << 8) | yLow);
    
    // Get direction arrow
    String arrow = getDirectionArrow(deltaX, deltaY);
    
    // Show the results in a friendly way
    Serial.print("Motion detected! ");
    Serial.print(arrow);
    Serial.print(" X:");
    Serial.print(deltaX);
    Serial.print(" Y:");
    Serial.println(deltaY);
    
  } else {
    Serial.println("🔴 No movement");
  }
  
  delay(200); // Don't overwhelm the sensor - like pausing between questions
}
