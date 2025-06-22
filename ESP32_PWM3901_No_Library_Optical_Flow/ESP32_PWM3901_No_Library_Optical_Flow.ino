#include <SPI.h>

// Pin definitions - think of these as the "address" where each wire connects
#define CS_PIN    5   // Chip Select - like raising your hand to say "I want to talk"
#define MOSI_PIN  23  // Master Out Slave In - ESP32 talking to sensor
#define MISO_PIN  19  // Master In Slave Out - sensor talking back to ESP32
#define SCK_PIN   18  // Clock - the "metronome" that keeps everyone in sync

// PWM3901 Register addresses - like house numbers for different data rooms
#define PWM3901_PRODUCT_ID        0x00
#define PWM3901_MOTION            0x02
#define PWM3901_DELTA_X_L         0x03
#define PWM3901_DELTA_X_H         0x04
#define PWM3901_DELTA_Y_L         0x05
#define PWM3901_DELTA_Y_H         0x06
#define PWM3901_SQUAL             0x07

class PWM3901 {
private:
  int csPin;
  
  // Write to a register - like sending a letter to a specific address
  void writeRegister(uint8_t reg, uint8_t value) {
    digitalWrite(csPin, LOW);  // "Hey sensor, listen up!"
    SPI.transfer(reg | 0x80);  // Address with write flag (like putting "WRITE" on envelope)
    SPI.transfer(value);       // The actual data
    digitalWrite(csPin, HIGH); // "Okay, I'm done talking"
    delayMicroseconds(50);     // Give it a moment to process
  }
  
  // Read from a register - like asking for mail from a specific address
  uint8_t readRegister(uint8_t reg) {
    digitalWrite(csPin, LOW);   // "Hey sensor, I want to ask you something"
    SPI.transfer(reg & 0x7F);   // Address with read flag (like asking "what's at this address?")
    delayMicroseconds(100);     // Wait for the sensor to find the info
    uint8_t data = SPI.transfer(0x00); // "Send me the data please"
    digitalWrite(csPin, HIGH);  // "Thanks, we're done"
    delayMicroseconds(19);      // Brief pause before next operation
    return data;
  }

public:
  // Constructor - setting up our communication channel
  PWM3901(int chipSelect = CS_PIN) {
    csPin = chipSelect;
  }
  
  // Initialize the sensor - like introducing yourself before a conversation
  bool begin() {
    pinMode(csPin, OUTPUT);
    digitalWrite(csPin, HIGH);
    
    // Start SPI - set up our communication protocol
    SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, csPin);
    SPI.setDataMode(SPI_MODE3);  // Specific "handshake" style for this sensor
    SPI.setClockDivider(SPI_CLOCK_DIV64); // Slow and steady conversation speed
    
    delay(100); // Let the sensor wake up - like waiting for someone to answer the door
    
    // Check if sensor is there and responding
    uint8_t productId = readRegister(PWM3901_PRODUCT_ID);
    Serial.print("Product ID: 0x");
    Serial.println(productId, HEX);
    
    // PWM3901 should return 0x49 - like checking someone's ID card
    if (productId != 0x49) {
      Serial.println("PWM3901 not found!");
      return false;
    }
    
    Serial.println("PWM3901 initialized successfully!");
    return true;
  }
  
  // Read motion data - like asking "what did you see?"
  struct MotionData {
    bool motion;      // Did anything move?
    int16_t deltaX;   // How much movement left/right
    int16_t deltaY;   // How much movement up/down
    uint8_t quality;  // How confident are you? (0-255)
  };
  
  MotionData readMotion() {
    MotionData data;
    
    // Check if there's motion - like asking "did you see anything interesting?"
    data.motion = readRegister(PWM3901_MOTION) & 0x80;
    
    // Get the movement amounts - like asking "how much and which direction?"
    uint8_t deltaXLow = readRegister(PWM3901_DELTA_X_L);
    uint8_t deltaXHigh = readRegister(PWM3901_DELTA_X_H);
    uint8_t deltaYLow = readRegister(PWM3901_DELTA_Y_L);
    uint8_t deltaYHigh = readRegister(PWM3901_DELTA_Y_H);
    
    // Combine high and low bytes - like putting together a two-part message
    data.deltaX = (int16_t)((deltaXHigh << 8) | deltaXLow);
    data.deltaY = (int16_t)((deltaYHigh << 8) | deltaYLow);
    
    // Get quality metric - like asking "how sure are you about this?"
    data.quality = readRegister(PWM3901_SQUAL);
    
    return data;
  }
};

// Create our sensor object - like having a translator ready
PWM3901 flowSensor;

void setup() {
  Serial.begin(115200);
  Serial.println("Starting PWM3901 Optical Flow Sensor Test");
  
  // Initialize the sensor - like saying hello and making sure it's working
  if (!flowSensor.begin()) {
    Serial.println("Failed to initialize sensor!");
    while(1); // Stop here if sensor isn't working
  }
  
  delay(1000); // Give everything a moment to settle
}

void loop() {
  // Read the latest motion data - like checking your messages
  PWM3901::MotionData motion = flowSensor.readMotion();
  
  // Print the results in a friendly format
  Serial.print("Motion: ");
  Serial.print(motion.motion ? "YES" : "NO");
  Serial.print(" | X: ");
  Serial.print(motion.deltaX);
  Serial.print(" | Y: ");
  Serial.print(motion.deltaY);
  Serial.print(" | Quality: ");
  Serial.print(motion.quality);
  Serial.println();
  
  delay(100); // Don't spam the sensor - give it time to breathe
}