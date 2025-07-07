#include <SPI.h>
#include "Bitcraze_PMW3901.h"

// Your existing pin definitions...
static constexpr uint8_t PIN_SCK  = 3;
static constexpr uint8_t PIN_MISO = 6;
static constexpr uint8_t PIN_MOSI = 1;
static constexpr uint8_t PIN_CS   = 7;

Bitcraze_PMW3901 flow(PIN_CS);

// Movement threshold - like the minimum push needed to register a direction
const int MOVEMENT_THRESHOLD = 3;

// Get direction indicator - like a compass showing where you're headed
String getDirectionIndicator(int16_t deltaX, int16_t deltaY) {
  // Check if movement is significant enough to matter
  if (abs(deltaX) < MOVEMENT_THRESHOLD && abs(deltaY) < MOVEMENT_THRESHOLD) {
    return "🔴 STATIONARY";
  }
  
  // Determine primary direction based on larger component
  // Think of it like asking "am I moving more sideways or front-back?"
  if (abs(deltaX) > abs(deltaY)) {
    // Horizontal movement dominates
    if (deltaX > 0) {
      return "➡️  RIGHT";
    } else {
      return "⬅️  LEFT";
    }
  } else {
    // Vertical movement dominates
    if (deltaY > 0) {
      return "⬇️  BACKWARD";  // Positive Y typically means moving away from sensor
    } else {
      return "⬆️  FORWARD";   // Negative Y typically means moving toward sensor
    }
  }
}

// Get detailed movement info - like a GPS showing speed and direction
String getMovementDetails(int16_t deltaX, int16_t deltaY) {
  if (abs(deltaX) < MOVEMENT_THRESHOLD && abs(deltaY) < MOVEMENT_THRESHOLD) {
    return "No significant movement";
  }
  
  String details = "";
  
  // Add magnitude info - like showing how fast you're going
  int magnitude = sqrt(deltaX * deltaX + deltaY * deltaY);
  details += "Speed: " + String(magnitude) + " | ";
  
  // Add component breakdown
  details += "X:" + String(deltaX) + " Y:" + String(deltaY);
  
  return details;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }

  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);

  if (!flow.begin()) {
    Serial.println("ERROR: PMW3901 init failed! Check wiring and 3.3V supply.");  
    while (1) { delay(100); }
  }
  Serial.println("✓ PMW3901 initialized successfully");
  Serial.println("Move the sensor to see directional indicators...\n");
}

void loop() {
  int16_t deltaX = 0, deltaY = 0;
  flow.readMotionCount(&deltaX, &deltaY);

  // Get direction and movement info
  String direction = getDirectionIndicator(deltaX, deltaY);
  String details = getMovementDetails(deltaX, deltaY);
  
  // Print formatted output
  Serial.print(direction);
  Serial.print(" | ");
  Serial.println(details);

  delay(50);
}
