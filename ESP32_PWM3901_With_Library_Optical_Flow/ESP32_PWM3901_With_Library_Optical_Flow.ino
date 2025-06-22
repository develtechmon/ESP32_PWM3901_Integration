#include <SPI.h>
#include "Bitcraze_PMW3901.h"

// ── PIN DEFINITIONS ─────────────────────────────────────────────────────────────
static constexpr uint8_t PIN_SCK  = 18;
static constexpr uint8_t PIN_MISO = 19;
static constexpr uint8_t PIN_MOSI = 23;
static constexpr uint8_t PIN_CS   = 5;

// Create the sensor instance on CS pin
Bitcraze_PMW3901 flow(PIN_CS);

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }

  // Initialize SPI with our pin mapping
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);

  // Try to initialize the sensor
  if (!flow.begin()) {
    Serial.println("ERROR: PMW3901 init failed! Check wiring and 3.3V supply.");  
    while (1) { delay(100); }
  }
  Serial.println("✓ PMW3901 initialized successfully");
}

void loop() {
  int16_t deltaX = 0, deltaY = 0;

  // Read motion counts since last call
  flow.readMotionCount(&deltaX, &deltaY);  // returns via pointers :contentReference[oaicite:0]{index=0}

  // Print results
  Serial.print("ΔX: "); Serial.print(deltaX);
  Serial.print("    ΔY: "); Serial.println(deltaY);

  delay(50);  // adjust update rate as needed
}
