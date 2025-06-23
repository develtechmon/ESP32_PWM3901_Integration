#include <Wire.h>
#include <ESP32Servo.h>
#include <SPI.h>
#include <VL53L0X.h>
#include <math.h>
#include "Bitcraze_PMW3901.h"
#include <Adafruit_NeoPixel.h>

// Define the pin for the built-in LED
#define LED_PIN 48  
#define NUM_LEDS 1

// ===== PPM Definitions =====
#define PPM_PIN 13             
#define NUM_CHANNELS 8         
#define PPM_SYNC_THRESHOLD 3000 
#define CHANNEL_MIN 1000       
#define CHANNEL_MAX 2000       

// Define your custom I2C pins
#define CUSTOM_SDA_PIN 4   
#define CUSTOM_SCL_PIN 5   

// ===== Position Hold Sensor Pins =====
#define FLOW_CS_PIN    21   // PWM3901 Chip Select

// ===== Position Hold Variables =====
VL53L0X distanceSensor;
Bitcraze_PMW3901 flowSensor(FLOW_CS_PIN);  // Create flow sensor object

// Position tracking - like keeping track of where you are on a map
float positionX = 0;        // How far left/right from start point
float positionY = 0;        // How far forward/back from start point
float targetPositionX = 0;  // Where we want to be (X)
float targetPositionY = 0;  // Where we want to be (Y)
float targetAltitude = 0;   // How high we want to stay

// Position hold PID controllers - like autopilot assistants
float positionPIDX = 0;     // X-axis position correction
float positionPIDY = 0;     // Y-axis position correction  
float altitudePID = 0;      // Height correction

// Position hold gains - how aggressively to correct position
float KP_position = 0.8;    // How quickly to react to position error
float KI_position = 0.1;    // How much to remember past errors
float KD_position = 0.2;    // How much to predict future errors

// Altitude hold gains - TUNED FOR STABILITY
float KP_altitude = 3.5;    // Strong response to height error
float KI_altitude = 0.8;    // Remember altitude drift over time
float KD_altitude = 0.15;   // Smooth out height oscillations

// Enhanced altitude variables for better control
float altitudeVelocity = 0;           // How fast we're climbing/descending
float prevAltitude = 0;               // Previous altitude reading
float baseThrottle = 1500;            // Neutral throttle for hovering
float altitudeDeadband = 2.0;         // Ignore small altitude errors (cm)

// Error tracking for PID
float prevErrorX = 0, prevErrorY = 0, prevErrorAlt = 0;
float integralX = 0, integralY = 0, integralAlt = 0;

bool positionHoldMode = false;  // Are we in position hold mode?
bool altitudeHoldMode = false;  // Separate altitude hold mode
float currentAltitude = 0;      // Current height from ground
float filteredAltitude = 0;     // Smoothed altitude reading

// ===== Arming/Disarming Variables =====
bool armed = false;  // Drone armed state
unsigned long armDisarmTimer = 0;
const unsigned long armHoldTime = 1000; // milliseconds required to hold stick

// ===== NeoPixel LED =====
Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// ===== Original Flight Controller Variables =====
uint32_t LoopTimer;
volatile float MotorInput1, MotorInput2, MotorInput3, MotorInput4;
volatile float RatePitch, RateRoll, RateYaw;
float RateCalibrationPitch, RateCalibrationRoll, RateCalibrationYaw,AccXCalibration,AccYCalibration,AccZCalibration;

volatile int ReceiverValue[NUM_CHANNELS] = {1500,1500,1000,1500,1500,1500,1500,1500};
volatile int channelIndex = 0;
volatile unsigned long lastTime = 0;

int ESCfreq = 500;
int channelValues[NUM_CHANNELS];
float PAngleRoll=2;       float PAnglePitch=PAngleRoll;
float IAngleRoll=0.5;     float IAnglePitch=IAngleRoll;
float DAngleRoll=0.007;   float DAnglePitch=DAngleRoll;

float PRateRoll = 0.625;  float PRatePitch = PRateRoll;
float IRateRoll = 2.1;    float IRatePitch = IRateRoll;
float DRateRoll = 0.0088; float DRatePitch = DRateRoll;

float PRateYaw = 4;
float IRateYaw = 3;
float DRateYaw = 0;

int ThrottleIdle = 1170;
int ThrottleCutOff = 1000;
int led_time = 100;

volatile float PtermRoll, ItermRoll, DtermRoll, PIDOutputRoll;
volatile float PtermPitch, ItermPitch, DtermPitch, PIDOutputPitch;
volatile float PtermYaw, ItermYaw, DtermYaw, PIDOutputYaw;

volatile float DesiredRateRoll, DesiredRatePitch, DesiredRateYaw;
volatile float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
volatile float InputRoll, InputThrottle, InputPitch, InputYaw;
volatile float PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw;
volatile float PrevItermRateRoll, PrevItermRatePitch, PrevItermRateYaw;

volatile float AccX, AccY, AccZ;
volatile float AngleRoll, AnglePitch;
volatile float KalmanAngleRoll=0, KalmanUncertaintyAngleRoll=2*2;
volatile float KalmanAnglePitch=0, KalmanUncertaintyAnglePitch=2*2;
volatile float Kalman1DOutput[]={0,0};
volatile float DesiredAngleRoll, DesiredAnglePitch;
volatile float ErrorAngleRoll, ErrorAnglePitch;
volatile float PrevErrorAngleRoll, PrevErrorAnglePitch;
volatile float PrevItermAngleRoll, PrevItermAnglePitch;

float Voltage;

Servo mot1, mot2, mot3, mot4;
const int mot1_pin = 11;
const int mot2_pin = 10;
const int mot3_pin = 9;
const int mot4_pin = 8;

const float t = 0.004; 

// ===== Position Hold Functions - like having a personal navigator =====
void initializePositionSensors() {
  Serial.println("🚀 Starting position hold sensors...");
  
  // Initialize PWM3901 optical flow sensor using the library
  if (flowSensor.begin()) {
    Serial.println("✅ PWM3901 optical flow initialized");
  } else {
    Serial.println("❌ PWM3901 failed to initialize");
  }
  
  // Initialize VL53L0X distance sensor
  if (distanceSensor.init()) {
    Serial.println("✅ VL53L0X initialized");
    distanceSensor.setTimeout(500);
    distanceSensor.startContinuous();
  } else {
    Serial.println("❌ VL53L0X failed to initialize");
  }
  
  Serial.println("✅ Position sensors ready!");
}

// Update position tracking - like updating your location on a map
void updatePosition() {
  // Read optical flow movement using the library - much simpler!
  int16_t deltaX, deltaY;
  flowSensor.readMotionCount(&deltaX, &deltaY);
  
  // Convert flow to actual movement (this needs calibration for your specific setup)
  // Think of this like converting "mouse movement" to actual distance
  float movementX = deltaX * 0.01; // Scale factor - you'll need to tune this
  float movementY = deltaY * 0.01; // Scale factor - you'll need to tune this
  
  // Update our position estimate
  positionX += movementX;
  positionY += movementY;
  
  // Enhanced altitude reading with filtering for stability
  float rawAltitude = distanceSensor.readRangeContinuousMillimeters() / 10.0; // Convert to cm
  
  if (distanceSensor.timeoutOccurred() || rawAltitude > 400) {
    // Sensor error or out of range - keep previous reading
    rawAltitude = currentAltitude;
  }
  
  // Apply simple low-pass filter - like smoothing out bumpy road
  // This removes noise and makes altitude control much more stable
  filteredAltitude = (filteredAltitude * 0.85) + (rawAltitude * 0.15);
  currentAltitude = filteredAltitude;
  
  // Calculate altitude velocity (rate of climb/descent)
  altitudeVelocity = (currentAltitude - prevAltitude) / t;
  prevAltitude = currentAltitude;
}

// Enhanced altitude hold PID controller - like cruise control for height
void calculateAltitudeHold() {
  if (!altitudeHoldMode) return;
  
  // Calculate altitude error with deadband - ignore tiny errors
  float errorAlt = targetAltitude - currentAltitude;
  
  // Apply deadband - like ignoring small steering corrections in a car
  if (abs(errorAlt) < altitudeDeadband) {
    errorAlt = 0;
  }
  
  // Altitude PID calculation - much more sophisticated
  float proportionalAlt = KP_altitude * errorAlt;
  
  // Integral term with windup protection
  integralAlt += errorAlt * t;
  integralAlt = constrain(integralAlt, -100, 100); // Prevent integral windup
  float integralTerm = KI_altitude * integralAlt;
  
  // Derivative term using altitude velocity for smoother control
  float derivativeTerm = KD_altitude * (-altitudeVelocity); // Negative because we want to oppose velocity
  
  // Combine all terms
  altitudePID = proportionalAlt + integralTerm + derivativeTerm;
  
  // Limit altitude correction to prevent crazy throttle changes
  altitudePID = constrain(altitudePID, -300, 300);
  
  // If we're way off, reset integral to prevent windup
  if (abs(errorAlt) > 20) {
    integralAlt = 0;
  }
}
// Position hold PID controller - like an autopilot that keeps you in place
void calculatePositionHold() {
  if (!positionHoldMode) return;
  
  // Calculate position errors - "how far are we from where we want to be?"
  float errorX = targetPositionX - positionX;
  float errorY = targetPositionY - positionY;
  
  // X-axis position PID (left/right correction)
  float proportionalX = KP_position * errorX;
  integralX += errorX * t;
  integralX = constrain(integralX, -10, 10); // Don't let it get too crazy
  float derivativeX = (errorX - prevErrorX) / t;
  positionPIDX = proportionalX + (KI_position * integralX) + (KD_position * derivativeX);
  positionPIDX = constrain(positionPIDX, -15, 15); // Limit correction strength
  
  // Y-axis position PID (forward/back correction)
  float proportionalY = KP_position * errorY;
  integralY += errorY * t;
  integralY = constrain(integralY, -10, 10);
  float derivativeY = (errorY - prevErrorY) / t;
  positionPIDY = proportionalY + (KI_position * integralY) + (KD_position * derivativeY);
  positionPIDY = constrain(positionPIDY, -15, 15);
  
  // Remember errors for next time
  prevErrorX = errorX;
  prevErrorY = errorY;
}

// ===== PPM Interrupt Handler =====
void IRAM_ATTR ppmInterruptHandler() {
  unsigned long currentTime = micros();
  unsigned long pulseWidth = currentTime - lastTime;
  lastTime = currentTime;

  if (pulseWidth > PPM_SYNC_THRESHOLD) {
    channelIndex = 0;
  } else if (channelIndex < NUM_CHANNELS) {
    if(pulseWidth < CHANNEL_MIN) pulseWidth = CHANNEL_MIN;
    else if(pulseWidth > CHANNEL_MAX) pulseWidth = CHANNEL_MAX;
    ReceiverValue[channelIndex] = pulseWidth;
    channelIndex++;
  }
}

void battery_voltage(void) {
  Voltage=(float)analogRead(1)/237;
}

void read_receiver(int *channelValues) {
  noInterrupts();
  for (int i = 0; i < NUM_CHANNELS; i++) {
    channelValues[i] = ReceiverValue[i];
  }
  interrupts();
}

void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
    KalmanState=KalmanState + (t*KalmanInput);
    KalmanUncertainty=KalmanUncertainty + (t*t*4*4);
    float KalmanGain=KalmanUncertainty * 1/(1*KalmanUncertainty + 3 * 3);
    KalmanState=KalmanState+KalmanGain * (KalmanMeasurement-KalmanState);
    KalmanUncertainty=(1-KalmanGain) * KalmanUncertainty;
    Kalman1DOutput[0]=KalmanState; 
    Kalman1DOutput[1]=KalmanUncertainty;
}

void setup() {
  Serial.begin(115200);

  // Initialize NeoPixel LED
  strip.begin();
  strip.show(); // Turn all pixels off initially

  pinMode(2, OUTPUT);
  pinMode(48, OUTPUT);

  for (int i = 0; i < 6; i++) {
    digitalWrite(48, HIGH); delay(led_time);
    digitalWrite(48, LOW); delay(led_time);
  }
  
  pinMode(PPM_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PPM_PIN), ppmInterruptHandler, FALLING);
  delay(100);

  Wire.setClock(400000);
  Wire.begin(CUSTOM_SDA_PIN, CUSTOM_SCL_PIN);
  delay(250);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  delay(1000);
  mot1.attach(mot1_pin, 1000, 2000); delay(1000); mot1.setPeriodHertz(ESCfreq); delay(100);
  mot2.attach(mot2_pin, 1000, 2000); delay(1000); mot2.setPeriodHertz(ESCfreq); delay(100);
  mot3.attach(mot3_pin, 1000, 2000); delay(1000); mot3.setPeriodHertz(ESCfreq); delay(100);
  mot4.attach(mot4_pin, 1000, 2000); delay(1000); mot4.setPeriodHertz(ESCfreq); delay(100);

  mot1.writeMicroseconds(1000);
  mot2.writeMicroseconds(1000);
  mot3.writeMicroseconds(1000);
  mot4.writeMicroseconds(1000);

  // Initialize position hold sensors
  initializePositionSensors();

  RateCalibrationRoll=-4.74;
  RateCalibrationPitch=0.79;
  RateCalibrationYaw=-0.88;
  AccXCalibration=0.04;
  AccYCalibration=0.01;
  AccZCalibration=0.07;

  battery_voltage();

  if (Voltage < 11.6) {
    // Red LED for low battery
    strip.setPixelColor(0, strip.Color(255, 0, 0));  
    strip.show();
    mot1.writeMicroseconds(ThrottleCutOff);
    mot2.writeMicroseconds(ThrottleCutOff);
    mot3.writeMicroseconds(ThrottleCutOff);
    mot4.writeMicroseconds(ThrottleCutOff);
    PrevErrorRateRoll = PrevErrorRatePitch = PrevErrorRateYaw = 0;
    PrevItermRateRoll = PrevItermRatePitch = PrevItermRateYaw = 0;
    PrevErrorAngleRoll = PrevErrorAnglePitch = 0;
    PrevItermAngleRoll = PrevItermAnglePitch = 0;
    return;
  }

  // Green LED - system ready but disarmed
  strip.setPixelColor(0, strip.Color(0, 255, 0));
  strip.show();
  delay(500);
  
  strip.setPixelColor(0, strip.Color(0, 0, 0));
  strip.show();

  LoopTimer = micros();
}

void loop() {
  read_receiver(channelValues);

  // ===== Arming/Disarming Logic =====
  if (channelValues[2] < 1050) { // Throttle stick down
    if (!armed && channelValues[3] > 1900) { // Yaw stick right to arm
      if (armDisarmTimer == 0) {
        armDisarmTimer = millis();
      } else if (millis() - armDisarmTimer > armHoldTime) {
        armed = true;
        
        // Yellow LED (armed)
        strip.setPixelColor(0, strip.Color(255, 255, 0));
        strip.show();
        delay(1000);
        
        strip.setPixelColor(0, strip.Color(0, 0, 0));
        strip.show();
        
        Serial.println("🔓 Drone ARMED");
        armDisarmTimer = 0;
      }
    } else if (armed && channelValues[3] < 1100) { // Yaw stick left to disarm
      if (armDisarmTimer == 0) {
        armDisarmTimer = millis();
      } else if (millis() - armDisarmTimer > armHoldTime) {
        armed = false;
        
        // White LED (disarmed)
        strip.setPixelColor(0, strip.Color(255, 255, 255));
        strip.show();
        delay(1000);
        
        strip.setPixelColor(0, strip.Color(0, 0, 0));
        strip.show();
        
        Serial.println("🔒 Drone DISARMED");
        armDisarmTimer = 0;
      }
    } else {
      armDisarmTimer = 0;
    }
  } else {
    armDisarmTimer = 0;
  }
  
  // If not armed, stop all motors and exit
  if (!armed) {
    MotorInput1 = MotorInput2 = MotorInput3 = MotorInput4 = ThrottleCutOff;
    mot1.writeMicroseconds(MotorInput1);
    mot2.writeMicroseconds(MotorInput2);
    mot3.writeMicroseconds(MotorInput3);
    mot4.writeMicroseconds(MotorInput4);
    
    while (micros() - LoopTimer < (t * 1000000));
    LoopTimer = micros();
    return;
  }

  // Update position tracking
  updatePosition();
  
  // Check for altitude hold mode (channel 6)
  if (channelValues[5] > 1700) { // Channel 6 switch high
    if (!altitudeHoldMode) {
      altitudeHoldMode = true;
      targetAltitude = currentAltitude;
      baseThrottle = channelValues[2]; // Remember current throttle as baseline
      integralAlt = 0; // Reset altitude integral
      Serial.println("🔒 Altitude Hold ACTIVATED");
    }
  } else {
    if (altitudeHoldMode) {
      altitudeHoldMode = false;
      Serial.println("🔓 Altitude Hold DEACTIVATED");
      integralAlt = 0;
    }
  }
  
  // Check if position hold mode should be activated (channel 5)
  if (channelValues[4] > 1700) { // Channel 5 switch high
    if (!positionHoldMode) {
      // Just entered position hold mode - set current position as target
      positionHoldMode = true;
      targetPositionX = positionX;
      targetPositionY = positionY;
      if (!altitudeHoldMode) { // Only set target altitude if not already in altitude hold
        targetAltitude = currentAltitude;
      }
      Serial.println("🔒 Position Hold ACTIVATED");
    }
  } else {
    if (positionHoldMode) {
      positionHoldMode = false;
      Serial.println("🔓 Position Hold DEACTIVATED");
      // Reset PID integrators
      integralX = integralY = 0;
    }
  }
  
  // Calculate altitude hold corrections (runs independently)
  calculateAltitudeHold();
  
  // Calculate position hold corrections
  calculatePositionHold();
  
  // ===== Original IMU Reading Code =====
  Wire.beginTransmission(0x68);
  Wire.write(0x1A); Wire.write(0x05); Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1C); Wire.write(0x10); Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x3B); Wire.endTransmission(); 
  Wire.requestFrom(0x68,6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();
  Wire.beginTransmission(0x68);
  Wire.write(0x1B); Wire.write(0x8); Wire.endTransmission();                                                   
  Wire.beginTransmission(0x68);
  Wire.write(0x43); Wire.endTransmission();
  Wire.requestFrom(0x68,6);
  int16_t GyroX=Wire.read()<<8 | Wire.read();
  int16_t GyroY=Wire.read()<<8 | Wire.read();
  int16_t GyroZ=Wire.read()<<8 | Wire.read();
  
  RateRoll=(float)GyroX/65.5;
  RatePitch=(float)GyroY/65.5;
  RateYaw=(float)GyroZ/65.5;
  AccX=(float)AccXLSB/4096;
  AccY=(float)AccYLSB/4096;
  AccZ=(float)AccZLSB/4096;

  RateRoll  -= RateCalibrationRoll;
  RatePitch -= RateCalibrationPitch;
  RateYaw   -= RateCalibrationYaw;
  AccX      -= AccXCalibration;
  AccY      -= AccYCalibration;
  AccZ      -= AccZCalibration;

  AngleRoll  = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * 57.29;
  AnglePitch = -atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * 57.29;
  
  kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
  KalmanAngleRoll=Kalman1DOutput[0]; KalmanUncertaintyAngleRoll=Kalman1DOutput[1];
  kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
  KalmanAnglePitch=Kalman1DOutput[0]; KalmanUncertaintyAnglePitch=Kalman1DOutput[1];

  KalmanAngleRoll = constrain(KalmanAngleRoll, -20, 20);
  KalmanAnglePitch = constrain(KalmanAnglePitch, -20, 20);

  // ===== Modified Control Input Processing =====
  if (positionHoldMode) {
    // In position hold mode, use position corrections instead of stick inputs
    DesiredAngleRoll = positionPIDX;   // Position correction becomes desired angle
    DesiredAnglePitch = positionPIDY;  // Position correction becomes desired angle
  } else {
    // Normal angle mode - use stick inputs
    DesiredAngleRoll = 0.1 * (ReceiverValue[0] - 1500);
    DesiredAnglePitch = 0.1 * (ReceiverValue[1] - 1500);
  }
  
  // Throttle handling with altitude hold
  if (altitudeHoldMode) {
    // Use base throttle + altitude correction
    InputThrottle = baseThrottle + altitudePID;
    // Allow pilot to adjust target altitude with small stick movements
    float throttleStickInput = ReceiverValue[2] - baseThrottle;
    if (abs(throttleStickInput) > 50) { // Significant stick movement
      targetAltitude += throttleStickInput * 0.01; // Adjust target altitude
      baseThrottle = ReceiverValue[2]; // Update base throttle
    }
  } else {
    // Normal throttle control
    InputThrottle = ReceiverValue[2];
  }
  
  DesiredRateYaw = 0.15 * (ReceiverValue[3] - 1500);

  // ===== Rest of Original PID Code (unchanged) =====
  ErrorAngleRoll = DesiredAngleRoll - KalmanAngleRoll;
  PtermRoll = PAngleRoll * ErrorAngleRoll;
  ItermRoll = PrevItermAngleRoll + (IAngleRoll * (ErrorAngleRoll + PrevErrorAngleRoll) * (t / 2));
  ItermRoll = constrain(ItermRoll, -400, 400);
  DtermRoll = DAngleRoll * ((ErrorAngleRoll - PrevErrorAngleRoll) / t);
  PIDOutputRoll = PtermRoll + ItermRoll + DtermRoll;
  PIDOutputRoll = constrain(PIDOutputRoll, -400, 400);
  DesiredRateRoll = PIDOutputRoll;
  PrevErrorAngleRoll = ErrorAngleRoll;
  PrevItermAngleRoll = ItermRoll;

  ErrorAnglePitch = DesiredAnglePitch - KalmanAnglePitch;
  PtermPitch = PAnglePitch * ErrorAnglePitch;
  ItermPitch = PrevItermAnglePitch + (IAnglePitch * (ErrorAnglePitch + PrevErrorAnglePitch) * (t / 2));
  ItermPitch = constrain(ItermPitch, -400, 400);
  DtermPitch = DAnglePitch * ((ErrorAnglePitch - PrevErrorAnglePitch) / t);
  PIDOutputPitch = PtermPitch + ItermPitch + DtermPitch;
  PIDOutputPitch = constrain(PIDOutputPitch, -400, 400);
  DesiredRatePitch = PIDOutputPitch;
  PrevErrorAnglePitch = ErrorAnglePitch;
  PrevItermAnglePitch = ItermPitch;

  ErrorRateRoll = DesiredRateRoll - RateRoll;
  ErrorRatePitch = DesiredRatePitch - RatePitch;
  ErrorRateYaw = DesiredRateYaw - RateYaw;

  PtermRoll = PRateRoll * ErrorRateRoll;
  ItermRoll = PrevItermRateRoll + (IRateRoll * (ErrorRateRoll + PrevErrorRateRoll) * (t / 2));
  ItermRoll = constrain(ItermRoll, -400, 400);
  DtermRoll = DRateRoll * ((ErrorRateRoll - PrevErrorRateRoll) / t);
  PIDOutputRoll = PtermRoll + ItermRoll + DtermRoll;
  PIDOutputRoll = constrain(PIDOutputRoll, -400, 400);
  InputRoll = PIDOutputRoll;
  PrevErrorRateRoll = ErrorRateRoll;
  PrevItermRateRoll = ItermRoll;

  PtermPitch = PRatePitch * ErrorRatePitch;
  ItermPitch = PrevItermRatePitch + (IRatePitch * (ErrorRatePitch + PrevErrorRatePitch) * (t / 2));
  ItermPitch = constrain(ItermPitch, -400, 400);
  DtermPitch = DRatePitch * ((ErrorRatePitch - PrevErrorRatePitch) / t);
  PIDOutputPitch = PtermPitch + ItermPitch + DtermPitch;
  PIDOutputPitch = constrain(PIDOutputPitch, -400, 400);
  InputPitch = PIDOutputPitch;
  PrevErrorRatePitch = ErrorRatePitch;
  PrevItermRatePitch = ItermPitch;

  PtermYaw = PRateYaw * ErrorRateYaw;
  ItermYaw = PrevItermRateYaw + (IRateYaw * (ErrorRateYaw + PrevErrorRateYaw) * (t / 2));
  ItermYaw = constrain(ItermYaw, -400, 400);
  DtermYaw = DRateYaw * ((ErrorRateYaw - PrevErrorRateYaw) / t);
  PIDOutputYaw = PtermYaw + ItermYaw + DtermYaw;
  PIDOutputYaw = constrain(PIDOutputYaw, -400, 400);
  InputYaw = PIDOutputYaw;
  PrevErrorRateYaw = ErrorRateYaw;
  PrevItermRateYaw = ItermYaw;

  InputThrottle = constrain(InputThrottle, 1000, 1800);

  MotorInput1 = (InputThrottle - InputRoll - InputPitch - InputYaw);
  MotorInput2 = (InputThrottle - InputRoll + InputPitch + InputYaw);
  MotorInput3 = (InputThrottle + InputRoll + InputPitch - InputYaw);
  MotorInput4 = (InputThrottle + InputRoll - InputPitch + InputYaw);

  MotorInput1 = constrain(MotorInput1, ThrottleIdle, 1999);
  MotorInput2 = constrain(MotorInput2, ThrottleIdle, 1999);
  MotorInput3 = constrain(MotorInput3, ThrottleIdle, 1999);
  MotorInput4 = constrain(MotorInput4, ThrottleIdle, 1999);

  if (ReceiverValue[2] < 1030) {
    MotorInput1 = MotorInput2 = MotorInput3 = MotorInput4 = ThrottleCutOff;
    PrevErrorRateRoll = PrevErrorRatePitch = PrevErrorRateYaw = 0;
    PrevItermRateRoll = PrevItermRatePitch = PrevItermRateYaw = 0;
    PrevErrorAngleRoll = PrevErrorAnglePitch = 0;    
    PrevItermAngleRoll = PrevItermAnglePitch = 0;
    // Reset position hold integrators too
    integralX = integralY = integralAlt = 0;
  }

  mot1.writeMicroseconds(MotorInput1);
  mot2.writeMicroseconds(MotorInput2);
  mot3.writeMicroseconds(MotorInput3);
  mot4.writeMicroseconds(MotorInput4);

  while (micros() - LoopTimer < (t*1000000));
  LoopTimer = micros();
}