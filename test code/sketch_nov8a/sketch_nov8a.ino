#include <TMCStepper.h>
#include <AccelStepper.h>
#include <Encoder.h>
#include <TeensyTimerTool.h>
using namespace TeensyTimerTool;

// ========== CONFIGURATION ==========
#define MAX_AXIS 6

// Example for 2 axes now (expand later)
#define NUM_AXIS 2  

// Motor current and UART settings
#define R_SENSE 0.11f  // Sense resistor on your TMC board
#define DRIVER_ADDR 0b00 // Default TMC2209 address (set differently if needed)

// Teensy pins for STEP/DIR per axis
// const uint8_t stepPins[MAX_AXIS] = {2, 4, 8, 11, 18, 25};
// const uint8_t dirPins[MAX_AXIS]  = {26, 28, 9, 12, 19, 26};
const uint8_t stepPins[MAX_AXIS] = {2, 4};
const uint8_t dirPins[MAX_AXIS]  = {26, 28};

// UART Serial ports (only if using UART mode)
HardwareSerial *driverSerials[MAX_AXIS] = {&Serial1, &Serial2, &Serial3, &Serial4, &Serial5, &Serial6};

// Encoder pins (only for closed-loop axes)
const uint8_t encA[MAX_AXIS] = {22, 23, 0, 0, 0, 0};
const uint8_t encB[MAX_AXIS] = {24, 25, 0, 0, 0, 0};
bool isClosedLoop[MAX_AXIS]  = {true, true, false, false, false, false};

// ========== GLOBALS ==========
TMC2209Stepper *drivers[MAX_AXIS];
AccelStepper *steppers[MAX_AXIS];
Encoder *encoders[MAX_AXIS];
PeriodicTimer updateTimer;

// ========== PID SETTINGS (for closed-loop axes) ==========
float Kp = 0.5, Ki = 0.0, Kd = 0.0;
long encoderTarget[MAX_AXIS] = {0};
long encoderError[MAX_AXIS]  = {0};

// ========== FUNCTIONS ==========

// Closed-loop update function (runs every 2ms)
void updateLoop() {
  for (int i = 0; i < NUM_AXIS; i++) {
    if (isClosedLoop[i]) {
      long pos = encoders[i]->read();
      encoderError[i] = encoderTarget[i] - pos;
      float correction = Kp * encoderError[i];  // Minimal PID (P only)
      steppers[i]->moveTo(encoderTarget[i] + correction);
    }
    steppers[i]->run();
  }
}

// ========== SETUP ==========
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Teensy Multi-Axis Robot Init");

  for (int i = 0; i < NUM_AXIS; i++) {
    // --- DRIVER SETUP ---
    driverSerials[i]->begin(115200);
    drivers[i] = new TMC2209Stepper(driverSerials[i], R_SENSE, DRIVER_ADDR);
    drivers[i]->begin();
    drivers[i]->rms_current(1200);      // Set motor current (mA)
    drivers[i]->microsteps(16);         // Smooth stepping
    drivers[i]->en_spreadCycle(false);  // Enable stealthChop for silence
    drivers[i]->pwm_autoscale(true);    // Auto PWM scaling

    // --- STEPPER SETUP ---
    steppers[i] = new AccelStepper(AccelStepper::DRIVER, stepPins[i], dirPins[i]);
    // steppers[i]->setMaxSpeed(4000);    // Adjust based on mechanics
    // steppers[i]->setAcceleration(8000);
    if (i == 1) {
      steppers[i]->setMaxSpeed(4000*30);    // Adjust based on mechanics
      steppers[i]->setAcceleration(8000*30);
    } else {
      steppers[i]->setMaxSpeed(4000);    // Adjust based on mechanics
      steppers[i]->setAcceleration(8000);
    }

    // --- ENCODER SETUP ---
    if (isClosedLoop[i]) {
      encoders[i] = new Encoder(encA[i], encB[i]);
      encoderTarget[i] = 0;
    }
  }

  // 500 Hz update loop (2ms)
  updateTimer.begin(updateLoop, 2000);
}

// ========== LOOP ==========
void loop() {
  // Example: simple motion pattern
  static uint32_t t = millis();
  if (millis() - t > 3000) {
    t = millis();
    for (int i = 0; i < NUM_AXIS; i++) {
      long newTarget = random(-5000, 5000);
      steppers[i]->moveTo(newTarget);
      if (isClosedLoop[i]) encoderTarget[i] = newTarget;
      Serial.printf("Axis %d -> Target %ld\n", i, newTarget);
    }
  }
}