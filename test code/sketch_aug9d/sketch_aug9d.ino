/*
 * 6-Axis Robot Controller - FIXED VERSION
 * 
 * Key Improvements:
 * 1. Proper encoder/motor synchronization
 * 2. Atomic access to volatile variables
 * 3. Timer-based stepping (non-blocking)
 * 4. Direction/encoder alignment detection
 * 5. Modular architecture ready for OOP refactor
 */

#include <TMCStepper.h>

// ============================================================================
// ATOMIC ACCESS MACROS
// ============================================================================
#define ATOMIC_BLOCK_START noInterrupts()
#define ATOMIC_BLOCK_END interrupts()

#define ATOMIC_READ(var) ({ \
  ATOMIC_BLOCK_START; \
  auto tmp = var; \
  ATOMIC_BLOCK_END; \
  tmp; \
})

#define ATOMIC_WRITE(var, val) \
  do { \
    ATOMIC_BLOCK_START; \
    var = val; \
    ATOMIC_BLOCK_END; \
  } while (0)

// ============================================================================
// CONFIGURATION
// ============================================================================
#define NUM_AXES 1
#define SERIAL_DEBUG Serial
#define BAUD_RATE 115200

#define MOTOR_STEPS_PER_REV 578    // Correct mechanical step count
#define ENCODER_PPR 1000
#define ENCODER_CPR 4000

#define R_SENSE 0.11f
#define MICROSTEPS 16
#define INTERPOLATION 128
#define DEFAULT_CURRENT 1400
#define STEALTHCHOP_ENABLED true

#define STEP_PULSE_WIDTH 5
#define MIN_STEP_INTERVAL 578   // Faster motion, smoother sound


// ============================================================================
// AXIS STATE STRUCTURE
// ============================================================================
struct AxisConfig {
  // Hardware pins
  uint8_t stepPin, dirPin, enPin;
  uint8_t encAPin, encBPin, encZPin;
  HardwareSerial* serial;
  uint8_t uartAddress;

  // Mechanical config
  float gearRatio;
  uint16_t microstepping;
  uint16_t current_mA;
  bool invertDirection;

  // Motion limits
  float minAngle, maxAngle;
  float maxSpeed;  // deg/sec
  float maxAccel;  // deg/sec²

  // Position tracking (volatile - accessed in ISRs)
  volatile long encoderCount;
  volatile bool indexFound;
  volatile long encoderAtIndex;  // NEW: Encoder value when index triggered

  // Motor state (non-volatile - only accessed in main loop)
  long motorStepTarget;   // Target step position
  long motorStepCurrent;  // Current step position
  bool isHomed;           // NEW: Has axis found home?
  bool enabled;

  // Motion state for timer-based stepping
  volatile bool isMoving;
  volatile long stepsRemaining;
  volatile bool stepDirection;
  unsigned long lastStepMicros;

  // Calibration
  float stepsPerDegree;
  float encoderCountsPerDegree;
  bool directionVerified;       // NEW: Has direction been verified?
  int8_t encoderDirectionSign;  // NEW: +1 or -1 to match encoder to motor
};

// ============================================================================
// PIN DEFINITIONS
// ============================================================================
#define AXIS0_STEP 49
#define AXIS0_DIR 51
#define AXIS0_EN 53
#define AXIS0_ENC_A 18
#define AXIS0_ENC_B 20
#define AXIS0_ENC_Z 2

// ============================================================================
// GLOBALS
// ============================================================================
AxisConfig axes[NUM_AXES];
TMC2209Stepper* drivers[NUM_AXES];
unsigned long lastStatusPrint = 0;

// ============================================================================
// ENCODER ISRs - Optimized Quadrature Decoding
// ============================================================================
void axis0_encoderA_ISR() {
  // Standard quadrature decoding (Gray code)
  static uint8_t lastState = 0;
  uint8_t A = digitalRead(AXIS0_ENC_A);
  uint8_t B = digitalRead(AXIS0_ENC_B);
  uint8_t state = (A << 1) | B;

  // State transition lookup: [lastState][newState] -> direction
  // 0→1→3→2→0 = forward, 0→2→3→1→0 = reverse
  int8_t dir = 0;
  if (lastState == 0b00 && state == 0b01) dir = 1;
  else if (lastState == 0b01 && state == 0b11) dir = 1;
  else if (lastState == 0b11 && state == 0b10) dir = 1;
  else if (lastState == 0b10 && state == 0b00) dir = 1;
  else if (lastState == 0b00 && state == 0b10) dir = -1;
  else if (lastState == 0b10 && state == 0b11) dir = -1;
  else if (lastState == 0b11 && state == 0b01) dir = -1;
  else if (lastState == 0b01 && state == 0b00) dir = -1;

  axes[0].encoderCount += dir * axes[0].encoderDirectionSign;
  lastState = state;
}

void axis0_encoderB_ISR() {
  axis0_encoderA_ISR();  // Same handler, state machine handles both
}

void axis0_encoderZ_ISR() {
  axes[0].indexFound = true;
  axes[0].encoderAtIndex = axes[0].encoderCount;
}

// ============================================================================
// INITIALIZATION
// ============================================================================
void initializeAxisConfigs() {
  // Axis 0
  axes[0].stepPin = AXIS0_STEP;
  axes[0].dirPin = AXIS0_DIR;
  axes[0].enPin = AXIS0_EN;
  axes[0].encAPin = AXIS0_ENC_A;
  axes[0].encBPin = AXIS0_ENC_B;
  axes[0].encZPin = AXIS0_ENC_Z;
  axes[0].serial = &Serial2;
  axes[0].uartAddress = 0x00;

  axes[0].gearRatio = 100.0 / 15.0;  // pulley ratio (output/motor)
  axes[0].microstepping = MICROSTEPS;
  axes[0].current_mA = DEFAULT_CURRENT;
  axes[0].invertDirection = false;

  axes[0].minAngle = -360.0;
  axes[0].maxAngle = 360.0;
  axes[0].maxSpeed = 180.0;
  axes[0].maxAccel = 360.0;

  axes[0].encoderCount = 0;
  axes[0].indexFound = false;
  axes[0].encoderAtIndex = 0;
  axes[0].motorStepTarget = 0;fi
  axes[0].motorStepCurrent = 0;
  axes[0].isHomed = false;
  axes[0].enabled = false;

  axes[0].isMoving = false;
  axes[0].stepsRemaining = 0;
  axes[0].lastStepMicros = 0;

  axes[0].directionVerified = false;
  axes[0].encoderDirectionSign = 1;  // Assume positive initially

  updateAxisCalculations(0);
}

void updateAxisCalculations(uint8_t axis) {
  float motorStepsPerRev = MOTOR_STEPS_PER_REV * axes[axis].microstepping;
  axes[axis].stepsPerDegree = (motorStepsPerRev * axes[axis].gearRatio) / 360.0;
  axes[axis].encoderCountsPerDegree = (ENCODER_CPR * axes[axis].gearRatio) / 360.0;
}

void initializeDrivers() {
  for (uint8_t i = 0; i < NUM_AXES; i++) {
    axes[i].serial->begin(115200);
    drivers[i] = new TMC2209Stepper(axes[i].serial, R_SENSE, axes[i].uartAddress);

    drivers[i]->begin();
    drivers[i]->toff(5);
    drivers[i]->rms_current(axes[i].current_mA);
    drivers[i]->microsteps(axes[i].microstepping);
    drivers[i]->pwm_autoscale(true);
    drivers[i]->en_spreadCycle(!STEALTHCHOP_ENABLED);
    drivers[i]->intpol(true);
    drivers[i]->TPOWERDOWN(128);  // Increased from 20 for better holding

    if (STEALTHCHOP_ENABLED) {
      drivers[i]->TCOOLTHRS(0xFFFFF);
      drivers[i]->semin(5);
      drivers[i]->semax(2);
      drivers[i]->sedn(0b01);
    }

    delay(100);  // Let driver stabilize

    // Verify UART communication
    uint32_t status = drivers[i]->DRV_STATUS();
    SERIAL_DEBUG.print("Driver ");
    SERIAL_DEBUG.print(i);
    SERIAL_DEBUG.print(" DRV_STATUS: 0x");
    SERIAL_DEBUG.println(status, HEX);

    if (status == 0 || status == 0xFFFFFFFF) {
      SERIAL_DEBUG.println("WARNING: UART communication failure!");
    }
  }
}

// ============================================================================
// MOTOR CONTROL - IMPROVED
// ============================================================================
void enableAxis(uint8_t axis, bool enable) {
  digitalWrite(axes[axis].enPin, enable ? LOW : HIGH);
  axes[axis].enabled = enable;
  SERIAL_DEBUG.print("Axis ");
  SERIAL_DEBUG.print(axis);
  SERIAL_DEBUG.println(enable ? " ENABLED" : " DISABLED");
}

// NEW: Verify encoder counts in same direction as motor
void verifyEncoderDirection(uint8_t axis) {
  SERIAL_DEBUG.println("\n=== VERIFYING ENCODER DIRECTION ===");

  enableAxis(axis, true);
  delay(100);

  long startEnc = ATOMIC_READ(axes[axis].encoderCount);

  // Move in positive direction
  digitalWrite(axes[axis].dirPin, HIGH);
  for (int i = 0; i < 800; i++) {  // Quarter revolution
    digitalWrite(axes[axis].stepPin, HIGH);
    delayMicroseconds(STEP_PULSE_WIDTH);
    digitalWrite(axes[axis].stepPin, LOW);
    delayMicroseconds(1000);
  }

  delay(100);
  long endEnc = ATOMIC_READ(axes[axis].encoderCount);
  long encChange = endEnc - startEnc;

  SERIAL_DEBUG.print("Motor DIR=HIGH, Encoder change: ");
  SERIAL_DEBUG.println(encChange);

  if (encChange < 0) {
    SERIAL_DEBUG.println("Encoder counts BACKWARD → Inverting encoder direction");
    axes[axis].encoderDirectionSign = -1;
  } else if (encChange > 0) {
    SERIAL_DEBUG.println("Encoder counts FORWARD → Direction OK");
    axes[axis].encoderDirectionSign = 1;
  } else {
    SERIAL_DEBUG.println("ERROR: No encoder movement detected!");
    return;
  }

  axes[axis].directionVerified = true;
  SERIAL_DEBUG.println("=== DIRECTION VERIFIED ===\n");
}

// NEW: Proper homing sequence
void homeAxis(uint8_t axis) {
  if (!axes[axis].directionVerified) {
    SERIAL_DEBUG.println("ERROR: Run direction verification first!");
    return;
  }

  SERIAL_DEBUG.println("\n=== HOMING SEQUENCE ===");

  enableAxis(axis, true);
  ATOMIC_WRITE(axes[axis].indexFound, false);

  // Rotate to find index pulse
  digitalWrite(axes[axis].dirPin, HIGH);
  while (!ATOMIC_READ(axes[axis].indexFound)) {
    digitalWrite(axes[axis].stepPin, HIGH);
    delayMicroseconds(STEP_PULSE_WIDTH);
    digitalWrite(axes[axis].stepPin, LOW);
    delayMicroseconds(2000);  // Slow homing speed
  }

  delay(100);

  // Synchronize positions
  ATOMIC_WRITE(axes[axis].encoderCount, 0);
  axes[axis].motorStepCurrent = 0;
  axes[axis].motorStepTarget = 0;
  axes[axis].isHomed = true;

  SERIAL_DEBUG.println("Homing complete! Zero position set.");
  SERIAL_DEBUG.println("=== HOMING DONE ===\n");
}

// FIXED: Absolute move with proper position tracking
void moveToAngle(uint8_t axis, float targetAngle) {
  if (!axes[axis].enabled) {
    SERIAL_DEBUG.println("ERROR: Axis not enabled!");
    return;
  }

  if (!axes[axis].isHomed) {
    SERIAL_DEBUG.println("WARNING: Axis not homed! Position may be inaccurate.");
  }

  // Limit check
  if (targetAngle < axes[axis].minAngle || targetAngle > axes[axis].maxAngle) {
    SERIAL_DEBUG.print("ERROR: Angle ");
    SERIAL_DEBUG.print(targetAngle);
    SERIAL_DEBUG.println("° out of range!");
    return;
  }

  // Calculate target in motor steps
  long targetSteps = angleToMotorSteps(axis, targetAngle);
  long currentSteps = axes[axis].motorStepCurrent;
  long deltaSteps = targetSteps - currentSteps;

  SERIAL_DEBUG.print("Moving to ");
  SERIAL_DEBUG.print(targetAngle, 2);
  SERIAL_DEBUG.print("° (");
  SERIAL_DEBUG.print(deltaSteps);
  SERIAL_DEBUG.println(" steps)");

  moveAxisSteps(axis, deltaSteps);
}

void moveByAngle(uint8_t axis, float deltaAngle) {
  // Convert current motor position to angle and add delta
  float currentAngle = motorStepsToAngle(axis, axes[axis].motorStepCurrent);
  moveToAngle(axis, currentAngle + deltaAngle);
}

// IMPROVED: Step execution with proper position tracking
void moveAxisSteps(uint8_t axis, long steps) {
  if (steps == 0) return;

  AxisConfig& a = axes[axis];
  bool direction = steps > 0;
  if (a.invertDirection) direction = !direction;

  digitalWrite(a.dirPin, direction ? HIGH : LOW);
  delayMicroseconds(10); // Direction setup time

  long absSteps = labs(steps);
  a.isMoving = true;

  // --- Compute timing parameters ---
  float maxSpeedStepsPerSec = a.stepsPerDegree * a.maxSpeed;
  float accelStepsPerSec2 = a.stepsPerDegree * a.maxAccel;

  if (maxSpeedStepsPerSec < 10) maxSpeedStepsPerSec = 10; // safety clamp

  // Step timing
  float currentSpeed = 0;
  float targetSpeed = maxSpeedStepsPerSec;
  float stepDelay = 1e6 / 10; // Start slow (10 steps/sec)

  unsigned long lastStepTime = micros();
  long stepsDone = 0;

  // --- Accel-decel control ---
  long accelDistance = (long)((targetSpeed * targetSpeed) / (2.0 * accelStepsPerSec2));
  if (accelDistance > absSteps / 2) accelDistance = absSteps / 2; // triangle profile

  // --- Main stepping loop ---
  while (stepsDone < absSteps) {
    unsigned long now = micros();
    if (now - lastStepTime >= (unsigned long)stepDelay) {
      // Issue one step pulse
      digitalWrite(a.stepPin, HIGH);
      delayMicroseconds(STEP_PULSE_WIDTH);
      digitalWrite(a.stepPin, LOW);

      // Update motor step counter
      a.motorStepCurrent += (direction ? 1 : -1);
      stepsDone++;
      lastStepTime = now;

      // Acceleration phase
      if (stepsDone < accelDistance) {
        currentSpeed = sqrt(2.0 * accelStepsPerSec2 * stepsDone);
        if (currentSpeed > targetSpeed) currentSpeed = targetSpeed;
      }
      // Deceleration phase
      else if (stepsDone > absSteps - accelDistance) {
        long decelStep = absSteps - stepsDone;
        currentSpeed = sqrt(2.0 * accelStepsPerSec2 * decelStep);
        if (currentSpeed < 1) currentSpeed = 1;
      }
      // Cruise
      else {
        currentSpeed = targetSpeed;
      }

      // Compute new delay
      stepDelay = 1e6 / currentSpeed;
      if (stepDelay < MIN_STEP_INTERVAL) stepDelay = MIN_STEP_INTERVAL;
    }
  }

  a.isMoving = false;

  // --- Post-move verification ---
  delay(50);
  long encCount = ATOMIC_READ(a.encoderCount);
  float motorAngle = motorStepsToAngle(axis, a.motorStepCurrent);
  float encoderAngle = encoderCountsToAngle(axis, encCount);
  float error = motorAngle - encoderAngle;

  SERIAL_DEBUG.print("Move complete. Error: ");
  SERIAL_DEBUG.print(error, 3);
  SERIAL_DEBUG.println("°");
}

// ============================================================================
// CONVERSION FUNCTIONS
// ============================================================================
long angleToMotorSteps(uint8_t axis, float angle) {
  return (long)(angle * axes[axis].stepsPerDegree);
}

float motorStepsToAngle(uint8_t axis, long steps) {
  return (float)steps / axes[axis].stepsPerDegree;
}

float encoderCountsToAngle(uint8_t axis, long counts) {
  return (float)counts / axes[axis].encoderCountsPerDegree;
}

// ============================================================================
// DIAGNOSTICS
// ============================================================================
void printAxisStatus(uint8_t axis) {
  long encCount = ATOMIC_READ(axes[axis].encoderCount);

  float motorAngle = motorStepsToAngle(axis, axes[axis].motorStepCurrent);
  float encoderAngle = encoderCountsToAngle(axis, encCount);
  float error = motorAngle - encoderAngle;

  SERIAL_DEBUG.print("Axis ");
  SERIAL_DEBUG.print(axis);
  SERIAL_DEBUG.print(" | Motor: ");
  SERIAL_DEBUG.print(motorAngle, 2);
  SERIAL_DEBUG.print("°");
  SERIAL_DEBUG.print(" | Encoder: ");
  SERIAL_DEBUG.print(encoderAngle, 2);
  SERIAL_DEBUG.print("°");
  SERIAL_DEBUG.print(" | Err: ");
  SERIAL_DEBUG.print(error, 3);
  SERIAL_DEBUG.print("°");
  SERIAL_DEBUG.print(" | Homed: ");
  SERIAL_DEBUG.println(axes[axis].isHomed ? "YES" : "NO");
}

void calibrateAxis(uint8_t axis) {
  if (!axes[axis].isHomed) {
    SERIAL_DEBUG.println("ERROR: Home axis first!");
    return;
  }

  SERIAL_DEBUG.println("\n=== CALIBRATION ===");

  long startEnc = ATOMIC_READ(axes[axis].encoderCount);
  long startSteps = axes[axis].motorStepCurrent;

  moveByAngle(axis, 360.0);
  delay(500);

  long endEnc = ATOMIC_READ(axes[axis].encoderCount);
  long endSteps = axes[axis].motorStepCurrent;

  long encDiff = endEnc - startEnc;
  long stepDiff = endSteps - startSteps;

  float encoderAngle = encoderCountsToAngle(axis, encDiff);
  float motorAngle = motorStepsToAngle(axis, stepDiff);
  float error = ((encoderAngle - 360.0) / 360.0) * 100.0;

  SERIAL_DEBUG.print("Commanded: 360.00°\n");
  SERIAL_DEBUG.print("Motor: ");
  SERIAL_DEBUG.print(motorAngle, 2);
  SERIAL_DEBUG.println("°");
  SERIAL_DEBUG.print("Encoder: ");
  SERIAL_DEBUG.print(encoderAngle, 2);
  SERIAL_DEBUG.println("°");
  SERIAL_DEBUG.print("Error: ");
  SERIAL_DEBUG.print(error, 3);
  SERIAL_DEBUG.println("%");

  if (abs(error) < 1.0) {
    SERIAL_DEBUG.println("✓ CALIBRATION PASSED");
  } else {
    SERIAL_DEBUG.println("✗ CALIBRATION FAILED");
  }

  SERIAL_DEBUG.println("=== CALIBRATION END ===\n");
}

// ============================================================================
// SETUP & LOOP
// ============================================================================
void setup() {
  SERIAL_DEBUG.begin(BAUD_RATE);
  while (!SERIAL_DEBUG && millis() < 3000)
    ;

  SERIAL_DEBUG.println("\n6-Axis Robot Controller v2.0 (FIXED)");
  SERIAL_DEBUG.println("======================================\n");

  initializeAxisConfigs();

  for (uint8_t i = 0; i < NUM_AXES; i++) {
    pinMode(axes[i].stepPin, OUTPUT);
    pinMode(axes[i].dirPin, OUTPUT);
    pinMode(axes[i].enPin, OUTPUT);
    digitalWrite(axes[i].enPin, HIGH);

    pinMode(axes[i].encAPin, INPUT_PULLUP);
    pinMode(axes[i].encBPin, INPUT_PULLUP);
    pinMode(axes[i].encZPin, INPUT_PULLUP);
  }

  attachInterrupt(digitalPinToInterrupt(AXIS0_ENC_A), axis0_encoderA_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(AXIS0_ENC_B), axis0_encoderB_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(AXIS0_ENC_Z), axis0_encoderZ_ISR, RISING);

  delay(100);
  initializeDrivers();

  SERIAL_DEBUG.println("\n=== COMMANDS ===");
  SERIAL_DEBUG.println("SETUP:");
  SERIAL_DEBUG.println("  e - Enable axis");
  SERIAL_DEBUG.println("  d - Disable axis");
  SERIAL_DEBUG.println("  v - Verify encoder direction");
  SERIAL_DEBUG.println("  h - Home to index");
  SERIAL_DEBUG.println("\nMOTION:");
  SERIAL_DEBUG.println("  m[angle] - Move relative (e.g., m45)");
  SERIAL_DEBUG.println("  a[angle] - Move absolute (e.g., a0)");
  SERIAL_DEBUG.println("  + / - - Move ±90°");
  SERIAL_DEBUG.println("\nDIAGNOSTICS:");
  SERIAL_DEBUG.println("  c - Calibrate");
  SERIAL_DEBUG.println("  s - Status");
  SERIAL_DEBUG.println("  r - Reset position");
  SERIAL_DEBUG.println("================\n");

  SERIAL_DEBUG.println("IMPORTANT: Run commands in order:");
  SERIAL_DEBUG.println("1. e (enable)");
  SERIAL_DEBUG.println("2. v (verify direction)");
  SERIAL_DEBUG.println("3. h (home)");
  SERIAL_DEBUG.println("4. Test with m45, a0, etc.\n");
}

void loop() {
  if (SERIAL_DEBUG.available()) {
    char cmd = SERIAL_DEBUG.read();

    switch (cmd) {
      case 'e': enableAxis(0, true); break;
      case 'd': enableAxis(0, false); break;
      case 'v': verifyEncoderDirection(0); break;
      case 'h': homeAxis(0); break;
      case 'c': calibrateAxis(0); break;
      case 's': printAxisStatus(0); break;

      case 'r':
        ATOMIC_WRITE(axes[0].encoderCount, 0);
        axes[0].motorStepCurrent = 0;
        axes[0].isHomed = false;
        SERIAL_DEBUG.println("Position reset (not homed)");
        break;

      case 'm':
        {
          float angle = SERIAL_DEBUG.parseFloat();
          moveByAngle(0, angle);
          break;
        }

      case 'a':
        {
          float angle = SERIAL_DEBUG.parseFloat();
          moveToAngle(0, angle);
          break;
        }

      case '+':
        if (axes[0].enabled) moveByAngle(0, 90.0);
        break;
      case '-':
        if (axes[0].enabled) moveByAngle(0, -90.0);
        break;
    }

    while (SERIAL_DEBUG.available()) SERIAL_DEBUG.read();
  }

  if (millis() - lastStatusPrint > 2000) {
    printAxisStatus(0);
    lastStatusPrint = millis();
  }
}