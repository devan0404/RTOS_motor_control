#include <Arduino.h>

// ============================================================================
// HARDWARE CONFIGURATION
// ============================================================================

#define STEP_PIN 18
#define DIR_PIN  19

// ============================================================================
// MOTION PARAMETERS - TUNING CONSTANTS
// ============================================================================

// Mechanical configuration
const float STEPS_PER_MM = 5.0;           // Full steps per mm(200 steps/rev, 40mm/rev)
const float MAX_TRAVEL_MM = 290.0;        // Maximum axis travel

// Motion limits(all in mm and mm/s units)
const float MAX_VELOCITY = 100.0;         // mm/s - Maximum velocity
const float MAX_ACCELERATION = 500.0;     // mm/s² - Maximum acceleration
const float MAX_JERK = 5000.0;            // mm/s³ - Maximum jerk(rate of acceleration change)

// Default motion parameters
const float DEFAULT_FEEDRATE = 50.0;      // mm/s - Default feedrate for G0/G1

// Timer configuration
const uint32_t TIMER_FREQ_HZ = 1000000;   // 1 MHz timer base frequency
const uint32_t MIN_STEP_INTERVAL_US = 50; // Minimum time between steps(20kHz max step rate)

// Motion planning update rate
const uint32_t MOTION_UPDATE_MS = 1;      // Motion planner updates every 1ms

// ============================================================================
// GLOBAL STATE VARIABLES
// ============================================================================

// Hardware timer
hw_timer_t *stepTimer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// Motion state
volatile long currentPosition = 0;        // Current position in steps
volatile long targetPosition = 0;         // Target position in steps
volatile bool directionPositive = true;   // Current direction

// Velocity profile(steps/second and steps/second²)
volatile float currentVelocity = 0.0;     // Current velocity in steps/s
volatile float targetVelocity = 0.0;      // Target velocity in steps/s
volatile uint32_t stepIntervalUs = 0;     // Current step interval in microseconds
volatile bool stepsActive = false;        // Is motion active?

// Timer management for ESP32 Core 3.x
volatile uint32_t timerCounter = 0;       // Internal counter for step timing
volatile uint32_t nextStepCount = 0;      // When to generate next step

// Motion control flags
enum MotionState {
  MOTION_IDLE,
  MOTION_RUNNING,
  MOTION_PAUSED,
  MOTION_STOPPED
};
volatile MotionState motionState = MOTION_IDLE;

// Feed override(percentage, 100 = 100%)
volatile float feedOverride = 100.0;

// Position mode
bool absoluteMode = true;  // G90/G91

// Current feedrate setting
float currentFeedrate = DEFAULT_FEEDRATE;

// ============================================================================
// MOTION COMMAND STRUCTURE
// ============================================================================

struct MotionCommand {
  float targetPositionMm;  // Target position in mm
  float feedrate;          // Feedrate in mm/s
  bool isRapid;            // True for G0, false for G1
};

// FreeRTOS queue for motion commands
QueueHandle_t motionQueue;

// Task handles
TaskHandle_t serialTaskHandle;
TaskHandle_t motionTaskHandle;

// ============================================================================
// HARDWARE TIMER ISR - STEP PULSE GENERATION
// ESP32 Arduino Core 3.x compatible
// ============================================================================

void IRAM_ATTR onStepTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  
  // Increment timer counter(called every 10µs)
  timerCounter++;
  
  if (stepsActive && motionState == MOTION_RUNNING) {
    // Check if we've reached target
    if (currentPosition == targetPosition) {
      stepsActive = false;
      portEXIT_CRITICAL_ISR(&timerMux);
      return;
    }
    
    // Check if it's time for the next step
    if (timerCounter >= nextStepCount) {
      // Generate step pulse
      digitalWrite(STEP_PIN, HIGH);
      
      // Update position
      if (directionPositive) {
        currentPosition++;
      } else {
        currentPosition--;
      }
      
      // Pulse must be at least 1µs for A4988
      delayMicroseconds(2);
      digitalWrite(STEP_PIN, LOW);
      
      // Calculate next step time based on current velocity
      if (stepIntervalUs > 0) {
        nextStepCount = timerCounter + (stepIntervalUs / 10);  // Convert to 10µs units
      } else {
        stepsActive = false;
      }
    }
  }
  
  portEXIT_CRITICAL_ISR(&timerMux);
}

// ============================================================================
// MOTION PROFILE CALCULATION - S-CURVE APPROXIMATION
// ============================================================================

/*
 * This uses an online S-curve approximation rather than exact 7-segment planning.
 * The velocity is updated incrementally each motion update cycle, with the
 * acceleration ramped up and down smoothly using jerk limiting.
 * 
 * Profile phases:
 * 1. Jerk-limited acceleration increase(S-curve entry)
 * 2. Constant acceleration
 * 3. Jerk-limited acceleration decrease(S-curve transition)
 * 4. Constant velocity
 * 5. Jerk-limited deceleration increase(S-curve entry)
 * 6. Constant deceleration
 * 7. Jerk-limited deceleration decrease(S-curve exit)
 */

class MotionPlanner {
private:
  float currentAccel = 0.0;      // Current acceleration in mm/s²
  float targetAccel = 0.0;       // Target acceleration
  float plannedVelocity = 0.0;   // Planned velocity in mm/s
  float maxVel = 0.0;            // Maximum velocity for this move
  float decelDistance = 0.0;     // Distance needed to decelerate
  
public:
  
  void startMove(float targetPosMm, float feedrateMmS) {
    // Apply feed override
    float effectiveFeedrate = feedrateMmS * (feedOverride / 100.0);
    maxVel = min(effectiveFeedrate, MAX_VELOCITY);
    
    // Convert to steps
    long targetSteps = (long)(targetPosMm * STEPS_PER_MM);
    
    portENTER_CRITICAL(&timerMux);
    targetPosition = targetSteps;
    
    // Set direction
    long delta = targetPosition - currentPosition;
    if (delta > 0) {
      directionPositive = true;
      digitalWrite(DIR_PIN, HIGH);
    } else {
      directionPositive = false;
      digitalWrite(DIR_PIN, LOW);
    }
    
    stepsActive = true;
    motionState = MOTION_RUNNING;
    nextStepCount = timerCounter + 100;  // Start stepping soon
    portEXIT_CRITICAL(&timerMux);
    
    // Reset motion profile
    currentAccel = 0.0;
    targetAccel = 0.0;
    plannedVelocity = currentVelocity / STEPS_PER_MM; // Convert from steps/s to mm/s
  }
  
  void updateProfile(float deltaTimeS) {
    if (motionState != MOTION_RUNNING) {
      return;
    }
    
    // Get current position safely
    long curPos, targPos;
    portENTER_CRITICAL(&timerMux);
    curPos = currentPosition;
    targPos = targetPosition;
    portEXIT_CRITICAL(&timerMux);
    
    float remainingSteps = abs(targPos - curPos);
    float remainingMm = remainingSteps / STEPS_PER_MM;
    
    // If we're very close, come to a stop
    if (remainingMm < 0.01) {
      plannedVelocity = 0.0;
      currentAccel = 0.0;
      updateVelocity();
      return;
    }
    
    // Calculate deceleration distance needed from current velocity
    // Using trapezoidal approximation: d = v²/(2*a) + jerk_margin
    float jerkMargin = (plannedVelocity * plannedVelocity) / (2.0 * MAX_JERK * 0.1);
    decelDistance = (plannedVelocity * plannedVelocity) / (2.0 * MAX_ACCELERATION) + jerkMargin;
    
    // Determine if we should accelerate or decelerate
    if (remainingMm > decelDistance && plannedVelocity < maxVel) {
      // Accelerate toward max velocity with jerk limiting
      targetAccel = MAX_ACCELERATION;
    } else {
      // Decelerate toward zero with jerk limiting
      targetAccel = -MAX_ACCELERATION;
      
      // Adjust target velocity based on remaining distance
      float neededVel = sqrt(2.0 * MAX_ACCELERATION * remainingMm);
      if (neededVel < plannedVelocity) {
        targetAccel = -MAX_ACCELERATION;
      }
    }
    
    // Apply jerk limiting to acceleration
    float accelDelta = targetAccel - currentAccel;
    float maxAccelChange = MAX_JERK * deltaTimeS;
    
    if (abs(accelDelta) > maxAccelChange) {
      if (accelDelta > 0) {
        currentAccel += maxAccelChange;
      } else {
        currentAccel -= maxAccelChange;
      }
    } else {
      currentAccel = targetAccel;
    }
    
    // Update velocity based on acceleration
    plannedVelocity += currentAccel * deltaTimeS;
    
    // Clamp velocity
    if (plannedVelocity > maxVel) {
      plannedVelocity = maxVel;
      currentAccel = 0.0;
    }
    if (plannedVelocity < 0.0) {
      plannedVelocity = 0.0;
      currentAccel = 0.0;
    }
    
    // Update the ISR velocity
    updateVelocity();
  }
  
  void updateVelocity() {
    float velocityStepsPerSec = plannedVelocity * STEPS_PER_MM;
    
    portENTER_CRITICAL(&timerMux);
    currentVelocity = velocityStepsPerSec;
    
    if (currentVelocity > 0.0) {
      // Calculate step interval in microseconds
      float intervalUs = 1000000.0 / currentVelocity;
      if (intervalUs < MIN_STEP_INTERVAL_US) {
        intervalUs = MIN_STEP_INTERVAL_US;
      }
      stepIntervalUs = (uint32_t)intervalUs;
    } else {
      stepIntervalUs = 0;
      stepsActive = false;
    }
    portEXIT_CRITICAL(&timerMux);
  }
  
  void pause() {
    portENTER_CRITICAL(&timerMux);
    motionState = MOTION_PAUSED;
    portEXIT_CRITICAL(&timerMux);
  }
  
  void resume() {
    portENTER_CRITICAL(&timerMux);
    if (motionState == MOTION_PAUSED) {
      motionState = MOTION_RUNNING;
      stepsActive = true;
    }
    portEXIT_CRITICAL(&timerMux);
  }
  
  void emergencyStop() {
    portENTER_CRITICAL(&timerMux);
    motionState = MOTION_STOPPED;
    stepsActive = false;
    currentVelocity = 0.0;
    stepIntervalUs = 0;
    portEXIT_CRITICAL(&timerMux);
    
    // Reset planner state
    currentAccel = 0.0;
    plannedVelocity = 0.0;
  }
  
  bool isIdle() {
    portENTER_CRITICAL(&timerMux);
    bool idle = (motionState == MOTION_IDLE) || 
                (motionState == MOTION_RUNNING && !stepsActive);
    portEXIT_CRITICAL(&timerMux);
    return idle;
  }
};

MotionPlanner planner;

// ============================================================================
// CORE 1 TASK - MOTION PLANNING(HIGHEST PRIORITY)
// ============================================================================

void motionPlanningTask(void *parameter) {
  MotionCommand cmd;
  unsigned long lastUpdateTime = millis();
  
  Serial.println("[Core 1] Motion planning task started");
  
  while (true) {
    // Check for new motion commands
    if (xQueueReceive(motionQueue, &cmd, 0) == pdTRUE) {
      Serial.print("Executing move to X=");
      Serial.print(cmd.targetPositionMm, 3);
      Serial.print(" mm at F=");
      Serial.print(cmd.feedrate, 1);
      Serial.println(" mm/s");
      
      planner.startMove(cmd.targetPositionMm, cmd.feedrate);
    }
    
    // Update motion profile at fixed rate
    unsigned long currentTime = millis();
    unsigned long deltaTime = currentTime - lastUpdateTime;
    
    if (deltaTime >= MOTION_UPDATE_MS) {
      float deltaTimeS = deltaTime / 1000.0;
      planner.updateProfile(deltaTimeS);
      lastUpdateTime = currentTime;
    }
    
    // Check if motion completed
    if (motionState == MOTION_RUNNING && !stepsActive) {
      portENTER_CRITICAL(&timerMux);
      motionState = MOTION_IDLE;
      portEXIT_CRITICAL(&timerMux);
      Serial.println("Move complete");
    }
    
    // Small delay to prevent watchdog issues
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

// ============================================================================
// CORE 0 TASK - SERIAL COMMUNICATION
// ============================================================================

void parseGCode(String line) {
  line.trim();
  line.toUpperCase();
  
  if (line.length() == 0) return;
  
  // Handle single-character commands
  if (line.length() == 1) {
    char cmd = line.charAt(0);
    switch (cmd) {
      case 'P':
        Serial.println("PAUSE");
        planner.pause();
        break;
      case 'R':
        Serial.println("RESUME");
        planner.resume();
        break;
      case 'E':
        Serial.println("EMERGENCY STOP");
        planner.emergencyStop();
        // Clear motion queue
        xQueueReset(motionQueue);
        break;
      case '+':
        feedOverride += 10.0;
        if (feedOverride > 200.0) feedOverride = 200.0;
        Serial.print("Feed override: ");
        Serial.print(feedOverride, 0);
        Serial.println("%");
        break;
      case '-':
        feedOverride -= 10.0;
        if (feedOverride < 10.0) feedOverride = 10.0;
        Serial.print("Feed override: ");
        Serial.print(feedOverride, 0);
        Serial.println("%");
        break;
      case 'S':
        // Print status
        Serial.println("\n=== STATUS ===");
        portENTER_CRITICAL(&timerMux);
        float posMm = currentPosition / STEPS_PER_MM;
        float velMmS = currentVelocity / STEPS_PER_MM;
        MotionState state = motionState;
        portEXIT_CRITICAL(&timerMux);
        
        Serial.print("Position: X=");
        Serial.print(posMm, 3);
        Serial.println(" mm");
        Serial.print("Velocity: ");
        Serial.print(velMmS, 2);
        Serial.println(" mm/s");
        Serial.print("State: ");
        switch (state) {
          case MOTION_IDLE: Serial.println("IDLE"); break;
          case MOTION_RUNNING: Serial.println("RUNNING"); break;
          case MOTION_PAUSED: Serial.println("PAUSED"); break;
          case MOTION_STOPPED: Serial.println("STOPPED"); break;
        }
        Serial.print("Mode: ");
        Serial.println(absoluteMode ? "G90(Absolute)" : "G91(Relative)");
        Serial.print("Feed override: ");
        Serial.print(feedOverride, 0);
        Serial.println("%");
        Serial.println("==============\n");
        break;
    }
    return;
  }
  
  // Parse G-code
  if (line.startsWith("G")) {
    int gCode = -1;
    float xPos = 0.0;
    float feedrate = currentFeedrate;
    bool hasX = false;
    bool hasF = false;
    
    // Extract G code number
    int spaceIdx = line.indexOf(' ');
    if (spaceIdx > 0) {
      gCode = line.substring(1, spaceIdx).toInt();
    } else {
      gCode = line.substring(1).toInt();
    }
    
    // Parse parameters
    int xIdx = line.indexOf('X');
    if (xIdx >= 0) {
      hasX = true;
      int nextSpace = line.indexOf(' ', xIdx);
      if (nextSpace < 0) nextSpace = line.length();
      xPos = line.substring(xIdx + 1, nextSpace).toFloat();
    }
    
    int fIdx = line.indexOf('F');
    if (fIdx >= 0) {
      hasF = true;
      int nextSpace = line.indexOf(' ', fIdx);
      if (nextSpace < 0) nextSpace = line.length();
      feedrate = line.substring(fIdx + 1, nextSpace).toFloat();
    }
    
    // Update current feedrate if F was specified
    if (hasF) {
      currentFeedrate = feedrate;
    }
    
    // Handle G-codes
    switch (gCode) {
      case 0:  // Rapid move
      case 1:  // Linear move
        if (hasX) {
          float targetMm = xPos;
          
          // Convert to absolute if in relative mode
          if (!absoluteMode) {
            portENTER_CRITICAL(&timerMux);
            float currentMm = currentPosition / STEPS_PER_MM;
            portEXIT_CRITICAL(&timerMux);
            targetMm = currentMm + xPos;
          }
          
          // Check travel limits
          if (targetMm < 0.0 || targetMm > MAX_TRAVEL_MM) {
            Serial.print("ERROR: Position ");
            Serial.print(targetMm, 2);
            Serial.println(" mm out of range [0, 290]");
            return;
          }
          
          // Create motion command
          MotionCommand cmd;
          cmd.targetPositionMm = targetMm;
          cmd.feedrate = currentFeedrate;
          cmd.isRapid = (gCode == 0);
          
          // Send to motion queue
          if (xQueueSend(motionQueue, &cmd, pdMS_TO_TICKS(100)) != pdTRUE) {
            Serial.println("ERROR: Motion queue full");
          }
        }
        break;
        
      case 90:  // Absolute positioning
        absoluteMode = true;
        Serial.println("G90: Absolute mode");
        break;
        
      case 91:  // Relative positioning
        absoluteMode = false;
        Serial.println("G91: Relative mode");
        break;
        
      default:
        Serial.print("Unsupported G-code: G");
        Serial.println(gCode);
        break;
    }
  }
}

void serialCommTask(void *parameter) {
  String inputBuffer = "";
  
  Serial.println("[Core 0] Serial communication task started");
  Serial.println("\n=== ESP32 Motor Controller Ready ===");
  Serial.println("Commands:");
  Serial.println("  G0/G1 X### F### - Move to position");
  Serial.println("  G90/G91 - Absolute/Relative mode");
  Serial.println("  P - Pause, R - Resume, E - Emergency Stop");
  Serial.println("  + - Speed up, - - Slow down");
  Serial.println("  S - Status");
  Serial.println("=====================================\n");
  
  while (true) {
    while (Serial.available()) {
      char c = Serial.read();
      
      if (c == '\n' || c == '\r') {
        if (inputBuffer.length() > 0) {
          Serial.print("> ");
          Serial.println(inputBuffer);
          parseGCode(inputBuffer);
          inputBuffer = "";
        }
      } else {
        inputBuffer += c;
      }
    }
    
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ============================================================================
// ARDUINO SETUP
// ============================================================================

void setup() {
  // Initialize serial
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n\n=== ESP32 Motor Controller Initializing ===");
  
  // Configure GPIO pins
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN, LOW);
  
  Serial.println("GPIO configured");
  
  // Create motion queue(holds up to 10 motion commands)
  motionQueue = xQueueCreate(10, sizeof(MotionCommand));
  if (motionQueue == NULL) {
    Serial.println("ERROR: Failed to create motion queue");
    while (1);
  }
  Serial.println("Motion queue created");
  
  // Configure hardware timer(ESP32 Arduino Core 3.x API)
  // Timer runs at 100kHz (every 10 microseconds)
  stepTimer = timerBegin(100000);  // 100kHz = 100,000 Hz
  timerAttachInterrupt(stepTimer, &onStepTimer);
  timerAlarm(stepTimer, 1, true, 0);  // Trigger every 1 count(10µs), autoreload, unlimited
  Serial.println("Hardware timer configured(100kHz, 10µs interval)");
  
  // Create FreeRTOS tasks
  // Core 0: Serial communication
  xTaskCreatePinnedToCore(
    serialCommTask,      // Function
    "SerialComm",        // Name
    4096,                // Stack size
    NULL,                // Parameter
    1,                   // Priority(lower)
    &serialTaskHandle,   // Handle
    0                    // Core 0
  );
  
  // Core 1: Motion planning(highest priority)
  xTaskCreatePinnedToCore(
    motionPlanningTask,  // Function
    "MotionPlanner",     // Name
    4096,                // Stack size
    NULL,                // Parameter
    3,                   // Priority(highest)
    &motionTaskHandle,   // Handle
    1                    // Core 1
  );
  
  Serial.println("FreeRTOS tasks created");
  Serial.println("=== Initialization Complete ===\n");
}

// ============================================================================
// ARDUINO LOOP(runs on Core 1, but we use FreeRTOS tasks)
// ============================================================================

void loop() {
  // Empty - all work done in FreeRTOS tasks
  vTaskDelay(pdMS_TO_TICKS(1000));
}