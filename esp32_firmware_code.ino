/*
 * ESP32 Dual-Core FreeRTOS Two-Axis Plotter Controller
 * 
 * Board: ESP32 DevKit V1(or similar ESP32 dual-core board)
 * 
 * Hardware:
 *   - ESP32 Dual Core
 *   - 2x A4988 Stepper Drivers(X and Y axes)
 *   - 2x 200 steps/rev stepper motors
 *   - GT2 belt, 20-tooth pulley(40mm/rev) for both axes
 *   - Resolution: 5 steps/mm at full step(both axes)
 *   - Micro servo for Z-axis(pen up/down)
 * 
 * Pin Configuration:
 *   X-Axis Stepper:
 *     STEP_X = GPIO18
 *     DIR_X  = GPIO19
 *   Y-Axis Stepper:
 *     STEP_Y = GPIO21
 *     DIR_Y  = GPIO22
 *   Z-Axis Servo:
 *     SERVO_PIN = GPIO23
 *   (ENABLE tied to GND, SLEEP/RESET tied to 3.3V for both drivers)
 * 
 * Architecture:
 *   - Core 0: Serial communication and command parsing
 *   - Core 1: Motion planning and profile generation(highest priority)
 *   - Hardware Timer ISR: Step pulse generation for both axes
 * 
 * Motion Profile:
 *   - Online S-curve approximation with jerk limiting
 *   - Coordinated 2-axis motion(both axes synchronized)
 *   - No abrupt velocity changes
 *   - Smooth acceleration and deceleration
 * 
 * Commands:
 *   G0 X### Y### F### - Rapid move to position
 *   G1 X### Y### F### - Linear move to position
 *   G90 - Absolute positioning mode
 *   G91 - Relative positioning mode
 *   M3 or G0 Z10 - Pen up
 *   M5 or G1 Z0 - Pen down
 *   G28 - Home(go to X0 Y0)
 *   P - Pause motion
 *   R - Resume motion
 *   E - Emergency stop
 *   + - Increase feed override(+10%)
 *   - - Decrease feed override(-10%)
 *   S - Print status
 * 
 * IMPORTANT: For ESP32 in Arduino IDE, only Arduino.h is needed!
 * FreeRTOS is built into the ESP32 Arduino core - no separate includes required.
 * 
 * ESP32 Arduino Core Version: 3.x compatible(tested on 3.3.8)
 * Note: Timer API changed significantly in Core 3.x
 */

#include 
#include   // Built-in servo library for ESP32

// ============================================================================
// HARDWARE CONFIGURATION
// ============================================================================

// X-Axis pins
#define STEP_X_PIN 18
#define DIR_X_PIN  19

// Y-Axis pins
#define STEP_Y_PIN 21
#define DIR_Y_PIN  22

// Z-Axis servo pin
#define SERVO_Z_PIN 23

// ============================================================================
// MOTION PARAMETERS - TUNING CONSTANTS
// ============================================================================

// Mechanical configuration(same for both X and Y axes)
const float STEPS_PER_MM = 5.0;           // Full steps per mm(200 steps/rev, 40mm/rev)
const float MAX_TRAVEL_X_MM = 290.0;      // Maximum X-axis travel
const float MAX_TRAVEL_Y_MM = 290.0;      // Maximum Y-axis travel

// Motion limits(all in mm and mm/s units)
const float MAX_VELOCITY = 100.0;         // mm/s - Maximum velocity
const float MAX_ACCELERATION = 500.0;     // mm/s² - Maximum acceleration
const float MAX_JERK = 5000.0;            // mm/s³ - Maximum jerk(rate of acceleration change)

// Default motion parameters
const float DEFAULT_FEEDRATE = 50.0;      // mm/s - Default feedrate for G0/G1

// Servo configuration(Z-axis pen lift)
const int SERVO_PEN_UP = 90;              // Servo angle for pen up(degrees)
const int SERVO_PEN_DOWN = 30;            // Servo angle for pen down(degrees)
const int SERVO_MOVE_DELAY = 300;         // Delay after servo move(ms)

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

// Servo object
Servo servoZ;
volatile bool penIsDown = false;

// Motion state - Two axes(X and Y)
volatile long currentPositionX = 0;       // Current X position in steps
volatile long currentPositionY = 0;       // Current Y position in steps
volatile long targetPositionX = 0;        // Target X position in steps
volatile long targetPositionY = 0;        // Target Y position in steps

// Direction for each axis
volatile bool directionXPositive = true;
volatile bool directionYPositive = true;

// Bresenham-style step timing for coordinated motion
volatile long stepCountX = 0;             // Steps taken on X
volatile long stepCountY = 0;             // Steps taken on Y
volatile long totalStepsX = 0;            // Total steps needed for X
volatile long totalStepsY = 0;            // Total steps needed for Y
volatile long dominantSteps = 0;          // Steps in dominant axis
volatile long errorAccumX = 0;            // Bresenham error for X
volatile long errorAccumY = 0;            // Bresenham error for Y

// Velocity profile(in terms of dominant axis steps/second)
volatile float currentVelocity = 0.0;     // Current velocity in steps/s
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
  float targetPositionX;   // Target X position in mm
  float targetPositionY;   // Target Y position in mm
  float feedrate;          // Feedrate in mm/s
  bool isRapid;            // True for G0, false for G1
  bool hasX;               // X coordinate specified
  bool hasY;               // Y coordinate specified
  bool hasZ;               // Z command specified
  bool zUp;                // True = pen up, False = pen down
};

// Shared state for Z-axis command
volatile bool pendingZUp = false;
volatile bool pendingZCommand = false;

// FreeRTOS queues
QueueHandle_t rawCommandQueue;      // UART → Parser
QueueHandle_t motionQueue;          // Parser → Motion Coordinator

// FreeRTOS Binary Semaphores
SemaphoreHandle_t zCommandSem;      // Coordinator → Z Task
SemaphoreHandle_t zCompleteSem;     // Z Task → Coordinator
SemaphoreHandle_t xyCommandSem;     // Coordinator → XY Task
SemaphoreHandle_t xyCompleteSem;    // XY Task → Coordinator

// FreeRTOS Mutex
SemaphoreHandle_t positionMutex;    // Protects shared position variables

// Task handles
TaskHandle_t uartReceiverHandle;
TaskHandle_t gCodeParserHandle;
TaskHandle_t shapeGeneratorHandle;
TaskHandle_t motionCoordinatorHandle;
TaskHandle_t zAxisControlHandle;
TaskHandle_t xyMotionControlHandle;

// ============================================================================
// HARDWARE TIMER ISR - STEP PULSE GENERATION(TWO AXES)
// ESP32 Arduino Core 3.x compatible
// Uses Bresenham-style algorithm for coordinated 2-axis motion
// ============================================================================

void IRAM_ATTR onStepTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  
  // Increment timer counter(called every 10µs)
  timerCounter++;
  
  if (stepsActive && motionState == MOTION_RUNNING) {
    // Check if we've reached target on both axes
    if (stepCountX >= totalStepsX && stepCountY >= totalStepsY) {
      stepsActive = false;
      portEXIT_CRITICAL_ISR(&timerMux);
      return;
    }
    
    // Check if it's time for the next step in the dominant axis
    if (timerCounter >= nextStepCount) {
      bool stepTaken = false;
      
      // Determine which axis/axes to step using Bresenham algorithm
      // This ensures coordinated motion along the desired path
      
      if (stepCountX < totalStepsX) {
        errorAccumX += totalStepsX;
        if (errorAccumX >= dominantSteps) {
          // Step X axis
          digitalWrite(STEP_X_PIN, HIGH);
          errorAccumX -= dominantSteps;
          stepTaken = true;
          
          // Update position
          if (directionXPositive) {
            currentPositionX++;
          } else {
            currentPositionX--;
          }
          stepCountX++;
        }
      }
      
      if (stepCountY < totalStepsY) {
        errorAccumY += totalStepsY;
        if (errorAccumY >= dominantSteps) {
          // Step Y axis
          digitalWrite(STEP_Y_PIN, HIGH);
          errorAccumY -= dominantSteps;
          stepTaken = true;
          
          // Update position
          if (directionYPositive) {
            currentPositionY++;
          } else {
            currentPositionY--;
          }
          stepCountY++;
        }
      }
      
      // Pulse must be at least 1µs for A4988
      if (stepTaken) {
        delayMicroseconds(2);
        digitalWrite(STEP_X_PIN, LOW);
        digitalWrite(STEP_Y_PIN, LOW);
      }
      
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
  float totalDistance = 0.0;     // Total move distance in mm
  
public:
  
  void startMove(float targetXmm, float targetYmm, float feedrateMmS, bool hasX, bool hasY) {
    // If no axes specified, do nothing
    if (!hasX && !hasY) {
      return;
    }
    
    // Apply feed override
    float effectiveFeedrate = feedrateMmS * (feedOverride / 100.0);
    maxVel = min(effectiveFeedrate, MAX_VELOCITY);
    
    // Convert to steps
    long targetStepsX = (long)(targetXmm * STEPS_PER_MM);
    long targetStepsY = (long)(targetYmm * STEPS_PER_MM);
    
    // Take mutex for position access
    if (xSemaphoreTake(positionMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      portENTER_CRITICAL(&timerMux);
      
      // Update target positions
      if (hasX) targetPositionX = targetStepsX;
      if (hasY) targetPositionY = targetStepsY;
      
      // Calculate deltas
      long deltaX = targetPositionX - currentPositionX;
      long deltaY = targetPositionY - currentPositionY;
    
    totalStepsX = abs(deltaX);
    totalStepsY = abs(deltaY);
    
    // Set directions
    if (deltaX > 0) {
      directionXPositive = true;
      digitalWrite(DIR_X_PIN, HIGH);
    } else {
      directionXPositive = false;
      digitalWrite(DIR_X_PIN, LOW);
    }
    
    if (deltaY > 0) {
      directionYPositive = true;
      digitalWrite(DIR_Y_PIN, HIGH);
    } else {
      directionYPositive = false;
      digitalWrite(DIR_Y_PIN, LOW);
    }
    
    // Dominant axis is the one with more steps
    dominantSteps = max(totalStepsX, totalStepsY);
    
    // Reset step counters and Bresenham error
    stepCountX = 0;
    stepCountY = 0;
    errorAccumX = 0;
    errorAccumY = 0;
    
    // Calculate actual distance in mm(Pythagorean)
    float deltaXmm = deltaX / STEPS_PER_MM;
    float deltaYmm = deltaY / STEPS_PER_MM;
    totalDistance = sqrt(deltaXmm * deltaXmm + deltaYmm * deltaYmm);
    
      stepsActive = true;
      motionState = MOTION_RUNNING;
      nextStepCount = timerCounter + 100;  // Start stepping soon
      portEXIT_CRITICAL(&timerMux);
      
      xSemaphoreGive(positionMutex);
    }
    
    // Reset motion profile
    currentAccel = 0.0;
    targetAccel = 0.0;
    plannedVelocity = currentVelocity / STEPS_PER_MM; // Convert from steps/s to mm/s
  }
  
  void updateProfile(float deltaTimeS) {
    if (motionState != MOTION_RUNNING) {
      return;
    }
    
    // Get current step counts safely
    long stepsX, stepsY, totalX, totalY;
    portENTER_CRITICAL(&timerMux);
    stepsX = stepCountX;
    stepsY = stepCountY;
    totalX = totalStepsX;
    totalY = totalStepsY;
    portEXIT_CRITICAL(&timerMux);
    
    // Calculate remaining steps in dominant axis
    float remainingStepsX = totalX - stepsX;
    float remainingStepsY = totalY - stepsY;
    float remainingDominant = max(remainingStepsX, remainingStepsY);
    
    // Calculate remaining distance in mm
    float remainingMm = (remainingDominant / STEPS_PER_MM) * (totalDistance / dominantSteps);
    
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
// CORE 1 TASKS - MOTION CONTROL
// ============================================================================

// ----------------------------------------------------------------------------
// TASK: Z-Axis Servo Control(Core 1, Priority 3)
// ----------------------------------------------------------------------------
void zAxisControlTask(void *parameter) {
  Serial.println("[Core 1] Z-Axis servo control task started");
  
  while (true) {
    // Wait for Z command signal from coordinator
    if (xSemaphoreTake(zCommandSem, portMAX_DELAY) == pdTRUE) {
      // Read Z command
      bool shouldBeUp = pendingZUp;
      
      if (shouldBeUp) {
        servoZ.write(SERVO_PEN_UP);
        penIsDown = false;
        Serial.println("[Z-Axis] Pen UP");
      } else {
        servoZ.write(SERVO_PEN_DOWN);
        penIsDown = true;
        Serial.println("[Z-Axis] Pen DOWN");
      }
      
      // Wait for servo to complete movement
      vTaskDelay(pdMS_TO_TICKS(SERVO_MOVE_DELAY));
      
      // Signal completion
      xSemaphoreGive(zCompleteSem);
    }
  }
}

// ----------------------------------------------------------------------------
// TASK: XY Motion Control(Core 1, Priority 3)
// ----------------------------------------------------------------------------
void xyMotionControlTask(void *parameter) {
  unsigned long lastUpdateTime = millis();
  
  Serial.println("[Core 1] XY motion control task started");
  
  while (true) {
    // Wait for XY command signal from coordinator
    if (xSemaphoreTake(xyCommandSem, pdMS_TO_TICKS(1)) == pdTRUE) {
      // Motion is already started by coordinator, just monitor until complete
      while (motionState == MOTION_RUNNING || stepsActive) {
        // Update motion profile at fixed rate
        unsigned long currentTime = millis();
        unsigned long deltaTime = currentTime - lastUpdateTime;
        
        if (deltaTime >= MOTION_UPDATE_MS) {
          float deltaTimeS = deltaTime / 1000.0;
          planner.updateProfile(deltaTimeS);
          lastUpdateTime = currentTime;
        }
        
        vTaskDelay(pdMS_TO_TICKS(1));
      }
      
      // Motion complete - update state
      if (xSemaphoreTake(positionMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        motionState = MOTION_IDLE;
        xSemaphoreGive(positionMutex);
      }
      
      Serial.println("[XY Motion] Move complete");
      
      // Signal completion
      xSemaphoreGive(xyCompleteSem);
    } else {
      // If no command, just update motion profile if running
      if (motionState == MOTION_RUNNING) {
        unsigned long currentTime = millis();
        unsigned long deltaTime = currentTime - lastUpdateTime;
        
        if (deltaTime >= MOTION_UPDATE_MS) {
          float deltaTimeS = deltaTime / 1000.0;
          planner.updateProfile(deltaTimeS);
          lastUpdateTime = currentTime;
        }
      }
    }
    
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

// ----------------------------------------------------------------------------
// TASK: Motion Coordinator(Core 1, Priority 4 - HIGHEST)
// Sequences Z-axis FIRST, then XY motion
// ----------------------------------------------------------------------------
void motionCoordinatorTask(void *parameter) {
  MotionCommand cmd;
  
  Serial.println("[Core 1] Motion Coordinator task started(HIGHEST PRIORITY)");
  
  while (true) {
    // Wait for motion commands from parser
    if (xQueueReceive(motionQueue, &cmd, portMAX_DELAY) == pdTRUE) {
      
      // ========================================
      // STEP 1: Execute Z-axis FIRST(if present)
      // ========================================
      if (cmd.hasZ) {
        Serial.print("[Coordinator] Z-axis command: ");
        Serial.println(cmd.zUp ? "Pen UP" : "Pen DOWN");
        
        // Set Z command for Z task
        pendingZUp = cmd.zUp;
        pendingZCommand = true;
        
        // Signal Z task to execute
        xSemaphoreGive(zCommandSem);
        
        // Wait for Z task to complete
        xSemaphoreTake(zCompleteSem, portMAX_DELAY);
        
        Serial.println("[Coordinator] Z-axis complete");
      }
      
      // ========================================
      // STEP 2: Execute XY motion(if present)
      // ========================================
      if (cmd.hasX || cmd.hasY) {
        Serial.print("[Coordinator] XY move to");
        if (cmd.hasX) {
          Serial.print(" X=");
          Serial.print(cmd.targetPositionX, 3);
        }
        if (cmd.hasY) {
          Serial.print(" Y=");
          Serial.print(cmd.targetPositionY, 3);
        }
        Serial.print(" mm at F=");
        Serial.print(cmd.feedrate, 1);
        Serial.println(" mm/s");
        
        // Start the motion
        planner.startMove(cmd.targetPositionX, cmd.targetPositionY, cmd.feedrate, cmd.hasX, cmd.hasY);
        
        // Signal XY task to monitor and complete
        xSemaphoreGive(xyCommandSem);
        
        // Wait for XY motion to complete
        xSemaphoreTake(xyCompleteSem, portMAX_DELAY);
        
        Serial.println("[Coordinator] XY motion complete");
      }
      
      // Command fully executed
      Serial.println("[Coordinator] Command complete, ready for next\n");
    }
  }
}

// ============================================================================
// CORE 0 TASKS - COMMUNICATION & PARSING
// ============================================================================

// ----------------------------------------------------------------------------
// TASK: UART Receiver(Core 0, Priority 1)
// Receives raw bytes from ATmega328P via Serial2(GPIO16/17) and buffers complete lines
// ----------------------------------------------------------------------------
void uartReceiverTask(void *parameter) {
  String inputBuffer = "";
  
  Serial.println("[Core 0] UART Receiver task started - listening on Serial2(ATmega328P)");
  
  while (true) {
    // Read from Serial2(ATmega328P connection)
    while (Serial2.available()) {
      char c = Serial2.read();
      
      if (c == '\n' || c == '\r') {
        if (inputBuffer.length() > 0) {
          // Echo to USB Serial for debugging
          Serial.print("[ATmega→ESP32] ");
          Serial.println(inputBuffer);
          
          // Send acknowledgment back to ATmega
          Serial2.println("Message received");
          
          // Send complete line to parser queue
          char *cmdCopy = (char*)malloc(inputBuffer.length() + 1);
          if (cmdCopy != NULL) {
            strcpy(cmdCopy, inputBuffer.c_str());
            if (xQueueSend(rawCommandQueue, &cmdCopy, pdMS_TO_TICKS(100)) != pdTRUE) {
              Serial.println("[UART] ERROR: Raw command queue full");
              Serial2.println("ERROR: Queue full");
              free(cmdCopy);
            }
          }
          inputBuffer = "";
        }
      } else {
        inputBuffer += c;
      }
    }
    
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ----------------------------------------------------------------------------
// TASK: Shape Generator(Core 0, Priority 1)
// Generates G-code for hardcoded shapes
// ----------------------------------------------------------------------------
void shapeGeneratorTask(void *parameter) {
  Serial.println("[Core 0] Shape Generator task started");
  
  while (true) {
    // This task can be triggered by special commands
    // For now, it's a placeholder
    // You can expand this to generate squares, rectangles, circles, etc.
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// Helper function to generate a square
void generateSquare(float size, float startX, float startY) {
  Serial.print("[ShapeGen] Generating square: size=");
  Serial.print(size);
  Serial.println(" mm");
  
  MotionCommand cmd;
  cmd.feedrate = DEFAULT_FEEDRATE;
  cmd.isRapid = false;
  
  // Pen up and move to start
  cmd.hasX = true;
  cmd.hasY = true;
  cmd.hasZ = true;
  cmd.zUp = true;
  cmd.targetPositionX = startX;
  cmd.targetPositionY = startY;
  xQueueSend(motionQueue, &cmd, portMAX_DELAY);
  vTaskDelay(pdMS_TO_TICKS(100));
  
  // Pen down
  cmd.hasX = false;
  cmd.hasY = false;
  cmd.hasZ = true;
  cmd.zUp = false;
  xQueueSend(motionQueue, &cmd, portMAX_DELAY);
  vTaskDelay(pdMS_TO_TICKS(100));
  
  // Draw square
  // Side 1: Right
  cmd.hasX = true;
  cmd.hasY = true;
  cmd.hasZ = false;
  cmd.targetPositionX = startX + size;
  cmd.targetPositionY = startY;
  xQueueSend(motionQueue, &cmd, portMAX_DELAY);
  vTaskDelay(pdMS_TO_TICKS(100));
  
  // Side 2: Up
  cmd.targetPositionX = startX + size;
  cmd.targetPositionY = startY + size;
  xQueueSend(motionQueue, &cmd, portMAX_DELAY);
  vTaskDelay(pdMS_TO_TICKS(100));
  
  // Side 3: Left
  cmd.targetPositionX = startX;
  cmd.targetPositionY = startY + size;
  xQueueSend(motionQueue, &cmd, portMAX_DELAY);
  vTaskDelay(pdMS_TO_TICKS(100));
  
  // Side 4: Down
  cmd.targetPositionX = startX;
  cmd.targetPositionY = startY;
  xQueueSend(motionQueue, &cmd, portMAX_DELAY);
  vTaskDelay(pdMS_TO_TICKS(100));
  
  // Pen up
  cmd.hasX = false;
  cmd.hasY = false;
  cmd.hasZ = true;
  cmd.zUp = true;
  xQueueSend(motionQueue, &cmd, portMAX_DELAY);
  
  Serial.println("[ShapeGen] Square generation complete");
}

// Helper function to generate a rectangle
void generateRectangle(float width, float height, float startX, float startY) {
  Serial.print("[ShapeGen] Generating rectangle: ");
  Serial.print(width);
  Serial.print("x");
  Serial.print(height);
  Serial.println(" mm");
  
  MotionCommand cmd;
  cmd.feedrate = DEFAULT_FEEDRATE;
  cmd.isRapid = false;
  
  // Pen up and move to start
  cmd.hasX = true;
  cmd.hasY = true;
  cmd.hasZ = true;
  cmd.zUp = true;
  cmd.targetPositionX = startX;
  cmd.targetPositionY = startY;
  xQueueSend(motionQueue, &cmd, portMAX_DELAY);
  vTaskDelay(pdMS_TO_TICKS(100));
  
  // Pen down
  cmd.hasX = false;
  cmd.hasY = false;
  cmd.hasZ = true;
  cmd.zUp = false;
  xQueueSend(motionQueue, &cmd, portMAX_DELAY);
  vTaskDelay(pdMS_TO_TICKS(100));
  
  // Draw rectangle
  cmd.hasX = true;
  cmd.hasY = true;
  cmd.hasZ = false;
  
  cmd.targetPositionX = startX + width;
  cmd.targetPositionY = startY;
  xQueueSend(motionQueue, &cmd, portMAX_DELAY);
  vTaskDelay(pdMS_TO_TICKS(100));
  
  cmd.targetPositionX = startX + width;
  cmd.targetPositionY = startY + height;
  xQueueSend(motionQueue, &cmd, portMAX_DELAY);
  vTaskDelay(pdMS_TO_TICKS(100));
  
  cmd.targetPositionX = startX;
  cmd.targetPositionY = startY + height;
  xQueueSend(motionQueue, &cmd, portMAX_DELAY);
  vTaskDelay(pdMS_TO_TICKS(100));
  
  cmd.targetPositionX = startX;
  cmd.targetPositionY = startY;
  xQueueSend(motionQueue, &cmd, portMAX_DELAY);
  vTaskDelay(pdMS_TO_TICKS(100));
  
  // Pen up
  cmd.hasX = false;
  cmd.hasY = false;
  cmd.hasZ = true;
  cmd.zUp = true;
  xQueueSend(motionQueue, &cmd, portMAX_DELAY);
  
  Serial.println("[ShapeGen] Rectangle generation complete");
}

// ----------------------------------------------------------------------------
// TASK: G-Code Parser(Core 0, Priority 2)
// Parses G-code and creates motion commands
// ----------------------------------------------------------------------------

void parseGCode(String line) {
  line.trim();
  line.toUpperCase();
  
  if (line.length() == 0) return;
  
  // Handle special shape commands
  if (line.startsWith("DRAW SQUARE")) {
    int spaceIdx = line.indexOf(' ', 12);
    float size = 50.0;  // Default size
    if (spaceIdx > 0) {
      size = line.substring(12).toFloat();
    }
    generateSquare(size, 10.0, 10.0);
    return;
  }
  
  if (line.startsWith("DRAW RECTANGLE")) {
    // Format: DRAW RECTANGLE width height
    int firstSpace = line.indexOf(' ', 15);
    int secondSpace = line.indexOf(' ', firstSpace + 1);
    float width = 50.0, height = 30.0;
    if (firstSpace > 0 && secondSpace > 0) {
      width = line.substring(15, secondSpace).toFloat();
      height = line.substring(secondSpace + 1).toFloat();
    }
    generateRectangle(width, height, 10.0, 10.0);
    return;
  }
  
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
        if (xSemaphoreTake(positionMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
          float posXmm = currentPositionX / STEPS_PER_MM;
          float posYmm = currentPositionY / STEPS_PER_MM;
          float velMmS = currentVelocity / STEPS_PER_MM;
          MotionState state = motionState;
          bool penDown = penIsDown;
          
          Serial.print("Position: X=");
          Serial.print(posXmm, 3);
          Serial.print(" mm, Y=");
          Serial.print(posYmm, 3);
          Serial.println(" mm");
          Serial.print("Pen: ");
          Serial.println(penDown ? "DOWN" : "UP");
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
          
          xSemaphoreGive(positionMutex);
        }
        Serial.println("==============\n");
        break;
    }
    return;
  }
  
  // Parse G-code
  if (line.startsWith("G")) {
    int gCode = -1;
    float xPos = 0.0;
    float yPos = 0.0;
    float zPos = 0.0;
    float feedrate = currentFeedrate;
    bool hasX = false;
    bool hasY = false;
    bool hasZ = false;
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
    
    int yIdx = line.indexOf('Y');
    if (yIdx >= 0) {
      hasY = true;
      int nextSpace = line.indexOf(' ', yIdx);
      if (nextSpace < 0) nextSpace = line.length();
      yPos = line.substring(yIdx + 1, nextSpace).toFloat();
    }
    
    int zIdx = line.indexOf('Z');
    if (zIdx >= 0) {
      hasZ = true;
      int nextSpace = line.indexOf(' ', zIdx);
      if (nextSpace < 0) nextSpace = line.length();
      zPos = line.substring(zIdx + 1, nextSpace).toFloat();
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
        // Handle Z axis(servo pen up/down) - will be sequenced by coordinator
        if (hasZ) {
          // Z command will be sent to motion queue and handled by coordinator
        }
        
        // Handle X and Y motion
        if (hasX || hasY) {
          float targetXmm = xPos;
          float targetYmm = yPos;
          
          // Convert to absolute if in relative mode
          if (!absoluteMode) {
            portENTER_CRITICAL(&timerMux);
            float currentXmm = currentPositionX / STEPS_PER_MM;
            float currentYmm = currentPositionY / STEPS_PER_MM;
            portEXIT_CRITICAL(&timerMux);
            
            if (hasX) targetXmm = currentXmm + xPos;
            else targetXmm = currentXmm;
            
            if (hasY) targetYmm = currentYmm + yPos;
            else targetYmm = currentYmm;
          } else {
            // In absolute mode, keep current position if axis not specified
            if (!hasX) {
              portENTER_CRITICAL(&timerMux);
              targetXmm = currentPositionX / STEPS_PER_MM;
              portEXIT_CRITICAL(&timerMux);
            }
            if (!hasY) {
              portENTER_CRITICAL(&timerMux);
              targetYmm = currentPositionY / STEPS_PER_MM;
              portEXIT_CRITICAL(&timerMux);
            }
          }
          
          // Check travel limits
          if (targetXmm < 0.0 || targetXmm > MAX_TRAVEL_X_MM) {
            Serial.print("ERROR: X position ");
            Serial.print(targetXmm, 2);
            Serial.print(" mm out of range [0, ");
            Serial.print(MAX_TRAVEL_X_MM, 0);
            Serial.println("]");
            return;
          }
          if (targetYmm < 0.0 || targetYmm > MAX_TRAVEL_Y_MM) {
            Serial.print("ERROR: Y position ");
            Serial.print(targetYmm, 2);
            Serial.print(" mm out of range [0, ");
            Serial.print(MAX_TRAVEL_Y_MM, 0);
            Serial.println("]");
            return;
          }
          
          // Create motion command
          MotionCommand cmd;
          cmd.targetPositionX = targetXmm;
          cmd.targetPositionY = targetYmm;
          cmd.feedrate = currentFeedrate;
          cmd.isRapid = (gCode == 0);
          cmd.hasX = hasX;
          cmd.hasY = hasY;
          cmd.hasZ = hasZ;
          cmd.zUp = (zPos > 5.0);  // Z > 5 means pen up
          
          // Send to motion queue
          if (xQueueSend(motionQueue, &cmd, pdMS_TO_TICKS(100)) != pdTRUE) {
            Serial.println("ERROR: Motion queue full");
          }
        }
        break;
      
      case 28:  // Home - go to X0 Y0
        Serial.println("Homing to X0 Y0...");
        {
          MotionCommand cmd;
          cmd.targetPositionX = 0.0;
          cmd.targetPositionY = 0.0;
          cmd.feedrate = DEFAULT_FEEDRATE;
          cmd.isRapid = true;
          cmd.hasX = true;
          cmd.hasY = true;
          cmd.hasZ = false;
          
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
  
  // Parse M-code(for pen up/down)
  if (line.startsWith("M")) {
    int mCode = line.substring(1).toInt();
    
    MotionCommand cmd;
    cmd.hasX = false;
    cmd.hasY = false;
    cmd.hasZ = true;
    cmd.feedrate = DEFAULT_FEEDRATE;
    cmd.isRapid = false;
    
    switch (mCode) {
      case 3:  // Pen down(spindle on in traditional CNC)
        Serial.println("M3: Pen DOWN");
        cmd.zUp = false;
        if (xQueueSend(motionQueue, &cmd, pdMS_TO_TICKS(100)) != pdTRUE) {
          Serial.println("ERROR: Motion queue full");
        }
        break;
        
      case 5:  // Pen up(spindle off in traditional CNC)
        Serial.println("M5: Pen UP");
        cmd.zUp = true;
        if (xQueueSend(motionQueue, &cmd, pdMS_TO_TICKS(100)) != pdTRUE) {
          Serial.println("ERROR: Motion queue full");
        }
        break;
        
      default:
        Serial.print("Unsupported M-code: M");
        Serial.println(mCode);
        break;
    }
  }
}

void gCodeParserTask(void *parameter) {
  char *rawCommand;
  
  Serial.println("[Core 0] G-Code Parser task started");
  
  while (true) {
    // Wait for raw commands from UART
    if (xQueueReceive(rawCommandQueue, &rawCommand, portMAX_DELAY) == pdTRUE) {
      Serial.print("> ");
      Serial.println(rawCommand);
      
      // Parse the command
      String cmdString = String(rawCommand);
      parseGCode(cmdString);
      
      // Free the allocated memory
      free(rawCommand);
    }
  }
}



// ============================================================================
// ARDUINO SETUP
// ============================================================================

void setup() {
  // Initialize USB Serial for debugging
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n\n=== ESP32 2-Axis Plotter Controller(CLIENT) Initializing ===");
  
  // Initialize Serial2 for communication with ATmega328P(SERVER)
  // RX2 = GPIO16, TX2 = GPIO17
  Serial2.begin(115200, SERIAL_8N1, 16, 17);
  Serial.println("Serial2 initialized(115200 baud, RX=GPIO16, TX=GPIO17)");
  Serial.println("Waiting for commands from ATmega328P...");
  
  // Configure GPIO pins
  pinMode(STEP_X_PIN, OUTPUT);
  pinMode(DIR_X_PIN, OUTPUT);
  pinMode(STEP_Y_PIN, OUTPUT);
  pinMode(DIR_Y_PIN, OUTPUT);
  digitalWrite(STEP_X_PIN, LOW);
  digitalWrite(DIR_X_PIN, LOW);
  digitalWrite(STEP_Y_PIN, LOW);
  digitalWrite(DIR_Y_PIN, LOW);
  
  Serial.println("GPIO configured");
  
  // Initialize servo
  servoZ.attach(SERVO_Z_PIN);
  servoZ.write(SERVO_PEN_UP);  // Start with pen up
  penIsDown = false;
  Serial.println("Servo initialized(Pen UP)");
  
  // Create FreeRTOS queues
  rawCommandQueue = xQueueCreate(20, sizeof(char*));
  if (rawCommandQueue == NULL) {
    Serial.println("ERROR: Failed to create raw command queue");
    while (1);
  }
  Serial.println("Raw command queue created");
  
  motionQueue = xQueueCreate(50, sizeof(MotionCommand));
  if (motionQueue == NULL) {
    Serial.println("ERROR: Failed to create motion queue");
    while (1);
  }
  Serial.println("Motion queue created");
  
  // Create binary semaphores
  zCommandSem = xSemaphoreCreateBinary();
  zCompleteSem = xSemaphoreCreateBinary();
  xyCommandSem = xSemaphoreCreateBinary();
  xyCompleteSem = xSemaphoreCreateBinary();
  
  if (zCommandSem == NULL || zCompleteSem == NULL || 
      xyCommandSem == NULL || xyCompleteSem == NULL) {
    Serial.println("ERROR: Failed to create semaphores");
    while (1);
  }
  Serial.println("Binary semaphores created");
  
  // Create mutex
  positionMutex = xSemaphoreCreateMutex();
  if (positionMutex == NULL) {
    Serial.println("ERROR: Failed to create position mutex");
    while (1);
  }
  Serial.println("Position mutex created");
  
  // Configure hardware timer(ESP32 Arduino Core 3.x API)
  // Timer runs at 100kHz (every 10 microseconds)
  stepTimer = timerBegin(100000);  // 100kHz = 100,000 Hz
  timerAttachInterrupt(stepTimer, &onStepTimer);
  timerAlarm(stepTimer, 1, true, 0);  // Trigger every 1 count(10µs), autoreload, unlimited
  Serial.println("Hardware timer configured(100kHz, 10µs interval)");
  
  // Create FreeRTOS tasks
  Serial.println("\n=== Creating RTOS Tasks ===");
  
  // CORE 0 TASKS(Communication & Parsing)
  
  // Task 1: UART Receiver
  xTaskCreatePinnedToCore(
    uartReceiverTask,
    "UARTReceiver",
    4096,
    NULL,
    1,                   // Priority 1
    &uartReceiverHandle,
    0                    // Core 0
  );
  Serial.println("Created: UART Receiver(Core 0, Priority 1)");
  
  // Task 2: G-Code Parser
  xTaskCreatePinnedToCore(
    gCodeParserTask,
    "GCodeParser",
    4096,
    NULL,
    2,                   // Priority 2
    &gCodeParserHandle,
    0                    // Core 0
  );
  Serial.println("Created: G-Code Parser(Core 0, Priority 2)");
  
  // Task 3: Shape Generator
  xTaskCreatePinnedToCore(
    shapeGeneratorTask,
    "ShapeGenerator",
    4096,
    NULL,
    1,                   // Priority 1
    &shapeGeneratorHandle,
    0                    // Core 0
  );
  Serial.println("Created: Shape Generator(Core 0, Priority 1)");
  
  // CORE 1 TASKS(Motion Control)
  
  // Task 4: Motion Coordinator(HIGHEST PRIORITY)
  xTaskCreatePinnedToCore(
    motionCoordinatorTask,
    "MotionCoordinator",
    8192,
    NULL,
    4,                   // Priority 4 (HIGHEST)
    &motionCoordinatorHandle,
    1                    // Core 1
  );
  Serial.println("Created: Motion Coordinator(Core 1, Priority 4 - HIGHEST)");
  
  // Task 5: Z-Axis Control
  xTaskCreatePinnedToCore(
    zAxisControlTask,
    "ZAxisControl",
    4096,
    NULL,
    3,                   // Priority 3
    &zAxisControlHandle,
    1                    // Core 1
  );
  Serial.println("Created: Z-Axis Control(Core 1, Priority 3)");
  
  // Task 6: XY Motion Control
  xTaskCreatePinnedToCore(
    xyMotionControlTask,
    "XYMotionControl",
    8192,
    NULL,
    3,                   // Priority 3
    &xyMotionControlHandle,
    1                    // Core 1
  );
  Serial.println("Created: XY Motion Control(Core 1, Priority 3)");
  
  Serial.println("\n=== All Tasks Created Successfully ===");
  Serial.println("\n=== ESP32 2-Axis Plotter Controller Ready ===");
  Serial.println("Commands:");
  Serial.println("  G0/G1 X### Y### Z### F### - Move/Pen control");
  Serial.println("  G28 - Home(X0 Y0)");
  Serial.println("  M3 - Pen down, M5 - Pen up");
  Serial.println("  DRAW SQUARE [size] - Draw square");
  Serial.println("  DRAW RECTANGLE [width] [height] - Draw rectangle");
  Serial.println("  P - Pause, R - Resume, E - E-Stop");
  Serial.println("  S - Status");
  Serial.println("=============================================\n");
}

// ============================================================================
// ARDUINO LOOP(runs on Core 1, but we use FreeRTOS tasks)
// ============================================================================

void loop() {
  // Empty - all work done in FreeRTOS tasks
  vTaskDelay(pdMS_TO_TICKS(1000));
}
