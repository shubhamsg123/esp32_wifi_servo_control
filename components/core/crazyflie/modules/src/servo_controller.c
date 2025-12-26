/**
 * ESP-Drone Firmware - Servo Controller
 * 
 * Minimal controller that bypasses stabilization and directly maps
 * CRTP thrust/roll/pitch/yaw commands to MG995 servo motors.
 *
 * Motor Mapping:
 * - Thrust -> Servo 1 (M1)
 * - Roll   -> Servo 2 (M2)
 * - Pitch  -> Servo 3 (M3)
 * - Yaw    -> Servo 4 (M4)
 *
 * Copyright 2019-2020 Espressif Systems (Shanghai)
 * GPL3.0 License
 */

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "servo_controller.h"
#include "crtp.h"
#include "crtp_commander.h"
#include "motors.h"
#include "system.h"
#include "log.h"
#include "param.h"
#include "stm32_legacy.h"
#include "static_mem.h"
#include "platform.h"

#define DEBUG_MODULE "SERVO"
#include "debug_cf.h"

// Task configuration
#define SERVO_TASK_STACKSIZE  (2 * configMINIMAL_STACK_SIZE)
#define SERVO_TASK_NAME       "SERVO"
#define SERVO_TASK_PRI        5
#define SERVO_UPDATE_RATE_HZ  50

// CRTP packet format for thrust/roll/pitch/yaw
struct CommanderCrtpValues {
    float roll;       // degrees
    float pitch;      // degrees
    float yaw;        // deg/s
    uint16_t thrust;  // 0-65535
} __attribute__((packed));

static bool isInit = false;
static uint16_t servo1_us = SERVO_MID_US;
static uint16_t servo2_us = SERVO_MID_US;
static uint16_t servo3_us = SERVO_MID_US;
static uint16_t servo4_us = SERVO_MID_US;

STATIC_MEM_TASK_ALLOC(servoControllerTask, SERVO_TASK_STACKSIZE);

// Clamp value to range
static inline float clampf(float val, float min, float max) {
    if (val < min) return min;
    if (val > max) return max;
    return val;
}

// Map thrust (0-65535) to servo microseconds (1500-2000, forward only)
static uint16_t thrustToMicroseconds(uint16_t thrust) {
    // 0 = stop (1500us), 65535 = full forward (2000us)
    return SERVO_MID_US + (thrust * 500 / 65535);
}

// Map angle (-45 to +45 deg) to servo microseconds (1000-2000)
static uint16_t angleToMicroseconds(float angle) {
    // -45 = 1000us, 0 = 1500us, +45 = 2000us
    float clamped = clampf(angle, -45.0f, 45.0f);
    return (uint16_t)(SERVO_MID_US + (clamped * 500.0f / 45.0f));
}

// Map yaw rate (-200 to +200 deg/s) to servo microseconds (1000-2000)
static uint16_t rateToMicroseconds(float rate) {
    // -200 = 1000us, 0 = 1500us, +200 = 2000us
    float clamped = clampf(rate, -200.0f, 200.0f);
    return (uint16_t)(SERVO_MID_US + (clamped * 500.0f / 200.0f));
}

// CRTP callback for commander port
static void servoCommanderCallback(CRTPPacket *pk) {
    struct CommanderCrtpValues *cmd = (struct CommanderCrtpValues *)pk->data;
    
    // Map CRTP values to servo microseconds
    servo1_us = thrustToMicroseconds(cmd->thrust);
    servo2_us = angleToMicroseconds(cmd->roll);
    servo3_us = angleToMicroseconds(cmd->pitch);
    servo4_us = rateToMicroseconds(cmd->yaw);
    
    // Apply to servos
    servoWriteMicroseconds(MOTOR_M1, servo1_us);
    servoWriteMicroseconds(MOTOR_M2, servo2_us);
    servoWriteMicroseconds(MOTOR_M3, servo3_us);
    servoWriteMicroseconds(MOTOR_M4, servo4_us);
}

static void servoControllerTask(void *param) {
    systemWaitStart();
    
    DEBUG_PRINT("Servo Controller started\n");
    
    // Initialize motors for servo mode
    motorsInit(platformConfigGetMotorMapping());
    
    // Set all servos to neutral on startup
    servoStopAll();
    
    // Register CRTP callback for commander packets
    crtpRegisterPortCB(CRTP_PORT_SETPOINT, servoCommanderCallback);
    
    // Main loop - just wait, callbacks handle commands
    while (1) {
        vTaskDelay(M2T(1000 / SERVO_UPDATE_RATE_HZ));
    }
}

void servoControllerInit(void) {
    if (isInit) return;
    
    STATIC_MEM_TASK_CREATE(servoControllerTask, servoControllerTask, 
                           SERVO_TASK_NAME, NULL, SERVO_TASK_PRI);
    
    isInit = true;
    DEBUG_PRINT("Servo Controller initialized\n");
}

bool servoControllerTest(void) {
    return isInit;
}

// Logging
LOG_GROUP_START(servo)
LOG_ADD(LOG_UINT16, s1_us, &servo1_us)
LOG_ADD(LOG_UINT16, s2_us, &servo2_us)
LOG_ADD(LOG_UINT16, s3_us, &servo3_us)
LOG_ADD(LOG_UINT16, s4_us, &servo4_us)
LOG_GROUP_STOP(servo)
