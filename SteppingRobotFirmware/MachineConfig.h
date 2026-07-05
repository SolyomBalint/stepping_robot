#ifndef MACHINE_CONFIG_H
#define MACHINE_CONFIG_H

#include "RobotTypes.h"

//==================================================
// Firmware configuration
//==================================================

constexpr uint8_t MOTION_QUEUE_SIZE = 8;

//==================================================
// Pin assignments
//==================================================

// X axis
constexpr uint8_t X_STEP_PIN = 2;
constexpr uint8_t X_DIR_PIN  = 5;

// Y axis
constexpr uint8_t Y_STEP_PIN = 3;
constexpr uint8_t Y_DIR_PIN  = 6;

// Shared enable (CNC Shield V3)
constexpr uint8_t ENABLE_PIN = 8;

// Limit switches
constexpr uint8_t X_LIMIT_PIN = 9;
constexpr uint8_t Y_LIMIT_PIN = 10;

// Optional emergency stop
constexpr uint8_t ESTOP_PIN = 11;

//==================================================
// Electrical configuration
//==================================================

struct AxisHardware
{
    uint8_t stepPin;
    uint8_t dirPin;

    uint8_t negativeLimitPin;

    bool invertDirection;
};

struct IOConfig
{
    AxisHardware x;

    AxisHardware y;

    uint8_t enablePin;

    uint8_t estopPin;

    bool enableActiveLow;

    bool limitActiveLow;
};

//==================================================
// Motion configuration
//==================================================

struct MotionConfig
{
    TrajectoryLimits defaults;

    bool queueEnabled;

    bool blendingEnabled;

    bool velocityModeEnabled;

    uint8_t queueSize;
};

//==================================================
// Homming
//==================================================

struct HomingConfig
{
    float fastSpeed;

    float slowSpeed;

    float backoffDistance;

    float switchDebounceTime;
};

//==================================================
// Timing
//==================================================

struct TimingConfig
{
    Milliseconds telemetryPeriod;

    Milliseconds idleDisableDelay;

    Milliseconds motionTimeout;

    Milliseconds homingTimeout;
};

//==================================================
// Complete machine configuration
//==================================================

struct MachineConfig
{
    HardwareConfig hardware;

    IOConfig io;

    MotionConfig motion;

    TimingConfig timing;
};

//==================================================
// Default machine configuration
//==================================================

const MachineConfig MACHINE =
{
    HardwareConfig
    {
        80.0f,

        AxisConfig
        {
            0.0f,
            400.0f,
            HomeDirection::NEGATIVE
        },

        AxisConfig
        {
            0.0f,
            400.0f,
            HomeDirection::NEGATIVE
        },

        HardwareLimits
        {
            300.0f,
            1500.0f
        }
    },

    IOConfig
    {
        {
            X_STEP_PIN,
            X_DIR_PIN,
            X_LIMIT_PIN,
            false
        },

        {
            Y_STEP_PIN,
            Y_DIR_PIN,
            Y_LIMIT_PIN,
            false
        },

        ENABLE_PIN,
        ESTOP_PIN,
        true,
        true
    },

    MotionConfig
    {
        TrajectoryLimits
        {
            200.0f,
            800.0f
        },

        true,
        true,
        true,
        MOTION_QUEUE_SIZE
    },

    TimingConfig
    {
        50,
        1000,
        5000,
        20000
    }
};

#endif