#ifndef ROBOT_TYPES_H
#define ROBOT_TYPES_H

#include <stdint.h>

//==================================================
// Common typedefs
//==================================================

typedef uint32_t MotionID;
typedef uint32_t Milliseconds;

//==================================================
// Geometry
//==================================================

struct Pose2D
{
    float x;
    float y;
};

struct Velocity2D
{
    float vx;
    float vy;
};

struct Acceleration2D
{
    float ax;
    float ay;
};

//==================================================
// Axis identifiers
//==================================================

enum class AxisID : uint8_t
{
    X = 0,
    Y = 1
};

enum class AxisMask : uint8_t
{
    NONE = 0,

    X = 1,

    Y = 2,

    BOTH = 3
};

//==================================================
// Hardware
//==================================================

struct HardwareLimits
{
    float maxSpeed;
    float maxAcceleration;
};

struct TrajectoryLimits
{
    float maxSpeed;
    float maxAcceleration;
};

enum class HomeDirection : uint8_t
{
    NEGATIVE,
    POSITIVE
};

struct AxisConfig
{
    float minPosition;
    float maxPosition;

    HomeDirection homeDirection;
};

struct HardwareConfig
{
    float stepsPerMM;

    AxisConfig x;

    AxisConfig y;

    HardwareLimits limits;
};

//==================================================
// Motion
//==================================================

enum class MotionMode : uint8_t
{
    POSITION,
    VELOCITY
};

enum class MotionState : uint8_t
{
    IDLE,
    MOVING,
    HOLDING,
    FAILED
};

enum class MotionResult : uint8_t
{
    NONE,

    RUNNING,

    SUCCESS,

    LIMIT,

    TIMEOUT,

    ESTOP,

    FAULT
};

struct MotionRequest
{
    MotionID id;

    MotionMode mode;

    Pose2D target;

    Velocity2D velocity;

    float targetSpeed;

    TrajectoryLimits limits;
};

struct MotionFeedback
{
    MotionID id;

    Milliseconds timestamp;

    MotionState state;

    MotionResult result;

    Pose2D position;

    Velocity2D velocity;

    Acceleration2D acceleration;

    Pose2D target;

    Pose2D positionError;

    float distanceRemaining;

    bool moving;
};

//==================================================
// Controller
//==================================================

enum class ControllerState : uint8_t
{
    STARTUP,
    READY,
    HOMING,
    DISABLED,
    FAULT
};

enum class HomingState : uint8_t
{
    IDLE,

    Y_FAST,
    Y_BACKOFF,
    Y_SLOW,

    X_FAST,
    X_BACKOFF,
    X_SLOW,

    COMPLETE,
    FAILED
};

enum class SafetyState : uint8_t
{
    NORMAL,
    WARNING,
    ESTOP,
    FAULT
};

struct MachineStatus
{
    ControllerState controller;

    HomingState homing;

    SafetyState safety;

    bool enabled;

    bool homed;
};

#endif