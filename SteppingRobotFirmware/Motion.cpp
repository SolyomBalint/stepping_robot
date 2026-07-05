#include "Motion.h"

#include <math.h>

#include <string.h>

//--------------------------------------------------
// Constructor
//--------------------------------------------------

Motion::Motion()
:
stepperX(
    AccelStepper::DRIVER,
    X_STEP_PIN,
    X_DIR_PIN
),
stepperY(
    AccelStepper::DRIVER,
    Y_STEP_PIN,
    Y_DIR_PIN
)
{
}

//--------------------------------------------------
// Helpers
//--------------------------------------------------

long Motion::mmToSteps(
    float mm
) const
{
    return (long)(
        mm *
        machine.hardware.stepsPerMM
    );
}

float Motion::stepsToMM(
    long steps
) const
{
    return
        (float)steps
        /
        machine.hardware.stepsPerMM;
}

//--------------------------------------------------
// Initialization
//--------------------------------------------------

void Motion::begin(
    const MachineConfig& config
)
{
    machine = config;

    pinMode(
        machine.io.enablePin,
        OUTPUT
    );

    disable();

    stepperX.setCurrentPosition(0);
    stepperY.setCurrentPosition(0);

    enabled = false;
    motionActive = false;
    requestChanged = false;

    previousPositionX = 0.0f;
    previousPositionY = 0.0f;

    previousVelocityX = 0.0f;
    previousVelocityY = 0.0f;

    lastResult =
        MotionResult::NONE;

    Milliseconds now =
        millis();

    motionStartTime =
        now;

    lastMovementTime =
        now;

    previousUpdateTime =
        now;

    memset(&feedback, 0, sizeof(feedback));

    feedback.state = MotionState::IDLE;
    feedback.result = MotionResult::NONE;
}

//--------------------------------------------------
// Submit new motion
//--------------------------------------------------

bool Motion::submit(
    const MotionRequest& request
)
{
    activeMotion =
        request;

    // Clamp target

    if(activeMotion.target.x <
       machine.hardware.x.minPosition)
    {
        activeMotion.target.x =
            machine.hardware.x.minPosition;
    }

    if(activeMotion.target.x >
       machine.hardware.x.maxPosition)
    {
        activeMotion.target.x =
            machine.hardware.x.maxPosition;
    }

    if(activeMotion.target.y <
       machine.hardware.y.minPosition)
    {
        activeMotion.target.y =
            machine.hardware.y.minPosition;
    }

    if(activeMotion.target.y >
       machine.hardware.y.maxPosition)
    {
        activeMotion.target.y =
            machine.hardware.y.maxPosition;
    }

    // Clamp limits

    if(activeMotion.targetSpeed >
       machine.hardware.limits.maxSpeed)
    {
        activeMotion.targetSpeed =
            machine.hardware.limits.maxSpeed;
    }

    if(activeMotion.limits.maxAcceleration >
       machine.hardware.limits.maxAcceleration)
    {
        activeMotion.limits.maxAcceleration =
            machine.hardware.limits.maxAcceleration;
    }

    requestChanged = true;

    motionActive = true;

    lastResult =
        MotionResult::RUNNING;

    motionStartTime =
        millis();

    return true;
}

//--------------------------------------------------
// Main update
//--------------------------------------------------

void Motion::update()
{
    updateExecutor();

    updateFeedback();

    updateState();

    checkTimeout();
}

//--------------------------------------------------
// Motion executor
//--------------------------------------------------

void Motion::updateExecutor()
{
    if(requestChanged)
    {
        enable();

        float speed = min(
            activeMotion.targetSpeed,
            activeMotion.limits.maxSpeed
        );

        if(speed <= 0.0f)
        {
            speed =
                machine.motion.defaults.maxSpeed;
        }

        if(speed >
           machine.hardware.limits.maxSpeed)
        {
            speed =
                machine.hardware.limits.maxSpeed;
        }

        float accel =
            activeMotion.limits.maxAcceleration;

        if(accel <= 0.0f)
        {
            accel =
                machine.motion.defaults.maxAcceleration;
        }

        if(accel >
           machine.hardware.limits.maxAcceleration)
        {
            accel =
                machine.hardware.limits.maxAcceleration;
        }

        stepperX.setMaxSpeed(
            speed *
            machine.hardware.stepsPerMM
        );

        stepperY.setMaxSpeed(
            speed *
            machine.hardware.stepsPerMM
        );

        stepperX.setAcceleration(
            accel *
            machine.hardware.stepsPerMM
        );

        stepperY.setAcceleration(
            accel *
            machine.hardware.stepsPerMM
        );

        switch(activeMotion.mode)
        {
        case MotionMode::POSITION:

            stepperX.moveTo(
                mmToSteps(
                    activeMotion.target.x
                )
            );

            stepperY.moveTo(
                mmToSteps(
                    activeMotion.target.y
                )
            );

            break;

        case MotionMode::VELOCITY:

            stepperX.setSpeed(
                activeMotion.velocity.vx *
                machine.hardware.stepsPerMM
            );

            stepperY.setSpeed(
                activeMotion.velocity.vy *
                machine.hardware.stepsPerMM
            );

            break;
        }

        requestChanged = false;
    }

    switch(activeMotion.mode)
    {
    case MotionMode::POSITION:

        stepperX.run();
        stepperY.run();

        break;

    case MotionMode::VELOCITY:

        stepperX.runSpeed();
        stepperY.runSpeed();

        break;
    }
}

//--------------------------------------------------
// Feedback
//--------------------------------------------------

void Motion::updateFeedback()
{
    Milliseconds now = millis();

    float dt =
        (float)(now - previousUpdateTime) / 1000.0f;

    if(dt <= 0.0f)
    {
        dt = 0.001f;
    }

    previousUpdateTime = now;

    //----------------------------------
    // Position
    //----------------------------------

    feedback.position.x =
        stepsToMM(
            stepperX.currentPosition()
        );

    feedback.position.y =
        stepsToMM(
            stepperY.currentPosition()
        );

    //----------------------------------
    // Velocity estimate
    //----------------------------------

    feedback.velocity.vx =
        (feedback.position.x - previousPositionX) / dt;

    feedback.velocity.vy =
        (feedback.position.y - previousPositionY) / dt;

    //----------------------------------
    // Acceleration estimate
    //----------------------------------

    feedback.acceleration.ax =
        (feedback.velocity.vx - previousVelocityX) / dt;

    feedback.acceleration.ay =
        (feedback.velocity.vy - previousVelocityY) / dt;

    previousPositionX =
        feedback.position.x;

    previousPositionY =
        feedback.position.y;

    previousVelocityX =
        feedback.velocity.vx;

    previousVelocityY =
        feedback.velocity.vy;

    //----------------------------------
    // Target
    //----------------------------------

    feedback.target =
        activeMotion.target;

    //----------------------------------
    // Error
    //----------------------------------

    feedback.positionError.x =
        feedback.target.x -
        feedback.position.x;

    feedback.positionError.y =
        feedback.target.y -
        feedback.position.y;

    //----------------------------------
    // Distance remaining
    //----------------------------------

    float dx =
        feedback.positionError.x;

    float dy =
        feedback.positionError.y;

    feedback.distanceRemaining =
        sqrt(
            dx * dx +
            dy * dy
        );

    //----------------------------------
    // Timestamp
    //----------------------------------

    feedback.timestamp =
        now;

    //----------------------------------
    // Moving
    //----------------------------------

    feedback.moving =
        (stepperX.distanceToGo() != 0) ||
        (stepperY.distanceToGo() != 0);

    if(feedback.moving)
    {
        lastMovementTime =
            now;
    }

    feedback.result =
        lastResult;
}

//--------------------------------------------------
// State machine
//--------------------------------------------------

void Motion::updateState()
{
    if(lastResult != MotionResult::RUNNING)
    {
        feedback.state =
            MotionState::IDLE;

        motionActive = false;

        return;
    }

    if(feedback.moving)
    {
        feedback.state =
            MotionState::MOVING;
    }
    else
    {
        feedback.state =
            MotionState::HOLDING;

        lastResult =
            MotionResult::SUCCESS;
    }
}

//--------------------------------------------------
// Timeout detection
//--------------------------------------------------

void Motion::checkTimeout()
{
    if(!feedback.moving)
    {
        return;
    }

    if(
        millis() -
        lastMovementTime
        >
        machine.timing.motionTimeout
    )
    {
        stop();

        lastResult =
            MotionResult::TIMEOUT;

        feedback.state =
            MotionState::FAILED;
    }
}


//--------------------------------------------------
// Enable
//--------------------------------------------------

void Motion::enable()
{
    if(machine.io.enableActiveLow)
    {
        digitalWrite(
            machine.io.enablePin,
            LOW
        );
    }
    else
    {
        digitalWrite(
            machine.io.enablePin,
            HIGH
        );
    }

    enabled = true;
}

//--------------------------------------------------
// Disable
//--------------------------------------------------

void Motion::disable()
{
    if(machine.io.enableActiveLow)
    {
        digitalWrite(
            machine.io.enablePin,
            HIGH
        );
    }
    else
    {
        digitalWrite(
            machine.io.enablePin,
            LOW
        );
    }

    enabled = false;
}

//--------------------------------------------------
// Stop
//--------------------------------------------------

void Motion::stop()
{
    stepperX.stop();
    stepperY.stop();

    motionActive = false;
}

void Motion::emergencyStop()
{
    stepperX.stop();
    stepperY.stop();

    disable();

    motionActive = false;

    lastResult =
        MotionResult::ESTOP;
}

//--------------------------------------------------
// Status
//--------------------------------------------------

bool Motion::busy() const
{
    return
    feedback.moving;
}

MotionFeedback Motion::getFeedback() const
{
    return feedback;
}