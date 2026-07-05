#ifndef MOTION_H
#define MOTION_H

#include <AccelStepper.h>

#include "RobotTypes.h"
#include "MachineConfig.h"

class Motion
{
public:

    Motion();

    void begin(
        const MachineConfig& config
    );

    void update();

    bool submit(
        const MotionRequest& request
    );

    void stop();

    void emergencyStop();

    void enable();

    void disable();

    bool busy() const;

    MotionFeedback getFeedback() const;

private:

    //--------------------------------------------------
    // Internal update functions
    //--------------------------------------------------

    void updateExecutor();

    void updateFeedback();

    void updateState();

    void checkTimeout();

    //--------------------------------------------------
    // Helpers
    //--------------------------------------------------

    long mmToSteps(
        float mm
    ) const;

    float stepsToMM(
        long steps
    ) const;

    //--------------------------------------------------
    // Configuration
    //--------------------------------------------------

    MachineConfig machine;

    //--------------------------------------------------
    // Stepper drivers
    //--------------------------------------------------

    AccelStepper stepperX;

    AccelStepper stepperY;

    //--------------------------------------------------
    // Current motion
    //--------------------------------------------------

    MotionRequest activeMotion;

    MotionFeedback feedback;

    //--------------------------------------------------
    // Runtime
    //--------------------------------------------------

    bool enabled;

    bool motionActive;

    bool requestChanged;

    //--------------------------------------------------
    // Timing
    //--------------------------------------------------

    Milliseconds motionStartTime;

    Milliseconds lastMovementTime;

    Milliseconds previousUpdateTime;

    //--------------------------------------------------
    // History
    //--------------------------------------------------

    float previousVelocityX;

    float previousVelocityY;

    float previousPositionX;

    float previousPositionY;

    MotionResult lastResult;
};

#endif