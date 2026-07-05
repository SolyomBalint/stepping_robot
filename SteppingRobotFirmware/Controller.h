#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "RobotTypes.h"
#include "MachineConfig.h"
#include "Motion.h"

class Controller
{
public:

    void begin();

    void update();

    Motion& getMotion()
    {
        return motion;
    }

    bool move(
        const MotionRequest& request
    );

    void stop();

    MotionFeedback feedback() const;

private:

    MachineConfig machine;

    MachineStatus status;

    Motion motion;

    void updateController();
};

#endif