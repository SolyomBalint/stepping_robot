#include "Controller.h"

bool Controller::move(
    const MotionRequest& request
)
{
    return motion.submit(request);
}

void Controller::stop()
{
    motion.stop();
}

MotionFeedback Controller::feedback() const
{
    return motion.getFeedback();
}

void Controller::begin()
{
    machine = MACHINE;

    status = {};

    status.controller =
        ControllerState::STARTUP;

    motion.begin(machine);

    status.controller =
        ControllerState::READY;
}

void Controller::update()
{
    motion.update();

    updateController();
}

void Controller::updateController()
{
    MotionFeedback feedback =
        motion.getFeedback();

    switch(status.controller)
    {
    case ControllerState::STARTUP:

        break;

    case ControllerState::READY:

        break;

    case ControllerState::HOMING:

        break;

    case ControllerState::DISABLED:

        break;

    case ControllerState::FAULT:

        break;
    }


}