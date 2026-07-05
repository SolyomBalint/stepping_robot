#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <Arduino.h>

#include "RobotTypes.h"

class Controller;

class Communication
{
public:

    void begin(
        unsigned long baud = 115200
    );

    void update(
        Controller& controller
    );

private:

    String rxBuffer;

    void processLine(
        const String& line,
        Controller& controller
    );

    void sendTelemetry(
        Controller& controller
    );
};

#endif