#include "Communication.h"

#include "Controller.h"

void Communication::begin(
    unsigned long baud
)
{
    Serial.begin(baud);

    rxBuffer.reserve(128);
}

void Communication::update(
    Controller& controller
)
{
    while(Serial.available())
    {
        char c = Serial.read();

        if(c == '\r')
            continue;

        if(c == '\n')
        {
            processLine(
                rxBuffer,
                controller
            );

            rxBuffer = "";
        }
        else
        {
            rxBuffer += c;
        }
    }
}

void Communication::processLine(
    const String& line,
    Controller& controller
)
{
    //--------------------------------------------------
    // MOVE x y speed
    //--------------------------------------------------

    if(line.startsWith("MOVE"))
    {
        MotionRequest request;

        sscanf(
            line.c_str(),
            "MOVE %f %f %f",
            &request.target.x,
            &request.target.y,
            &request.targetSpeed
        );

        request.mode = MotionMode::POSITION;

        request.limits =
            MACHINE.motion.defaults;

        controller.move(request);

        Serial.println("OK");
        return;
    }

    //--------------------------------------------------
    // STOP
    //--------------------------------------------------

    if(line == "STOP")
    {
        controller.stop();

        Serial.println("OK");

        return;
    }

    //--------------------------------------------------
    // STATUS
    //--------------------------------------------------

    if(line == "STATUS")
    {
        sendTelemetry(controller);

        return;
    }

    Serial.println("ERR");
}

void Communication::sendTelemetry(
    Controller& controller
)
{
    MotionFeedback fb =
        controller.feedback();

    Serial.print("POS,");

    Serial.print(fb.position.x);

    Serial.print(",");

    Serial.print(fb.position.y);

    Serial.print(",");

    Serial.print(fb.velocity.vx);

    Serial.print(",");

    Serial.print(fb.velocity.vy);

    Serial.print(",");

    Serial.println(
        fb.distanceRemaining
    );
}