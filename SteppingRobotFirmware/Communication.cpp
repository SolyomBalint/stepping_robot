#include "Communication.h"

#include "Controller.h"

#include <stdio.h>

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

    if(
        millis() - lastTelemetry >= 100
    )
    {
        sendTelemetry(controller);

        lastTelemetry = millis();
    }
}

void Communication::processLine(
    String line,
    Controller& controller
)
{
    line.trim();

    if(line.length() == 0)
        return;

    //----------------------------------------
    // PING
    //----------------------------------------

    if(line == "PING")
    {
        Serial.println("PONG");
        return;
    }

    //----------------------------------------
    // STATUS
    //----------------------------------------

    if(line == "STATUS")
    {
        sendTelemetry(controller);
        return;
    }

    //----------------------------------------
    // ENABLE
    //----------------------------------------

    if(line == "ENABLE")
    {
        controller.getMotion().enable();

        Serial.println("OK");

        return;
    }

    //----------------------------------------
    // DISABLE
    //----------------------------------------

    if(line == "DISABLE")
    {
        controller.getMotion().disable();

        Serial.println("OK");

        return;
    }

    //----------------------------------------
    // STOP
    //----------------------------------------

    if(line == "STOP")
    {
        controller.stop();

        Serial.println("OK");

        return;
    }

    //----------------------------------------
    // MOVE x y speed
    //----------------------------------------

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

        if(controller.move(request))
        {
            Serial.println("OK");
        }
        else
        {
            Serial.println("BUSY");
        }

        return;
    }

    //----------------------------------------
    // VEL vx vy
    //----------------------------------------

    if(line.startsWith("VEL"))
    {
        MotionRequest request;

        request.mode =
            MotionMode::VELOCITY;

        sscanf(
            line.c_str(),
            "VEL %f %f",
            &request.velocity.vx,
            &request.velocity.vy
        );

        request.targetSpeed =
            MACHINE.motion.defaults.maxSpeed;

        request.limits =
            MACHINE.motion.defaults;

        if(controller.move(request))
        {
            Serial.println("OK");
        }
        else
        {
            Serial.println("BUSY");
        }

        return;
    }

    //----------------------------------------
    // Unknown command
    //----------------------------------------

    Serial.println("ERR");
}

void Communication::sendTelemetry(
    Controller& controller
)
{
    MotionFeedback fb =
        controller.feedback();

    Serial.print("TEL,");

    Serial.print(fb.position.x);
    Serial.print(",");

    Serial.print(fb.position.y);
    Serial.print(",");

    Serial.print(fb.velocity.vx);
    Serial.print(",");

    Serial.print(fb.velocity.vy);
    Serial.print(",");

    Serial.print(fb.distanceRemaining);
    Serial.print(",");

    Serial.print(fb.moving);
    Serial.print(",");

    Serial.println(
        (int)fb.state
    );
}