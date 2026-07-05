#include "Controller.h"
#include "Communication.h"

Controller controller;
Communication communication;

void setup()
{
    controller.begin();

    communication.begin();
}

void loop()
{
    controller.update();

    communication.update(controller);
}