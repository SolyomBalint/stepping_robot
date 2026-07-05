#include <AccelStepper.h>

//==================================================
// Pins
//==================================================

#define X_STEP 2
#define X_DIR  5

#define Y_STEP 3
#define Y_DIR  6

#define X_HOME 8
#define Y_HOME 9

#define ENABLE_PIN 8

//==================================================
// Machine and control constants
//==================================================

const float STEPS_PER_MM = 80.0;

const float X_MAX_MM = 400.0;
const float Y_MAX_MM = 400.0;

const float HOME_FAST_SPEED = 1000;
const float HOME_SLOW_SPEED = 200;
const float HOME_BACKOFF_MM = 5.0;

//==================================================
// Stepper objects
//==================================================

AccelStepper stepperX(
    AccelStepper::DRIVER,
    X_STEP,
    X_DIR
);

AccelStepper stepperY(
    AccelStepper::DRIVER,
    Y_STEP,
    Y_DIR
);

//==================================================
// Motors enable and disable
//==================================================

void motorsEnable()
{
    digitalWrite(
        ENABLE_PIN,
        LOW
    );
}

void motorsDisable()
{
    digitalWrite(
        ENABLE_PIN,
        HIGH
    );
}

//==================================================
// Motion state
//==================================================

struct MotionState
{
    // Current estimated state
    float posX;
    float posY;

    float velX;
    float velY;

    float accelX;
    float accelY;

    // Commanded state
    float targetX;
    float targetY;

    float targetSpeed;

    // Limits
    float maxSpeed;
    float maxAccel;

    // Diagnostic
    float errorX;
    float errorY;
};

MotionState motion = {0};

//==================================================
// Runtime flags
//==================================================

bool targetChanged = false;
bool motionChanged = false;

//==================================================
// Acceleration estimation
//==================================================

unsigned long lastUpdate = 0;

float previousVelX = 0;
float previousVelY = 0;

//==================================================
// Helpers
//==================================================

long mmToSteps(float mm)
{
    return (long)(mm * STEPS_PER_MM);
}

float clamp(
    float value,
    float low,
    float high
)
{
    return max(
        low,
        min(
            value,
            high
        )
    );
}

//==================================================
// Motion API
//==================================================

void setTarget(
    float x,
    float y
)
{
    motion.targetX =
        clamp(
            x,
            0,
            X_MAX_MM
        );

    motion.targetY =
        clamp(
            y,
            0,
            Y_MAX_MM
        );

    targetChanged =
        true;
}

void configureMotion(
    float speed,
    float accel
)
{
    motion.maxSpeed =
        speed;

    motion.maxAccel =
        accel;

    motionChanged =
        true;
}

//==================================================
// Motion update
//==================================================

void updateMotion()
{
    // Apply motion settings

    if (
        motionChanged
    )
    {
        stepperX.setMaxSpeed(
            motion.maxSpeed
        );

        stepperY.setMaxSpeed(
            motion.maxSpeed
        );

        stepperX.setAcceleration(
            motion.maxAccel
        );

        stepperY.setAcceleration(
            motion.maxAccel
        );

        motionChanged =
            false;
    }

    // Apply new targets

    if (
        targetChanged
    )
    {
        stepperX.moveTo(
            mmToSteps(
                motion.targetX
            )
        );

        stepperY.moveTo(
            mmToSteps(
                motion.targetY
            )
        );

        targetChanged =
            false;
    }

    // Execute motion

    stepperX.run();

    stepperY.run();

    // Estimate state

    motion.posX =
        stepperX.currentPosition()
        /
        STEPS_PER_MM;

    motion.posY =
        stepperY.currentPosition()
        /
        STEPS_PER_MM;

    motion.velX =
        stepperX.speed()
        /
        STEPS_PER_MM;

    motion.velY =
        stepperY.speed()
        /
        STEPS_PER_MM;

    unsigned long now =
        millis();

    float dt =
        (
            now
            -
            lastUpdate
        )
        /
        1000.0;

    if (
        dt > 0
    )
    {
        motion.accelX =
            (
                motion.velX
                -
                previousVelX
            )
            /
            dt;

        motion.accelY =
            (
                motion.velY
                -
                previousVelY
            )
            /
            dt;
    }

    previousVelX =
        motion.velX;

    previousVelY =
        motion.velY;

    lastUpdate =
        now;

    motion.errorX =
        motion.targetX
        -
        motion.posX;

    motion.errorY =
        motion.targetY
        -
        motion.posY;
}

//==================================================
// Homing and position calibration
//==================================================

void homeAxis(
    AccelStepper& stepper,
    int switchPin
)
{
    stepper.setMaxSpeed(
        1000
    );

    stepper.setSpeed(
        -400
    );

    unsigned long start =
        millis();
    
    while (
        digitalRead(
            switchPin
        )
        ==
        HIGH
    )
    {
    stepper.runSpeed();

    if (
        millis()
        -
        start
        >
        10000
    )
    {
        break;
    }
}

    stepper.stop();

    stepper.setCurrentPosition(
        0
    );
}

void home()
{
    homeAxis(
        stepperX,
        X_HOME
    );

    homeAxis(
        stepperY,
        Y_HOME
    );

    setTarget(
        0,
        0
    );

    stepperX.setMaxSpeed(
        motion.maxSpeed
    );
    
    stepperY.setMaxSpeed(
        motion.maxSpeed
    );
    
    stepperX.setAcceleration(
        motion.maxAccel
    );
    
    stepperY.setAcceleration(
        motion.maxAccel
    );

    motion.posX = 0;
    motion.posY = 0;
    
    motion.targetX = 0;
    motion.targetY = 0;
}

void calibratePosition(
    float x,
    float y
)
{
    stepperX.setCurrentPosition(
        mmToSteps(
            x
        )
    );

    stepperY.setCurrentPosition(
        mmToSteps(
            y
        )
    );

    motion.posX = x;
    motion.posY = y;

    motion.targetX = x;
    motion.targetY = y;

    targetChanged =
      true;

    
}

//==================================================
// Sending feedback
//==================================================

unsigned long lastTelemetry = 0;

const int TELEMETRY_MS = 50;

void sendState()
{
    unsigned long now =
        millis();

    if (
        now
        -
        lastTelemetry
        <
        TELEMETRY_MS
    )
    {
        return;
    }

    lastTelemetry =
        now;

    Serial.print("S ");

    Serial.print(
        motion.posX,
        1
    );

    Serial.print(" ");

    Serial.print(
        motion.posY,
        1
    );

    Serial.print(" ");

    Serial.print(
        motion.velX,
        1
    );

    Serial.print(" ");

    Serial.print(
        motion.velY,
        1
    );

    Serial.print(" ");

    Serial.print(
        motion.errorX,
        1
    );
    
    Serial.print(" ");
    
    Serial.print(
        motion.errorY,
        1
    );
    
    Serial.print(" ");
    
    Serial.println(
        (
            abs(
                motion.velX
            )
            +
            abs(
                motion.velY
            )
        )
        >
        1
    );
}

//==================================================
// Command parser
//==================================================

void parseCommand(char* msg)
{
    char cmd;

    if (
        sscanf(
            msg,
            "%c",
            &cmd
        )
        != 1
    )
    {
        return;
    }

    float a = 0;
    float b = 0;

    switch (cmd)
    {
        case 'P':

            if (
                sscanf(
                    msg,
                    "%c %f %f",
                    &cmd,
                    &a,
                    &b
                )
                == 3
            )
            {
                setTarget(
                    a,
                    b
                );
            }

            break;

        case 'M':

            if (
                sscanf(
                    msg,
                    "%c %f %f",
                    &cmd,
                    &a,
                    &b
                )
                == 3
            )
            {
                configureMotion(
                    a,
                    b
                );
            }

            break;

        case 'C':

            if (
                sscanf(
                    msg,
                    "%c %f %f",
                    &cmd,
                    &a,
                    &b
                )
                == 3
            )
            {
                calibratePosition(
                    a,
                    b
                );
            }

            break;

        case 'H':

            home();

            break;
    }
}

//==================================================
// Serial input
//==================================================

char buffer[32];

int idx = 0;

void readCommand()
{
    while (
        Serial.available()
    )
    {
        char c =
            Serial.read();

        if (
            c
            ==
            '\n'
        )
        {
            buffer[idx] =
                '\0';

            parseCommand(
                buffer
            );

            idx =
                0;
        }

        else if (
            idx < 31
        )
        {
            buffer[idx++] =
                c;
        }
        else
        {
            idx =
                0;
        }
    }
}

//==================================================
// Setup
//==================================================

void setup()
{
    pinMode(
        ENABLE_PIN,
        OUTPUT
    );
    
    digitalWrite(
        ENABLE_PIN,
        LOW
    );
  
    Serial.begin(
        115200
    );

    configureMotion(
        3000,
        1500
    );

    setTarget(
        0,
        0
    );

    lastUpdate =
        millis();

    pinMode(
        X_HOME,
        INPUT_PULLUP
    );
    
    pinMode(
        Y_HOME,
        INPUT_PULLUP
    );

    stepperX.setCurrentPosition(0);

    stepperY.setCurrentPosition(0);
}

//==================================================
// Main loop
//==================================================

void loop()
{
    readCommand();

    updateMotion();

    sendState();
}
