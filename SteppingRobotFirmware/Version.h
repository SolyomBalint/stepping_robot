#ifndef VERSION_H
#define VERSION_H

//--------------------------------------------------
// Firmware identity
//--------------------------------------------------

constexpr char FIRMWARE_NAME[] =
    "SteppingRobotFirmware";

constexpr char FIRMWARE_AUTHOR[] =
    "Balint Solyom";

constexpr char FIRMWARE_BOARD[] =
    "MEGA2560 PRO";

constexpr char FIRMWARE_TARGET[] =
    "2-Axis Interactive Motion Controller";

//--------------------------------------------------
// Semantic version
//--------------------------------------------------

constexpr uint8_t VERSION_MAJOR = 2;
constexpr uint8_t VERSION_MINOR = 0;
constexpr uint8_t VERSION_PATCH = 0;

//--------------------------------------------------
// Protocol version
//--------------------------------------------------

constexpr uint8_t PROTOCOL_VERSION = 1;

//--------------------------------------------------
// Build information
//--------------------------------------------------

constexpr char BUILD_DATE[] = __DATE__;

constexpr char BUILD_TIME[] = __TIME__;

//--------------------------------------------------
// URL
//--------------------------------------------------

constexpr char PROJECT_URL[] =
    "github.com/SolyomBalint/stepping_robot";

#endif