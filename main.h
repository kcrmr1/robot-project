#ifndef MAIN_H
#define MAIN_H

// standard namespace
using namespace std;

// light color values
enum Color {
    LRED,
    LBLUE,
    EMPTY
};

// constants
constexpr double IGWAN_MAX_VOLTAGE = 9; // V
constexpr double FITEC_MAX_VOLTAGE = 6; // V
constexpr double WHEEL_DIAMETER = 3; // inches
constexpr double BETWEEN_WHEELS = 8.4; // inches
constexpr double TIME_PER_CORR = .1; // seconds
constexpr int COUNTS_PER_REV = 318;
constexpr int LEFT_MOTOR_FORWARD = -1;
constexpr int RIGHT_MOTOR_FORWARD = 1;
constexpr int SWITCH_ACTIVE = 0;
constexpr int SWITCH_INACTIVE = 1;
constexpr int MAX_DIFF = 4;

constexpr int X_MAX = 319;
constexpr int Y_MAX = 239;

#endif
