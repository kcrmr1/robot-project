// header files
#include <Arduino.h>
#include <FEHIO.h>
#include <FEHLCD.h>
#include <FEHMotor.h>
#include <FEHServo.h>
#include <FEHUtility.h>
#include <FEHRCS.h>

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
constexpr double TIME_PER_CORR = .2; // seconds
constexpr int COUNTS_PER_REV = 318;
constexpr int LEFT_MOTOR_FORWARD = -1;
constexpr int RIGHT_MOTOR_FORWARD = 1;
constexpr int SWITCH_ACTIVE = 0;
constexpr int SWITCH_INACTIVE = 1;
constexpr int MAX_DIFF = 4;
constexpr int LEFT_RIGHT_DIFF = (MAX_DIFF/2) * LEFT_MOTOR_FORWARD;

constexpr int X_MAX = 319;
constexpr int Y_MAX = 239;

// define motor and io pins
FEHMotor leftMotor(FEHMotor::Motor2, IGWAN_MAX_VOLTAGE);
FEHMotor rightMotor(FEHMotor::Motor0, IGWAN_MAX_VOLTAGE);
FEHServo horizontalServo(FEHServo::Servo0);
FEHServo verticalServo(FEHServo::Servo1);
DigitalEncoder leftEncoder(FEHIO::Pin10);
DigitalEncoder rightEncoder(FEHIO::Pin8);
DigitalInputPin frontLeftSwitch(FEHIO::Pin0);
DigitalInputPin frontRightSwitch(FEHIO::Pin1);
DigitalInputPin backLeftSwitch(FEHIO::Pin2);
DigitalInputPin backRightSwitch(FEHIO::Pin3);
AnalogInputPin cdsCell(FEHIO::Pin0);

// classes for organization

class Drive {
    // priv constructor b/c class is static
    Drive();
    // declare any important consts below
    static constexpr double rampLength = 12; // inches
    static constexpr float timeBuffer = .2; // s
    static const int8_t percentBeforeJump = 45;

    public:
    /** 
     * Drive forward a specific distance in inches. **DEPRECIATED**
     * 
     * @param d
     *      Distance in inches.
     * @param percent
     *      Motor percent.
     * 
     * @return Number of counts traveled.
    */
    double static LinearForward(double d, int8_t percent);
    /** 
     * Drive forward a specific distance in inches. Automatically correct difference in motors.
     * 
     * @param d
     *      Distance in inches.
     * @param percent
     *      Motor percent.
     * 
     * @return Number of counts traveled.
    */
    double static Forward(double d, int8_t percent);
        /** 
     * Drive forward a specific amount of time. Automatically correct difference in motors.
     * 
     * @param t
     *      Time in seconds.
     * @param percent
     *      Motor percent.
     * 
     * @return Number of counts traveled.
    */
    double static TimedForward(double t, int8_t percent);
    /**
     * Drive forward until switches indicate stop.
     * 
     * @param includeBoth
     *      True if both switches need activated, false otherwise.
     * @param percent
     *      Motor percent.
     */
    void static ToObj(bool includeBoth, int8_t percent);
    /**
     * Drive foward until specific distance reached or switches indicate stop.
     * 
     * @param d
     *      Distance in inches.
     * @param includeBoth
     *      True if both switches need activated, false otherwise.
     * @param percent
     *      Motor percent.
     * 
     * @return Number of counts traveled.
     */
    double static ForwardOrToObj(double d, bool includeBoth, int8_t percent);
    /**
     * Reverse a specific distance in inches. 
     * 
     * @param d
     *      Distance to travel in inches.
     * @param percent
     *      Motor percent.
     */
    double static Reverse(double d, int8_t percent);
    /**
     * Reverse a specific distance in inches or until switches indicate stop.
     * 
     * @param d
     *      Distance to travel in inches.
     * @param includeBoth
     *      True if both switches need activated, false otherwise.
     * @param percent
     *      Motor percent.
     */
    double static ReverseOrToObj(double d, bool includeBoth, int8_t percent);
};
class Maneuver {
    // priv constructor b/c class is static
    Maneuver();
    // declare any important consts below
    static constexpr float timeBuffer = .5;
    static const int8_t defaultPercent = 15;

    public:
    /**
     * Turn right or left in place for a specific degree value.
     * 
     * @param direction
     *      'R' or 'L'.
     * @param degrees
     *      Value in degrees for turn angle.
     * @param percent
     *      Motor percent.
     * 
     * @return Number of counts traveled.
     */
    double static Turn(char direction, int degrees, int8_t percent);
    /** 
     * Turn right or left about the opposite, stationary wheel for a specific degree value.
     * 
     * @param direction
     *      'R' or 'L'.
     * @param degrees
     *      Value in degrees for turn angle
     * @param percent
     *      Motor percent
     * 
     * @return Number of counts traveled.
    */
    int static TurnOneWheel(char direction, int degrees, int8_t percent);
    /**
     * Flatten the back side against a wall with the back switches.
     * 
     * @param direction
     *      'R' or 'L'.
     * @param percent
     *      Motor percent.
     */
    void static FlattenAgainstWall(char direction, int8_t percent);
};
class Forklift {
    // static
    Forklift();

    constexpr static int offDegrees = 88;
    constexpr static int leftDegrees = 83;
    constexpr static int rightDegrees = 94;

    constexpr static double dbTime = .25;
public:
    /**
     * Move forklift horizontally for a certain amount of seconds.
     * 
     * @param d
     *      direction of movement ('L' or 'R')
     * @param s
     *      time in seconds to move
     */
    static void moveHorizontal(char d, double s);
};
class LightInput {
    // static
    LightInput();
    // consts
    static constexpr float redMax = .9, blueMax = 1.8, emptyMax = 5; // V
    static constexpr float waitTime = 3., readBuffer = .2; // seconds

public:
    /**
     * Get the current color reading.
     * 
     * @param wait
     *      True if wait for value other than EMPTY, false otherwise.
     * 
     * @return color reading
     */
    Color static GetColorReading(bool wait);
    /**
     * Wait for the start light input of LRED.
     */
    void static WaitForStart();
};

/**
 * Test for color values.
 */
void colorTest() {
        while (true) {
        String output;
        LCD.Clear();
        Color reading = LightInput::GetColorReading(false);
        switch (reading) {
        case LRED:
            output = "RED";
            break;
        case LBLUE:
            output = "BLUE";
            break;
        case EMPTY:
            output = "EMPTY";
            break;
        default:
            output = "UNHANDLED EXCEPTION";
            break;
        }
        LCD.WriteLine(output.c_str());
        Sleep(.25);
    }
}

void openWindow(double d, int8_t percent) {
    // find wanted count value, reset encoders, and drive
    double wantedCounts = (d / (PI*WHEEL_DIAMETER)) * COUNTS_PER_REV;
    leftEncoder.ResetCounts();
    rightEncoder.ResetCounts();
    int leftPercent = LEFT_MOTOR_FORWARD * percent + LEFT_RIGHT_DIFF;
    int rightPercent = RIGHT_MOTOR_FORWARD * percent;
    leftMotor.SetPercent(leftPercent + 2);
    rightMotor.SetPercent(rightPercent);

    // while avg counts of both encoders less than wanted, wait
    while (leftEncoder.Counts() < wantedCounts) {}

    // stop motors
    //LCD.WriteLine(leftEncoder.Counts());
    //LCD.WriteLine(rightEncoder.Counts());
    leftMotor.Stop();
    rightMotor.Stop();
    Sleep(.25);
}

/**
 * Main function code for milestone one.
 */
void milestoneOne() {
    // x y vars
    int x, y;

    // draw screen
    LCD.DrawVerticalLine(.5*X_MAX, 0, Y_MAX);
    LCD.WriteAt("Straight", .1*X_MAX, .5*Y_MAX);
    LCD.WriteAt("Up Ramp", .6*X_MAX, .5*Y_MAX);

    // wait for touch & release
    while (!LCD.Touch(&x, &y)) {}
    while (LCD.Touch(&x, &y)) {}
    
    // countdown
    for (int i = 3; i > 0; i--) {
        LCD.Clear();
        LCD.WriteAt(i, .45*X_MAX, .45*Y_MAX);
        Sleep(1.);
    }

    // intermittent text
    LCD.Clear();
    LCD.WriteLine("Doing task...");

    // read button press
    if (x <= .5*X_MAX) {
        Drive::Forward(30, 30);
    } else {
        Drive::Forward(36, 35);
        Maneuver::TurnOneWheel('L', 45, 20);
        Maneuver::Turn('L', 135, 20);
        Drive::Forward(30, 35);
    }

    // complete
    LCD.Clear();
    LCD.WriteLine("Complete!");
}

/**
 * Main function for milestone two.
 */
void milestoneTwo() {
// wait for start light and press button
    LightInput::WaitForStart();
    Drive::Forward(2.4, 25); 
    Drive::Forward(2.4, -25);

    // turn and move to make space to go up ramp
    Maneuver::Turn('L', 90, 25);
    Drive::Forward(4.75, 25);
    Maneuver::TurnOneWheel('L', 48, 25);

    // go up ramp
    Drive::Forward(30.75, 35);

    // drive to cds cell
    Maneuver::Turn('L', 88, 25);
    Drive::Forward(12.4, 25);

    // get color reading
    Color humidifierButton = LightInput::GetColorReading(false);
    int buffer;

    // press correct button
    if (humidifierButton == LBLUE) {
        Maneuver::Turn('L', 90, 25);
        Drive::Forward(2, 25);
        Maneuver::Turn('R', 90, 25);
        buffer = -1;
    } else if (humidifierButton == LRED) {
        Maneuver::Turn('R', 90, 25);
        Drive::Forward(2, 25);
        Maneuver::Turn('L', 90, 25);
        buffer = 1;
    } else {
        Maneuver::Turn('L', 90, 25);
        Drive::Forward(2, 25);
        Maneuver::Turn('R', 90, 25);
        buffer = -1;
    }
    Drive::Forward(6.75, 25);

    // return to button
    Drive::Forward(2, -25);
    Maneuver::Turn('R', 180, 25);
    Drive::Forward(4.75 + 12.4, 25);
    Maneuver::Turn('R', 90, 25);
    Drive::Forward(30.75 + (2 * buffer), 30);

    // press button
    Drive::Forward(5, 25);
}

/**
 * Main function for milestone three.
 */
void milestoneThree() {
    // start light
    LightInput::WaitForStart();

    // press button and space from it
    Drive::TimedForward(.75, 20);
    Drive::Reverse(2.4, 25);
    
    // turn and move to make space to go up ramp
    Maneuver::Turn('L', 90, 25);
    Drive::Forward(4.75, 25);
    Maneuver::TurnOneWheel('L', 45, 25);

    // go up ramp
    Drive::Forward(25.4, 35);

    // turn and open window
    Maneuver::Turn('L', 88, 25);
    Drive::Forward(13.5, 35);

    // wrap around window handle
    Drive::Reverse(3, 20);
    Maneuver::TurnOneWheel('R', 45, 30);
    Drive::Forward(6, 20);
    Maneuver::TurnOneWheel('L', 90, 30);
    Drive::Forward(6, 20);
    Maneuver::TurnOneWheel('R', 45, 30);

    // close window
    Drive::Reverse(10, 35);
}

/**
 * Main function for milestone four.
 */
void milestoneFour() {
    
}

// main
void ERCMain() {
    // battery
    LCD.WriteLine(BatteryVoltage());

    // do stuff
    TestGUI();
}

// class function definitions

double Drive::LinearForward(double d, int8_t percent) {
    // find wanted count value, reset encoders, and drive
    double wantedCounts = (d / (PI*WHEEL_DIAMETER)) * COUNTS_PER_REV;
    leftEncoder.ResetCounts();
    rightEncoder.ResetCounts();
    leftMotor.SetPercent(LEFT_MOTOR_FORWARD * percent);
    rightMotor.SetPercent(RIGHT_MOTOR_FORWARD * percent);

    // while avg counts of both encoders less than wanted, wait
    while (((leftEncoder.Counts() + rightEncoder.Counts()) / 2) < wantedCounts) {}

    // stop motors
    leftMotor.Stop();
    rightMotor.Stop();
    LCD.WriteLine(leftEncoder.Counts());
    LCD.WriteLine(rightEncoder.Counts());
    Sleep(timeBuffer);

    // return traveled counts
    return ((leftEncoder.Counts() + rightEncoder.Counts()) / 2);
} 
double Drive::Forward(double d, int8_t percent) {
    // find wanted count value, reset encoders, and drive
    double wantedCounts = (d / (PI*WHEEL_DIAMETER)) * COUNTS_PER_REV;
    leftEncoder.ResetCounts();
    rightEncoder.ResetCounts();
    int leftPercent = LEFT_MOTOR_FORWARD * percent + LEFT_RIGHT_DIFF;
    int rightPercent = RIGHT_MOTOR_FORWARD * percent;
    leftMotor.SetPercent(leftPercent);
    rightMotor.SetPercent(rightPercent);

    float timeSinceCorrection = TimeNow();

    // while avg counts of both encoders less than wanted, wait
    while (((leftEncoder.Counts() + rightEncoder.Counts()) / 2) < wantedCounts) {
        // if time per correction has passed, adjust percentages
        if (TimeNow() - timeSinceCorrection > TIME_PER_CORR) {
            int diff = leftPercent*LEFT_MOTOR_FORWARD-rightPercent*RIGHT_MOTOR_FORWARD;
            if (leftEncoder.Counts() > rightEncoder.Counts() && diff > 0) {
                leftPercent = leftPercent - (1 * LEFT_MOTOR_FORWARD);
                leftMotor.SetPercent(leftPercent);
                timeSinceCorrection = TimeNow();
            } else if (leftEncoder.Counts() < rightEncoder.Counts() && diff < MAX_DIFF) {
                leftPercent = leftPercent + (1 * LEFT_MOTOR_FORWARD);
                leftMotor.SetPercent(leftPercent);
                timeSinceCorrection = TimeNow();
            }
        }
    }

    // stop motors
    //LCD.WriteLine(leftEncoder.Counts());
    //LCD.WriteLine(rightEncoder.Counts());
    leftMotor.Stop();
    rightMotor.Stop();
    Sleep(timeBuffer);

    // return traveled counts
    return ((leftEncoder.Counts() + rightEncoder.Counts()) / 2);
}
double Drive::TimedForward(double t, int8_t percent) {
    // find wanted count value, reset encoders, and drive
    int leftPercent = LEFT_MOTOR_FORWARD * percent + LEFT_RIGHT_DIFF;
    int rightPercent = RIGHT_MOTOR_FORWARD * percent;
    leftMotor.SetPercent(leftPercent);
    rightMotor.SetPercent(rightPercent);

    float timeSinceCorrection = TimeNow();
    float timeStart = TimeNow();

    // while avg counts of both encoders less than wanted, wait
    while (TimeNow() - timeStart < t) {
        // if time per correction has passed, adjust percentages
        if (TimeNow() - timeSinceCorrection > TIME_PER_CORR) {
            int diff = leftPercent*LEFT_MOTOR_FORWARD-rightPercent*RIGHT_MOTOR_FORWARD;
            if (leftEncoder.Counts() > rightEncoder.Counts() && diff > 0) {
                leftPercent = leftPercent - (1 * LEFT_MOTOR_FORWARD);
                leftMotor.SetPercent(leftPercent);
                timeSinceCorrection = TimeNow();
            } else if (leftEncoder.Counts() < rightEncoder.Counts() && diff < MAX_DIFF) {
                leftPercent = leftPercent + (1 * LEFT_MOTOR_FORWARD);
                leftMotor.SetPercent(leftPercent);
                timeSinceCorrection = TimeNow();
            }
        }
    }

    // stop motors
    //LCD.WriteLine(leftEncoder.Counts());
    //LCD.WriteLine(rightEncoder.Counts());
    leftMotor.Stop();
    rightMotor.Stop();
    Sleep(timeBuffer);

    // return traveled counts
    return ((leftEncoder.Counts() + rightEncoder.Counts()) / 2);
}
void Drive::ToObj(bool includeBoth, int8_t percent) {
    // drive
    leftMotor.SetPercent(LEFT_MOTOR_FORWARD * percent + LEFT_RIGHT_DIFF);
    rightMotor.SetPercent(RIGHT_MOTOR_FORWARD * percent);

    if (includeBoth) {
        // wait until both pressed
        while (!(frontLeftSwitch.Value() || frontRightSwitch.Value())) {}
    } else {
        // wait until a single one is pressed
        while (frontLeftSwitch.Value() && frontRightSwitch.Value()) {}
    }

    // stop
    leftMotor.Stop();
    rightMotor.Stop();
    Sleep(timeBuffer);
}
double Drive::ForwardOrToObj(double d, bool includeBoth, int8_t percent) {
    // find wanted count value, reset encoders, and drive
    double wantedCounts = (d / (PI * WHEEL_DIAMETER)) * COUNTS_PER_REV;
    leftEncoder.ResetCounts();
    rightEncoder.ResetCounts();
    leftMotor.SetPercent(LEFT_MOTOR_FORWARD * percent + LEFT_RIGHT_DIFF);
    rightMotor.SetPercent(RIGHT_MOTOR_FORWARD * percent);

    // wait conditions
    if (includeBoth) {
        while (!(frontLeftSwitch.Value() || frontRightSwitch.Value())
         && (((leftEncoder.Counts() + rightEncoder.Counts()) / 2) < wantedCounts)) {}
    } else {
        while ((frontLeftSwitch.Value() && frontRightSwitch.Value())
         && (((leftEncoder.Counts() + rightEncoder.Counts()) / 2) < wantedCounts)) {}
    }

    // stop motors & wait
    leftMotor.Stop();
    rightMotor.Stop();
    Sleep(timeBuffer);

    // return traveled counts
    return ((leftEncoder.Counts() + rightEncoder.Counts()) / 2);
}
double Drive::Reverse(double d, int8_t percent) {
    // find wanted count value, reset encoders, and drive
    double wantedCounts = (d / (PI*WHEEL_DIAMETER)) * COUNTS_PER_REV;
    leftEncoder.ResetCounts();
    rightEncoder.ResetCounts();
    int leftPercent = -1 * LEFT_MOTOR_FORWARD * percent + -1 * LEFT_RIGHT_DIFF;
    int rightPercent = -1 * RIGHT_MOTOR_FORWARD * percent;
    leftMotor.SetPercent(leftPercent);
    rightMotor.SetPercent(rightPercent);

    float timeSinceCorrection = TimeNow();

    // while avg counts of both encoders less than wanted, wait
    while (((leftEncoder.Counts() + rightEncoder.Counts()) / 2) < wantedCounts) {
        // if time per correction has passed, adjust percentages
        if (TimeNow() - timeSinceCorrection > TIME_PER_CORR) {
            int diff = leftPercent*LEFT_MOTOR_FORWARD-rightPercent*RIGHT_MOTOR_FORWARD;
            if (leftEncoder.Counts() > rightEncoder.Counts() && diff > 0) {
                leftPercent = leftPercent - (-1 * LEFT_MOTOR_FORWARD);
                leftMotor.SetPercent(leftPercent);
                timeSinceCorrection = TimeNow();
            } else if (leftEncoder.Counts() < rightEncoder.Counts() && diff < MAX_DIFF) {
                leftPercent = leftPercent + (-1 * LEFT_MOTOR_FORWARD);
                leftMotor.SetPercent(leftPercent);
                timeSinceCorrection = TimeNow();
            }
        }
    }

    // stop motors
    //LCD.WriteLine(leftEncoder.Counts());
    //LCD.WriteLine(rightEncoder.Counts());
    leftMotor.Stop();
    rightMotor.Stop();
    Sleep(timeBuffer);

    // return traveled counts
    return ((leftEncoder.Counts() + rightEncoder.Counts()) / 2);
}
double Drive::ReverseOrToObj(double d, bool includeBoth, int8_t percent) {
    // find wanted count value, reset encoders, and drive
    double wantedCounts = (d / (PI*WHEEL_DIAMETER)) * COUNTS_PER_REV;
    leftEncoder.ResetCounts();
    rightEncoder.ResetCounts();
    int leftPercent = -1 * LEFT_MOTOR_FORWARD * percent + -1 * LEFT_RIGHT_DIFF;
    int rightPercent = -1 * RIGHT_MOTOR_FORWARD * percent;
    leftMotor.SetPercent(leftPercent);
    rightMotor.SetPercent(rightPercent);

    float timeSinceCorrection = TimeNow();
    bool switchStop = false;

    // while avg counts of both encoders less than wanted, wait
    while (((leftEncoder.Counts() + rightEncoder.Counts()) / 2) < wantedCounts && !switchStop) {
        // if time per correction has passed, adjust percentages
        if (TimeNow() - timeSinceCorrection > TIME_PER_CORR) {
            int diff = leftPercent*LEFT_MOTOR_FORWARD-rightPercent*RIGHT_MOTOR_FORWARD;
            if (leftEncoder.Counts() > rightEncoder.Counts() && diff > 0) {
                leftPercent = leftPercent - (-1 * LEFT_MOTOR_FORWARD);
                leftMotor.SetPercent(leftPercent);
                timeSinceCorrection = TimeNow();
            } else if (leftEncoder.Counts() < rightEncoder.Counts() && diff < MAX_DIFF) {
                leftPercent = leftPercent + (-1 * LEFT_MOTOR_FORWARD);
                leftMotor.SetPercent(leftPercent);
                timeSinceCorrection = TimeNow();
            }
        }

        if (includeBoth) {
            switchStop = (backLeftSwitch.Value() == SWITCH_ACTIVE 
                && backRightSwitch.Value() == SWITCH_ACTIVE);
        } else {
            switchStop = (backLeftSwitch.Value() == SWITCH_ACTIVE
                || backRightSwitch.Value() == SWITCH_ACTIVE);
        }
    }

    // stop motors
    leftMotor.Stop();
    rightMotor.Stop();
    Sleep(timeBuffer);

    // return traveled counts
    return ((leftEncoder.Counts() + rightEncoder.Counts()) / 2);
}
double Maneuver::Turn(char direction, int degrees, int8_t percent) {
    /* 
     * fraction of the circumference to travel 
     * times the circle made around the wheel axle
    */
    double distance = (degrees / 360.) * (PI * BETWEEN_WHEELS);
    double wantedCounts = (distance / (PI * WHEEL_DIAMETER)) * COUNTS_PER_REV;

    // set percent values
    int8_t leftPercent = LEFT_MOTOR_FORWARD * percent + LEFT_RIGHT_DIFF;
    int8_t rightPercent = RIGHT_MOTOR_FORWARD * percent;
    if (direction == 'r' || direction == 'R') {
        rightPercent *= -1;
    } else if (direction == 'l' || direction == 'L') {
        leftPercent *= -1;
    }

    // reset encoder counts and start motors
    leftEncoder.ResetCounts();
    rightEncoder.ResetCounts();
    leftMotor.SetPercent(leftPercent);
    rightMotor.SetPercent(rightPercent);

    float timeSinceCorrection = TimeNow();

    // wait until distance traveled
    while (((leftEncoder.Counts() + rightEncoder.Counts()) / 2) < wantedCounts) {
        // if time per correction has passed, adjust percentages
        if (TimeNow() - timeSinceCorrection > TIME_PER_CORR) {
            int diff = leftPercent*LEFT_MOTOR_FORWARD-rightPercent*RIGHT_MOTOR_FORWARD;
            if (leftEncoder.Counts() > rightEncoder.Counts() && diff > 0) {
                leftPercent = leftPercent - (1 * LEFT_MOTOR_FORWARD);
                leftMotor.SetPercent(leftPercent);
                timeSinceCorrection = TimeNow();
            } else if (leftEncoder.Counts() < rightEncoder.Counts() && diff < MAX_DIFF) {
                leftPercent = leftPercent + (1 * LEFT_MOTOR_FORWARD);
                leftMotor.SetPercent(leftPercent);
                timeSinceCorrection = TimeNow();
            }
        }
    }

    // stop motors & wait
    leftMotor.Stop();
    rightMotor.Stop();
    Sleep(timeBuffer);

    // return traveled counts
    return ((leftEncoder.Counts() + rightEncoder.Counts()) / 2);
} 
int Maneuver::TurnOneWheel(char direction, int degrees, int8_t percent) {
    /* 
     * fraction of the circumference to travel 
     * times the circle made around the wheel axle
    */
    double distance = (degrees / 360.) * (PI * 2 * BETWEEN_WHEELS);
    double wantedCounts = (distance / (PI * WHEEL_DIAMETER)) * COUNTS_PER_REV;

    // reset both counts and start motor
    leftEncoder.ResetCounts();
    rightEncoder.ResetCounts();
    if (direction == 'r' || direction == 'R') {
        leftMotor.SetPercent(percent*LEFT_MOTOR_FORWARD + LEFT_RIGHT_DIFF);
    } else if (direction == 'l' || direction == 'L') {
        rightMotor.SetPercent(percent*RIGHT_MOTOR_FORWARD);
    }

    // while neither has traveled the wanted counts, wait
    while (leftEncoder.Counts() + rightEncoder.Counts() < wantedCounts) {}

    // stop both & sleep
    leftMotor.Stop();
    rightMotor.Stop();
    Sleep(timeBuffer);

    return (leftEncoder.Counts() + rightEncoder.Counts());
}
void Maneuver::FlattenAgainstWall(char direction, int8_t percent) {
    // set percent values
    int8_t firstPercent;
    int8_t secondPercent;

    // set order
    FEHMotor* first;
    FEHMotor* second;
    DigitalInputPin* firstSwitch;
    DigitalInputPin* secondSwitch;
    if (direction == 'r' || direction == 'R') {
        first = &rightMotor;
        firstPercent = -1 * RIGHT_MOTOR_FORWARD * percent;
        firstSwitch = &backRightSwitch;

        second = &leftMotor;
        secondPercent = -1 * LEFT_MOTOR_FORWARD * percent + LEFT_RIGHT_DIFF;
        secondSwitch = &backLeftSwitch;
    } else if (direction == 'l' || direction == 'L') {
        first = &leftMotor;
        firstPercent = -1 * LEFT_MOTOR_FORWARD * percent + LEFT_RIGHT_DIFF;
        firstSwitch = &backLeftSwitch;

        second = &rightMotor;
        secondPercent = -1 * RIGHT_MOTOR_FORWARD * percent;
        secondSwitch = &backRightSwitch;
    }

    // drive
    (*first).SetPercent(firstPercent);
    while ((*firstSwitch).Value() == SWITCH_INACTIVE) {}
    (*first).Stop();
    Sleep(timeBuffer);
    (*second).SetPercent(secondPercent);
    while ((*secondSwitch).Value() == SWITCH_INACTIVE) {}
    (*second).Stop();
    Sleep(timeBuffer);
}
void Forklift::moveHorizontal(char d, double s) {
    // find direction of movement, none if d input invalid
    int degrees = offDegrees;
    if (d == 'l' || d == 'L') {
        degrees = leftDegrees;
    } else if (d == 'r' || d == 'R') {
        degrees = rightDegrees;
    }

    // set to move and wait
    horizontalServo.SetDegree(degrees);
    float startTime = TimeNow();
    while (TimeNow() - startTime < s) {}
    // turn off
    horizontalServo.SetDegree(offDegrees);

    // wait db time
    Sleep(dbTime);
}
Color LightInput::GetColorReading(bool wait) {
    float value = cdsCell.Value();
    LCD.WriteLine(value);
    Color reading;
    if (wait) {
        // wait for value other than empty
        float startTime = TimeNow(); // seconds
        while (value > blueMax && value <= emptyMax && (TimeNow() - startTime) < waitTime) {
            Sleep(readBuffer);
            value = cdsCell.Value();
        }
        if (value > 0 && value <= redMax) {
            reading = LRED;
        } else if (value > redMax && value <= blueMax) {
            reading = LBLUE;
        } else if ((TimeNow() - startTime) < waitTime) {
            LCD.WriteLine("Max time exceeded for color reading!");
        } else {
            LCD.WriteLine("Unhandled color value exception!");
        }
    } else {
        // return whatever value matches
        if (value >= .2 && value <= .8) {
            reading = LRED;
        } else if (value >= 1.1 && value <= 1.7) {
            reading = LBLUE;
        } else {
            reading = EMPTY;
        }
    }

    return reading;
}
void LightInput::WaitForStart() {
    while (GetColorReading(false) != LRED) {}
}
