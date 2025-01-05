#ifndef PARAMS_H
#define PARAMS_H

#define PI 3.14159

// Wire Parameters
#define MAX_SPOOL_LENGTH 1000.0 //ft
#define MIN_DIST_FROM_MIDPOINT 0.5//in, Minimum distance between midpoint and incision

// LCD Parameters
#define SPLASH_SCREEN_LOAD_TIME 1000//ms

#define WIRE_LEVEL_MED  50//% left
#define WIRE_LEVEL_LOW   25//% left
#define WIRE_INCREMENT   0.2//in, Increment amount of parameters

typedef enum {
    FULL_STEP = 0,
    HALF_STEP = 1,
    QUARTER_STEP = 2,
    EIGHTH_STEP = 3,
    SIXTEENTH_STEP = 7
} StepperResolution;

typedef enum {
    STEPPER_FWD = 1,
    STEPPER_REV = 0
} StepperDirection;

#define FEEDER_MOTOR_SPEED 10000.0
#define FEEDER_MOTOR_STEPS EIGHTH_STEP
#define FEEDER_WHEEL_DIAMETER 0.5 // inches
#define FEEDER_WHEEL_CIRCUMFERENCE PI*FEEDER_WHEEL_DIAMETER // Inches per revolution
#define FEEDER_RESOLUTION 200.0 // Steps per revolution
#define FEEDER_STEP_PER_INCH (FEEDER_RESOLUTION/FEEDER_WHEEL_CIRCUMFERENCE)

#define CUTTER_MOTOR_SPEED 1.0
#define CUTTER_INCISION_COUNTS 3
#define CUTTER_CUT_COUNTS 3
#define CUTTER_TIME 1.0 //seconds

#define POS_STRIP 155
#define POS_CUT 142

typedef enum {
    ONE_PRESSED     =0x11,
    ONE_RELEASED    =0x10,
    TWO_PRESSED     =0x21,
    TWO_RELEASED    =0x20,
    THREE_PRESSED   =0x31,
    THREE_RELEASED  =0x30,
    FOUR_PRESSED    =0x41,
    FOUR_RELEASED   =0x40,
    UP_PRESSED      =0x51,
    UP_RELEASED     =0x50,
    DOWN_PRESSED    =0x61,
    DOWN_RELEASED   =0x60,
    LEFT_PRESSED    =0x71,
    LEFT_RELEASED   =0x70,
    RIGHT_PRESSED   =0x81,
    RIGHT_RELEASED  =0x80,
    INVALID         =0xFF
} ButtonState;

typedef enum {
    MENU,
    CUTTING_ONE,
    CUTTING_TWO,
    SETTINGS_ONE,
    SETTINGS_RESET,
    SETTINGS_FEED,
    SETTINGS_CUTTER,
    SETTINGS_GUIDE
} State;

typedef enum {
    IDLE,
    GOT_EXCLAM,
    GOT_B,
    GOT_BTN,
    GOT_HIT
} BleState;

#endif