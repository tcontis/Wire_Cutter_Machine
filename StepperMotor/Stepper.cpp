#include "Stepper.h"
#include "mbed.h"

Stepper::Stepper(PinName _en, PinName ms1, PinName ms2, PinName ms3, PinName _stepPin, PinName dir):en(_en),
    microstepping(ms1, ms2, ms3),
    stepPin(_stepPin),
    direction(dir)
{
}

void Stepper::step(int microstep, int dir, float speed)
{
    if (microstep == 1) {
        microstepping = 0;
    } else if (microstep <= 4) {
        microstepping = microstep / 2;
    } else if (microstep > 4) {
        microstepping = (microstep / 2) - 1;
    }
    if (dir == 1) {
        direction = 0;
    } else if (dir == 0) {
        direction = 1;
    }
    
    //  Step...
    stepPin = 1;
    wait(1/speed);
    stepPin = 0;
    wait(1/speed);
}

void Stepper::enable()
{
    en = 0;
}

void Stepper::disable()
{
    en = 1;
}