#ifndef MBED_STEPPER_H
#define MBED_STEPPER_H
#endif

#include "mbed.h"

class Stepper
{
public:
    Stepper(PinName _en, PinName ms1, PinName ms2, PinName ms3, PinName _stepPin, PinName dir);
    void step(int microstep, int dir, float speed);
    void enable();
    void disable();
private:
    DigitalOut en;
    BusOut microstepping;
    DigitalOut stepPin;
    DigitalOut direction;
};