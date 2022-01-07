#ifndef MOTOR_H_
#define MOTOR_H_

#include "DigitalIoPin.h"

#define CLOCKWISE 0
#define COUNTER_CLOCKWISE 1

class Motor {
public:

    Motor(
        DigitalIoPin* stepper,
        DigitalIoPin* direction,
        DigitalIoPin* limOrigin,
        DigitalIoPin* limMax,
        bool originDirection
    );

    bool readStepper()     { return stepper->read(); }
    bool readDirection()   { return direction->read(); }
    bool readMinLimit()    { return originDirection == CLOCKWISE ? minLimit->read() : maxLimit->read(); }
    bool readMaxLimit()    { return originDirection == CLOCKWISE ? maxLimit->read() : minLimit->read(); }
    bool readOriginLimit() { return readMinLimit(); } // "Alias" for readMinLimit

    void writeStepper(bool step)  { stepper->write(step); }
    void writeDirection(bool dir) { direction->write(dir); }

    bool isOriginDirection() { return direction->read() ==  originDirection; }

    bool getOriginDirection() 	      { return originDirection; }
    void setOriginDirection(bool dir) { originDirection = dir; }
private:
    DigitalIoPin* stepper;
    DigitalIoPin* direction;
    DigitalIoPin* minLimit;
    DigitalIoPin* maxLimit;
    bool originDirection;
};
#endif /* MOTOR_H_ */
