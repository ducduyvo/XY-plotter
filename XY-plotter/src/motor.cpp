#include "motor.h"

Motor::Motor(
        DigitalIoPin* stepper,
        DigitalIoPin* direction,
        DigitalIoPin* minLimit,
        DigitalIoPin* maxLimit,
        bool originDirection
    ) :
        stepper(stepper),
        direction(direction),
        minLimit (minLimit),
        maxLimit(maxLimit),
        originDirection(originDirection)
{ }
