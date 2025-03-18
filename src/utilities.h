#ifndef UTILITIES__H__
#define UTILITIES__H__

#include "nu32dip.h"

typedef enum {
    IDLE,  // IDLE = 0
    PWM,   // PWM = 1
    ITEST, // ITEST = 2
    HOLD,  // HOLD = 3
    TRACK  // TRACK = 4
} Mode;

void setMode(Mode m);
Mode getMode(void);

#endif // UTILITIES__H__