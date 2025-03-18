#include "nu32dip.h"

enum Mode {
    IDLE,  // IDLE = 0
    PWM,   // PWM = 1
    ITEST,  // ITEST = 2
    HOLD,   // HOLD = 3
    TRACK    // WEST = 4
};

void setMode(enum Mode m);

enum Mode getMode(void);