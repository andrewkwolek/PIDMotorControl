#include "utilities.h"

static volatile Mode mode = IDLE;

void setMode(Mode m) {
    mode = m;
}

Mode getMode(void) {
    return mode;
}