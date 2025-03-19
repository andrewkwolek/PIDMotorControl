#ifndef CURRENTCONTROL__H__
#define CURRENTCONTROL__H__

#include <xc.h> // processor SFR definitions
#include <sys/attribs.h> // __ISR macro
#include "nu32dip.h"
#include "ina219.h"
#include "utilities.h"

// Define for Timer4 frequency
#define CURRENT_CONTROL_FREQ 5000 // 5kHz control frequency

void makeWaveform(void);

// Initialize the current controller with default values and set up Timer4 ISR
void CurrentControl_Init(void);

// Set PI gains for the current controller
void CurrentControl_SetGains(float kp, float ki);

// Get current PI gains
void CurrentControl_GetGains(float *kp, float *ki);

// Reset the integrator
void CurrentControl_ResetIntegrator(void);

// Test current control with a 100 Hz square wave reference
// Returns number of samples recorded
void CurrentControl_Test(void);

// PWM control
void PWMControl(int pwm_val);

// Current reference value to be used by current controller
float getCurrentReference(void);
void setCurrentReference(float ref);

#endif // CURRENTCONTROL__H__