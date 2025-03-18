#ifndef CURRENTCONTROL__H__
#define CURRENTCONTROL__H__

#include <xc.h> // processor SFR definitions
#include <sys/attribs.h> // __ISR macro
#include "nu32dip.h"
#include "ina219.h"
#include "utilities.h"

// Define for Timer4 frequency
#define CURRENT_CONTROL_FREQ 5000 // 5kHz control frequency

// Initialize the current controller with default values and set up Timer4 ISR
void CurrentControl_Init(void);

// Set PI gains for the current controller
void CurrentControl_SetGains(float kp, float ki);

// Get current PI gains
void CurrentControl_GetGains(float *kp, float *ki);

// Set the target current for the controller
void CurrentControl_SetTarget(float targetCurrent);

// Apply the calculated control output to the motor
void CurrentControl_ApplyOutput(float controlOutput);

// Reset the integrator
void CurrentControl_ResetIntegrator(void);

void MakeITESTWaveform(void);

// Start the current control loop
void CurrentControl_Enable(void);

// Stop the current control loop
void CurrentControl_Disable(void);

void PlotData(void);

#endif // CURRENTCONTROL__H__