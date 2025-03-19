#ifndef POSITIONCONTROL__H__
#define POSITIONCONTROL__H__

#include <xc.h> // processor SFR definitions
#include <sys/attribs.h> // __ISR macro
#include "nu32dip.h"
#include "currentcontrol.h"
#include "encoder.h"
#include "utilities.h"

// Define for Timer5 frequency
#define POSITION_CONTROL_FREQ 200 // 200Hz control frequency

// Initialize position controller and Timer5
void PositionControl_Init(void);

// Set position controller gains
void PositionControl_SetGains(float kp, float ki, float kd);

// Get position controller gains
void PositionControl_GetGains(float *kp, float *ki, float *kd);

// Set the reference position in degrees
void PositionControl_SetReference(float reference_deg);

// Get the current reference position
float PositionControl_GetReference(void);

// Reset the position controller integrator
void PositionControl_ResetIntegrator(void);

// Load trajectory data points
void PositionControl_LoadTrajectory(float *trajectory, int length);

// Execute loaded trajectory
void PositionControl_ExecuteTrajectory(void);

#endif // POSITIONCONTROL__H__