#include "positioncontrol.h"
#include <stdio.h>
#include <stdlib.h>

#define MAX_TRAJECTORY_LENGTH 1000
#define PLOTPTS 200

// Global static volatile controller variables
static volatile float kp = 2.0;       // Proportional gain
static volatile float ki = 0.1;       // Integral gain
static volatile float kd = 0.0;       // Derivative gain
static volatile float integral = 0.0;  // Integral error term
static volatile float prev_error = 0.0; // Previous error for derivative
static volatile float reference_position = 0.0; // Reference position in degrees
static volatile float Eintmax = 100.0; // Anti-windup limit

// Trajectory variables
static volatile float trajectory[MAX_TRAJECTORY_LENGTH];
static volatile int trajectory_length = 0;
static volatile int trajectory_index = 0;
static volatile int tracking_active = 0;

// Arrays for storing test data
static volatile float REFarray[PLOTPTS];
static volatile float MEASarray[PLOTPTS];
static volatile int StoringData = 0;
static volatile int plot_index = 0;
static volatile int decimation_counter = 0;
static volatile int DECIMATION = 5;

// ISR for position control loop (200 Hz)
void __ISR(_TIMER_5_VECTOR, IPL4SOFT) PositionControlISR(void) {
    static float current_command = 0.0;
    float actual_position, error, derivative;
    
    switch (getMode()) {
        case HOLD:
        case TRACK:
            // Read current position from encoder
            actual_position = get_encoder_angle(get_encoder_count());
            
            // If tracking mode, update reference from trajectory
            if (getMode() == TRACK && tracking_active) {
                if (trajectory_index < trajectory_length) {
                    reference_position = trajectory[trajectory_index++];
                } else {
                    // End of trajectory
                    tracking_active = 0;
                    // Keep holding the last position
                    setMode(HOLD);
                }
            }
            
            // Calculate error
            error = reference_position - actual_position;
            
            // Normalize error to handle wrap-around (keep error between -180 and 180 degrees)
            while (error > 180.0) error -= 360.0;
            while (error < -180.0) error += 360.0;
            
            // Calculate derivative term
            derivative = error - prev_error;
            prev_error = error;
            
            // Update integral term with anti-windup
            integral += error;
            if (integral > Eintmax) {
                integral = Eintmax;
            } else if (integral < -Eintmax) {
                integral = -Eintmax;
            }
            
            // Calculate PID control output
            current_command = kp * error + ki * integral + kd * derivative;
            
            // Set current reference for the current controller
            // This will be implemented by sending to current controller
            
            // If we're storing data for plotting
            if (StoringData) {
                decimation_counter++;
                if (decimation_counter >= DECIMATION) {
                    decimation_counter = 0;
                    if (plot_index < PLOTPTS) {
                        REFarray[plot_index] = reference_position;
                        MEASarray[plot_index] = actual_position;
                        plot_index++;
                    } else {
                        StoringData = 0;
                        plot_index = 0;
                    }
                }
            }
            break;
            
        default:
            // In other modes, position controller is inactive
            integral = 0.0;
            prev_error = 0.0;
            break;
    }
    
    // Clear the timer interrupt flag
    IFS0bits.T5IF = 0;
}

// Initialize the position controller with default values and set up Timer5 ISR
void PositionControl_Init(void) {
    // Initialize controller parameters
    kp = 2.0;        // Default proportional gain
    ki = 0.1;        // Default integral gain
    kd = 0.0;        // Default derivative gain
    integral = 0.0;  // Reset integrator
    prev_error = 0.0; // Reset previous error
    
    // Set up Timer5 for 200Hz interrupt frequency
    __builtin_disable_interrupts();
    
    // Configure Timer5
    T5CONbits.TCKPS = 0b011;    // Timer5 prescaler 1:8
    PR5 = (NU32DIP_SYS_FREQ / (8 * POSITION_CONTROL_FREQ)) - 1; // Period for 200Hz
    TMR5 = 0;                   // Initialize timer to 0
    
    // Set up Timer5 interrupt with priority 4
    IPC5bits.T5IP = 4;          // Priority level 4 (lower than current control)
    IPC5bits.T5IS = 0;          // Subpriority level 0
    IFS0bits.T5IF = 0;          // Clear interrupt flag
    IEC0bits.T5IE = 1;          // Enable Timer5 interrupt
    
    // Timer5 initially on
    T5CONbits.ON = 1;
    
    __builtin_enable_interrupts();
}

// Set PID gains for the position controller
void PositionControl_SetGains(float new_kp, float new_ki, float new_kd) {
    // Use critical section to modify controller parameters
    __builtin_disable_interrupts();
    
    kp = new_kp;
    ki = new_ki;
    kd = new_kd;
    // Reset integrator when gains change to prevent unexpected behavior
    integral = 0.0;
    prev_error = 0.0;
    
    __builtin_enable_interrupts();
}

// Get current PID gains
void PositionControl_GetGains(float *out_kp, float *out_ki, float *out_kd) {
    *out_kp = kp;
    *out_ki = ki;
    *out_kd = kd;
}

// Set the reference position in degrees
void PositionControl_SetReference(float reference_deg) {
    reference_position = reference_deg;
}

// Get the current reference position
float PositionControl_GetReference(void) {
    return reference_position;
}

// Hold the current position
void PositionControl_Hold(void) {
    // Read current position
    float current_position = get_encoder_angle(get_encoder_count());
    
    // Set reference to current position
    reference_position = current_position;
    
    // Reset controller states
    integral = 0.0;
    prev_error = 0.0;
    
    // Set mode to HOLD
    setMode(HOLD);
}

// Test position control with a step response
void PositionControl_TestStep(void) {
    char message[100];
    int i = 0;
    
    // Reset controller states
    integral = 0.0;
    prev_error = 0.0;
    
    // Start at current position
    float start_position = get_encoder_angle(get_encoder_count());
    reference_position = start_position;
    
    // Set mode to HOLD
    setMode(HOLD);
    
    // Allow controller to stabilize
    _CP0_SET_COUNT(0);
    while(_CP0_GET_COUNT() < NU32DIP_SYS_FREQ/4) { } // Wait 0.25s
    
    // Start data collection
    plot_index = 0;
    decimation_counter = 0;
    StoringData = 1;
    
    // Step change in reference
    reference_position = start_position + 45.0; // Step of 45 degrees
    
    // Wait for data collection to finish
    while(StoringData) { }
    
    // Send data to client
    sprintf(message, "%d\r\n", PLOTPTS);
    NU32DIP_WriteUART1(message);
    
    for (i = 0; i < PLOTPTS; i++) {
        sprintf(message, "%f %f\r\n", REFarray[i], MEASarray[i]);
        NU32DIP_WriteUART1(message);
    }
}

// Load trajectory data points
void PositionControl_LoadTrajectory(float *new_trajectory, int length) {
    int i;
    
    // Check for valid length
    if (length > MAX_TRAJECTORY_LENGTH) {
        length = MAX_TRAJECTORY_LENGTH;
    }
    
    // Copy trajectory data
    for (i = 0; i < length; i++) {
        trajectory[i] = new_trajectory[i];
    }
    
    trajectory_length = length;
    trajectory_index = 0;
}

// Execute loaded trajectory
void PositionControl_ExecuteTrajectory(void) {
    // Reset trajectory index
    trajectory_index = 0;
    
    // Set initial position
    reference_position = trajectory[0];
    
    // Reset controller states
    integral = 0.0;
    prev_error = 0.0;
    
    // Start trajectory tracking
    tracking_active = 1;
    
    // Set mode to TRACK
    setMode(TRACK);
}