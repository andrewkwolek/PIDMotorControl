#include "currentcontrol.h"
#include <stdio.h>
#include <stdlib.h>

// Global static volatile controller variables
static volatile float kp = 0.5;       // Proportional gain
static volatile float ki = 0.1;       // Integral gain
static volatile float integral = 0.0; // Integral error term
static volatile float target = 0.0;   // Target current in mA
static volatile float actual = 0.0;   // Measured current
static volatile float error = 0.0;    // Error term
static volatile float u = 0.0;        // Control output
static volatile int enabled = 0;      // Control enabled flag

// Arrays for storing test data
#define MAX_SAMPLES 250
static volatile float reference[MAX_SAMPLES];
static volatile float measurement[MAX_SAMPLES];

// Constants
static const float umax = 100.0;     // Maximum control output
static const float umin = -100.0;    // Minimum control output

// ISR for current control loop
void __ISR(_TIMER_4_VECTOR, IPL5SOFT) CurrentControlISR(void) {
    // Only execute control calculations if enabled
    if (enabled) {
        // Read current from sensor
        actual = INA219_read_current();
        
        // Calculate error
        error = target - actual;
        
        // Update integral term with anti-windup
        integral += error;
        
        // Calculate control output (PI controller)
        u = kp * error + ki * integral;
        
        // Apply output limits with anti-windup
        if (u > umax) {
            u = umax;
            // Prevent further integration in this direction
            integral -= error;
        } else if (u < umin) {
            u = umin;
            // Prevent further integration in this direction
            integral -= error;
        }
        
        // Apply the control output
        CurrentControl_ApplyOutput(u);
    }
    
    // Clear the timer interrupt flag
    IFS0bits.T4IF = 0;
}

// Initialize the current controller with default values and set up Timer4 ISR
void CurrentControl_Init(void) {
    // Initialize controller parameters
    kp = 0.5;        // Default proportional gain
    ki = 0.1;        // Default integral gain
    integral = 0.0;  // Reset integrator
    target = 0.0;    // Default target current (no current)
    actual = 0.0;    // Initial measured current
    error = 0.0;     // Initial error
    u = 0.0;         // Initial control output
    enabled = 0;     // Start with control loop disabled
    
    // Set up Timer4 for 5kHz interrupt frequency
    __builtin_disable_interrupts();
    
    // Configure Timer4
    T4CONbits.TCKPS = 0b000;    // Timer4 prescaler 1:1
    PR4 = (NU32DIP_SYS_FREQ / CURRENT_CONTROL_FREQ) - 1; // Period for 5kHz
    TMR4 = 0;                   // Initialize timer to 0
    
    // Set up Timer4 interrupt with priority 5
    IPC4bits.T4IP = 5;          // Priority level 5
    IPC4bits.T4IS = 0;          // Subpriority level 0
    IFS0bits.T4IF = 0;          // Clear interrupt flag
    IEC0bits.T4IE = 1;          // Enable Timer4 interrupt
    
    // Timer4 initially off
    T4CONbits.ON = 0;
    
    __builtin_enable_interrupts();
}

// Set PI gains for the current controller
void CurrentControl_SetGains(float new_kp, float new_ki) {
    // Use critical section to modify controller parameters
    __builtin_disable_interrupts();
    
    kp = new_kp;
    ki = new_ki;
    // Reset integrator when gains change to prevent unexpected behavior
    integral = 0.0;
    
    __builtin_enable_interrupts();
}

// Get current PI gains
void CurrentControl_GetGains(float *out_kp, float *out_ki) {
    *out_kp = kp;
    *out_ki = ki;
}

// Set the target current for the controller
void CurrentControl_SetTarget(float targetCurrent) {
    target = targetCurrent;
}

// Apply the calculated control output to the motor
void CurrentControl_ApplyOutput(float controlOutput) {
    // Set direction based on sign of control output
    if (controlOutput < 0) {
        // Negative direction
        LATBbits.LATB10 = 1;
    } else {
        // Positive direction
        LATBbits.LATB10 = 0;
    }
    
    // Set magnitude of PWM (convert controlOutput [-100, 100] to PWM duty cycle)
    int pwm_abs = abs((int)controlOutput);
    if (pwm_abs > 100) {
        pwm_abs = 100; // Safety clamp to maximum allowed value
    }
    
    // PR3 is defined in main.c as (NU32DIP_SYS_FREQ/PWM) - 1
    // Use the same formula to calculate OC3RS
    unsigned int PR3_VAL = (NU32DIP_SYS_FREQ/20000) - 1; // 20kHz PWM frequency
    OC3RS = (unsigned int)((pwm_abs / 100.0) * PR3_VAL);
}

// Reset the integrator
void CurrentControl_ResetIntegrator(void) {
    __builtin_disable_interrupts();
    integral = 0.0;
    __builtin_enable_interrupts();
}

// Start the current control loop
void CurrentControl_Enable(void) {
    CurrentControl_ResetIntegrator(); // Reset integrator when enabling
    enabled = 1;
    T4CONbits.ON = 1; // Turn on Timer4 to start ISR
}

// Stop the current control loop
void CurrentControl_Disable(void) {
    enabled = 0;
    T4CONbits.ON = 0; // Turn off Timer4
    OC3RS = 0; // Set PWM to 0
}

// Get the current control status
int CurrentControl_IsEnabled(void) {
    return enabled;
}

// Test current control with a 100 Hz square wave reference
// Returns number of samples recorded
int CurrentControl_Test(void) {
    // Save the previous target current
    float prevTarget = target;
    
    // Variables for the ±200 mA, 100 Hz square wave reference
    float squareWaveAmp = 200.0; // mA amplitude
    int cycleCount = 0;          // Count the number of complete cycles
    int maxCycles = 3;           // Number of cycles to capture (2-4)
    
    // Array to store the data index that we're currently on
    volatile int sampleIndex = 0;
    
    // Reset the integrator at the start of the test
    CurrentControl_ResetIntegrator();
    
    // Enable current control loop
    int wasEnabled = CurrentControl_IsEnabled();
    if (!wasEnabled) {
        CurrentControl_Enable();
    }
    
    // Set up a timer to track the square wave generation
    // We're running at 5kHz ISR rate, and want a 100 Hz square wave
    // So we toggle every 25 samples (5000/100/2 = 25)
    const int samplesPerHalfCycle = CURRENT_CONTROL_FREQ / (100 * 2);
    int halfCycleSampleCount = 0;
    
    // Set initial reference to positive amplitude
    target = squareWaveAmp;
    
    // Now collect data while the ISR runs the controller
    // We'll store both the reference and the actual current values
    
    // Compute the total number of samples for the desired number of cycles
    int samplesPerCycle = CURRENT_CONTROL_FREQ / 100;
    int totalSamplesToCapture = samplesPerCycle * maxCycles;
    
    // Make sure we don't exceed the array size
    if (totalSamplesToCapture > MAX_SAMPLES) {
        totalSamplesToCapture = MAX_SAMPLES;
    }
    
    // Store the current mode to return to after the test
    int currentMode = 1; // Current control mode
    
    // Main test loop
    while (sampleIndex < totalSamplesToCapture) {
        // Record the reference and actual values
        reference[sampleIndex] = target;
        measurement[sampleIndex] = actual;
        
        // Update the sample index
        sampleIndex++;
        
        // Check if we need to toggle the reference value
        halfCycleSampleCount++;
        if (halfCycleSampleCount >= samplesPerHalfCycle) {
            // Toggle the reference between positive and negative
            target = -target;
            halfCycleSampleCount = 0;
            
            // Track completed cycles (after every two half-cycles)
            if (target > 0) {
                cycleCount++;
            }
        }
        
        // Give time for the ISR to run (avoid hogging CPU)
        // The ISR will update 'actual' at 5kHz
        _CP0_SET_COUNT(0);
        while (_CP0_GET_COUNT() < NU32DIP_SYS_FREQ / CURRENT_CONTROL_FREQ) {
            ; // Wait for approximately one ISR period
        }
    }
    
    // Test complete, return to previous state
    target = 0.0; // Set target current to 0
    
    // Wait for current to settle near zero
    _CP0_SET_COUNT(0);
    while (_CP0_GET_COUNT() < NU32DIP_SYS_FREQ / 10) { // Wait 0.1 seconds
        ; // Allow time for current to settle
    }
    
    // Restore previous state
    target = prevTarget;
    
    // If controller wasn't enabled before, disable it now
    if (!wasEnabled) {
        CurrentControl_Disable();
    }
    
    // Return the number of samples collected
    return sampleIndex;
}

// Get the reference and measurement data
void CurrentControl_GetTestData(float *ref_dest, float *meas_dest, int numSamples) {
    int i;
    for (i = 0; i < numSamples && i < MAX_SAMPLES; i++) {
        ref_dest[i] = reference[i];
        meas_dest[i] = measurement[i];
    }
}