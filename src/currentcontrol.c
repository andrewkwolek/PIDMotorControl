#include "currentcontrol.h"
#include <stdio.h>
#include <stdlib.h>

#define PLOTPTS 200
#define DECIMATION 10
#define NUMSAMPS 50

// Global static volatile controller variables
static volatile float kp = 0.5;       // Proportional gain
static volatile float ki = 0.1;       // Integral gain
static volatile float integral = 0.0; // Integral error term
static volatile float target = 0.0;   // Target current in mA
static volatile float actual = 0.0;   // Measured current
static volatile float error = 0.0;    // Error term
static volatile float u = 0.0;        // Control output
static volatile int enabled = 0;      // Control enabled flag

// Constants
static const float umax = 100.0;     // Maximum control output
static const float umin = -100.0;    // Minimum control output

static volatile enum Mode mode = IDLE;

static volatile int CurrentWaveform[NUMSAMPS];
static volatile float CurrentACTUALarray[PLOTPTS];
static volatile float CurrentREFarray[PLOTPTS];
static volatile int StoringCurrentData = 0;

// ISR for current control loop
void __ISR(_TIMER_4_VECTOR, IPL5SOFT) CurrentControlISR(void) {
    static int counter = 0; // initialize counter once
    static int plotind = 0;
    static int decctr = 0;
    static float actual = 0.f;

    mode = getMode();

    // Only execute control calculations if enabled
    switch (mode) {
        case ITEST:
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

            if (StoringCurrentData) {
                decctr++;
                if (decctr == DECIMATION) { 
                    decctr = 0; 
                    CurrentACTUALarray[plotind] = actual; 
                    CurrentREFarray[plotind] = CurrentWaveform[counter];
                    plotind++;
                }
                if (plotind == PLOTPTS) { 
                    plotind = 0;
                    StoringCurrentData = 0; 
                }
            }
            
            // Apply the control output
            CurrentControl_ApplyOutput(u);

            break;
    }
    
    counter++; // add one to counter every time ISR is entered
    if (counter == NUMSAMPS) {
        counter = 0; // roll the counter over when needed
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

    MakeITESTWaveform();
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

// Generate reference waveform
void MakeITESTWaveform(void) {
    int i = 0;
    int center = 0;
    int A = 200; // square wave, fill in center value and amplitude
    for (i = 0; i < NUMSAMPS; ++i) {
        if ( i < NUMSAMPS/2) {
            CurrentWaveform[i] = center + A;
        } else {
            CurrentWaveform[i] = center - A;
        }
    }
}

void PlotData(void) {
    int i = 0;
    char message[100];
    StoringCurrentData = 1;
    while (StoringCurrentData) {
        ;
    }
    for (i=0; i<PLOTPTS; i++) { // send plot data to MATLAB
        sprintf(message, "%d %d %d\r\n", PLOTPTS-i, CurrentACTUALarray[i], CurrentREFarray[i]);
        NU32DIP_WriteUART1(message);
    }
}