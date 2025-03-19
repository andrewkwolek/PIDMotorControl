#include "currentcontrol.h"
#include <stdio.h>
#include <stdlib.h>

#define NUMSAMPS 100
#define PLOTPTS 50
#define DECIMATION 1

// Global static volatile controller variables
static volatile float kp = 0.5;       // Proportional gain
static volatile float ki = 0.1;       // Integral gain
static volatile float integral = 0.0; // Integral error term
static volatile float Eintmax = 1000.0;

// Arrays for storing test data
static volatile float REFarray[PLOTPTS];
static volatile float MEASarray[PLOTPTS];
static volatile int Waveform[NUMSAMPS];
static volatile int StoringData = 0;

static volatile Mode mode = IDLE;

// Constants
static const float umax = 100.0;     // Maximum control output
static const float umin = -100.0;    // Minimum control output

// ISR for current control loop
void __ISR(_TIMER_4_VECTOR, IPL5SOFT) CurrentControlISR(void) {
    static int counter = 0; // initialize counter once
    static int plotind = 0;
    static int decctr = 0;
    static float actual = 0.0;

    // Only execute control calculations if enabled
    switch (mode) {
        case ITEST:
            // Read current from sensor
            actual = INA219_read_current();
            
            // Calculate error
            float error = Waveform[counter] - actual;
            
            // Update integral term with anti-windup
            integral += error;
            if (integral > Eintmax) {
                integral = Eintmax;
            } else if (integral < -Eintmax) {
                integral = -Eintmax;
            }

            // Calculate control output (PI controller)
            float u = kp * error + ki * integral;
            
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
            
            if (u < 0) {
                // Negative direction
                LATBbits.LATB10 = 1;
            } else {
                // Positive direction
                LATBbits.LATB10 = 0;
            }
            
            // Set magnitude of PWM (convert controlOutput [-100, 100] to PWM duty cycle)
            int pwm_abs = abs((int)u);
            if (pwm_abs > 100) {
                pwm_abs = 100; // Safety clamp to maximum allowed value
            }
            
            // PR3 is defined in main.c as (NU32DIP_SYS_FREQ/PWM) - 1
            // Use the same formula to calculate OC3RS
            unsigned int PR3_VAL = (NU32DIP_SYS_FREQ/20000) - 1; // 20kHz PWM frequency
            OC3RS = (unsigned int)((pwm_abs / 100.0) * PR3_VAL);

            if (StoringData) {
                decctr++;
                if (decctr == DECIMATION) { 
                    decctr = 0; 
                    MEASarray[plotind] = actual; 
                    REFarray[plotind] = Waveform[counter];
                    plotind++;
                }
                if (plotind == PLOTPTS) { 
                    plotind = 0;
                    StoringData = 0; 
                }
            }

            counter++; // add one to counter every time ISR is entered
            if (counter == NUMSAMPS) {
                counter = 0; // roll the counter over when needed
                setMode(IDLE);
            }
            break;

        case IDLE:
            break;
        
        default:
            break;
    }
    
    // Clear the timer interrupt flag
    IFS0bits.T4IF = 0;
}

void makeWaveform() {
    int i = 0;
    int center = 0;
    int A = 200; // square wave, fill in center value and amplitude
    for (i = 0; i < NUMSAMPS; ++i) {
        if ( i < NUMSAMPS/2) {
            Waveform[i] = center + A;
        } else {
            Waveform[i] = center - A;
        }
    }
}

// Initialize the current controller with default values and set up Timer4 ISR
void CurrentControl_Init(void) {
    // Initialize controller parameters
    kp = 0.5;        // Default proportional gain
    ki = 0.1;        // Default integral gain
    integral = 0.0;  // Reset integrator

    makeWaveform();
    
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
    
    // Timer4 initially on
    T4CONbits.ON = 1;
    
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

// Reset the integrator
void CurrentControl_ResetIntegrator(void) {
    __builtin_disable_interrupts();
    integral = 0.0;
    __builtin_enable_interrupts();
}

// Test current control with a 100 Hz square wave reference
// Returns number of samples recorded
void CurrentControl_Test(void) {
    char message[100];
    int i = 0;
    // Reset the integrator at the start of the test
    CurrentControl_ResetIntegrator();
    
    StoringData = 1; // message to ISR to start storing data
    while (StoringData) { // wait until ISR says data storing is done
        ; // do nothing
    }
    for (i=0; i<PLOTPTS; i++) { // send plot data to MATLAB
        sprintf(message, "%d %d %d\r\n", PLOTPTS-i, MEASarray[i], REFarray[i]);
        NU32DIP_WriteUART1(message);
    }
}
