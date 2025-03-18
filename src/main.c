#include "nu32dip.h"
#include "ina219.h"
#include "encoder.h"

#define BUF_SIZE 200
#define PWM 20000
#define PR3_VAL (NU32DIP_SYS_FREQ/PWM) - 1

int main() {
    char buffer[BUF_SIZE];
    NU32DIP_Startup();
    NU32DIP_GREEN = 1;
    NU32DIP_YELLOW = 1;

    UART2_Startup();
    INA219_Startup();

    __builtin_disable_interrupts();
    // Disable analog on Port A
    ANSELA = 0;  

    // Set RA3 as digital output
    TRISAbits.TRISA3 = 0;  

    // Set RB10 as digital output for direction control
    TRISBbits.TRISB10 = 0;

    // Map RA3 to OC3 for PWM output
    RPA3Rbits.RPA3R = 0b101;  

    // Set Timer3 to 20kHz frequency
    T3CONbits.TCKPS = 0x0;
    PR3 = PR3_VAL;
    TMR3 = 0;

    // Use OC3 for PWM mode with TMR3
    T3CONbits.ON = 1;
    OC3CONbits.OCTSEL = 1;
    OC3CONbits.OCM = 0x6;
    OC3RS = PR3/2;
    OC3R = PR3/2;
    OC3CONbits.ON = 1;
    __builtin_enable_interrupts();

    while (1) {
        NU32DIP_ReadUART1(buffer, BUF_SIZE);
        NU32DIP_YELLOW = 1;
        switch(buffer[0]) {
            case 'a':
            {
                // Read current sensor (ADC counts)
                NU32DIP_WriteUART1("Current sensor ADC is not implemented due to hardware change.");
                break;
            }
            case 'b':
            {
                // Read current sensor (mA)
                break;
            }
            case 'c':
            {
                // Read encoder
                WriteUART2("a");
                while(!get_encoder_flag()){}
                set_encoder_flag(0);
                char m[50];
                int p = get_encoder_count();
                sprintf(m, "%d\r\n", p);
                NU32DIP_WriteUART1(m);
                break;
            }
            case 'd':
            {
                // Read encoder
                int n = 0;
                NU32DIP_ReadUART1(buffer, BUF_SIZE);
                sscanf(buffer, "%d", &n);
                sprintf(buffer, "%d\r\n", n + 1); // return the number + 1
                NU32DIP_WriteUART1(buffer);
                break;
            }
            case 'e':
            {
                // Reset encoder
                WriteUART2("b");
                NU32DIP_WriteUART1("Encoder count reset\r\n");
                break;
            }
            case 'f':
            {
                // Set PWM (-100 to 100)
                NU32DIP_WriteUART1("Enter PWM (-100 to 100): ");
                int pwm_val;
                char pwm_buffer[50];
                
                // Read pwm value from buffer
                NU32DIP_ReadUART1(pwm_buffer, BUF_SIZE); 
                sscanf(pwm_buffer, "%d", &pwm_val);
                if (pwm_val >= -100 && pwm_val <= 100)
                {
                    char m[50];
                    sprintf(m, "PWM has been set to %d\r\n", pwm_val);
                    NU32DIP_WriteUART1(m);

                    // Set direction
                    if (pwm_val < 0) {
                        // Negative direction
                        LATBbits.LATB10 = 1;
                    } else {
                        // Positive direction
                        LATBbits.LATB10 = 0;
                    }
                    
                    // Set magnitude of PWM
                    int pwm_abs = abs(pwm_val);
                    OC3RS = (unsigned int)((pwm_abs / 100.0) * PR3);
                }
                else
                {
                    NU32DIP_WriteUART1("Invalid PWM value.\r\n");
                }
                break;
            }
            case 'g':
            {
                // Set current gains
                break;
            }
            case 'h':
            {
                // Get current gains
                break;
            }
            case 'i':
            {
                // Set position gains
                break;
            }
            case 'j':
            {
                // Get position gains
                break;
            }
            case 'k':
            {
                // Test current control
                break;
            }
            case 'l':
            {
                // Go to angle (deg)
                break;
            }
            case 'm':
            {
                // Load step trajectory
                break;
            }
            case 'n':
            {
                // Load cubic trajectory
                break;
            }
            case 'o':
            {
                // Execute trajectory
                break;
            }
            case 'p':
            {
                // Unpower the motor
                break;
            }
            case 'q':
            {
                // handle q for quit. Later you may want to return to idle mode here
                break;
            }
            case 'r':
            {
                // Get mode
                break;
            }
            default:
            {
                NU32DIP_YELLOW = 0; // Turn on to indicate error
                break;
            }
        }
    }

    return 0;
}