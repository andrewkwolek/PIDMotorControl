#include "nu32dip.h"
#include "ina219.h"
#include "encoder.h"
#include "currentcontrol.h"
#include "positioncontrol.h"
#include "utilities.h"

#define BUF_SIZE 200
#define PWM_VAL 20000
#define PR3_VAL (NU32DIP_SYS_FREQ/PWM_VAL) - 1
#define PLOTPTS 200
#define MAX_TRAJECTORY_LENGTH 1000

int main() {
    char buffer[BUF_SIZE];
    NU32DIP_Startup();
    NU32DIP_GREEN = 1;
    NU32DIP_YELLOW = 1;

    UART2_Startup();
    INA219_Startup();

    // Initialize current controller (sets up Timer4 for ISR)
    CurrentControl_Init();

    // Initialize position controller (sets up Timer5 for ISR)
    PositionControl_Init();

    char message[100];

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
    OC3RS = 0;
    OC3R = 0;
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
                double current = INA219_read_current();
                char m[50];
                sprintf(m, "%d\r\n", current);
                NU32DIP_WriteUART1(m);
                break;
            }
            case 'c':
            {
                // Read encoder (counts)
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
                // Read encoder (deg)
                WriteUART2("a");
                while(!get_encoder_flag()){}
                set_encoder_flag(0);
                char m[50];
                int p = get_encoder_count();
                float angle = get_encoder_angle(p);
                sprintf(m, "%f\r\n", angle);
                NU32DIP_WriteUART1(m);
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
                int pwm_val;
                char pwm_buffer[50];
                
                // Read pwm value from buffer
                NU32DIP_ReadUART1(pwm_buffer, BUF_SIZE); 
                sscanf(pwm_buffer, "%d", &pwm_val);
                if (pwm_val >= -100 && pwm_val <= 100) {
                    setMode(PWM);
                    PWMControl(pwm_val);
                }
                else {
                    NU32DIP_WriteUART1("Invalid PWM value.\r\n");
                }
                break;
            }
            case 'g':
            {
                // Set current gains
                float kp, ki;
                NU32DIP_ReadUART1(buffer, BUF_SIZE);
                sscanf(buffer, "%f", &kp);
                
                NU32DIP_ReadUART1(buffer, BUF_SIZE);
                sscanf(buffer, "%f", &ki);
                
                if (kp >= 0 && ki >= 0) {
                    CurrentControl_SetGains(kp, ki);
                    char m[100];
                    sprintf(m, "Sending Kp = %.3f and Ki = %.3f to the current controller.\r\n", kp, ki);
                    NU32DIP_WriteUART1(m);
                } else {
                    NU32DIP_WriteUART1("Invalid gains. Both must be non-negative.\r\n");
                }
                break;
            }
            case 'h':
            {
                // Get current gains
                float kp, ki;
                CurrentControl_GetGains(&kp, &ki);
                char m[100];
                sprintf(m, "The current controller is using Kp = %.3f and Ki = %.3f\r\n", kp, ki);
                NU32DIP_WriteUART1(m);
                break;
            }
            case 'i':
            {
                // Set position gains
                float posKp, posKi, posKd;
                
                // Read Kp value
                NU32DIP_ReadUART1(buffer, BUF_SIZE);
                sscanf(buffer, "%f", &posKp);
                
                // Read Ki value
                NU32DIP_ReadUART1(buffer, BUF_SIZE);
                sscanf(buffer, "%f", &posKi);
                
                // Read Kd value
                NU32DIP_ReadUART1(buffer, BUF_SIZE);
                sscanf(buffer, "%f", &posKd);
                
                // Validate gains (all should be non-negative)
                if (posKp >= 0 && posKi >= 0 && posKd >= 0) {
                    PositionControl_SetGains(posKp, posKi, posKd);
                    char m[100];
                    sprintf(m, "Position controller gains set to: Kp=%.3f, Ki=%.3f, Kd=%.3f\r\n", 
                            posKp, posKi, posKd);
                    NU32DIP_WriteUART1(m);
                } else {
                    NU32DIP_WriteUART1("Invalid gains. All values must be non-negative.\r\n");
                }
                break;
            }
            case 'j':
            {
                // Get position gains
                float posKp, posKi, posKd;
                
                // Get current gains from position controller
                PositionControl_GetGains(&posKp, &posKi, &posKd);
                
                // Report gains to user
                char m[100];
                sprintf(m, "The position controller is using: Kp=%.3f, Ki=%.3f, Kd=%.3f\r\n", 
                        posKp, posKi, posKd);
                NU32DIP_WriteUART1(m);
                break;
            }
            case 'k': {
                setMode(ITEST);
                CurrentControl_Test();
                break;
            }
            case 'l':
            {
                // Go to angle (deg)
                float angle;
                NU32DIP_ReadUART1(buffer, BUF_SIZE);
                sscanf(buffer, "%f", &angle);
                
                // Set the reference position for the position controller
                PositionControl_SetReference(angle);
                
                // Switch to HOLD mode
                setMode(HOLD);
                
                // Provide feedback to the user
                sprintf(message, "Going to angle: %.2f degrees\r\n", angle);
                NU32DIP_WriteUART1(message);
                break;
            }
            case 'm':
            {
                // Load step trajectory
                char count_str[20];
                int trajectory_length = 0;
                float trajectory_data[MAX_TRAJECTORY_LENGTH];
                
                // Read number of points
                NU32DIP_ReadUART1(count_str, BUF_SIZE);
                sscanf(count_str, "%d", &trajectory_length);
                
                // Clamp to maximum trajectory length if needed
                if (trajectory_length > MAX_TRAJECTORY_LENGTH) {
                    trajectory_length = MAX_TRAJECTORY_LENGTH;
                }
                
                // Read each trajectory point
                for (int i = 0; i < trajectory_length; i++) {
                    char point_str[20];
                    NU32DIP_ReadUART1(point_str, BUF_SIZE);
                    sscanf(point_str, "%f", &trajectory_data[i]);
                }
                
                // Load trajectory into position controller
                PositionControl_LoadTrajectory(trajectory_data, trajectory_length);
                
                // Send confirmation
                sprintf(message, "Loaded step trajectory with %d points\r\n", trajectory_length);
                NU32DIP_WriteUART1(message);
                break;
            }
            case 'n':
            {
                // Load cubic trajectory - same structure as 'm' case
                char count_str[20];
                int trajectory_length = 0;
                float trajectory_data[MAX_TRAJECTORY_LENGTH];
                
                // Read number of points
                NU32DIP_ReadUART1(count_str, BUF_SIZE);
                sscanf(count_str, "%d", &trajectory_length);
                
                // Clamp to maximum trajectory length if needed
                if (trajectory_length > MAX_TRAJECTORY_LENGTH) {
                    trajectory_length = MAX_TRAJECTORY_LENGTH;
                }
                
                // Read each trajectory point
                for (int i = 0; i < trajectory_length; i++) {
                    char point_str[20];
                    NU32DIP_ReadUART1(point_str, BUF_SIZE);
                    sscanf(point_str, "%f", &trajectory_data[i]);
                }
                
                // Load trajectory into position controller
                PositionControl_LoadTrajectory(trajectory_data, trajectory_length);
                
                // Send confirmation
                sprintf(message, "Loaded cubic trajectory with %d points\r\n", trajectory_length);
                NU32DIP_WriteUART1(message);
                break;
            }
            case 'o':
            {
                // Execute trajectory
                PositionControl_ExecuteTrajectory();
                break;
            }
            case 'p':
            {
                setMode(IDLE);
                break;
            }
            case 'q':
            {
                // handle q for quit. Later you may want to return to idle mode here
                setMode(IDLE);
                break;
            }
            case 'r':
            {
                // Get mode
                switch(getMode()) {
                    case IDLE:
                        NU32DIP_WriteUART1("The PIC32 controller mode is currently IDLE.\r\n");
                        break;
                    case PWM:
                        NU32DIP_WriteUART1("The PIC32 controller mode is currently PWM.\r\n");
                        break;
                    case ITEST:
                        NU32DIP_WriteUART1("The PIC32 controller mode is currently ITEST.\r\n");
                        break;
                    case HOLD:
                        NU32DIP_WriteUART1("The PIC32 controller mode is currently HOLD.\r\n");
                        break;
                    case TRACK:
                        NU32DIP_WriteUART1("The PIC32 controller mode is currently TRACK.\r\n");
                        break;
                }
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