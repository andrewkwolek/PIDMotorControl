#ifndef ENCODER__H__
#define ENCODER__H__

#include <xc.h> // processor SFR definitions
#include <sys/attribs.h> // __ISR macro

#include "nu32dip.h"

void UART2_Startup();
void WriteUART2(const char * string);
int get_encoder_flag();
void set_encoder_flag(int f);
int get_encoder_count();
float get_encoder_angle(int encoder_counts);

#endif // ENCODER__H__
