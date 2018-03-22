#include "ink.h"

//cm_acc Header
#ifndef APP_INTR_H_
#define APP_INTR_H_

/* FFT characteristics definitions */
//Dummy values 
#define N_SAMPLES               128
#define INPUT_AMPLITUDE         0.5
#define INPUT_FREQUENCY         64
#define SAMPL_FREQ              512
#define PI                      3.1415926536
#define FREQ_THRESHOLD          1500

//UART communication codes
#define ACC 13
#define SIGNAL 52
#define FREE 166
#define STOP 26

//Boundaries in "amplitude" for raising a pin 
#define BOUND_SINGAL 1000
#define BOUND_ACC 1250

#define ACC_SAMPLES 3

//filter bound 
#define BOUND 2000

//high priority signal filter
uint8_t valid_signal(void);

//setup ADC
void ADC_config();

//mcu settings 
void setup_mcu();

#endif //APP_INTR_H_
