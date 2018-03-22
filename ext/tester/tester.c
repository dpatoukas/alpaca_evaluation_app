#include <msp430.h>
#include <stdbool.h>
#include "tester.h"

#pragma PERSISTENT(noise_idx)
unsigned int noise_idx = 0;

extern uint32_t current_ticks;
//#define __nv  __attribute__((section(".nv_vars")))

__nv uint16_t energy_buffer;
__nv float consm_rate = 1;
__nv float lpm_consm_rate = 0;

void set_rate(float rt)
{
  consm_rate = rt;
}

void set_eu(uint16_t energy_units)
{
  energy_buffer = energy_units;
}

void reseter(uint16_t interval)
{
  //TA0CCTL0 = CCIE;                          // TACCR0 interrupt enabled
  //TA0CCR0 = interval;
  //TA0CTL = TASSEL__SMCLK | MC__UP | TACLR;   // SMCLK, UP mode
  //TA0CTL &= ~TAIFG;

  TA0CCR0 = interval; // Take current count + desired count
  TA0CCTL0 = CCIE; // Enable Interrupts on Comparator register
  TA0CTL = TASSEL__ACLK | MC__UP | TACLR; // start timer
  __bis_SR_register(GIE);       // enable general interrupt
}

void eb_tester_start(void* noise_pattern)
{

    P4OUT |= BIT3;

	set_eu(((int16_t*) noise_pattern)[noise_idx]);

    if (++noise_idx >= NOISE_LEN) {
        noise_idx = 0;
    }

 	reseter((uint16_t) energy_buffer/consm_rate);
  
  	set_rate(consm_rate);
}

void eb_tester_reseter(float depletion_rate){
  
  set_eu((energy_buffer-(TA0R*consm_rate)));
  
  reseter((uint16_t) energy_buffer/(depletion_rate));
  
  set_rate(depletion_rate);
}


// Timer0_A0 interrupt service routine
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = TIMER0_A0_VECTOR
__interrupt void Timer0_A0_ISR (void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER0_A0_VECTOR))) Timer0_A0_ISR (void)
#else
#error Compiler not supported!
#endif
{
   // TODO: modify as needed
   __bic_SR_register(GIE);       // disable general interrupt
   //__no_operation();
   //current_ticks = __get_time();
   PMMCTL0 = PMMPW | PMMSWBOR;
}
