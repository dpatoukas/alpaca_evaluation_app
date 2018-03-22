/*
 * tester.h
 *
 *  Created on: Mar 11, 2018
 *      Author: dpatoukas
 */

#ifndef TESTER_TESTER_H_
#define TESTER_TESTER_H_
//#include <msp430.h>
//#include "tester.h"
//#include <stdbool.h>
#include <stdint.h>
#define NOISE_LEN 200

void set_rate(float rt);
void set_eu(uint16_t energy_units);
void reseter(uint16_t interval);
void eb_tester_start(void* noise_pattern);
void eb_tester_reseter(float depletion_rate);


#endif /* TESTER_TESTER_H_ */
