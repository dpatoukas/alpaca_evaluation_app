
#include <msp430.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "../ext/i2c_lib/adxl345_i2c_lib.h"
#include "../ext/app_specs/app_intrinsics.h"
#include "../ext/dsplib/include/DSPLib.h"
#include "../ext/tester/tester.h"

uint16_t noise[] = {
                    209,90,678,329,34,691,616,527,846,793,683,259,448,938,539,653,936,610,1118,1324,632,1081,486,617,764,1016,1006,989,1526,906,1027,1760,1633,1568,1275,2210,2022,1824,2436,2200,1990,1731,2634,2550,2763,2185,2886,2614,2415,2543,3040,2951,2806,3739,3058,3991,3423,3736,3796,3545,3739,4705,4340,4182,4729,4977,4829,5154,4862,5378,5717,5324,6022,5846,6380,6026,6174,6912,6895,6745,6912,7058,7272,7316,8038,7277,8335,8227,8109,8478,8148,9042,9390,9339,9300,9370,9575,10253,9705,10043,10324,11196,10423,10636,11020,11319,11713,11906,12174,12062,12818,12808,12917,13070,13283,14145,14065,13930,14676,14647,15018,15197,15615,15645,15502,15893,16317,17056,16699,17477,17792,17196,17920,18656,18377,18318,19222,18981,19238,20275,20408,20551,20927,21208,21731,21528,22128,22077,22394,22812,22984,23192,23677,23456,24431,24205,24834,25267,25267,25744,26536,25986,27203,26817,27112,27987,27899,28812,28869,29278,29542,29641,29978,30878,30711,31128,31320,31972,32577,32345,33244,32942,34071,33527,34495,34391,34888,34998,36105,36112,36725,37299,36960,37576,38467,38517,39301,38882,39475,40234
};

//tasks
TASK(1 , init);
TASK(2 , task_schedule);
TASK(3 , task_sample_accel);
TASK(4 , task_fft_accel);
TASK(5 , task_sample_mic);
TASK(6 , task_fft_mic);
TASK(7 , task_filter_data);
TASK(8 , task_init);
TASK(9 , task_selectMode);
TASK(10 , task_resetStats);
TASK(11, task_sample);
TASK(12 , task_transform);
TASK(13 , task_featurize);
TASK(14 , task_classify);
TASK(15 , task_stats);
TASK(16 , task_warmup);
TASK(17 , task_train);
TASK(18 , task_idle);

typedef enum
{
	MIC,
	WAIT,
	FILTER,
	ACCEL

}sched_t;

static sched_t _ISR_flag = WAIT;

GLOBAL_SB(uint8_t,inited)

//scheduler
GLOBAL_SB(sched_t,flag)

//SOUND FFT
GLOBAL_SB(_q15,pers_sdata,FFT_SAMPLES);

//ACCEL FFT
GLOBAL_SB(_q15,data_array,N_SAMPLES);

//pesistent vars
GLOBAL_SB(uint16_t, pinState);
GLOBAL_SB(unsigned, discardedSamplesCount);
GLOBAL_SB(run_mode_t, class);
GLOBAL_SB(unsigned, totalCount);
GLOBAL_SB(unsigned, movingCount);
GLOBAL_SB(unsigned, stationaryCount);
GLOBAL_SB(accelReading, window, ACCEL_WINDOW_SIZE);
GLOBAL_SB(features_t, features);
GLOBAL_SB(features_t, model_stationary, MODEL_SIZE);
GLOBAL_SB(features_t, model_moving, MODEL_SIZE);
GLOBAL_SB(unsigned, trainingSetSize);
GLOBAL_SB(unsigned, samplesInWindow);
GLOBAL_SB(run_mode_t, mode);
GLOBAL_SB(unsigned, seed);
GLOBAL_SB(unsigned, count);

void configure_mcu()
{
  WDTCTL = WDTPW | WDTHOLD; // Stop watchdog timer

  __delay_cycles(3);

  // Configure GPIO
  // Pin 1.2 will be the input to fire ISR1
  //enable_P1P2();
  P1OUT &= ~BIT2;                             // Pull-down resistor on P1.2
  P1REN |= BIT2;                              // Select down-up mode for P1.2
  P1DIR &= ~BIT2;                             // Set P1.2 to output direction
  P1IES &= ~BIT2;                             // P1.2 Lo/Hi edge

  P1OUT &= ~BIT4;                             // Pull-down resistor on P1.4
  P1REN |= BIT4;                              // Select down-up mode for P1.4
  P1DIR &= ~BIT4;                              // Set all but P1.4 to output direction
  P1IES &= ~BIT4;                             // P1.4 Lo/Hi edge                                               // Clear all P1 interrupt flags
  P1IFG = 0;                                  // Clear all P1 interrupt flags
  
  //Logic analyzer pins
  P3OUT &= ~(BIT5); 
  P3DIR |=  BIT5;
  //Logic analyzer pins
  P3OUT &= ~(BIT4); 
  P3DIR |=  BIT4;

  P4OUT &= ~(BIT3 | BIT2);
  P4DIR |=  (BIT3 | BIT2);

  P3OUT &= ~(BIT6|BIT0); 
  P3DIR |=  (BIT6|BIT0);

  P2OUT &= ~(BIT6|BIT5|BIT2|BIT4); 
  P2DIR |=  (BIT6|BIT5|BIT2|BIT4);

  //LED
  P1OUT &= ~BIT0;                           // Clear P1.0 output latch
  P1DIR |= BIT0;                            // For LED on P1.0
  P4OUT &= ~BIT6;                           // Clear P4.6 output latch
  P4DIR |= BIT6;                            // For LED on P4.6


  PM5CTL0 &= ~LOCKLPM5; // Disable the GPIO power-on default high-impedance mode

  // Clock System Setup
  CSCTL0_H = CSKEY >> 8;   // Unlock CS registers
  CSCTL2 |= SELA__VLOCLK;
  CSCTL3 |= DIVA__1 ;     // Set all dividers to 1
  CSCTL0_H = 0;           // Lock CS registe

}

void init(){

	configure_mcu();
#ifdef POWER_TEST
    eb_tester_start(noise);
#endif

    P1IE |= BIT4;                              // P1.4 interrupt enabled low priority thread
    P1IE |= BIT2;                              // P1.2 interrupt enabled high priority thread
    __delay_cycles(15);

    _ISR_flag = WAIT;
    
    if (!GV(inited))
    {
    	GV(inited) = 1;
    	GV(flag) = WAIT;
    }
    __enable_interrupt();

}

void mcu_sleep()
{	
#ifdef POWER_TEST
    eb_tester_reseter(0.3);
	P2OUT |= BIT2;
	__bis_SR_register(LPM3_bits | GIE)
    P2OUT &= ~BIT2;
    eb_tester_reseter(1);
#else
	__bis_SR_register(LPM3_bits | GIE)

#endif
	TRANSITION_TO(task_schedule);
}

void task_schedule()
{
	uint16_t tmp = 100;
    while(tmp--) 
    {
        rand();
    }

#ifdef NO_INT_SOURCE
//if interrupts are not available use this section
	   if ((rand() % 2) == 0) {

	       _ISR_flag = ACCEL;
	   }

	   else {

	       _ISR_flag = MIC;
	  }
#endif

	if (GV(flag) != WAIT)
	{
		GV(flag) = _ISR_flag;
	}
	_ISR_flag = WAIT;

	switch GV(flag)
	{
		case MIC:
			TRANSITION_TO(task_sample_mic);
			break;
		case ACCEL:
			TRANSITION_TO(task_sample_accel);
			break;
		case FILTER:
			TRANSITION_TO(task_filter_data);
			break;
		case WAIT:
			mcu_sleep();
			break;
			default:
			TRANSITION_TO(task_schedule);
	}

}


_interrupt(PORT1_VECTOR)
{
  switch(__even_in_range(P1IV, P1IV_P1IFG7))
    {
        case P1IV_P1IFG2:
            P1IE &= ~BIT2;
            P1IFG &= ~BIT2;                         // Clear P1.2 IFG

            _ISR_flag = FILTER;
            /* turn on CPU */
            __bic_SR_register_on_exit(LPM3_bits);
			TRANSITION_TO(task_schedule);
            break;                                  // Vector  6:  P1.2 interrupt flag
        case P1IV_P1IFG4:                           // Vector  10:  P1.4 interrupt flag
            P1IE &= ~BIT4;
            P1IFG &= ~BIT4;                         // Clear P1.4 IFG

			_ISR_flag = ACCEL;            
            /* turn on CPU */
            __bic_SR_register_on_exit(LPM3_bits);
			TRANSITION_TO(task_schedule);
            break;
        default: break;
    }
 	  
}


/*FFT characteristics definitions*/
#define FFT_SAMPLES             128
#define INPUT_AMPLITUDE         0.5
#define INPUT_FREQUENCY         64
#define SAMPL_FREQ              512
#define FFT_FREQ_THRESHOLD      1500
#define REAL_ADC_FREQ           1000000 / (4 * 4 * 16)
#define INDEX_THRESHOLD         FFT_FREQ_THRESHOLD / (REAL_ADC_FREQ / (2 * FFT_SAMPLES))

void task_filter_data()
{
	if (valid_signal())
		GV(flag) = MIC;
	else
		GV(flag) = WAIT;

	TRANSITION_TO(task_schedule);
}



DSPLIB_DATA(tb_fftd, MSP_ALIGN_FFT_Q15(FFT_SAMPLES))
 _q15 tb_fftd[FFT_SAMPLES];

uint16_t counter;
uint16_t is;
_q15 sampled_input[FFT_SAMPLES];

void task_sample_mic()
{  
  	P3OUT |= BIT5;
  	__disable_interrupt();
	// save interrupt state and then disable interrupts
	is = __get_interrupt_state();

	// configure ADC
	ADC_config();

	counter = 0;

	__enable_interrupt();

	while(counter < FFT_SAMPLES);
	
	// disable interrupt for (only) MEM0
	ADC12IER0 &= ~ADC12IE0;


	// turn off the ADC to save energy
	ADC12CTL0 &= ~ADC12ON;
  
	// restore interrupt state
	__set_interrupt_state(is);

 	uint8_t i;
	for (i = 0; i < FFT_SAMPLES; i++)
	{
    	GV(pers_sdata[i]) = sampled_input[i];
	}
  	P3OUT &= ~BIT5;
	TRANSITION_TO(task_fft_mic);
}

void task_fft_mic()
{
  P3OUT |= BIT5;
  msp_status status;

  uint8_t i;
  for (i = 0; i < FFT_SAMPLES; i++)
  {
    tb_fftd[i] = GV(pers_sdata[i]);
  }
  //  /* Configure parameters for FFT */
  msp_fft_q15_params fftParams;
  msp_abs_q15_params absParams;
  msp_max_q15_params maxParams;

  fftParams.length = FFT_SAMPLES;
  fftParams.bitReverse = 1;
  fftParams.twiddleTable = NULL;
#if !defined(MSP_USE_LEA)
  fftParams.twiddleTable = msp_cmplx_twiddle_table_128_q15;
#else
  fftParams.twiddleTable = NULL;
#endif

  absParams.length = FFT_SAMPLES;

  maxParams.length = FFT_SAMPLES;

  uint16_t shift = 0;
  uint16_t max_index = 0;

  /* Perform FFT */
  status = msp_fft_auto_q15(&fftParams, tb_fftd, &shift);

  /* Remove DC component  */
  tb_fftd[0] = 0;

  /* Compute absolute value of FFT */
  status = msp_abs_q15(&absParams, tb_fftd, tb_fftd);
  //msp_checkStatus(status);

  /* Get peak frequency */
  status = msp_max_q15(&maxParams, tb_fftd, NULL, &max_index); 

  P3OUT &= ~BIT5;

  //Flag completion of this "Thread"
  P3OUT |= BIT6;
  P3OUT &= ~BIT6;
  
  P1IE |= BIT2;                              // P1.2 interrupt enabled

 TRANSITION_TO(task_schedule);
}
/**
 * ADC Interrupt handler
 */

_interrupt(ADC12_VECTOR)
{
    switch(__even_in_range(ADC12IV,12))
    {
    case 12:                                // Vector 12:  ADC12BMEM0 Interrupt
        if (counter < FFT_SAMPLES)
            // Read ADC12MEM0 value
            sampled_input[counter++] = ADC12MEM0;
        else {
            // disable ADC conversion and disable interrupt request for MEM0
            ADC12CTL0 &= ~ADC12ENC;
            ADC12IER0 &= ~ADC12IE0;
        }
        break;
    default: break;
    }
}

DSPLIB_DATA(accel_input, MSP_ALIGN_FFT_Q15(N_SAMPLES))
 _q15 accel_input[N_SAMPLES];

uint8_t acc_data[NUM_BYTES_RX];

void task_sample_accel()
{

  
  P3OUT |= BIT0;
  
  setup_mcu();

  i2c_init();
  
  i2c_write(ADXL_345 , ADXL_CONF_REG , 0x00);
  i2c_write(ADXL_345, ADXL_CONF_REG, 0x10);
  i2c_write(ADXL_345, ADXL_CONF_REG, 0x08);

    
  //get samples 
  uint8_t collected = 0;
  uint16_t z;
  while(N_SAMPLES-collected){

        i2c_read_multi(ADXL_345, READ_REG, NUM_BYTES_RX, &acc_data);
        z = (((int16_t)acc_data[5]) << 8) | acc_data[4];
        GV(data_array[collected++]) = z;
  }

  P3OUT &= ~BIT0;
 
  TRANSITION_TO(task_fft_accel);

}

void task_fft_accel()
{


  P3OUT |= BIT0;
  msp_status status;

  uint8_t i;
  for (i = 0; i < N_SAMPLES; i++)
  {
    accel_input[i] = GV(data_array[i]);
  }
  //  /* Configure parameters for FFT */
  msp_fft_q15_params fftParams;
  msp_abs_q15_params absParams;
  msp_max_q15_params maxParams;

  fftParams.length = N_SAMPLES;
  fftParams.bitReverse = 1;
  fftParams.twiddleTable = msp_cmplx_twiddle_table_128_q15;

  absParams.length = N_SAMPLES;

  maxParams.length = N_SAMPLES;

  uint16_t max_index = 0;

  /* Perform FFT */
  status = msp_fft_fixed_q15(&fftParams, accel_input);

  /* Remove DC component  */
  accel_input[0] = 0;

  /* Compute absolute value of FFT */
  status = msp_abs_q15(&absParams, accel_input, accel_input);

  /* Get peak frequency */
  status = msp_max_q15(&maxParams, accel_input, NULL, &max_index); 

  P3OUT &= ~BIT0;

  //Initialize the AR-app 
  //AR test app is used here for an estimation 
  //of a high computation application 
  TRANSITION_TO(task_init);
}

//Dummy data sampling
void ACCEL_singleSample_(threeAxis_t_8* result){
   
  i2c_init();

  i2c_write(ADXL_345 , ADXL_CONF_REG , 0x00);
  i2c_write(ADXL_345, ADXL_CONF_REG, 0x10);
  i2c_write(ADXL_345, ADXL_CONF_REG, 0x08);

  i2c_read_multi(ADXL_345, READ_REG, NUM_BYTES_RX, &acc_data);
  
  result->x = (((int16_t)acc_data[1]) << 8) | acc_data[0];//(__GET(_v_seed)*17)%85;
  result->y = (((int16_t)acc_data[3]) << 8) | acc_data[2];//(__GET(_v_seed)*17*17)%85;
  result->z = (((int16_t)acc_data[5]) << 8) | acc_data[4];;//(__GET(_v_seed)*17*17*17)%85;
        
}

// Number of samples to discard before recording training set
#define NUM_WARMUP_SAMPLES 3

#define ACCEL_WINDOW_SIZE 3
#define MODEL_SIZE 16
#define SAMPLE_NOISE_FLOOR 10 // TODO: made up value

// Number of classifications to complete in one experiment
#define SAMPLES_TO_COLLECT 256

typedef struct 
{
    int16_t x,y,z;

}threeAxis_t_8;

typedef threeAxis_t_8 accelReading;

typedef accelReading accelWindow[ACCEL_WINDOW_SIZE];

typedef struct {
	unsigned meanmag;
	unsigned stddevmag;
} features_t;

typedef enum {
	CLASS_STATIONARY,
	CLASS_MOVING,
} class_t;


typedef enum {
	MODE_IDLE = 3,
	MODE_TRAIN_STATIONARY = 2,
	MODE_TRAIN_MOVING = 1,
	MODE_RECOGNIZE = 0, // default
} run_mode_t;


void task_init()
{
    P3OUT |= BIT0;

	GV(pinState) = MODE_IDLE;

	GV(count) = 0;
	GV(seed) = 1;
	P3OUT &= ~BIT0;
	TRANSITION_TO(task_selectMode);

}
void task_selectMode()
{
	P3OUT |= BIT0;

	uint16_t pin_state=1;
	++GV(count);
	if(GV(count) >= 3) pin_state=2;
	if(GV(count)>=5) pin_state=0;

	run_mode_t mode;
	class_t class;


	// Don't re-launch training after finishing training
	if ((pin_state == MODE_TRAIN_STATIONARY ||
				pin_state == MODE_TRAIN_MOVING) &&
			pin_state == GV(pinState)) {
		pin_state = MODE_IDLE;
	} else {
		GV(pinState) = pin_state;
	}


	switch(pin_state) {
		case MODE_TRAIN_STATIONARY:		
			GV(discardedSamplesCount) = 0;
			GV(mode) = MODE_TRAIN_STATIONARY;
			GV(class) = CLASS_STATIONARY;
			GV(samplesInWindow) = 0;

			P3OUT &= ~BIT0;
	
			TRANSITION_TO(task_warmup);
			break;

		case MODE_TRAIN_MOVING:
			GV(discardedSamplesCount) = 0;
			GV(mode) = MODE_TRAIN_MOVING;
			GV(class) = CLASS_MOVING;
			GV(samplesInWindow) = 0;

			P3OUT &= ~BIT0;
	
			TRANSITION_TO(task_warmup);
			break;

		case MODE_RECOGNIZE:
			GV(mode) = MODE_RECOGNIZE;

			P3OUT &= ~BIT0;
	
			TRANSITION_TO(task_resetStats);
			break;

		default:
			P3OUT &= ~BIT0;
	
			TRANSITION_TO(task_idle);
	}
}

void task_resetStats()
{
	P3OUT |= BIT0;

	// NOTE: could roll this into selectMode task, but no compelling reason


	// NOTE: not combined into one struct because not all code paths use both
	GV(movingCount) = 0;
	GV(stationaryCount) = 0;
	GV(totalCount) = 0;

	GV(samplesInWindow) = 0;

			P3OUT &= ~BIT0;
	TRANSITION_TO(task_sample);
}

void task_sample()
{
	 P3OUT |= BIT0;


	accelReading sample;
	ACCEL_singleSample_(&sample);
	GV(window, _global_samplesInWindow) = sample;
	++GV(samplesInWindow);
			sample.x, sample.y, sample.z, GV(samplesInWindow));


	if (GV(samplesInWindow) < ACCEL_WINDOW_SIZE) {
			P3OUT &= ~BIT0;

		TRANSITION_TO(task_sample);
	} else {
		GV(samplesInWindow) = 0;
			P3OUT &= ~BIT0;

		TRANSITION_TO(task_transform);
	}
}

void task_transform()
{
	 P3OUT |= BIT0;

	unsigned i;

	accelReading *sample;
	accelReading transformedSample;

	for (i = 0; i < ACCEL_WINDOW_SIZE; i++) {
		if (GV(window, i).x < SAMPLE_NOISE_FLOOR ||
				GV(window, i).y < SAMPLE_NOISE_FLOOR ||
				GV(window, i).z < SAMPLE_NOISE_FLOOR) {

			GV(window, i).x = (GV(window, i).x > SAMPLE_NOISE_FLOOR)
				? GV(window, i).x : 0;
			GV(window, i).y = (GV(window, i).y > SAMPLE_NOISE_FLOOR)
				? GV(window, i).y : 0;
			GV(window, i).z = (GV(window, i).z > SAMPLE_NOISE_FLOOR)
				? GV(window, i).z : 0;
		}
	}
	P3OUT &= ~BIT0;
	TRANSITION_TO(task_featurize);
}

void task_featurize()
{
	P3OUT |= BIT0;

	accelReading mean, stddev;
	mean.x = mean.y = mean.z = 0;
	stddev.x = stddev.y = stddev.z = 0;
	features_t features;


	int i;
	for (i = 0; i < ACCEL_WINDOW_SIZE; i++) {
		mean.x += GV(window, i).x;
		mean.y += GV(window, i).y;
		mean.z += GV(window, i).z;
	}
	mean.x >>= 2;
	mean.y >>= 2;
	mean.z >>= 2;

	for (i = 0; i < ACCEL_WINDOW_SIZE; i++) {
		stddev.x += GV(window, i).x > mean.x ? GV(window, i).x - mean.x
			: mean.x - GV(window, i).x;
		stddev.y += GV(window, i).y > mean.y ? GV(window, i).y - mean.y
			: mean.y - GV(window, i).y;
		stddev.z += GV(window, i).z > mean.z ? GV(window, i).z - mean.z
			: mean.z - GV(window, i).z;
	}
	stddev.x >>= 2;
	stddev.y >>= 2;
	stddev.z >>= 2;

	unsigned meanmag = mean.x*mean.x + mean.y*mean.y + mean.z*mean.z;
	unsigned stddevmag = stddev.x*stddev.x + stddev.y*stddev.y + stddev.z*stddev.z;
	features.meanmag   = sqrt(meanmag);
	features.stddevmag = sqrt(stddevmag);
			features.meanmag, features.stddevmag);

	switch (GV(mode)) {
		case MODE_TRAIN_STATIONARY:
		case MODE_TRAIN_MOVING:
			GV(features) = features;
			P3OUT &= ~BIT0;
	
			TRANSITION_TO(task_train);
			break;
		case MODE_RECOGNIZE:
			GV(features) = features;
			P3OUT &= ~BIT0;
	
			TRANSITION_TO(task_classify);
			break;
		default:
			// TODO: abort
			break;
	}
}

void task_classify() {
	
	P3OUT |= BIT0;
int move_less_error = 0;
	int stat_less_error = 0;
	int i;
	class_t class;
	long int meanmag;
	long int stddevmag;
	meanmag = GV(features).meanmag;
	stddevmag = GV(features).stddevmag;

	for (i = 0; i < MODEL_SIZE; ++i) {
		long int stat_mean_err = (GV(model_stationary, i).meanmag > meanmag)
			? (GV(model_stationary, i).meanmag - meanmag)
			: (meanmag - GV(model_stationary, i).meanmag);

		long int stat_sd_err = (GV(model_stationary, i).stddevmag > stddevmag)
			? (GV(model_stationary, i).stddevmag - stddevmag)
			: (stddevmag - GV(model_stationary, i).stddevmag);

		long int move_mean_err = (GV(model_moving, i).meanmag > meanmag)
			? (GV(model_moving, i).meanmag - meanmag)
			: (meanmag - GV(model_moving, i).meanmag);

		long int move_sd_err = (GV(model_moving, i).stddevmag > stddevmag)
			? (GV(model_moving, i).stddevmag - stddevmag)
			: (stddevmag - GV(model_moving, i).stddevmag);
		if (move_mean_err < stat_mean_err) {
			move_less_error++;
		} else {
			stat_less_error++;
		}

		if (move_sd_err < stat_sd_err) {
			move_less_error++;
		} else {
			stat_less_error++;
		}
	}

	GV(class) = (move_less_error > stat_less_error) ? CLASS_MOVING : CLASS_STATIONARY;


	P3OUT &= ~BIT0;
	TRANSITION_TO(task_stats);
}

void task_stats()
{
	P3OUT |= BIT0;

	unsigned movingCount = 0, stationaryCount = 0;


	++GV(totalCount);

	switch (GV(class)) {
		case CLASS_MOVING:

			++GV(movingCount);
			break;
		case CLASS_STATIONARY:

			++GV(stationaryCount);
			break;
	}

	if (GV(totalCount) == SAMPLES_TO_COLLECT) {

		unsigned resultStationaryPct = GV(stationaryCount) * 100 / GV(totalCount);
		unsigned resultMovingPct = GV(movingCount) * 100 / GV(totalCount);

		unsigned sum = GV(stationaryCount) + GV(movingCount);
		       GV(stationaryCount), resultStationaryPct,
		       GV(movingCount), resultMovingPct,
		       GV(totalCount), sum, sum == GV(totalCount) ? 'V' : 'X');
		P3OUT &= ~BIT0;

		TRANSITION_TO(task_idle);
	} else {
		P3OUT &= ~BIT0;

		TRANSITION_TO(task_sample);
	}
}

void task_warmup()
{
	P3OUT |= BIT0;

	threeAxis_t_8 sample;

	if (GV(discardedSamplesCount) < NUM_WARMUP_SAMPLES) {

		ACCEL_singleSample_(&sample);
		++GV(discardedSamplesCount);
			P3OUT &= ~BIT0;

		TRANSITION_TO(task_warmup);
	} else {
		GV(trainingSetSize) = 0;
		P3OUT &= ~BIT0;

		TRANSITION_TO(task_sample);
	}
}

void task_train()
{
	P3OUT |= BIT0;

	unsigned trainingSetSize;;
	unsigned class;

	switch (GV(class)) {
		case CLASS_STATIONARY:
			GV(model_stationary, _global_trainingSetSize) = GV(features);
			break;
		case CLASS_MOVING:
			GV(model_moving, _global_trainingSetSize) = GV(features);
			break;
	}

	++GV(trainingSetSize);

	if (GV(trainingSetSize) < MODEL_SIZE) {
		P3OUT &= ~BIT0;

		TRANSITION_TO(task_sample);
	} else {
		//        PRINTF("train: class %u done (mn %u sd %u)\r\n",
		//               class, features.meanmag, features.stddevmag);
		P3OUT &= ~BIT0;

		TRANSITION_TO(task_idle);
	}
}

void task_idle() {

  	P3OUT |= BIT0;

	//flag the completion of "thread2"
	P3OUT |= BIT4;
  	P3OUT &=~BIT4;

	P1IE |= BIT4;                               // P1.4 interrupt enabled
	
	P3OUT &= ~BIT0;
	TRANSITION_TO(task_schedule);
}

INIT_FUNC(init)
ENTRY_TASK(task_schedule)
