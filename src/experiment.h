/*
 * experiment.h
 *
 * Created: 01/10/2012 10:59:59 PM
 *  Author: mdryden
 */ 


#ifndef EXPERIMENT_H_
#define EXPERIMENT_H_

#include "conf_experiment.h"

#include "ads1255.h"
#include "max5443.h"
#include <tc.h>
#include <rtc.h>
#include <stdint.h>
#include <compiler.h>
#include <math.h>
#include <ioport.h>
#include <delay.h>
#include <stdio_usb.h>
#include <dma.h>

#if BOARD_VER_MAJOR == 1 && BOARD_VER_MINOR == 1
	#define POT_GAIN_100 0
	#define POT_GAIN_300 1
	#define POT_GAIN_3k 2
	#define POT_GAIN_30k 3
	#define POT_GAIN_300k 4
	#define POT_GAIN_3M 5
	#define POT_GAIN_30M 6
	#define POT_GAIN_500M 7
#endif

#if BOARD_VER_MAJOR == 1 && BOARD_VER_MINOR == 2
	#define POT_GAIN_0 0
	#define POT_GAIN_100 1
	#define POT_GAIN_3k 2
	#define POT_GAIN_30k 3
	#define POT_GAIN_300k 4
	#define POT_GAIN_3M 5
	#define POT_GAIN_30M 6
	#define POT_GAIN_100M 7
#endif

#define PIN_POT_CE              IOPORT_CREATE_PIN(PORTB,5)
#define PIN_POT_2ELECTRODE      IOPORT_CREATE_PIN(PORTB,3)
#define PIN_POT_CTRL            IOPORT_CREATE_PIN(PORTB,2)
#define PIN_POT_WE              IOPORT_CREATE_PIN(PORTB,4)

#define POT_OCP 0
#define POT_POTENT 1

#define POT_LP_OFF 0
#define POT_LP_ON 1

#define ADS_F_2_5 0
#define ADS_F_5 1
#define ADS_F_10 2
#define ADS_F_15 3
#define ADS_F_25 4
#define ADS_F_30 5
#define ADS_F_50 6
#define ADS_F_60 7
#define ADS_F_100 8
#define ADS_F_500 9
#define ADS_F_1000 10
#define ADS_F_2000 11
#define ADS_F_3750 12
#define ADS_F_7500 13
#define ADS_F_15000 14
#define ADS_F_30000 15

#define SIN_IMP_CYCLEPTS 50

#define RTC_COMPARE_INT_LEVEL RTC_COMPINTLVL_HI_gc

#define EXP_TC0_0       TCC0
#define EXP_TC1_0       TCC1
#define EXP_TC0_1       TCF0

extern uint16_t g_gain;
extern uint8_t g_short;
extern uint8_t autogain_enable;

uint16_t set_timer_period(uint32_t period, volatile void *tc);
void experiment_handler(char command);
void pot_init(void);
void pot_set_gain(void);
void volt_exp_start(void);
void volt_exp_stop(void);
void precond(int16_t v1, uint16_t t1, int16_t v2, uint16_t t2);
void cv_experiment(int16_t v1, int16_t v2, int16_t start, uint8_t scans, uint16_t slope);
uint8_t lsv_experiment(int16_t start, int16_t stop, uint16_t slope, int8_t first_run);

#if BOARD_VER_MAJOR == 1 && BOARD_VER_MINOR >= 2
void ocp_exp_start(void);
void pot_exp_start(void);
void pot_experiment(uint16_t time_seconds, uint8_t exp_type);
#endif

void ca_experiment(uint16_t steps, uint16_t step_dac[], uint16_t step_seconds[]);
void swv_experiment(int16_t start, int16_t stop, uint16_t step, uint16_t pulse_height, uint16_t frequency, uint16_t scans);
void dpv_experiment(int16_t start, int16_t stop, uint16_t step, uint16_t pulse_height, uint16_t pulse_period, uint16_t pulse_width);

#endif /* EXPERIMENT_H_ */