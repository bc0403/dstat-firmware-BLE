/*
 * experiment.c
 *
 * Created: 01/10/2012 10:59:48 PM
 *  Author: mdryden
 */ 
/**
 * @file experiment.c
 * @author  Michael DM Dryden <mdryden@chem.utoronto.ca>
 * @version 1.0
 *
 *
 * @section DESCRIPTION
 *
 * Contains Functions for performing experiments and adjusting potentiostat settings other than DAC and ADC.
 */

#include "experiment.h"
#include "settings.h"
#include "tcs.h"
#include "config/conf_board.h"
#include "shutter.h"

//Public variable definitions
uint16_t g_gain = POT_GAIN_30k;
uint8_t g_short = 0;
uint8_t autogain_enable = 1;

//Private variables
volatile int32_t voltage = 0;
volatile uint16_t dacindex = 0;
uint16_t dacindex_stop = 0;
volatile int8_t up = 1;
volatile uint16_t iter = 0;
uint16_t* eis_ptr = 0;
volatile uint16_t cycles = 0;
volatile uint16_t samples = 0;
volatile uint16_t tcf0period = 0;
uint32_t skip_samples = 0;

//Private function declarations
//uint16_t set_timer_period(uint32_t period, volatile void *tc);
static void precond_rtc_callback(uint32_t time);
static void porte_int0_lsv(void);
static void tcf0_ovf_callback(void);
static void tce1_ovf_callback_lsv(void);
static void lsv_cca_callback(void);
static void ca_cca_callback(void);
static void portd_int0_ca(void);
static uint8_t _swv_singledir(uint16_t dacindex, uint16_t dacindex_stop, uint16_t dacindex_pulse_height, uint16_t dacindex_step, uint8_t direction);
static uint8_t _dpv_singledir(uint16_t dacindex, uint16_t dacindex_stop, uint16_t dacindex_pulse_height, uint16_t dacindex_step, uint8_t direction);
static void pmt_idle(void);

//interrupt callback setup
typedef void (*port_callback_t) (void);

static port_callback_t portd_int0_callback;
static port_callback_t portd_int1_callback;

void experiment_handler(char command){
    static int16_t p1, p2, p3;
    static uint16_t u1, u2, u3, u4;
    static uint8_t p5, o1, o2, o3;
    static int16_t pcv1, pcv2;
    static uint16_t pct1, pct2;
    uint16_t tcs_data[] = {0,0,0,0};
    uint16_t tcs_data1[] = {0,0,0,0};
    double p6;

    switch (command){
        case 'A': //ADS Buffer/rate/PGA values from ads1255.h
            scanf("%hhx%hhx%hhx",&o1,&o2,&o3);
            printf("#A: %x %x %x\r\n",o1,o2,o3);
            ads1255_setup(o1, o2, o3);
            break;
            
        case 'G': //Gain
            scanf("%u%hhu",&g_gain, &g_short);
            printf("#G: %u %u\r\n", g_gain, g_short);
            pot_set_gain(); //uses global g_gain, so no params
            break;
            
        case 'L': //LSV - start, stop, slope
            scanf("%u%u%i%i%i%i%u",&pct1,&pct2,&pcv1,&pcv2,&p1,&p2,&u1);
            precond(pcv1,pct1,pcv2,pct2);
            lsv_experiment(p1,p2,u1,2);
            break;
            
        case 'C': //CV - v1, v2, start, scans, slope
            scanf("%u%u%i%i%i%i%i%hhu%u",&pct1,&pct2,&pcv1,&pcv2,&p1,&p2,&p3,&p5,&u1);
            precond(pcv1,pct1,pcv2,pct2);
            cv_experiment(p1,p2,p3,p5,u1);
            break;
            
        case 'S': //SWV - start, stop, step size, pulse_height, frequency, scans
            scanf("%u%u%i%i%i%i%u%u%u%u",&pct1,&pct2,&pcv1,&pcv2,&p1,&p2,&u1,&u2,&u3,&u4);
            precond(pcv1,pct1,pcv2,pct2);
            swv_experiment(p1,p2,u1,u2,u3,u4);
            break;
            
        case 'D': //DPV - start, stop, step size, pulse_height, period, width
            scanf("%u%u%i%i%i%i%u%u%u%u",&pct1,&pct2,&pcv1,&pcv2,&p1,&p2,&u1,&u2,&u3,&u4);
            precond(pcv1,pct1,pcv2,pct2);
            dpv_experiment(p1,p2,u1,u2,u3,u4);
            break;
            
        case 'M': //PMT idle mode - holds voltage at 0 V with no data output
            pmt_idle();
            break;
            
        #if BOARD_VER_MAJOR == 1 && BOARD_VER_MINOR >= 2
        case 'P': //potentiometry - time, OCP/poteniometry
            scanf("%u%hhu",&pct1, &o1);
            pot_experiment(pct1, o1);
            break;
        #endif
            
        case 'R': //CA - steps, step_dac[], step_seconds[]
            scanf("%hhu",&p5); //get number of steps
            printf("#INFO: Steps: %u\n\r", p5);
            
            //allocate arrays for steps
            uint16_t * step_dac = malloc(p5*sizeof(uint16_t));
            uint16_t * step_seconds = malloc(p5*sizeof(uint16_t));
            
            //check for successful allocation
            if (!step_dac || !step_seconds){
                printf("#ERR: Could not allocate memory\n\r");
                break;
            }
            
            uint8_t i;
            
            for (i=0; i<p5; i++){
                scanf("%u", &step_dac[i]); // get voltage steps (dac units)
                printf("#INFO: DAC: %u\n\r", step_dac[i]);
            }
            
            for (i=0; i<p5; i++){
                scanf("%u", &step_seconds[i]); //get step durations (seconds)
                printf("#INFO: Time: %u\n\r", step_seconds[i]);
            }
            
            scanf("%hhu", &o1);
            printf("#INFO: TCS_check: %hhu\n\r", o1);
            if (o1 > 0) {
                if (settings.settings.tcs_enabled > 0){
                    tcs_readvalues(tcs_data);
                    tcs_readvalues(tcs_data1); // If sensor disconnected, second measurement should be exactly the same (unless 0 or saturated)
                    printf("#INFO: TCS0—%u %u %u %u\n\r", tcs_data[0], tcs_data[1], tcs_data[2], tcs_data[3]);
                    printf("#INFO: TCS1—%u %u %u %u\n\r", tcs_data1[0], tcs_data1[1], tcs_data1[2], tcs_data1[3]);
                    if (tcs_data[0] == tcs_data1[0]){
                        if (!(tcs_data[0] == 0 || tcs_data[0] == 65535)) {
                            printf("#ERR: Ambient light sensor seems to be disconnected \n\r");
                            return;
                        }
                    }
                    if (tcs_data[0] > settings.settings.tcs_clear_threshold){
                        printf("#ERR: Ambient light exceeds threshold %u\n\r", tcs_data[0]);
                        printf("#INFO: TCS—%u %u %u %u\n\r", tcs_data[0], tcs_data[1], tcs_data[2], tcs_data[3]);
                        return;
                    }
                }
                else {
                    printf("#ERR: Ambient light sensor disabled.\n\r");
                    return;
                }
            }
            
            ca_experiment(p5, step_dac, step_seconds);
            
            //free arrays
            free(step_dac);
            free(step_seconds);
            
            break;
            
        case 'Z': //Shutter sync
            scanf("%lg",&p6);
            shutter_cont(p6);
            break;
            
        case 'z':
            shutter_cont_stop();
            break;
            
        case '1':
            shutter_close();
            break;
            
        case '2':
            shutter_open();
            break;
            
        default:
            printf("#ERR: Command %c not recognized\n\r", command);
    }
}


uint16_t set_timer_period(uint32_t period, volatile void *tc)
{
	/**
	 * Sets a suitable timer source and sets period for a 16-bit timer
	 *
	 * @param period 32-bit period in CPU cycles
	 * @param *tc pointer to timer to set
	 * @return divider used.
	 */
	uint16_t temp_div = ceil((double)period/65536);
	uint16_t divider = 0;
	
	if (temp_div == 1)
	tc_write_clock_source(tc, TC_CLKSEL_DIV1_gc);
	else if (temp_div == 2){
		tc_write_clock_source(tc, TC_CLKSEL_DIV2_gc);
		divider = 2;
	}
	else if (temp_div <= 4){
		tc_write_clock_source(tc, TC_CLKSEL_DIV4_gc);
		divider = 4;
	}
	else if (temp_div <= 8){
		tc_write_clock_source(tc,TC_CLKSEL_DIV8_gc);
		divider = 8;
	}
	else if (temp_div <= 64){
		tc_write_clock_source(tc,TC_CLKSEL_DIV64_gc);
		divider = 64;
	}
	else if (temp_div <= 256){
		tc_write_clock_source(tc,TC_CLKSEL_DIV256_gc);
		divider = 256;
	}
	else if (temp_div <= 1024){
		tc_write_clock_source(tc,TC_CLKSEL_DIV1024_gc);
		divider = 1024;
	}
	else{
		printf("#Frequency/ADC rate is too low\n\r");
		return 0;
	}
	
	period /= divider;
	tc_write_period(tc, (uint16_t)period);
	return divider;
}

void pot_init(void){
	/**
	 * Initializes AVR port directions and levels
	 *
	 * @return Nothing.
	 */
	#if BOARD_VER_MAJOR == 1 && BOARD_VER_MINOR == 1
		ioport_set_port_dir(IOPORT_PORTB, PIN3_bm|PIN4_bm|PIN5_bm|PIN6_bm|PIN7_bm, IOPORT_DIR_OUTPUT);
		ioport_set_port_dir(IOPORT_PORTD, PIN4_bm, IOPORT_DIR_OUTPUT);
		ioport_set_port_level(IOPORT_PORTB, PIN3_bm|PIN4_bm|PIN5_bm|PIN6_bm|PIN7_bm, PIN3_bm|PIN6_bm|PIN7_bm);
		ioport_set_port_level(IOPORT_PORTD, PIN4_bm, PIN4_bm);
	#endif
	#if BOARD_VER_MAJOR == 1 && BOARD_VER_MINOR == 2
		ioport_set_port_dir(IOPORT_PORTB, PIN2_bm|PIN3_bm|PIN4_bm|PIN5_bm|PIN6_bm|PIN7_bm, IOPORT_DIR_OUTPUT);
		ioport_set_port_dir(IOPORT_PORTD, PIN4_bm, IOPORT_DIR_OUTPUT);
		ioport_set_port_level(IOPORT_PORTB, PIN2_bm|PIN3_bm|PIN4_bm|PIN5_bm|PIN6_bm|PIN7_bm, PIN3_bm|PIN6_bm|PIN7_bm);
		ioport_set_port_level(IOPORT_PORTD, PIN4_bm, PIN4_bm);
	#endif
}

void pot_set_gain(void){
	/**
	 * Sets iV gain according to current g_gain value
	 *
	 * @return Nothing.
	 */
	switch (g_gain){
		#if BOARD_VER_MAJOR == 1 && BOARD_VER_MINOR == 1
			case POT_GAIN_500M:
				ioport_set_port_level(IOPORT_PORTB, PIN6_bm|PIN7_bm, 0);
				ioport_set_port_level(IOPORT_PORTD, PIN4_bm, 0);
		
				printf("#INFO: 500M\n\r");
				break;
		
			case POT_GAIN_30M:
				ioport_set_port_level(IOPORT_PORTB, PIN6_bm|PIN7_bm, PIN6_bm);
				ioport_set_port_level(IOPORT_PORTD, PIN4_bm, 0);
				printf("#INFO: 30M\n\r");
				break;
		
			case POT_GAIN_3M:	
				ioport_set_port_level(IOPORT_PORTB, PIN6_bm|PIN7_bm, PIN7_bm);
				ioport_set_port_level(IOPORT_PORTD, PIN4_bm, 0);
				printf("#INFO: 3M\n\r");
				break;
		
			case POT_GAIN_300k:
				ioport_set_port_level(IOPORT_PORTB, PIN6_bm|PIN7_bm, PIN6_bm|PIN7_bm);
				ioport_set_port_level(IOPORT_PORTD, PIN4_bm, 0);
				printf("#INFO: 300k\n\r");
				break;
		
			case POT_GAIN_30k:
				ioport_set_port_level(IOPORT_PORTB, PIN6_bm|PIN7_bm, 0);
				ioport_set_port_level(IOPORT_PORTD, PIN4_bm, PIN4_bm);
				printf("#INFO: 30k\n\r");
				break;
		
			case POT_GAIN_3k:
				ioport_set_port_level(IOPORT_PORTB, PIN6_bm|PIN7_bm, PIN6_bm);
				ioport_set_port_level(IOPORT_PORTD, PIN4_bm, PIN4_bm);
				printf("#INFO: 3k\n\r");
				break;
		
			case POT_GAIN_300:
				ioport_set_port_level(IOPORT_PORTB, PIN6_bm|PIN7_bm, PIN7_bm);
				ioport_set_port_level(IOPORT_PORTD, PIN4_bm, PIN4_bm);
				printf("#INFO: 300\n\r");
				break;
		
			case POT_GAIN_100:
				ioport_set_port_level(IOPORT_PORTB, PIN6_bm|PIN7_bm, PIN6_bm|PIN7_bm);
				ioport_set_port_level(IOPORT_PORTD, PIN4_bm, PIN4_bm);
				printf("#INFO: 100\n\r");
				break;
		#endif
		
		#if BOARD_VER_MAJOR == 1 && BOARD_VER_MINOR == 2
			case POT_GAIN_100M:
				ioport_set_port_level(IOPORT_PORTB, PIN6_bm|PIN7_bm, 0);
				ioport_set_port_level(IOPORT_PORTD, PIN4_bm, 0);
				printf("#INFO: 100M\n\r");
				break;
				
			case POT_GAIN_30M:
				ioport_set_port_level(IOPORT_PORTB, PIN6_bm|PIN7_bm, PIN6_bm);
				ioport_set_port_level(IOPORT_PORTD, PIN4_bm, 0);
				printf("#INFO: 30M\n\r");
				break;
				
			case POT_GAIN_3M:
				ioport_set_port_level(IOPORT_PORTB, PIN6_bm|PIN7_bm, PIN7_bm);
				ioport_set_port_level(IOPORT_PORTD, PIN4_bm, 0);
				printf("#INFO: 3M\n\r");
				break;
				
			case POT_GAIN_300k:
				ioport_set_port_level(IOPORT_PORTB, PIN6_bm|PIN7_bm, PIN6_bm|PIN7_bm);
				ioport_set_port_level(IOPORT_PORTD, PIN4_bm, 0);
				printf("#INFO: 300k\n\r");
				break;
				
			case POT_GAIN_30k:
				ioport_set_port_level(IOPORT_PORTB, PIN6_bm|PIN7_bm, 0);
				ioport_set_port_level(IOPORT_PORTD, PIN4_bm, PIN4_bm);
				printf("#INFO: 30k\n\r");
				break;
				
			case POT_GAIN_3k:
				ioport_set_port_level(IOPORT_PORTB, PIN6_bm|PIN7_bm, PIN6_bm);
				ioport_set_port_level(IOPORT_PORTD, PIN4_bm, PIN4_bm);
				printf("#INFO: 3k\n\r");
				break;
				
			case POT_GAIN_0:
				ioport_set_port_level(IOPORT_PORTB, PIN6_bm|PIN7_bm, PIN7_bm);
				ioport_set_port_level(IOPORT_PORTD, PIN4_bm, PIN4_bm);
				printf("#INFO: 0\n\r");
				break;
				
			case POT_GAIN_100:
				ioport_set_port_level(IOPORT_PORTB, PIN6_bm|PIN7_bm, PIN6_bm|PIN7_bm);
				ioport_set_port_level(IOPORT_PORTD, PIN4_bm, PIN4_bm);
				printf("#INFO: 100\n\r");
				break;
		#endif
		
			default:
				printf("#WAR: Invalid pot gain.\n\r");
				break;
            
		return;
	}
}

void volt_exp_start(void){
	/**
	 * Connects measurement cell to rest of circuit.
	 */
	#if BOARD_VER_MAJOR == 1 && BOARD_VER_MINOR == 1
		ioport_set_port_level(IOPORT_PORTB, PIN3_bm|PIN4_bm|PIN5_bm, PIN3_bm|PIN4_bm);
	#endif
	
	#if BOARD_VER_MAJOR == 1 && BOARD_VER_MINOR == 2
        ioport_set_port_level(IOPORT_PORTB, PIN2_bm|PIN3_bm|PIN4_bm|PIN5_bm, PIN2_bm|PIN4_bm);
	#endif
    
    delay_ms(100); // Make sure WE circuit is connected before control voltage applied
    ioport_set_pin_level(PIN_POT_CE, 1);
	
    if (g_short == 1)
        ioport_set_pin_level(PIN_POT_2ELECTRODE, 1);
    else
        ioport_set_pin_level(PIN_POT_2ELECTRODE, 0);
}

void volt_exp_stop(void){
	/**
	 * Disconnects measurement cell.
	 */
	#if BOARD_VER_MAJOR == 1 && BOARD_VER_MINOR == 1
		ioport_set_port_level(IOPORT_PORTB, PIN3_bm|PIN5_bm, 0);
	#endif
	#if BOARD_VER_MAJOR == 1 && BOARD_VER_MINOR == 2
		ioport_set_port_level(IOPORT_PORTB, PIN2_bm|PIN3_bm|PIN5_bm, 0);
	#endif
    
    delay_ms(100);  // Make sure WE is last to disconnect
    ioport_set_pin_level(PIN_POT_WE, 0);
    
    ioport_set_pin_level(PIN_POT_2ELECTRODE, 1);
}

#if BOARD_VER_MAJOR == 1 && BOARD_VER_MINOR >= 2
void pot_exp_start(void){
	/**
	 * All switches open.
	 */
	ioport_set_port_level(IOPORT_PORTB, PIN2_bm|PIN3_bm|PIN4_bm|PIN5_bm, 0);

}

void ocp_exp_start(void){
	/**
	 * U3C closed
	 */
	ioport_set_port_level(IOPORT_PORTB, PIN2_bm|PIN3_bm|PIN4_bm|PIN5_bm, PIN4_bm);
}
#endif

void precond(int16_t v1, uint16_t t1, int16_t v2, uint16_t t2){ //assumes potentiostat switches are already set
	/**
	 * Performs experiment preconditioning.
	 *
	 * @param v1 First potential (DAC index).
	 * @param t1 First duration (s).
	 * @param v2 Second potential (DAC index).
	 * @param t2 Second duration (s).
	 */
	
	uint16_t time_old = 0;
	
	while (RTC.STATUS & RTC_SYNCBUSY_bm);
	RTC.PER = 65535;
	while (RTC.STATUS & RTC_SYNCBUSY_bm);
	RTC.CTRL = RTC_PRESCALER_DIV1024_gc; //1s tick
	rtc_set_callback((rtc_callback_t)precond_rtc_callback);
	
	up = 1;
	
	//first potential
	if (t1 > 0){
		max5443_set_voltage1(v1);
		rtc_set_alarm(t1);
		RTC.CNT = 0;
		volt_exp_start();
		while (up){
			if (udi_cdc_is_rx_ready()){
				if (getchar() == 'a'){
					precond_rtc_callback(t1);
					printf("##ABORT\n\r");
					goto aborting;
				}
			}
			if (time_old != RTC.CNT){
				time_old = RTC.CNT;
				printf("#%u\n\r",time_old);
			}
		}
	}
	
	up = 1;
	time_old = 0;
	
	if (t2 > 0){
		max5443_set_voltage1(v2);
		rtc_set_alarm(t2);
		RTC.CNT = 0;
		volt_exp_start();
		while (up){
			if (udi_cdc_is_rx_ready()){
				if (getchar() == 'a'){
					precond_rtc_callback(t2);
					printf("##ABORT\n\r");
					goto aborting;
				}
			}
			if (time_old != RTC.CNT){
				time_old = RTC.CNT;
				printf("#%u\n\r",time_old);
			}
		}
	}
	
	aborting:
		volt_exp_stop();
		return;
}

static void precond_rtc_callback(uint32_t time){
	up = 0;
	RTC.INTCTRL |= RTC_COMPINTLVL_OFF_gc;
}

void cv_experiment(int16_t v1, int16_t v2, int16_t start, uint8_t scans, uint16_t slope){
	/**
	 * Perform a CV experiment.
	 *
	 * Calls lsv_experiment several times to make a CV experiment.
	 * @param v1 Vertex 1 in mV.
	 * @param v2 Vertex 2 in mV.
	 * @param start Start voltage in mV.
	 * @param scans Number of scans.
	 * @param slope Scan rate in mV/s.
	 */
	// check if start is [v1,v2]
    
	int8_t firstrun = 1;
	
	if((start < v1 && start < v2) || (start > v1 && start > v2)){
		printf("#ERR: Start must be within [v1, v2]\n\r");
		return;
	}
	
	while(scans > 0){
		if (start != v1){
			if (lsv_experiment(start,v1,slope,firstrun) == 1)
				return;
			firstrun = 0;
		}
        
		if (start == v2 && scans == 1)
			firstrun = -1;
        
		if (lsv_experiment(v1,v2,slope,firstrun) == 1)
			return;
        
		if (scans == 1)
			firstrun = -1;
        
		if (start != v2)
			if(lsv_experiment(v2,start,slope,firstrun) == 1)
				return;
        
		--scans;
		firstrun = 0;
        
        printf("S\n\r"); //signal end of scan
	}
	
	printf("D\n\r"); //signal end of experiment
	
	return;
}

uint8_t lsv_experiment(int16_t start, int16_t stop, uint16_t slope, int8_t first_run){
	/**
	 * Perform a LSV experiment.
	 *
	 * Uses porte_int0_lsv to output to USB.
	 * @param start Start potential in mV.
	 * @param stop Stop potential in mV.
	 * @param slope Scan rate in mV/s.
	 * @param first_run Keeps track of number of scans so potentiostat isn't initialized twice or disconnected when doing CV. Set to 2 for normal LSV.
	 */
	
	uint8_t ret = 0;
	
	//check experiment limits
	if(start<-1500 || start>=1500 ||start==stop|| stop<-1500 || stop>=1500 || slope>7000)
	{
		printf("#ERR: Experiment parameters outside limits\n\r");
		return ret;
	}
	
	uint16_t dacindex_start = ceil(start*(65536/(double)3000)+32768);
	dacindex_stop = ceil(stop*(65536/(double)3000)+32768);
	uint32_t timer_period;
	uint16_t temp_div;


	max5443_set_voltage1(dacindex_start);
	
	if (first_run == 1 || first_run == 2){
		volt_exp_start();
		ads1255_rdatac();
		ads1255_sync();

		tc_enable(&EXP_TC0_0);
        tc_enable(&EXP_TC1_0);
		tc_set_overflow_interrupt_callback(&EXP_TC0_0, tcf0_ovf_callback);
		tc_set_overflow_interrupt_callback(&EXP_TC1_0, tce1_ovf_callback_lsv);
		tc_set_cca_interrupt_callback(&EXP_TC1_0, lsv_cca_callback);
		portd_int0_callback = porte_int0_lsv; //ADC read

		//set EVCH0 event
		EVSYS.CH0MUX = EVSYS_CHMUX_TCC0_OVF_gc;
		EVSYS.CH0CTRL = 0;

		timer_period = ceil(1/((double)slope/(3000./65536))*(F_CPU));
		temp_div = ceil(timer_period/65536.);
		
		if (temp_div <= 1)
			tc_write_clock_source(&TCC0,TC_CLKSEL_DIV1_gc);
		else if (temp_div == 2){
			tc_write_clock_source(&TCC0,TC_CLKSEL_DIV2_gc);
			timer_period /= 2;
		}
		else if (temp_div <= 4){
			tc_write_clock_source(&TCC0,TC_CLKSEL_DIV4_gc);
			timer_period /= 4;
		}
		else if (temp_div <= 8){
			tc_write_clock_source(&TCC0,TC_CLKSEL_DIV8_gc);
			timer_period /= 8;
		}
		else if (temp_div <= 64){
			tc_write_clock_source(&TCC0,TC_CLKSEL_DIV64_gc);
			timer_period /= 64;
		}
		else if (temp_div <= 256){
			tc_write_clock_source(&TCC0,TC_CLKSEL_DIV256_gc);
			timer_period /= 256;
		}
		else if (temp_div <= 1024){
			tc_write_clock_source(&TCC0,TC_CLKSEL_DIV1024_gc);
			timer_period /= 1024;
		}
		else{
			printf("ERR: Frequency/ADC rate is too low\n\r");
			return ret;
		}

		ads1255_wakeup();
		tc_write_period(&EXP_TC1_0, 0xffff);
		tc_write_period(&EXP_TC0_0, (uint16_t)timer_period);

 	}
	
	EXP_TC1_0.CNT = dacindex_start;
	
	if (stop > start)
	{
		up = 1;
		tc_set_direction(&EXP_TC1_0, TC_UP);
	}
	else
	{
		up = -1;
		tc_set_direction(&EXP_TC1_0, TC_DOWN);
	}
	
	tc_write_cc(&EXP_TC1_0, TC_CCA, dacindex_stop);
	tc_enable_cc_channels(&EXP_TC1_0, TC_CCAEN);
	EXP_TC0_0.CNT = 0;
	
	tc_set_cca_interrupt_level(&EXP_TC1_0, TC_INT_LVL_HI); //Stop experiment
	tc_set_overflow_interrupt_level(&EXP_TC0_0, TC_OVFINTLVL_LO_gc); //Set DAC
	PORTD.INTCTRL = PORT_INT0LVL_MED_gc; //ADC read
	
	tc_write_clock_source(&EXP_TC1_0, TC_CLKSEL_EVCH0_gc);
	
	//Experiment run with interrupts
	while (up != 0){
		if (udi_cdc_is_rx_ready()){
			if (getchar()=='a'){
				tce1_ovf_callback_lsv();
				ret = 1;
				goto aborting;
				
			}
		}
	}
	
	if (first_run == -1 || first_run == 2)
	{
		aborting:
			tc_disable(&EXP_TC0_0);
			EXP_TC0_0.CNT = 0x0;
			tc_disable(&EXP_TC1_0);
			volt_exp_stop();
			ads1255_standby();
			return ret;
	}
	
	return ret;
}

static void porte_int0_lsv(void){
	/**
	 * ISR for taking LSV measurements.
	 */
	struct
	{
		uint16_t index;
		int32_t result;
	} data;
	
 	data.result = ads1255_read_fast24();

	static uint16_t last_value = 0;
	uint32_t current = EXP_TC1_0.CNT;
	data.index = (current+last_value)>>1; //DAC value is average of current and last timer - approximation of center of averaging window
	
	printf("B\n");
	udi_cdc_write_buf(&data, 6);
	last_value = (uint16_t)current;
	printf("\n");
	
	return;
}

static void tcf0_ovf_callback(void){
	max5443_set_voltage1(EXP_TC1_0.CNT);
}

static void tce1_ovf_callback_lsv(void){
	PORTD.INTCTRL = PORT_INT0LVL_OFF_gc;
	tc_set_overflow_interrupt_level(&EXP_TC0_0, TC_OVFINTLVL_OFF_gc);
	tc_set_overflow_interrupt_level(&EXP_TC1_0, TC_OVFINTLVL_OFF_gc);
	up = 0;
	return;
}

static void lsv_cca_callback(void){
	PORTD.INTCTRL = PORT_INT0LVL_OFF_gc;
	tc_set_overflow_interrupt_level(&EXP_TC0_0, TC_OVFINTLVL_OFF_gc);
	tc_set_cca_interrupt_level(&EXP_TC1_0, TC_INT_LVL_OFF);
	up = 0;
	return;
}

#if BOARD_VER_MAJOR == 1 && BOARD_VER_MINOR >= 2

void pot_experiment(uint16_t time_seconds, uint8_t exp_type){
	/**
	 * Performs a potentiometry experiment.
	 *
	 * @param time_seconds Time until automatic stop. If 0, can only be canceled by abort signal.
	 * @param exp_type Type of experiment, POT_OCP for OCP, POT_POTENT for potentiometry
	 */
	while (RTC.STATUS & RTC_SYNCBUSY_bm);
	RTC.PER = 999;
	while (RTC.STATUS & RTC_SYNCBUSY_bm);
	RTC.CTRL = RTC_PRESCALER_DIV1_gc; //1ms tick
	RTC.CNT = 0;
	
	EVSYS.CH0MUX = EVSYS_CHMUX_RTC_OVF_gc; //EV CH0 -- RTC overflow 1s
	
	portd_int0_callback = portd_int0_ca; //ADC interrupt
	
	tc_enable(&EXP_TC0_0);
	
	ads1255_mux(ADS_MUX_POT);
	ads1255_rdatac();
	ads1255_wakeup();
	
	tc_write_period(&EXP_TC0_0,0xffff);
	tc_write_clock_source(&EXP_TC0_0, TC_CLKSEL_EVCH0_gc);
	tc_set_direction(&EXP_TC0_0, TC_UP);
	
	up = 1;
	if (time_seconds >= 1){ //only enable interrupt if non-zero timeout specified
		tc_set_cca_interrupt_callback(&EXP_TC0_0, ca_cca_callback);
		tc_write_cc(&EXP_TC0_0, TC_CCA, time_seconds-1);
		tc_enable_cc_channels(&EXP_TC0_0, TC_CCAEN);
		tc_clear_cc_interrupt(&EXP_TC0_0, TC_CCA);
		tc_set_cca_interrupt_level(&EXP_TC0_0, TC_INT_LVL_MED);
	}
	
	if (exp_type == POT_OCP)
	{
		ocp_exp_start();
	}
	else if (exp_type == POT_POTENT)
	{
		pot_exp_start();
	}
		
	RTC.CNT=0;
	PORTD.INTCTRL = PORT_INT0LVL_LO_gc;
	EXP_TC0_0.CNT = 0;
	
	while (up !=0){
		if (udi_cdc_is_rx_ready()){
			if (getchar() == 'a'){
				ca_cca_callback();
				printf("##ABORT\n\r");
				goto aborting;
			}
		}
	}
	
	aborting:
		tc_set_cca_interrupt_level(&EXP_TC0_0, TC_INT_LVL_OFF);
		tc_write_clock_source(&EXP_TC0_0, TC_CLKSEL_OFF_gc);
		tc_disable(&EXP_TC0_0);
		volt_exp_stop();
		ads1255_standby();

		return;
}
#endif

void pmt_idle(void){
    max5443_set_voltage1(65535);
    ioport_set_pin_level(PIN_POT_2ELECTRODE, 1);
    delay_ms(100);
    ioport_set_pin_level(PIN_POT_CE, 1);
    
    while (1){
        if (udi_cdc_is_rx_ready()){
            if (getchar() == 'a'){
                printf("##ABORT\n\r");
                // Doesn't disconnect control line to minimize time this is floating
                return;
            }
        }
    }
}

void ca_experiment(uint16_t steps, uint16_t step_dac[], uint16_t step_seconds[]){
	/**
	 * Performs a chronoamperometry experiment.
	 *
	 * @param steps Total number of steps.
	 * @param step_dac[] Array containing DAC indices.
	 * @param step_seconds[] Array containing step durations in seconds.
	 */
	while (RTC.STATUS & RTC_SYNCBUSY_bm);
	RTC.PER = 999;
	while (RTC.STATUS & RTC_SYNCBUSY_bm);
	RTC.CTRL = RTC_PRESCALER_DIV1_gc; //1ms tick
	RTC.CNT = 0;
	
	EVSYS.CH0MUX = EVSYS_CHMUX_RTC_OVF_gc; //EV CH0 -- RTC overflow 1s
	
	portd_int0_callback = portd_int0_ca; //ADC interrupt
	
	tc_enable(&EXP_TC0_0);
	tc_set_cca_interrupt_callback(&EXP_TC0_0, ca_cca_callback);
	
	ads1255_rdatac();
	ads1255_wakeup();
	
	tc_write_period(&EXP_TC0_0,0xffff);
	tc_write_clock_source(&EXP_TC0_0, TC_CLKSEL_EVCH0_gc);
	tc_set_direction(&EXP_TC0_0, TC_UP);
	tc_enable_cc_channels(&EXP_TC0_0, TC_CCAEN);
	tc_set_cca_interrupt_level(&EXP_TC0_0, TC_INT_LVL_MED);
	
	EXP_TC0_0.CNT = 0;
	
	max5443_set_voltage1(step_dac[0]);
	volt_exp_start();
	
	for (uint8_t i = 0; i < steps; ++i)
	{
		up = 1;
		tc_write_cc(&EXP_TC0_0, TC_CCA, EXP_TC0_0.CNT+step_seconds[i]-1);
		RTC.CNT=0;
		max5443_set_voltage1(step_dac[i]);
		printf("#DAC: %u\n\r", step_dac[i]);
		PORTD.INTCTRL = PORT_INT0LVL_LO_gc;
		while (up !=0){
			if (udi_cdc_is_rx_ready()){
				if (getchar() == 'a'){
					ca_cca_callback();
					printf("##ABORT\n\r");
					goto aborting;
				}
			}
		}
	}
	
	aborting:
		tc_set_cca_interrupt_level(&EXP_TC0_0, TC_INT_LVL_OFF);
		tc_write_clock_source(&EXP_TC0_0, TC_CLKSEL_OFF_gc);
		tc_disable(&EXP_TC0_0);
		volt_exp_stop();
		ads1255_standby();

		return;
}

static void portd_int0_ca(void){
	struct
	{
		uint16_t time1;
		uint16_t time2;
		int32_t current;
	} data;
	
	data.time1 = EXP_TC0_0.CNT;
	data.time2 = RTC.CNT;
	data.current = ads1255_read_fast24();

	printf("B\n");
	udi_cdc_write_buf(&data, 8);
	printf("\n");
}

static void ca_cca_callback(void){
	/**
	 * Interrupt handler for CA. Triggers when counter matches CC to stop potential step.
	 *
	 */
	PORTD.INTCTRL = PORT_INT0LVL_OFF_gc;
	up = 0;
	return;
}

void swv_experiment(int16_t start, int16_t stop, uint16_t step, uint16_t pulse_height, uint16_t frequency, uint16_t scans){
	/**
	 * Perform a SWV experiment
	 *
	 * @param start Start voltage in mV.
	 * @param stop Stop voltage in mV.
	 * @param step Step voltage in mV.
	 * @param pulse_height Pulse amplitude in mV.
	 * @param frequency Frequency in Hz.
	 * @param scans Number of scans (0 for single direction mode)
	 */

	uint8_t direction;
	uint16_t dacindex_start = ceil((start)*(65536/(double)3000))+32768;
	uint16_t dacindex_stop = ceil(stop*(65536/(double)3000))+32768;
	uint16_t dacindex_step = ceil(step*(65536/(double)3000));
	uint16_t dacindex_pulse_height = ceil(pulse_height*(65536/(double)3000));
	uint32_t period;
	
	if (start < stop)
		direction = 1;
	else
		direction = 0;
	
	tc_enable(&EXP_TC0_1);
	tc_enable(&EXP_TC0_0);
	
	frequency *= 2; //compensate for half-period triggers
	
	//calculate time to ADC trigger
	period = ceil((1/(double)frequency)*F_CPU);
	uint32_t adc_period = ceil(((1/(double)frequency)-(double)(sample_delay_ms_100div/1e5))*F_CPU);
	set_timer_period(period, &EXP_TC0_1);
	set_timer_period(adc_period, &EXP_TC0_0);
	
	ads1255_wakeup();
	ads1255_standby();
	
	volt_exp_start();
	
	do{
		EXP_TC0_1.CNT = 0;
		EXP_TC0_0.CNT = 0;
	
		if (_swv_singledir(dacindex_start, dacindex_stop, dacindex_pulse_height, dacindex_step, direction))
			goto aborting; //function will return non-zero if abort called over USB
		
		if (scans > 0){ //non-cyclic mode skips out after one direction
			EXP_TC0_1.CNT = 0;
			EXP_TC0_0.CNT = 0;
			if (_swv_singledir(dacindex_stop, dacindex_start, dacindex_pulse_height, dacindex_step, !direction)) //swap start and stop and invert direction for second half of scan
				goto aborting;
		}
		
		printf("S\n\r"); //signal end of scan
		
	} while (scans-- > 1); //will underflow after comparison for scans = 0 , but shouldn't matter 
	
	printf("D\n\r"); //signal end of experiment
	
	aborting:
		volt_exp_stop();
		tc_write_clock_source(&EXP_TC0_1, TC_CLKSEL_OFF_gc);
		tc_disable(&EXP_TC0_1);
		EXP_TC0_1.CNT = 0;
		tc_write_clock_source(&EXP_TC0_0, TC_CLKSEL_OFF_gc);
		tc_disable(&EXP_TC0_0);
		EXP_TC0_0.CNT = 0;
		ads1255_standby();
	
		return;
}

uint8_t _swv_singledir (uint16_t dacindex, uint16_t dacindex_stop, uint16_t dacindex_pulse_height, uint16_t dacindex_step, uint8_t direction){
	/**
	 * Internal function that performs a single direction sweep for SWV
	 *
	 * @param dacindex Starting voltage as dac index
	 * @param dacindex_stop Stop voltage in dac index.
	 * @param dacindex_step Step voltage in dac index.
	 * @param dacindex_pulse_height Pulse amplitude in dac index.
	 * @param direction Scan direction - 1 for up, 0 for down.
	 */
	int32_t forward = 0;
	int32_t reverse = 0;
	uint16_t lastindex = 0;
	
	if (direction == 1)
		max5443_set_voltage1(dacindex+dacindex_pulse_height);
	else
		max5443_set_voltage1(dacindex-dacindex_pulse_height);
	
	while ((dacindex <= dacindex_stop && direction == 1) || (dacindex >= dacindex_stop && direction == 0)){
		tc_clear_overflow(&EXP_TC0_1);
		tc_clear_overflow(&EXP_TC0_0);
				
		while (!tc_is_overflow(&EXP_TC0_0)){ //ADC tc overflow
			if (udi_cdc_is_rx_ready()){ //check for abort signal over USB
				if (getchar() == 'a')
					return 1;
			}
		}
		
		ads1255_wakeup();
		while (ioport_pin_is_high(IOPORT_CREATE_PIN(PORTD, 5)));
		forward = ads1255_read_single24();
		ads1255_standby();
		
		while (!tc_is_overflow(&EXP_TC0_1)); //wait for end of half-cycle
		EXP_TC0_0.CNT = 0;
		
		if (direction == 1) //switch voltage to other half of cycle
			max5443_set_voltage1(dacindex-dacindex_pulse_height);
		else
			max5443_set_voltage1(dacindex+dacindex_pulse_height);

		tc_clear_overflow(&EXP_TC0_1); //reset timer OVF
		tc_clear_overflow(&EXP_TC0_0);
				
		while (!tc_is_overflow(&EXP_TC0_0)){ //ADC tc overflow
			if (udi_cdc_is_rx_ready()){ //check for abort signal over USB
				if (getchar() == 'a')
				return 1;
			}
		}
		
		ads1255_wakeup();
		while (ioport_pin_is_high(IOPORT_CREATE_PIN(PORTD, 5)));
		reverse = ads1255_read_single24();
		ads1255_standby();
		
		while (!tc_is_overflow(&EXP_TC0_1)); //wait for end of half-cycle
		EXP_TC0_0.CNT = 0;
			
		lastindex = dacindex;
			
		//increment dacindex
		if (direction == 1){
			dacindex += dacindex_step;
			max5443_set_voltage1(dacindex+dacindex_pulse_height);
		}
		
		else{
			dacindex -= dacindex_step;
			max5443_set_voltage1(dacindex-dacindex_pulse_height);
		}
			
		//data output
		struct
		{
			uint16_t lastindex;
			int32_t forward;
			int32_t	reverse;
		} data;
		
		data.lastindex = lastindex;
		data.forward = forward;
		data.reverse = reverse;
		
		printf("B\n");
		udi_cdc_write_buf(&data, 10);
		printf("\n");
	}
	
	return 0;
}

void dpv_experiment(int16_t start, int16_t stop, uint16_t step, uint16_t pulse_height, uint16_t pulse_period, uint16_t pulse_width){
	/**
	 * Perform a DPV experiment
	 *
	 * @param start Start voltage in mV.
	 * @param stop Stop voltage in mV.
	 * @param step Step voltage in mV.
	 * @param pulse_height Pulse amplitude in mV.
	 * @param pulse_period Pulse period in ms.
	 * @param pulse_width Pulse width in ms.
	 */

	uint8_t direction;
	uint16_t dacindex_start = ceil((start)*(65536/(double)3000))+32768;
	uint16_t dacindex_stop = ceil(stop*(65536/(double)3000))+32768;
	uint16_t dacindex_step = ceil(step*(65536/(double)3000));
	uint16_t dacindex_pulse_height = ceil(pulse_height*(65536/(double)3000));
	uint32_t cpu_period;
	uint32_t cpu_width;
	
	if (start < stop)
		direction = 1;
	else
		direction = 0;
	
	tc_enable(&EXP_TC0_1);
	tc_enable(&EXP_TC0_0);
	
	//calculate time to ADC trigger
	cpu_period = ceil((double)pulse_period*1e-3*F_CPU);
	uint32_t adc_period = ceil((((double)pulse_period*1e-3)-(double)(sample_delay_ms_100div/1e5))*F_CPU);
	uint16_t divider = set_timer_period(cpu_period, &EXP_TC0_1);
	uint16_t adc_divider = set_timer_period(adc_period, &EXP_TC0_0);
	
	cpu_width = (double)pulse_width*1e-3*F_CPU;
	uint32_t adc_width = ceil((((double)pulse_width*1e-3)-(double)(sample_delay_ms_100div/1e5))*F_CPU);
	tc_write_cc(&EXP_TC0_1, TC_CCA, (uint16_t)(cpu_width/divider));
	tc_enable_cc_channels(&EXP_TC0_1, TC_CCAEN);
	tc_write_cc(&EXP_TC0_0, TC_CCA, (uint16_t)(adc_width/adc_divider));
	tc_enable_cc_channels(&EXP_TC0_0, TC_CCAEN);
	
	ads1255_wakeup();
	ads1255_standby();

	volt_exp_start();
	
	EXP_TC0_1.CNT = 0;
	EXP_TC0_0.CNT = 0;
	
	if (_dpv_singledir(dacindex_start, dacindex_stop, dacindex_pulse_height, dacindex_step, direction))
		goto aborting; //function will return non-zero if abort called over USB
	
	printf("D\n\r"); //signal end of experiment
	
	aborting:
		volt_exp_stop();
		tc_write_clock_source(&EXP_TC0_1, TC_CLKSEL_OFF_gc);
		tc_disable(&EXP_TC0_1);
		tc_write_clock_source(&EXP_TC0_0, TC_CLKSEL_OFF_gc);
		tc_disable(&EXP_TC0_0);
		EXP_TC0_1.CNT = 0;
		EXP_TC0_0.CNT = 0;
		ads1255_standby();
	
		return;
}

uint8_t _dpv_singledir (uint16_t dacindex, uint16_t dacindex_stop, uint16_t dacindex_pulse_height, uint16_t dacindex_step, uint8_t direction){
	/**
	 * Internal function that performs a single direction sweep for DPV
	 *
	 * @param dacindex Starting voltage as dac index
	 * @param dacindex_stop Stop voltage in dac index.
	 * @param dacindex_step Step voltage in dac index.
	 * @param dacindex_pulse_height Pulse amplitude in dac index.
	 * @param direction Scan direction - 1 for up, 0 for down.
	 */
	int32_t forward = 0;
	int32_t reverse = 0;
	uint16_t lastindex = 0;
	
	if (direction == 1)
		max5443_set_voltage1(dacindex+dacindex_pulse_height);
	else
		max5443_set_voltage1(dacindex-dacindex_pulse_height);
	
	while ((dacindex <= dacindex_stop && direction == 1) || (dacindex >= dacindex_stop && direction == 0)){
		tc_clear_overflow(&EXP_TC0_1);
		tc_clear_cc_interrupt(&EXP_TC0_1, TC_CCA);
		tc_clear_overflow(&EXP_TC0_0);
		tc_clear_cc_interrupt(&EXP_TC0_0, TC_CCA);
				
		while (!tc_is_cc_interrupt(&EXP_TC0_0, TC_CCA)){ //wait until ADC TC CCA match
			if (udi_cdc_is_rx_ready()){ //check for abort signal over USB
				if (getchar() == 'a')
					return 1;
			}
		}
		
		ads1255_wakeup();
		while (ioport_pin_is_high(IOPORT_CREATE_PIN(PORTD, 5)));
		forward = ads1255_read_single24();
		ads1255_standby();
		
		while (!tc_is_cc_interrupt(&EXP_TC0_1, TC_CCA)); //wait for end of half-cycle
		
		//switch voltage to baseline
		max5443_set_voltage1(dacindex);
				
		while (!tc_is_overflow(&EXP_TC0_0)){ //wait for ADC TC overflow
			if (udi_cdc_is_rx_ready()){
				if (getchar() == 'a')
					return 1;
			}
		}
		
		ads1255_wakeup();
		while (ioport_pin_is_high(IOPORT_CREATE_PIN(PORTD, 5)));
		reverse = ads1255_read_single24();
		ads1255_standby();
		
		while (!tc_is_overflow(&EXP_TC0_1)); //wait for end of half-cycle
		EXP_TC0_0.CNT = 0; // Resync ADC TC
		
		lastindex = dacindex;
			
		//increment dacindex
		if (direction == 1){
			dacindex += dacindex_step;
			max5443_set_voltage1(dacindex+dacindex_pulse_height);
		}
		
		else{
			dacindex -= dacindex_step;
			max5443_set_voltage1(dacindex-dacindex_pulse_height);
		}
			
		//data output
		struct
		{
			uint16_t lastindex;
			int32_t forward;
			int32_t	reverse;
		} data;
		
		data.lastindex = lastindex;
		data.forward = forward;
		data.reverse = reverse;
		
		printf("B\n");
		udi_cdc_write_buf(&data, 10);
		printf("\n");
	}
	
	return 0;
}

ISR(PORTD_INT0_vect){
	if (portd_int0_callback) {
		portd_int0_callback();
	}
}