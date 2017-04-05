// //------ usartC0 interrupt mode test code
// // in baud rate setting, using cpu-2MHz
// #include <avr/io.h>
// #include <avr/interrupt.h>
// #include "usartC0.h"
//
// volatile unsigned char c;
//
// ISR(USARTC0_RXC_vect)
// {
// 	  c = usartRx();
//     usartTx(c);
//   //usartTx('B');
// }
//
// int main (void)
// {
// 		usartInit();
//     usartFlush();
//     usartInterruptInit();
//
//     while (1){
//       // c = usartRx();
//       // usartTx(c);
//       //usartTx('A');
// 		}
//
//     return 0;
// }
// //------ end

// //------ usartC0 polling mode test code
// //in baud setting, using cpu-2MHz
// #include <avr/io.h>
// #include "usartC0.h"
//
// volatile unsigned char c;
//
// int main (void)
// {
//
//     usartInit();
//
//     while (1){
//        c = usartRx();
//        usartTx(c);
//       // usartTx('A');
// 		}
//
//     return 0;
// }
// //------ end

/*
 * main.c
 *
 * Created: 29/09/2012 2:13:52 AM
 *  Author: mdryden
 */


#include "experiment.h"
#include "asf.h"
#include "settings.h"
#include "tcs.h"
#include "shutter.h"
#include <string.h>
#include <math.h>
#include <stdint.h>
#include "conf_board.h"

//--- usartC0 polling
#include <avr/io.h>
#include <stdio.h>
#include "usartC0.h" /* in usartC0.h, set USBFLAG for usb or BLE connections */

//volatile unsigned char c;
//---
//
// //--- usartC0 interrupt
// #include <avr/io.h>
// //#include <avr/interrupt.h>
// #include "usartC0.h"

volatile unsigned char c;
//
// ISR(USARTC0_RXC_vect)
// {
// 	  c = usartRx();
//     //usartTx(c);
// 	  //printf("hi!\n");
//   //usartTx('B');
// }
//---

//Internal function declarations
int8_t command_handler(char command);

int8_t command_handler(char command){
	/**
	 * Deals with commands over USB
	 *
	 * Calls functions in
	 * @param command Command character input.
	 */

	switch (command){
        case 'E': //Experiment options
            experiment_handler(getchar());
            break;

        case 'S': //Settings options
            settings_handler(getchar());
            break;

        case 'T': ;
            uint16_t tcs_data[] = {0,0,0,0};
            if (settings.settings.tcs_enabled == 0){
                printf("T-1.-1.-1.-1\n\r");
            }
            else{
                tcs_readvalues(tcs_data);
                printf("#INFO: TCS—%u %u %u %u\n\r", tcs_data[0], tcs_data[1], tcs_data[2], tcs_data[3]);
                printf("T%u.%u.%u.%u\n\r", tcs_data[0], tcs_data[1], tcs_data[2], tcs_data[3]);
            }
            break;

        case 'R': //Restart USB
            udc_detach();
            delay_ms(100);
            udc_attach();
            break;

		case 'V': //check version
			printf("V%u.%u\n\r", BOARD_VER_MAJOR, BOARD_VER_MINOR);
			break;

		default:
			printf("#ERR: Command %c not recognized\n\r", command);
			return 1;
	}
	printf("no\n\r");
	return 0;
}

int main(void){
  //--- usartC0 poll
	  usartInit();
		usartFlush();
  //---

  //---usartC0 interrupt
	// if (stdioFlag  == 'A')
	// {
	//   usartInit();
	// }

  // usartFlush();
  // usartInterruptInit();
  // //---

  irq_initialize_vectors();
  cpu_irq_enable();

	board_init();
    #if BOARD_VER_MAJOR >= 1 && BOARD_VER_MINOR >= 2 && BOARD_VER_MICRO >=3
        ioport_set_pin_dir(LED1, IOPORT_DIR_OUTPUT);
        ioport_set_pin_dir(LED2, IOPORT_DIR_OUTPUT);
    #endif
	pot_init();
	pmic_init();

	sysclk_init(); //Disables ALL peripheral clocks D:
  //--- usartC0
  sysclk_enable_peripheral_clock(&USARTC0); //enable clocks for usartC0
  //---
	rtc_init();
  sleepmgr_init();
	sysclk_enable_module(SYSCLK_PORT_GEN, SYSCLK_EVSYS);

	pmic_set_scheduling(PMIC_SCH_ROUND_ROBIN);

	stdio_usb_init();

	ads1255_init_pins();
	ads1255_init_module();

	PORTD.INT0MASK = PIN5_bm;
	PORTD.INT1MASK = PIN5_bm;
	PORTD.INTCTRL = PORT_INT0LVL_OFF_gc | PORT_INT1LVL_OFF_gc;

	max5443_init_pins();
	max5443_init_module();

	ads1255_wakeup();
	ads1255_rdatac();
	ads1255_standby();

  ads1255_setup(ADS_BUFF_ON,ADS_DR_60,ADS_PGA_2);

  autogain_enable = 0;
  g_gain = POT_GAIN_30k;
  printf("Welcome!\r\n");
	//printf("%c",stdioFlag);
	printf("===\r\n");

  pot_set_gain();

  settings_read_eeprom();


  tcs_init();
  shutter_init();

  delay_s(1);
  udc_detach();
  delay_ms(100);
  udc_attach();
  stdio_usb_enable();

	program_loop:
        #if BOARD_VER_MAJOR >= 1 && BOARD_VER_MINOR >= 2 && BOARD_VER_MICRO >=3
            ioport_set_pin_level(LED1, 1);
        #endif
		while(getchar() != '!');
		printf ("C\r\n");
        #if BOARD_VER_MAJOR >= 1 && BOARD_VER_MINOR >= 2 && BOARD_VER_MICRO >=3
            ioport_set_pin_level(LED1, 0);
        #endif
		command_handler(getchar());
	goto program_loop;
  //--- usartC0 poll
	// printf("welcome!\n");
  // while (1){
  //   c = usartRx();
  //   usartTx(c);
  // }
  //---
  //--- usartC0 interrupt
  // while (1){
  //   // c = usartRx();
  //   // usartTx(c);
  // }
  // //---
}
