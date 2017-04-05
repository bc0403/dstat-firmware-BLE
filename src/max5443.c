/*
 * max5443.c
 *
 * Created: 06/04/2012 7:29:47 PM
 *  Author: mdryden
 */ 

#include <max5443.h>
#include <stdint.h>
#include <usart_spi.h>
#include <compiler.h>
#include <ioport.h>
#include <delay.h>
#include <stdio_usb.h>
#include "settings.h"


struct usart_spi_device spi_device_conf_c = {
          .id = IOPORT_CREATE_PIN(PORTC, 4)
      };

void max5443_init_pins(void){
	ioport_set_port_dir(IOPORT_PORTC, PIN4_bm|PIN5_bm|PIN7_bm, IOPORT_DIR_OUTPUT);
	ioport_set_port_level(IOPORT_PORTC, PIN4_bm|PIN5_bm|PIN7_bm, PIN4_bm|PIN5_bm|PIN7_bm);
}

void max5443_init_module(void){
	usart_spi_init(&USARTC1);
	usart_spi_setup_device(&USARTC1, &spi_device_conf_c, SPI_MODE_0, 24000000UL, 0);
}

void max5443_set_voltage1(uint16_t dacindex){
	static union{
		uint8_t ui8[2];
		uint16_t ui16;
	} buffer;
		
	if (settings.settings.max5443_offset < 0)
	{
		settings.settings.max5443_offset = abs(settings.settings.max5443_offset);
		if (((uint16_t)settings.settings.max5443_offset) > dacindex)
			buffer.ui16 = 0;
		else
			buffer.ui16 = dacindex - (uint16_t)settings.settings.max5443_offset;
	}
	else if (((uint16_t)settings.settings.max5443_offset) > (65535 - dacindex))
		buffer.ui16 = 65535;
	else
		buffer.ui16 = dacindex + (uint16_t)settings.settings.max5443_offset;
	
 	irqflags_t flags;
    flags = cpu_irq_save();

	usart_spi_select_device(&USARTC1, &spi_device_conf_c);
	usart_spi_write_single(&USARTC1, buffer.ui8[1]);
	usart_spi_write_single(&USARTC1, buffer.ui8[0]);
	usart_spi_deselect_device(&USARTC1, &spi_device_conf_c);
	
	cpu_irq_restore(flags);
	
	return;
}
