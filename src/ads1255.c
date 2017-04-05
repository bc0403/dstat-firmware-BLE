/*
 * ads1255.c
 *
 * Created: 04/04/2012 2:13:47 PM
 *  Author: mdryden
 */ 


#include <ads1255.h>

uint8_t buffer_iter = 0;
uint16_t sample_delay_ms_100div = 0;

struct usart_spi_device spi_device_conf = {
          .id = IOPORT_CREATE_PIN(PORTE, 4)
      };

void ads1255_init_pins(void)
   {
	   arch_ioport_set_pin_dir(IOPORT_CREATE_PIN(PORTE,6), IOPORT_DIR_INPUT);
	   arch_ioport_set_port_dir(IOPORT_PORTE, PIN4_bm|PIN5_bm|PIN7_bm, IOPORT_DIR_OUTPUT);
	   arch_ioport_set_port_level(IOPORT_PORTE, PIN4_bm|PIN5_bm|PIN7_bm, PIN4_bm|PIN7_bm);
	   arch_ioport_set_pin_dir(IOPORT_CREATE_PIN(PORTD,5), IOPORT_DIR_INPUT);
	   arch_ioport_set_pin_sense_mode(IOPORT_CREATE_PIN(PORTD,5), IOPORT_SENSE_FALLING);
   }

void ads1255_init_module(void){
	usart_spi_init(&USARTE1);
	usart_spi_setup_device(&USARTE1, &spi_device_conf, SPI_MODE_1, 1900000UL, 0);
  }
   
void ads1255_sync(void){
	usart_spi_select_device(&USARTE1, &spi_device_conf);
	usart_spi_transmit(&USARTE1, ADS_SYNC);
	usart_spi_deselect_device(&USARTE1, &spi_device_conf);
	return;
}

void ads1255_reg_read(uint8_t address){
	uint8_t command_buffer[2];
	command_buffer[0] = address;
	command_buffer[0] |= (1 << 4);
	command_buffer[1] = 4;
	uint8_t input_buffer[5];

	usart_spi_select_device(&USARTE1, &spi_device_conf);
	
	while (arch_ioport_get_pin_level(IOPORT_CREATE_PIN(PORTD, 5)))
	
	usart_spi_write_packet(&USARTE1, &(command_buffer[0]), 2);
	delay_us(6.5);
    usart_spi_read_packet(&USARTE1, (uint8_t*) &input_buffer, 5);
    usart_spi_deselect_device(&USARTE1, &spi_device_conf);
	
	for (int i=0;i<5;i++)
		printf("ADS1255: Register %u=%.2x\n\r",i+1,input_buffer[i]);
	
	return;
}

void ads1255_reset(){
	usart_spi_select_device(&USARTE1, &spi_device_conf);

	while (arch_ioport_get_pin_level(IOPORT_CREATE_PIN(PORTD, 5)));
	usart_spi_transmit(&USARTE1, ADS_RESET);
	
	#ifdef ADS1255_DBG
	printf("ADS1255: Sending RESET\n\r");
	printf("ADS1255: Waiting for calibration\n\r");
	#endif
	
	while (arch_ioport_get_pin_level(IOPORT_CREATE_PIN(PORTD, 5)));
	usart_spi_deselect_device(&USARTE1, &spi_device_conf);
	
	return;
}

void ads1255_setup(uint8_t buff, uint8_t rate, uint8_t pga){
	#if BOARD_VER_MAJOR == 1 && BOARD_VER_MINOR == 1
		uint8_t command_buffer[6] = {0x50,0x03,buff,0x01,pga,rate}; // write reg 0, write 4 registers, analog buffer, MUX AIN0-AIN1, pga, rate
	#endif
	
	#if BOARD_VER_MAJOR == 1 && BOARD_VER_MINOR == 2
		uint8_t command_buffer[6] = {0x50,0x03,buff,0x08,pga,rate}; // write reg 0, write 4 registers, analog buffer, MUX AIN0-AINCOM, pga, rate	
	#endif
	
	//Stores propagation delay of current ADC sample rate in sample_delay_ms_100div in .01 ms
	
	switch (rate)
	{
	case ADS_DR_2_5:
		sample_delay_ms_100div = 40022;
		break;
	case ADS_DR_5:
		sample_delay_ms_100div = 20022;
		break;
	case ADS_DR_10:
		sample_delay_ms_100div = 10022;
		break;
	case ADS_DR_15:
		sample_delay_ms_100div = 6688;
		break;
	case ADS_DR_25:
		sample_delay_ms_100div = 4022;
		break;
	case ADS_DR_30:
		sample_delay_ms_100div = 3355;
		break;
	case ADS_DR_50:
		sample_delay_ms_100div = 2022;
		break;
	case ADS_DR_60:
		sample_delay_ms_100div = 1688;
		break;
	case ADS_DR_100:
		sample_delay_ms_100div = 1022;
		break;
	case ADS_DR_500:
		sample_delay_ms_100div = 222;
		break;
	case ADS_DR_1000:
		sample_delay_ms_100div = 122;
		break;
	case ADS_DR_2000:
		sample_delay_ms_100div = 72;
		break;
	case ADS_DR_3750:
		sample_delay_ms_100div = 48;
		break;
	case ADS_DR_7500:
		sample_delay_ms_100div = 35;
		break;
	case ADS_DR_15000:
		sample_delay_ms_100div = 29;
		break;
	case ADS_DR_30000:
		sample_delay_ms_100div = 25;
		break;
	default:
		printf("#ERR: Invalid ADC data rate specified.\n\r");
		break;
	}
	
		
	usart_spi_select_device(&USARTE1, &spi_device_conf);
	usart_spi_transmit(&USARTE1, ADS_SDATAC);
	usart_spi_write_packet(&USARTE1, (uint8_t*)&command_buffer, 6);
	usart_spi_transmit(&USARTE1, ADS_SYNC);
	usart_spi_transmit(&USARTE1, ADS_SELFCAL);

	while (arch_ioport_get_pin_level(IOPORT_CREATE_PIN(PORTD, 5)))
		;
    usart_spi_deselect_device(&USARTE1, &spi_device_conf);
	
	return;
}

void ads1255_mux(uint8_t channel){
	#if BOARD_VER_MAJOR == 1 && BOARD_VER_MINOR == 2
		uint8_t command_buffer[6] = {0x51,0x0,channel}; // write reg 1, write 1 register, MUX AIN0
			
		usart_spi_select_device(&USARTE1, &spi_device_conf);
		usart_spi_transmit(&USARTE1, ADS_SDATAC);
		usart_spi_write_packet(&USARTE1, (uint8_t*)&command_buffer, 3);
		usart_spi_transmit(&USARTE1, ADS_SYNC);
		usart_spi_transmit(&USARTE1, ADS_SELFCAL);
		while (arch_ioport_get_pin_level(IOPORT_CREATE_PIN(PORTD, 5)))
			;
		usart_spi_deselect_device(&USARTE1, &spi_device_conf);
			
		return;
	#endif
	
}

void ads1255_standby(void){
	usart_spi_select_device(&USARTE1, &spi_device_conf);
	usart_spi_transmit(&USARTE1, ADS_STANDBY);
	usart_spi_deselect_device(&USARTE1, &spi_device_conf);

	return;
}

void ads1255_wakeup(void){
	usart_spi_select_device(&USARTE1, &spi_device_conf);
	usart_spi_transmit(&USARTE1, ADS_WAKEUP);
	usart_spi_deselect_device(&USARTE1, &spi_device_conf);
	
	return;
}

void ads1255_rdatac(void){
	usart_spi_select_device(&USARTE1, &spi_device_conf);
	usart_spi_transmit(&USARTE1, ADS_RDATAC);

	while (arch_ioport_get_pin_level(IOPORT_CREATE_PIN(PORTD, 5)))
		;
	usart_spi_deselect_device(&USARTE1, &spi_device_conf);
	
	return;
}

int16_t ads1255_read_fast(void){
	union{
		uint8_t uint[2];
		int16_t int16;
	} input_buffer;

	usart_spi_select_device(&USARTE1, &spi_device_conf);
	
	for (int i = 1; i >= 0; --i){
		while (usart_data_register_is_empty(&USARTE1) == false)
			;
		usart_put(&USARTE1, CONFIG_USART_SPI_DUMMY);
		while (usart_rx_is_complete(&USARTE1) == false)
			;
		input_buffer.uint[i] = usart_get(&USARTE1);
	}
	
	while (!usart_tx_is_complete(&USARTE1))
		;
	usart_clear_tx_complete(&USARTE1);
	
	usart_spi_deselect_device(&USARTE1, &spi_device_conf);
	
	return input_buffer.int16;
}

int16_t ads1255_read_fast_single(void){
	union{
		uint8_t uint[2];
		int16_t int16;
	} input_buffer;

	usart_spi_select_device(&USARTE1, &spi_device_conf);
	usart_spi_transmit(&USARTE1, ADS_RDATA);
	delay_us(6.5);
	for (int i = 1; i >= 0; --i){
		while (usart_data_register_is_empty(&USARTE1) == false)
			;
		usart_put(&USARTE1, CONFIG_USART_SPI_DUMMY);
		while (usart_rx_is_complete(&USARTE1) == false)
			;
		input_buffer.uint[i] = usart_get(&USARTE1);
	}
	
	while (!usart_tx_is_complete(&USARTE1))
		;
	usart_clear_tx_complete(&USARTE1);
	usart_spi_deselect_device(&USARTE1, &spi_device_conf);
	
	ads1255_standby();
	
	#ifdef ADS1255_DBG
	printf("ADS1255 result=%li\n\r", input_buffer.int16);
	#endif
	
	return input_buffer.int16;
}

int32_t ads1255_read_fast24(void){

	union{
		uint8_t uint[4];
		int32_t int32;
	} input_buffer;
	
	input_buffer.int32 = 0;
	
	irqflags_t flags;
	flags = cpu_irq_save();
	
	usart_spi_select_device(&USARTE1, &spi_device_conf);
	for (int i = 2; i >= 0; --i){
		while (usart_data_register_is_empty(&USARTE1) == false)
			;
		usart_put(&USARTE1, CONFIG_USART_SPI_DUMMY);
		while (usart_rx_is_complete(&USARTE1) == false)
			;
		input_buffer.uint[i] = usart_get(&USARTE1);
	}
	
	while (!usart_tx_is_complete(&USARTE1))
		;
	usart_clear_tx_complete(&USARTE1);
	usart_spi_deselect_device(&USARTE1, &spi_device_conf);
	
	if (input_buffer.uint[2] > 0x7F)
		input_buffer.uint[3] = 0xFF;
	else
		input_buffer.uint[3] = 0x0;
	
	cpu_irq_restore(flags);
	return input_buffer.int32;
}

int32_t ads1255_read_single24(void){

	union{
		uint8_t uint[4];
		int32_t int32;
	} input_buffer;
	
	input_buffer.int32 = 0;
	
	irqflags_t flags;
	flags = cpu_irq_save();
	
	usart_spi_select_device(&USARTE1, &spi_device_conf);
	usart_spi_transmit(&USARTE1, ADS_RDATA);
	delay_us(6.5);
	
	for (int i = 2; i >= 0; --i){
		while (usart_data_register_is_empty(&USARTE1) == false)
		;
		usart_put(&USARTE1, CONFIG_USART_SPI_DUMMY);
		while (usart_rx_is_complete(&USARTE1) == false)
		;
		input_buffer.uint[i] = usart_get(&USARTE1);
	}
	
	while (!usart_tx_is_complete(&USARTE1))
	;
	usart_clear_tx_complete(&USARTE1);
	usart_spi_deselect_device(&USARTE1, &spi_device_conf);
	
	if (input_buffer.uint[2] > 0x7F)
		input_buffer.uint[3] = 0xFF;
	else
		input_buffer.uint[3] = 0x0;
	
	cpu_irq_restore(flags);
	return input_buffer.int32;
}