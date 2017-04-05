/*
 * ads1255.h
 *
 * Created: 05/03/2012 12:19:33 AM
 *  Author: mdryden
 */ 


#ifndef ADS1255_H_
#define ADS1255_H_

#include <stdint.h>
#include <usart_spi.h>
#include <compiler.h>
#include <ioport.h>
#include <delay.h>
#include <stdio_usb.h>

//#define ADS1255_DBG

#define ADS_WAKEUP 0x0
#define ADS_RDATA 0x01
#define ADS_RDATAC 0x03
#define ADS_SDATAC 0x0f
#define ADS_SELFCAL 0xf0
#define ADS_SELFOCAL 0xf1
#define ADS_SELFGCAL 0xf2
#define ADS_SYSOCAL 0xf3
#define ADS_SYSGCAL 0xf4
#define ADS_SYNC 0xfc
#define ADS_STANDBY 0xfd
#define ADS_RESET 0xfe

#define ADS_DR_2_5 0b00000011 //0x03
#define ADS_DR_5 0b00010011 //0x13
#define ADS_DR_10 0b00100011 //0x23
#define ADS_DR_15 0b00110011 //0x33
#define ADS_DR_25 0b01000011 //0x43
#define ADS_DR_30 0b01010011 //0x53
#define ADS_DR_50 0b01100011 //0x63
#define ADS_DR_60 0b01110010 //0x72
#define ADS_DR_100 0b10000010 //0x82
#define ADS_DR_500 0b10010010 //0x92
#define ADS_DR_1000 0b10100001 //0xA1
#define ADS_DR_2000 0b10110000 //0xB0
#define ADS_DR_3750 0b11000000 //0xC0
#define ADS_DR_7500 0b11010000 //0xD0
#define ADS_DR_15000 0b11100000 //0xE0
#define ADS_DR_30000 0b11110000 //0xF0

#define ADS_PGA_1 0b000 //0x0
#define ADS_PGA_2 0b001 //0x1
#define ADS_PGA_4 0b010 //0x2
#define ADS_PGA_8 0b011 //0x3
#define ADS_PGA_16 0b100 //0x4
#define ADS_PGA_32 0b101 //0x5
#define ADS_PGA_64 0b110 //0x6

#define ADS_BUFF_OFF 0b0000 //0x0
#define ADS_BUFF_ON 0b0010 //0x2

#define ADS_MUX_VOLT 0x08
#define ADS_MUX_POT 0x18

extern uint16_t sample_delay_ms_100div;

void ads1255_sync(void);
void ads1255_init_pins(void);
void ads1255_init_module(void);
int16_t ads1255_read_fast(void);
int16_t ads1255_read_fast_single(void);
int32_t ads1255_read_fast24(void);
int32_t ads1255_read_single24(void);
void ads1255_reg_read(uint8_t address);
void ads1255_reset(void);
void ads1255_setup(uint8_t buff, uint8_t rate, uint8_t pga);
void ads1255_mux(uint8_t channel);
void ads1255_standby(void);
void ads1255_wakeup(void);
void ads1255_rdatac(void);



#endif /* ADS1255_H_ */

