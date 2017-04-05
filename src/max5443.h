/*
 * max5443.h
 *
 * Created: 06/04/2012 7:30:13 PM
 *  Author: mdryden
 */ 


#ifndef MAX5443_H_
#define MAX5443_H_

#include <stdint.h>
#include <compiler.h>

void max5443_init_pins(void);
void max5443_init_module(void);
void max5443_set_voltage1(uint16_t dacindex);

//void max5443_set_voltage_8bit(uint8_t dacindex);

#endif /* MAX5443_H_ */