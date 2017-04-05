//
//  tcs.h
//  dstat-firmware
//
//  Created by Michael Dryden on 2015-09-25.
//  Copyright © 2015 wheeler lab. All rights reserved.
//

#ifndef tcs_h
#define tcs_h

#include <stdio.h>
//#include <stdbool.h>

#define TCS_TWI TWIC

void tcs_init(void);
void tcs_readvalues(uint16_t data[4]);


#endif /* tcs_h */
