//
//  shutter.h
//  dstat-firmware
//
//  Created by Michael Dryden on 2016-01-27.
//  Copyright © 2016 wheeler lab. All rights reserved.
//

#ifndef shutter_h
#define shutter_h

#include <stdio.h>
#include <ioport.h>

#define SHUTTER_PIN     IOPORT_CREATE_PIN(PORTE,1)
#define SHUTTER_TC      TCE0

void shutter_init(void);
void shutter_open(void);
void shutter_close(void);
uint8_t shutter_cont(double freq);
void shutter_cont_stop(void);


#endif /* shutter_h */
