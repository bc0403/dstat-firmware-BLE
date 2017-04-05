//
//  shutter.c
//  dstat-firmware
//
//  Created by Michael Dryden on 2016-01-27.
//  Copyright © 2016 wheeler lab. All rights reserved.
//

#include "shutter.h"
#include <stdint.h>
#include <tc.h>
#include <math.h>

void shutter_init(void){
    ioport_set_pin_dir(SHUTTER_PIN, IOPORT_DIR_OUTPUT);
    ioport_set_pin_level(SHUTTER_PIN, 0);
}

void shutter_open(void){
    ioport_set_pin_level(SHUTTER_PIN, 1);
}

void shutter_close(void){
    ioport_set_pin_level(SHUTTER_PIN, 0);
}

uint8_t shutter_cont(double freq){
    tc_write_clock_source(&SHUTTER_TC, TC_CLKSEL_OFF_gc);
    if (freq > 30 || freq < 0.23842) {
        return 1;
    }

    else{
        tc_enable(&SHUTTER_TC);
        tc_set_wgm(&SHUTTER_TC, TC_WG_FRQ);
        tc_enable_cc_channels(&SHUTTER_TC, TC_CCBEN);
        
        tc_write_clock_source(&SHUTTER_TC, TC_CLKSEL_DIV64_gc);

        uint16_t temp_div = ceil((1/(2*freq))*F_CPU/65536);
        uint16_t divider = 0;
        
        if (temp_div <= 64){
            tc_write_clock_source(&SHUTTER_TC,TC_CLKSEL_DIV64_gc);
            divider = 64;
        }
        else if (temp_div <= 256){
            tc_write_clock_source(&SHUTTER_TC,TC_CLKSEL_DIV256_gc);
            divider = 256;
        }
        else if (temp_div <= 1024){
            tc_write_clock_source(&SHUTTER_TC,TC_CLKSEL_DIV1024_gc);
            divider = 1024;
        }
        else{
            printf("#ERR: Frequency/ADC rate is too low\n\r");
            return 0;
        }

        SHUTTER_TC.CCA = ((uint16_t)((F_CPU/divider)/(2*freq)))-1; //f=1/(2*(CCA+1)*f_clk)
    
        return 0;
    }
}

void shutter_cont_stop(void){
    tc_write_clock_source(&SHUTTER_TC, TC_CLKSEL_OFF_gc);
    tc_set_wgm(&SHUTTER_TC, TC_WG_NORMAL);
    tc_disable(&SHUTTER_TC);
    ioport_set_pin_level(SHUTTER_PIN, 0);
}