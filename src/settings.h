//
//  settings.h
//  dstat-firmware
//
//  Created by Michael Dryden on 2015-10-14.
//  Copyright © 2015 wheeler lab. All rights reserved.
//

#ifndef settings_h
#define settings_h

#include <stdio.h>

// From module: NVM - Non Volatile Memory
#include <nvm.h>

// From module: NVM - Non volatile memory access
#include <common_nvm.h>

struct settings_list { //Make sure this doesn't exceed 32 bytes
    uint8_t programmed;
    int16_t max5443_offset;
    uint8_t tcs_enabled;
    uint16_t tcs_clear_threshold;
    int16_t r100_trim;
    int16_t r3k_trim;
    int16_t r30k_trim;
    int16_t r300k_trim;
    int16_t r3M_trim;
    int16_t r30M_trim;
    int16_t r100M_trim; //20 bytes
};

union {
    struct settings_list settings;
    char temp_char[EEPROM_PAGE_SIZE]; //makes sure struct fills whole page
} settings;

void settings_handler(char command);
void settings_read_eeprom(void);
void settings_write_eeprom(void);
void settings_restore_defaults(void);

#endif /* settings_h */
