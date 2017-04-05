//
//  settings.c
//  dstat-firmware
//
//  Created by Michael Dryden on 2015-10-14.
//  Copyright © 2015 wheeler lab. All rights reserved.
//
//  Save/Retrieve settings from EEPROM

#include "settings.h"
#include "config/conf_board.h"
#include <udc.h>

#define SETTINGS_EEPROM_PARAM_PAGE 0
#define SETTINGS_EEPROM_OFFSET_ADDR SETTINGS_EEPROM_PARAM_PAGE * EEPROM_PAGE_SIZE

void update_firmware(void);

void settings_handler(char command){
    switch (command){
        case 'D': //Reset defaults
            settings_restore_defaults();
            break;
            
        case 'F': //Update Firmware
            update_firmware();
            break;
            
        case 'R': //Read settings from EEPROM
            settings_read_eeprom();
            break;
            
        case 'W': //Write new settings
            scanf("%i%hhu%u%i%i%i%i%i%i%i",
                  &settings.settings.max5443_offset,
                  &settings.settings.tcs_enabled,
                  &settings.settings.tcs_clear_threshold,
                  &settings.settings.r100_trim,
                  &settings.settings.r3k_trim,
                  &settings.settings.r30k_trim,
                  &settings.settings.r300k_trim,
                  &settings.settings.r3M_trim,
                  &settings.settings.r30M_trim,
                  &settings.settings.r100M_trim);
            settings_write_eeprom();
            break;
            
        default:
            printf("#ERR: Command %c not recognized\n\r", command);
    }
}

void(* start_bootloader)(void) = (void (*)(void))(BOOT_SECTION_START/2+0x1FC/2);

void update_firmware(void){
    /**
     * Jumps to bootloader to do firmware upgrade
     */
    udc_stop(); // Disable USB Stack
    cli(); // Disable Interrupts
    
    EIND = BOOT_SECTION_START>>17;
    start_bootloader();
}

void settings_read_eeprom(void){
    printf("#INFO: SETTINGS\n\r");
    nvm_eeprom_read_buffer(SETTINGS_EEPROM_OFFSET_ADDR, &settings, EEPROM_PAGE_SIZE);
    if (settings.settings.programmed != 1) {
        printf("#INFO: EEPROM not programmed\n\r");
        settings_restore_defaults();
    }
    printf("#INFO: max5443_offset = %u\n\r", settings.settings.max5443_offset);
    printf("#INFO: tcs_enabled = %u\n\r", settings.settings.tcs_enabled);
    printf("#INFO: tcs_clear_threshold = %u\n\r", settings.settings.tcs_clear_threshold);
    printf("#INFO: r100_trim = %i\n\r", settings.settings.r100_trim);
    printf("#INFO: r3k_trim = %i\n\r", settings.settings.r3k_trim);
    printf("#INFO: r30k_trim = %i\n\r", settings.settings.r30k_trim);
    printf("#INFO: r300k_trim = %i\n\r", settings.settings.r300k_trim);
    printf("#INFO: r3M_trim = %i\n\r", settings.settings.r3M_trim);
    printf("#INFO: r30M_trim = %i\n\r", settings.settings.r30M_trim);
    printf("#INFO: r100M_trim = %i\n\r", settings.settings.r100M_trim);
    printf("Smax5443_offset.%u:tcs_enabled.%u:tcs_clear_threshold.%u:r100_trim.%i:r3k_trim.%i:r30k_trim.%i:r300k_trim.%i:r3M_trim.%i:r30M_trim.%i:r100M_trim.%i\n\r",
           settings.settings.max5443_offset,
           settings.settings.tcs_enabled,
           settings.settings.tcs_clear_threshold,
           settings.settings.r100_trim,
           settings.settings.r3k_trim,
           settings.settings.r30k_trim,
           settings.settings.r300k_trim,
           settings.settings.r3M_trim,
           settings.settings.r30M_trim,
           settings.settings.r100M_trim);
}

void settings_write_eeprom(void){
    nvm_eeprom_load_page_to_buffer((uint8_t*)&settings);
    nvm_eeprom_atomic_write_page(SETTINGS_EEPROM_OFFSET_ADDR);
}

void settings_restore_defaults(void){
    printf("#INFO: Restoring EEPROM Defaults\n\r");
    settings.settings.programmed = 1;
    settings.settings.max5443_offset = SETTINGS_MAX5443_OFFSET;
    settings.settings.tcs_enabled = SETTINGS_TCS_ENABLED;
    settings.settings.tcs_clear_threshold = SETTINGS_TCS_CLEAR_THRESHOLD;
    settings.settings.r100_trim =  SETTINGS_R100_TRIM;
    settings.settings.r3k_trim = SETTINGS_R3k_TRIM;
    settings.settings.r30k_trim = SETTINGS_R30k_TRIM;
    settings.settings.r300k_trim = SETTINGS_R300k_TRIM;
    settings.settings.r3M_trim = SETTINGS_R3M_TRIM;
    settings.settings.r30M_trim = SETTINGS_R30M_TRIM;
    settings.settings.r100M_trim = SETTINGS_R100M_TRIM;
    
    settings_write_eeprom();
}

