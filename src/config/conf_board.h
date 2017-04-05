/**
 * \file
 *
 * \brief User board configuration template
 *
 */

#ifndef CONF_BOARD_H
#define CONF_BOARD_H

#include <ioport.h>

#if BOARD_VER_MAJOR >= 1 && BOARD_VER_MINOR >= 2 && BOARD_VER_MICRO >=3
    #define BOARD_XOSC_HZ          12000000
    #define BOARD_XOSC_TYPE        XOSC_TYPE_XTAL
    #define BOARD_XOSC_STARTUP_US  1000

    #define LED1    IOPORT_CREATE_PIN(PORTF,7)
    #define LED2    IOPORT_CREATE_PIN(PORTF,6)
#endif

// Default Settings (only used if EEPROM is empty)
#define SETTINGS_MAX5443_OFFSET 0
#define SETTINGS_TCS_ENABLED 1
#define SETTINGS_TCS_CLEAR_THRESHOLD 10000U
#define SETTINGS_R100_TRIM 0
#define SETTINGS_R3k_TRIM 0
#define SETTINGS_R30k_TRIM 0
#define SETTINGS_R300k_TRIM 0
#define SETTINGS_R3M_TRIM 0
#define SETTINGS_R30M_TRIM 0
#define SETTINGS_R100M_TRIM 0

#endif // CONF_BOARD_H
