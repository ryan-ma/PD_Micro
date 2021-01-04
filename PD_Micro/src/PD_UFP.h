
/**
 * PD_UFP.h
 *
 *  Created on: Nov 16, 2020
 *      Author: Ryan Ma
 *
 * Minimalist USB PD Ardunio Library for PD Micro board
 * Only support UFP(device) functionality
 * Requires FUSB302_UFP.h, PD_UFP_Protocol.h and Standard Arduino Library
 *
 */

#ifndef PD_UFP_H
#define PD_UFP_H

#include <stdint.h>

extern "C" {
    #include "FUSB302_UFP.h"
    #include "PD_UFP_Protocol.h"
}

#define PD_V(v) ((uint16_t)(v * 20))
#define PD_A(a) ((uint16_t)(a * 100))

enum {
    PD_UFP_VOLTAGE_LED_OFF      = 0,
    PD_UFP_VOLTAGE_LED_5V       = 1,
    PD_UFP_VOLTAGE_LED_9V       = 2,
    PD_UFP_VOLTAGE_LED_12V      = 3,
    PD_UFP_VOLTAGE_LED_15V      = 4,
    PD_UFP_VOLTAGE_LED_20V      = 5,
    PD_UFP_VOLTAGE_LED_AUTO     = 6
};
typedef uint8_t PD_UFP_VOLTAGE_LED_t;

enum {
    PD_UFP_CURRENT_LED_OFF      = 0,
    PD_UFP_CURRENT_LED_LE_1V    = 1,
    PD_UFP_CURRENT_LED_LE_3V    = 2,
    PD_UFP_CURRENT_LED_GT_3V    = 3,
    PD_UFP_CURRENT_LED_AUTO     = 4    
};
typedef uint8_t PD_UFP_CURRENT_LED_t;

class PD_UFP_c
{
    public:
        PD_UFP_c();
        void init(enum PD_power_option_t power_option);
        void run(void);
        bool is_power_ready(void);
        
        uint16_t get_voltage(void);     // Voltage in 50mV units
        uint16_t get_current(void);     // Current in 10mA units
        
        void set_output(uint8_t enable);
        void set_led(uint8_t enable);
        void set_led(PD_UFP_VOLTAGE_LED_t index_v, PD_UFP_CURRENT_LED_t index_a);
        void blink_led(uint16_t period);
        
        void set_power_option(enum PD_power_option_t power_option);
        void print_status(void);
        
    private:
        void handle_protocol_event(PD_protocol_event_t events);
        void handle_FUSB302_event(FUSB302_event_t events);
        bool timer(void);
        // Device
        FUSB302_dev_t FUSB302;
        PD_protocol_t protocol;
        // Power ready power
        uint16_t ready_voltage;
        uint16_t ready_current;
        // LED
        uint8_t led_blink_enable;
        uint8_t led_blink_status;
        uint16_t time_led_blink;
        uint16_t period_led_blink;
        PD_UFP_VOLTAGE_LED_t led_voltage;
        PD_UFP_CURRENT_LED_t led_current;
        void update_voltage_led(PD_UFP_VOLTAGE_LED_t index);
        void update_current_led(PD_UFP_CURRENT_LED_t index);
        void handle_led(void);
        // Status
        uint8_t status_initialized;
        uint8_t status_src_cap_received;
        uint8_t status_power_ready;
        // Timer and counter for PD Policy 
        uint16_t time_polling;
        uint16_t time_wait_src_cap;
        uint8_t wait_src_cap;
        uint8_t get_src_cap_retry_count;
        // Status output
        uint8_t print_dev;
        uint8_t print_cc;
        uint8_t print_src_cap;
        uint8_t print_power_ready;
};

#endif
