
/**
 * PD_UFP.h
 *
 *  Updated on: Jan 13, 2021
 *      Author: Ryan Ma
 *
 * Minimalist USB PD Ardunio Library for PD Micro board
 * Only support UFP(device) functionality
 * Requires FUSB302_UFP.h, PD_UFP_Protocol.h and Standard Arduino Library
 *
 * Support PD3.0 PPS
 * 
 */

#ifndef PD_UFP_H
#define PD_UFP_H

#include <stdint.h>

#include <Arduino.h>
#include <Wire.h>
#include <HardwareSerial.h>

extern "C" {
    #include "FUSB302_UFP.h"
    #include "PD_UFP_Protocol.h"
}

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

enum {
    STATUS_POWER_NA = 0,
    STATUS_POWER_TYP,
    STATUS_POWER_PPS
};
typedef uint8_t status_power_t;

struct status_log_t {
    uint8_t status;
    uint8_t msg_name_ref;
    uint16_t time;
};

class PD_UFP_c
{
    public:
        PD_UFP_c();
        void init(enum PD_power_option_t power_option = PD_POWER_OPTION_MAX_5V);
        void init_PPS(uint16_t PPS_voltage, uint8_t PPS_current, enum PD_power_option_t power_option = PD_POWER_OPTION_MAX_5V);
        
        void run(void);
        bool is_power_ready(void) { return status_power == STATUS_POWER_TYP; }
        bool is_PPS_ready(void)   { return status_power == STATUS_POWER_PPS; }
        
        bool is_ps_transition(void) { return send_request || wait_ps_rdy; }

        uint16_t get_voltage(void) { return ready_voltage; }    // Voltage in 50mV units, 20mV(PPS)
        uint16_t get_current(void) { return ready_current; }    // Current in 10mA units, 50mA(PPS)

        void set_output(uint8_t enable);
        void set_led(uint8_t enable);
        void set_led(PD_UFP_VOLTAGE_LED_t index_v, PD_UFP_CURRENT_LED_t index_a);
        void blink_led(uint16_t period);
        
        bool set_PPS(uint16_t PPS_voltage, uint8_t PPS_current);
        void set_power_option(enum PD_power_option_t power_option);

        void clock_prescale_set(uint8_t prescaler);

        int status_log_readline(char * buffer, int maxlen);
        void print_status(Serial_ & serial);
        void print_status(HardwareSerial & serial);

    private:
        void handle_protocol_event(PD_protocol_event_t events);
        void handle_FUSB302_event(FUSB302_event_t events);
        bool timer(void);
        void set_default_power(void);
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
        void calculate_led(uint16_t voltage, uint16_t current);
        void calculate_led_pps(uint16_t PPS_voltage, uint8_t PPS_current);
        void update_voltage_led(PD_UFP_VOLTAGE_LED_t index);
        void update_current_led(PD_UFP_CURRENT_LED_t index);
        void handle_led(void);
        // PPS setup
        uint16_t PPS_voltage_next;
        // Status
        uint8_t status_initialized;
        uint8_t status_load_sw;
        uint8_t status_src_cap_received;
        status_power_t status_power;
        // Timer and counter for PD Policy
        uint16_t clock_ms(void);
        uint16_t time_polling;
        uint16_t time_wait_src_cap;
        uint16_t time_wait_ps_rdy;
        uint16_t time_PPS_request;
        uint8_t get_src_cap_retry_count;
        uint8_t wait_src_cap;
        uint8_t wait_ps_rdy;
        uint8_t send_request;
        uint8_t clock_prescaler;
        // Status output
        void status_log_tx_msg(void);
        void status_log_rx_msg(void);
        void status_log_push(uint8_t status, uint8_t msg_name_ref = 0);
        status_log_t status_log[16];
        uint8_t status_log_read;
        uint8_t status_log_write;
        uint8_t status_log_counter;
};

#endif
