
/**
 * PD_UFP.h
 *
 *  Updated on: Jan 4, 2021
 *      Author: Ryan Ma
 *
 * Minimalist USB PD Ardunio Library for PD Micro board
 * Only support UFP(device) functionality
 * Requires FUSB302_UFP.h, PD_UFP_Protocol.h and Standard Arduino Library
 *
 * Support PD3.0 PPS
 * 
 */
 
#include <stdint.h>
#include <string.h>

#include <Arduino.h>
#include <Wire.h>
#include <HardwareSerial.h>

#include "PD_UFP.h"

#define t_PD_POLLING            100
#define t_TypeCSinkWaitCap      350
#define t_RequestToPSReady      580     // combine t_SenderResponse and t_PSTransition
#define t_PPSRequest            5000    // must less than 10000 (10s)

#define PIN_OUTPUT_ENABLE       10
#define PIN_FUSB302_INT         7

#define PIN_LED_CURRENT_1       13
#define PIN_LED_CURRENT_2       12
#define PIN_LED_VOLTAGE_1       // PE2 not support by ardunio library, manipulate register directly
#define PIN_LED_VOLTAGE_2       22
#define PIN_LED_VOLTAGE_3       23
#define PIN_LED_VOLTAGE_4       11

#define LOG(...)      do { char buf[80]; sprintf(buf, __VA_ARGS__); Serial.print(buf); } while(0)

static FUSB302_ret_t FUSB302_i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint8_t count)
{
    Wire.beginTransmission(dev_addr);
    Wire.write(reg_addr);
    Wire.endTransmission();
    Wire.requestFrom(dev_addr, count);
    while (Wire.available() && count > 0) {
        *data++ = Wire.read();
        count--;
    }
    return count == 0 ? FUSB302_SUCCESS : FUSB302_ERR_READ_DEVICE;
}

static FUSB302_ret_t FUSB302_i2c_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint8_t count)
{
    Wire.beginTransmission(dev_addr);
    Wire.write(reg_addr);
    while (count > 0) {
        Wire.write(*data++);
        count--;
    }
    Wire.endTransmission();
    return FUSB302_SUCCESS;
}

static FUSB302_ret_t FUSB302_delay_ms(uint32_t t)
{
    delay(t);
    return FUSB302_SUCCESS;
}

void PD_UFP_c::handle_protocol_event(PD_protocol_event_t events)
{    
    if (events & PD_PROTOCOL_EVENT_SRC_CAP) {
        wait_src_cap = 0;
        get_src_cap_retry_count = 0;
        print_src_cap = 1;
        wait_ps_rdy = 1;
        time_wait_ps_rdy = clock_ms();
    }
    if (events & PD_PROTOCOL_EVENT_REJECT) {
        if (wait_ps_rdy) {
            wait_ps_rdy = 0;
            print_request_reject = 1;
        }
    }    
    if (events & PD_PROTOCOL_EVENT_PS_RDY) {
        PD_power_info_t p;
        uint8_t i, selected_power = PD_protocol_get_selected_power(&protocol);
        PD_protocol_get_power_info(&protocol, selected_power, &p);
        wait_ps_rdy = 0;
        if (p.type == PD_PDO_TYPE_AUGMENTED_PDO) {
            FUSB302_set_vbus_sense(&FUSB302, 0);
            ready_voltage = PD_protocol_get_PPS_voltage(&protocol);
            ready_current = PD_protocol_get_PPS_current(&protocol);
            if (PPS_voltage_next) {
                // Two stage startup for PPS voltage < 5V
                PD_protocol_set_PPS(&protocol, PPS_voltage_next, ready_current, false);
                PPS_voltage_next = 0;
                send_request = 1;
            } else {
                calculate_led_pps(ready_voltage, ready_current);
                time_PPS_request = clock_ms();
                status_power = STATUS_POWER_PPS;
                print_power = STATUS_POWER_PPS;
            }
        } else {
            FUSB302_set_vbus_sense(&FUSB302, 1);
            ready_voltage = p.max_v;
            ready_current = p.max_i;
            calculate_led(ready_voltage, ready_current);
            status_power = STATUS_POWER_TYP;
            print_power = STATUS_POWER_TYP;
        }
    }
}

void PD_UFP_c::handle_FUSB302_event(FUSB302_event_t events)
{
    if (events & FUSB302_EVENT_DETACHED) {
        PD_protocol_reset(&protocol);
        return;
    }
    if (events & FUSB302_EVENT_ATTACHED) {
        print_cc = 1;
        uint8_t cc1 = 0, cc2 = 0, cc = 0;
        FUSB302_get_cc(&FUSB302, &cc1, &cc2);
        PD_protocol_reset(&protocol);
        if (cc1 && cc2 == 0) {
            cc = cc1;
        } else if (cc2 && cc1 == 0) {
            cc = cc2;
        }
        /* TODO: handle no cc detected error */
        led_current = cc;
        led_voltage = PD_UFP_VOLTAGE_LED_5V;
        if (cc > 1) {
            wait_src_cap = 1;
        } else {
            set_default_power();
        }
    }
    if (events & FUSB302_EVENT_RX_SOP) {
        PD_protocol_event_t protocol_event = 0;
        uint16_t header;
        uint32_t obj[7];
        FUSB302_get_message(&FUSB302, &header, obj);
        PD_protocol_handle_msg(&protocol, header, obj, &protocol_event);
        if (protocol_event) {
            handle_protocol_event(protocol_event);
        }
    }
    if (events & FUSB302_EVENT_GOOD_CRC_SENT) {
        uint16_t header;
        uint32_t obj[7];
        delay(2);  /* Delay respond in case there are retry messages */
        if (PD_protocol_respond(&protocol, &header, obj)) {
            FUSB302_tx_sop(&FUSB302, header, obj);
        }
    }
}

bool PD_UFP_c::timer(void)
{
    uint16_t t = clock_ms();
    if (wait_src_cap && t - time_wait_src_cap > t_TypeCSinkWaitCap) {
        time_wait_src_cap = t;
        if (get_src_cap_retry_count < 3) {
            uint16_t header;
            get_src_cap_retry_count += 1;
            /* Try to request soruce capabilities message (will not cause power cycle VBUS) */
            PD_protocol_create_get_src_cap(&protocol, &header);
            FUSB302_tx_sop(&FUSB302, header, 0);
        } else {
            get_src_cap_retry_count = 0;
            /* Hard reset will cause the source power cycle VBUS. */
            FUSB302_tx_hard_reset(&FUSB302);
            PD_protocol_reset(&protocol);
        }
    }
    if (wait_ps_rdy) {
        if (t - time_wait_ps_rdy > t_RequestToPSReady) {
            wait_ps_rdy = 0;
            set_default_power();
        }
    } else if (send_request || (status_power == STATUS_POWER_PPS && t - time_PPS_request > t_PPSRequest)) {
        wait_ps_rdy = 1;
        send_request = 0;
        time_PPS_request = t;
        uint16_t header;
        uint32_t obj[7];
        /* Send request if option updated or regularly in PPS mode to keep power alive */
        PD_protocol_create_request(&protocol, &header, obj);
        FUSB302_tx_sop(&FUSB302, header, obj);
    }
    if (t - time_polling > t_PD_POLLING) {
        time_polling = t;
        return true;
    }
    return false;
}

void PD_UFP_c::set_default_power(void)
{
    ready_voltage = PD_V(5);
    ready_current = PD_A(1);
    status_power = STATUS_POWER_TYP;
}

void PD_UFP_c::calculate_led(uint16_t voltage, uint16_t current)
{
    uint8_t i;
    const uint16_t PD_V_level[4] = {PD_V(9.0), PD_V(12.0), PD_V(15.0), PD_V(20.0)};
    const uint16_t PD_A_level[2] = {PD_A(1.5), PD_A(3.0)};
    for (i = 0; i < 4 && voltage >= PD_V_level[i]; i++) {}
    led_voltage = PD_UFP_VOLTAGE_LED_5V + i;
    for (i = 0; i < 2 && current >= PD_A_level[i]; i++) {}
    led_current = PD_UFP_CURRENT_LED_LE_1V + i;
}

void PD_UFP_c::calculate_led_pps(uint16_t PPS_voltage, uint8_t PPS_current)
{
    uint8_t i;
    const uint16_t PPS_V_level[4] = {PPS_V(9.0), PPS_V(12.0), PPS_V(15.0), PPS_V(20.0)};
    const uint8_t PPS_A_level[2] = {PPS_A(1.5), PPS_A(3.0)};
    for (i = 0; i < 4 && PPS_voltage >= PPS_V_level[i]; i++) {}
    led_voltage = PD_UFP_VOLTAGE_LED_5V + i;
    for (i = 0; i < 2 && PPS_current >= PPS_A_level[i]; i++) {}
    led_current = PD_UFP_CURRENT_LED_LE_1V + i;
}

void PD_UFP_c::update_voltage_led(PD_UFP_VOLTAGE_LED_t index)
{
    if (index >= PD_UFP_VOLTAGE_LED_AUTO) {
        index = led_voltage;
    }
    if (index == PD_UFP_VOLTAGE_LED_OFF) {
        DDRE &= 0xFB;  // pinMode PE2 INPUT
        PORTE &= 0xFB;  // Clear PE2
        pinMode(PIN_LED_VOLTAGE_2, INPUT);
        pinMode(PIN_LED_VOLTAGE_3, INPUT);
        pinMode(PIN_LED_VOLTAGE_4, INPUT);
    } else {
        const uint8_t led1[5] = {0, 0, 0, 0, 1};
        const uint8_t led2[5] = {1, 0, 0, 0, 1};
        const uint8_t led3[5] = {1, 1, 0, 0, 1};
        const uint8_t led4[5] = {1, 1, 1, 0, 1};
        index -= 1;
        if (led1[index]) {
            PORTE |= 0x04;  // Set PE2
        } else {
            PORTE &= 0xFB;  // Clear PE2
        }
        digitalWrite(PIN_LED_VOLTAGE_2, led2[index]);
        digitalWrite(PIN_LED_VOLTAGE_3, led3[index]);
        digitalWrite(PIN_LED_VOLTAGE_4, led4[index]);
        DDRE |= 0x04;  // pinMode PE2 OUTPUT
        pinMode(PIN_LED_VOLTAGE_2, OUTPUT);
        pinMode(PIN_LED_VOLTAGE_3, OUTPUT);
        pinMode(PIN_LED_VOLTAGE_4, OUTPUT);
    }
}

void PD_UFP_c::update_current_led(PD_UFP_CURRENT_LED_t index)
{
    if (index >= PD_UFP_CURRENT_LED_AUTO) {
        index = led_current;
    }
    if (index == PD_UFP_CURRENT_LED_OFF) {
        pinMode(PIN_LED_CURRENT_1, INPUT);
        pinMode(PIN_LED_CURRENT_2, INPUT);
    } else {
        const uint8_t led1[3] = {0, 1, 1};
        const uint8_t led2[3] = {0, 0, 1};
        index -= 1;
        digitalWrite(PIN_LED_CURRENT_1, led1[index]);
        digitalWrite(PIN_LED_CURRENT_2, led2[index]);
        pinMode(PIN_LED_CURRENT_1, OUTPUT);
        pinMode(PIN_LED_CURRENT_2, OUTPUT);
    }
}
        
void PD_UFP_c::handle_led(void)
{
    if (led_blink_enable) {
        uint16_t t = clock_ms();
        if (t - time_led_blink > period_led_blink) {
            time_led_blink = t;
            if (led_blink_status) {
                update_voltage_led(PD_UFP_VOLTAGE_LED_OFF);
                update_current_led(PD_UFP_CURRENT_LED_OFF);
                led_blink_status = 0;
            } else {
                update_voltage_led(PD_UFP_VOLTAGE_LED_AUTO);
                update_current_led(PD_UFP_CURRENT_LED_AUTO);
                led_blink_status = 1;
            }
        }
    }
}

PD_UFP_c::PD_UFP_c():
    ready_voltage(0),
    ready_current(0),
    led_blink_enable(0),
    led_blink_status(0),
    time_led_blink(0),
    period_led_blink(0),
    led_voltage(PD_UFP_VOLTAGE_LED_OFF),
    led_current(PD_UFP_CURRENT_LED_OFF),
    PPS_voltage_next(0),
    status_initialized(0),
    status_src_cap_received(0),
    status_power(STATUS_POWER_NA),
    time_polling(0),
    time_wait_src_cap(0),
    time_wait_ps_rdy(0),
    time_PPS_request(0),
    get_src_cap_retry_count(0),
    wait_src_cap(0),
    wait_ps_rdy(0),
    send_request(0),
    clock_prescaler(1),
    print_dev(0),
    print_cc(0),
    print_src_cap(0),
    print_request_reject(0),
    print_power(STATUS_POWER_NA)
{
    memset(&FUSB302, 0, sizeof(FUSB302_dev_t));
    memset(&protocol, 0, sizeof(PD_protocol_t));
}

void PD_UFP_c::init(enum PD_power_option_t power_option)
{
    init_PPS(0, 0, power_option);
}

void PD_UFP_c::init_PPS(uint16_t PPS_voltage, uint8_t PPS_current, enum PD_power_option_t power_option)
{
    digitalWrite(PIN_OUTPUT_ENABLE, 0);
    pinMode(PIN_OUTPUT_ENABLE, OUTPUT);
        
    pinMode(PIN_FUSB302_INT, INPUT_PULLUP);    // sets the digital pin 7 as input

    update_voltage_led(PD_UFP_VOLTAGE_LED_OFF);
    update_current_led(PD_UFP_CURRENT_LED_OFF);

    // Initialize FUSB302
    FUSB302.i2c_address = 0x22;
    FUSB302.i2c_read = FUSB302_i2c_read;
    FUSB302.i2c_write = FUSB302_i2c_write;
    FUSB302.delay_ms = FUSB302_delay_ms;
    if (FUSB302_init(&FUSB302) == FUSB302_SUCCESS && FUSB302_get_ID(&FUSB302, 0, 0) == FUSB302_SUCCESS) {
        status_initialized = 1;
    }

    // Two stage startup for PPS Voltge < 5V
    if (PPS_voltage && PPS_voltage < PPS_V(5.0)) {
        PPS_voltage_next = PPS_voltage;
        PPS_voltage = PPS_V(5.0);
    }

    // Initialize PD protocol engine
    PD_protocol_init(&protocol);
    PD_protocol_set_power_option(&protocol, power_option);
    PD_protocol_set_PPS(&protocol, PPS_voltage, PPS_current, false);

    print_dev = 1;
}

void PD_UFP_c::run(void)
{
    if (timer() || digitalRead(PIN_FUSB302_INT) == 0) {
        FUSB302_event_t FUSB302_events = 0;
        for (uint8_t i = 0; i < 3 && FUSB302_alert(&FUSB302, &FUSB302_events) != FUSB302_SUCCESS; i++) {}
        if (FUSB302_events) {
            handle_FUSB302_event(FUSB302_events);
        }
    }
    handle_led();
}

void PD_UFP_c::set_output(uint8_t enable)
{
    digitalWrite(PIN_OUTPUT_ENABLE, enable);
}

void PD_UFP_c::set_led(PD_UFP_VOLTAGE_LED_t index_v, PD_UFP_CURRENT_LED_t index_a)
{
    led_blink_enable = 0;
    update_voltage_led(index_v);
    update_current_led(index_a);
}

void PD_UFP_c::set_led(uint8_t enable)
{
    led_blink_enable = 0;
    if (enable) {
        update_voltage_led(PD_UFP_VOLTAGE_LED_AUTO);
        update_current_led(PD_UFP_CURRENT_LED_AUTO);
    } else {
        update_voltage_led(PD_UFP_VOLTAGE_LED_OFF);
        update_current_led(PD_UFP_CURRENT_LED_OFF);
    }
}

void PD_UFP_c::blink_led(uint16_t period)
{
    led_blink_enable = 1;
    period_led_blink = period >> 1;
}

bool PD_UFP_c::set_PPS(uint16_t PPS_voltage, uint8_t PPS_current)
{
    if (status_power == STATUS_POWER_PPS && PD_protocol_set_PPS(&protocol, PPS_voltage, PPS_current, true)) {
        send_request = 1;
        return true;
    }
    return false;
}

void PD_UFP_c::set_power_option(enum PD_power_option_t power_option)
{
    if (PD_protocol_set_power_option(&protocol, power_option)) {
        send_request = 1;
    }
}

void PD_UFP_c::clock_prescale_set(uint8_t prescaler)
{
    if (prescaler) {
        clock_prescaler = prescaler;
    }
}

uint16_t PD_UFP_c::clock_ms(void)
{
    return (uint16_t)millis() * clock_prescaler;
}

void PD_UFP_c::print_status(void)
{
    if (!Serial) {
        return;
    } else if (print_dev) {
        print_dev = 0;
        if (status_initialized) {
            uint8_t version_ID = 0, revision_ID = 0;
            FUSB302_get_ID(&FUSB302, &version_ID, &revision_ID);
            LOG("FUSB302 version ID:%c_rev%c\n", 'A' + version_ID, 'A' + revision_ID);
        } else {
            LOG("FUSB302 init error\n");
        }
    } else if (print_cc) {
        print_cc = 0;
        const char *detection_type_str[] = {"vRd-USB", "vRd-1.5", "vRd-3.0"};
        uint8_t cc1 = 0, cc2 = 0;
        FUSB302_get_cc(&FUSB302, &cc1, &cc2);
        if (cc1 == 0 && cc2 == 0) {
            LOG("PD: USB attached vRA\n");
        } else if (cc1 && cc2 == 0) {
            LOG("PD: USB attached CC1 %s\n", detection_type_str[cc1 - 1]);
        } else if (cc2 && cc1 == 0) {
            LOG("PD: USB attached CC2 %s\n", detection_type_str[cc2 - 1]);
        } else {
            LOG("PD: USB attached unknown\n");
        }
    } else if (print_src_cap) {
        print_src_cap = 0;
        PD_power_info_t p;
        uint8_t selected = PD_protocol_get_selected_power(&protocol);
        for (uint8_t i = 0; PD_protocol_get_power_info(&protocol, i, &p); i++) {
            char min_v[8] = {0}, max_v[8] = {0}, power[8] = {0};
            if (p.min_v) snprintf(min_v, 8, "%d.%02dV", p.min_v / 20, (p.min_v * 5) % 100);
            if (p.max_v) snprintf(max_v, 8, "%d.%02dV", p.max_v / 20, (p.max_v * 5) % 100);
            if (p.max_i) {
                snprintf(power, 8, "%d.%02dA", p.max_i / 100, p.max_i % 100);
            } else {
                snprintf(power, 8, "%d.%02dW", p.max_p / 4, p.max_p * 25);
            }
            LOG("   [%d] %s%s %s%s\n", i, min_v, max_v, power, i == selected ? " *" : "");
        }
    } else if (print_power == STATUS_POWER_TYP) {
        print_power = STATUS_POWER_NA;
        PD_power_info_t p;
        uint8_t selected_power = PD_protocol_get_selected_power(&protocol);
        if (PD_protocol_get_power_info(&protocol, selected_power, &p)) {
            LOG("PD: %d.%02dV supply ready\n", p.max_v / 20, (p.max_v * 5) % 100);
        }
    } else if (print_power == STATUS_POWER_PPS) {
        print_power = STATUS_POWER_NA;
        LOG("PD: PPS supply ready\n");
    } else if (print_request_reject) {
        print_request_reject = 0;
        LOG("PD: Request Rejected\n");
    }    
}
