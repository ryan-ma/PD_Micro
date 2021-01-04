
/**
 * PD_UFP_Protocol.h
 *
 *  Created on: Nov 16, 2020
 *      Author: Ryan Ma
 *
 * Minimalist USB PD implement with only UFP(device) functionality
 * Requires PD PHY to do automatic GoodCRC response on valid SOP messages.
 * Requires only stdint.h, stdbool.h and string.h
 * No use of bit-field for better cross-platform compatibility
 *
 */

#ifndef PD_UFP_PROTOCOL_H
#define PD_UFP_PROTOCOL_H

#include <stdbool.h>
#include <stdint.h>

#define PD_PROTOCOL_EVENT_SRC_CAP       (1 << 0)
#define PD_PROTOCOL_EVENT_PS_RDY        (1 << 1)
typedef uint8_t PD_protocol_event_t;

enum PD_power_option_t {
    PD_POWER_OPTION_MAX_5V      = 0,
    PD_POWER_OPTION_MAX_9V      = 1,
    PD_POWER_OPTION_MAX_12V     = 2,
    PD_POWER_OPTION_MAX_15V     = 3,
    PD_POWER_OPTION_MAX_20V     = 4,
    PD_POWER_OPTION_MAX_VOLTAGE = 5,
    PD_POWER_OPTION_MAX_CURRENT = 6,
    PD_POWER_OPTION_MAX_POWER   = 7,
};

typedef struct {
    const char *name;
    uint8_t id;
    uint8_t spec_rev;
    uint8_t num_of_obj;
} PD_msg_info_t;

typedef struct {
    uint16_t min_v;     /* Voltage in 50mV units */
    uint16_t max_v;     /* Voltage in 50mV units */
    uint16_t max_i;     /* Current in 10mA units */
    uint16_t max_p;     /* Power in 250mW units */
} PD_power_info_t;

struct PD_msg_state_t;
typedef struct {
    const struct PD_msg_state_t *msg_state;
    const char *tx_msg_name;
    uint16_t tx_msg_header;
    uint16_t rx_msg_header;
    uint8_t message_id;

    enum PD_power_option_t power_option;
    uint32_t power_data_obj[6];
    uint8_t power_data_obj_count;
    uint8_t power_data_obj_selected;
} PD_protocol_t;

void PD_protocol_handle_msg(PD_protocol_t *p, uint16_t header, uint32_t *obj, PD_protocol_event_t *events);
bool PD_protocol_respond(PD_protocol_t *p, uint16_t *h, uint32_t *obj);

void PD_protocol_create_get_src_cap(PD_protocol_t *p, uint16_t *h);
void PD_protocol_create_request(PD_protocol_t *p, uint16_t *h, uint32_t *obj);

static inline uint8_t PD_protocol_get_selected_power(PD_protocol_t *p) { return p->power_data_obj_selected; }
bool PD_protocol_get_power_info(PD_protocol_t *p, uint8_t index, PD_power_info_t *power_info);
bool PD_protocol_get_tx_msg_info(PD_protocol_t *p, PD_msg_info_t *msg_info);
bool PD_protocol_get_rx_msg_info(PD_protocol_t *p, PD_msg_info_t *msg_info);

bool PD_protocol_set_power_option(PD_protocol_t *p, enum PD_power_option_t option);
bool PD_protocol_select_power(PD_protocol_t *p, uint8_t index);

void PD_protocol_reset(PD_protocol_t *p);
void PD_protocol_init(PD_protocol_t *p);

#endif
