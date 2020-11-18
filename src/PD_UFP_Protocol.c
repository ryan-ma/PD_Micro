
/**
 * PD_UFP_Protocol.c
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
 
#include <string.h>
#include "PD_UFP_Protocol.h"

#define PD_SPECIFICATION_REVISION       0x1

#define PD_CONTROL_MSG_TYPE_ACCEPT      0x3
#define PD_CONTROL_MSG_TYPE_REJECT      0x4
#define PD_DATA_MSG_TYPE_REQUEST        0x2
#define PD_DATA_MSG_TYPE_SINK_CAP       0x4
#define PD_DATA_MSG_TYPE_GET_SRC_CAP    0x7
#define PD_DATA_MSG_TYPE_VENDOR_DEFINED 0xF

#define T(name)     name

enum PD_power_data_obj_type_t {   /* Power data object type */
    PD_PDO_TYPE_FIXED_SUPPLY    = 0,
    PD_PDO_TYPE_BATTERY         = 1,
    PD_PDO_TYPE_VARIABLE_SUPPLY = 2
};

typedef struct {
    uint8_t type;
    uint8_t spec_rev;
    uint8_t id;
    uint8_t num_of_obj;
} PD_msg_header_info_t;

typedef struct {
    uint16_t limit;
    uint8_t use_voltage;
    uint8_t use_current;
} PD_power_option_setting_t;

struct PD_msg_state_t {
    const char * name;
    void (*handler)(PD_protocol_t * p, uint16_t header, uint32_t * obj, PD_protocol_event_t * events);
    bool (*responder)(PD_protocol_t * p, uint16_t * header, uint32_t * obj);
};

static void handler_good_crc   (PD_protocol_t * p, uint16_t header, uint32_t * obj, PD_protocol_event_t * events);
static void handler_goto_min   (PD_protocol_t * p, uint16_t header, uint32_t * obj, PD_protocol_event_t * events);
static void handler_ps_rdy     (PD_protocol_t * p, uint16_t header, uint32_t * obj, PD_protocol_event_t * events);
static void handler_source_cap (PD_protocol_t * p, uint16_t header, uint32_t * obj, PD_protocol_event_t * events);
static void handler_vender_def (PD_protocol_t * p, uint16_t header, uint32_t * obj, PD_protocol_event_t * events);

static bool responder_get_sink_cap  (PD_protocol_t * p, uint16_t * header, uint32_t * obj);
static bool responder_reject        (PD_protocol_t * p, uint16_t * header, uint32_t * obj);
static bool responder_soft_reset    (PD_protocol_t * p, uint16_t * header, uint32_t * obj);
static bool responder_source_cap    (PD_protocol_t * p, uint16_t * header, uint32_t * obj);
static bool responder_vender_def    (PD_protocol_t * p, uint16_t * header, uint32_t * obj);

static const struct PD_msg_state_t ctrl_msg_list[16] = {
    {.name = T("[CONTROL 0]"),  .handler = 0,                   .responder = 0},
    {.name = T("GoodCRC"),      .handler = handler_good_crc,    .responder = 0},
    {.name = T("GotoMin"),      .handler = handler_goto_min,    .responder = 0},
    {.name = T("Accept"),       .handler = 0,                   .responder = 0},
    {.name = T("Reject"),       .handler = 0,                   .responder = 0},
    {.name = T("Ping"),         .handler = 0,                   .responder = 0},
    {.name = T("PS_RDY"),       .handler = handler_ps_rdy,      .responder = 0},
    {.name = T("Get_Src_Cap"),  .handler = 0,                   .responder = 0},
    {.name = T("Get_Sink_Cap"), .handler = 0,                   .responder = responder_get_sink_cap},
    {.name = T("DR_Swap"),      .handler = 0,                   .responder = responder_reject},
    {.name = T("PR_Swap"),      .handler = 0,                   .responder = responder_reject},
    {.name = T("VCONN_Swap"),   .handler = 0,                   .responder = responder_reject},
    {.name = T("Wait"),         .handler = 0,                   .responder = 0},
    {.name = T("Soft_Reset"),   .handler = 0,                   .responder = responder_soft_reset},
    {.name = T("[CONTROL 14]"), .handler = 0,                   .responder = 0},
    {.name = T("[CONTROL 15]"), .handler = 0,                   .responder = 0}
};

static const struct PD_msg_state_t data_msg_list[16] = {
    {.name = T("[DATA 0]"),     .handler = 0,                   .responder = 0},
    {.name = T("Src_Cap"),      .handler = handler_source_cap,  .responder = responder_source_cap},
    {.name = T("Request"),      .handler = 0,                   .responder = 0},
    {.name = T("BIST"),         .handler = 0,                   .responder = 0},
    {.name = T("Sink_Cap"),     .handler = 0,                   .responder = 0},
    {.name = T("[DATA 5]"),     .handler = 0,                   .responder = 0},
    {.name = T("[DATA 6]"),     .handler = 0,                   .responder = 0},
    {.name = T("[DATA 7]"),     .handler = 0,                   .responder = 0},
    {.name = T("[DATA 8]"),     .handler = 0,                   .responder = 0},
    {.name = T("[DATA 9]"),     .handler = 0,                   .responder = 0},
    {.name = T("[DATA 10]"),    .handler = 0,                   .responder = 0},
    {.name = T("[DATA 11]"),    .handler = 0,                   .responder = 0},
    {.name = T("[DATA 12]"),    .handler = 0,                   .responder = 0},
    {.name = T("[DATA 13]"),    .handler = 0,                   .responder = 0},
    {.name = T("[DATA 14]"),    .handler = 0,                   .responder = 0},
    {.name = T("VDM"),          .handler = handler_vender_def,  .responder = responder_vender_def}
};

static const PD_power_option_setting_t power_option_setting[8] = {
    {.limit = 25,   .use_voltage = 1, .use_current = 0},    /* PD_POWER_OPTION_MAX_5V */
    {.limit = 45,   .use_voltage = 1, .use_current = 0},    /* PD_POWER_OPTION_MAX_9V */
    {.limit = 60,   .use_voltage = 1, .use_current = 0},    /* PD_POWER_OPTION_MAX_12V */
    {.limit = 75,   .use_voltage = 1, .use_current = 0},    /* PD_POWER_OPTION_MAX_15V */
    {.limit = 100,  .use_voltage = 1, .use_current = 0},    /* PD_POWER_OPTION_MAX_20V */
    {.limit = 100,  .use_voltage = 1, .use_current = 0},    /* PD_POWER_OPTION_MAX_VOLTAGE */
    {.limit = 125,  .use_voltage = 0, .use_current = 1},    /* PD_POWER_OPTION_MAX_CURRENT */
    {.limit = 12500,.use_voltage = 1, .use_current = 1},    /* PD_POWER_OPTION_MAX_POWER */  
};

static void evaluate_src_cap(PD_protocol_t * p)
{
    uint8_t option = p->power_option;
    uint8_t selected = 0;
    if (option < sizeof(power_option_setting) / sizeof(power_option_setting[0])) {
        const PD_power_option_setting_t setting = power_option_setting[option];
        PD_power_info_t info;
        for (uint8_t n = 0; PD_protocol_get_power_info(p, n, &info); n++) {
            uint8_t v = setting.use_voltage ? info.max_v >> 2 : 1;
            uint8_t i = setting.use_current ? info.max_i >> 2 : 1;
            uint16_t power = (uint16_t)v * i;  /* reduce 10-bit power info to 8-bit and use 8-bit x 8-bit multiplication */
            if (power <= setting.limit) {
                selected = n;
            }
        }
    }
    p->power_data_obj_selected = selected;
}

static void parse_header(PD_msg_header_info_t * info, uint16_t header)
{
    /* Reference: USB_PD_R2_0 V1.3 - 20170112   Page 143   6.2.1.1 Message Header */ 
    info->type = (header >> 0) & 0xF;                   /*   3...0  Message Type */
    info->spec_rev = (header >> 6) & 0x3;               /*   7...6  Specification Revision */
    info->id = (header >> 9) & 0x7;                     /*  11...9  MessageID */
    info->num_of_obj = (header >> 12) & 0x7;            /* 14...12  Number of Data Objects */
}

static uint16_t generate_header(PD_protocol_t * p, uint8_t type, uint8_t obj_count)
{
    uint16_t data = ((uint32_t)type << 0) |                       /*   3...0  Message Type */
                    ((uint32_t)PD_SPECIFICATION_REVISION << 6) |  /*   7...6  Specification Revision */
                    ((uint32_t)p->message_id << 9) |              /*  11...9  MessageID */
                    ((uint32_t)obj_count << 12);                  /* 14...12  Number of Data Objects */
    p->tx_msg_name = obj_count ? data_msg_list[type].name : ctrl_msg_list[type].name;
    p->tx_msg_header = data;
    return data;
}

static void handler_good_crc(PD_protocol_t * p, uint16_t header, uint32_t * obj, PD_protocol_event_t * events)
{
    /* Reference: USB_PD_R2_0 V1.3 - 20170112   Page 143   6.2.1.3 Message ID 
       MessageIDCounter Shall be initialized to zero at power-on / reset, increment when receive GoodCRC Message */
    uint8_t message_id = p->message_id;
    if (++message_id > 7) {
        message_id = 0;
    }
    p->message_id = message_id;
}

static void handler_goto_min(PD_protocol_t * p, uint16_t header, uint32_t * obj, PD_protocol_event_t * events)
{
    // Not implemented
}

static void handler_ps_rdy(PD_protocol_t * p, uint16_t header, uint32_t * obj, PD_protocol_event_t * events)
{
    if (events) {
        *events |= PD_PROTOCOL_EVENT_PS_RDY;
    }
}

static void handler_source_cap(PD_protocol_t * p, uint16_t header, uint32_t * obj, PD_protocol_event_t * events)
{
    PD_msg_header_info_t h;
    parse_header(&h, header);
    p->power_data_obj_count = h.num_of_obj;
    for (uint8_t i = 0; i < h.num_of_obj; i++) {
        p->power_data_obj[i] = obj[i];
    }
    evaluate_src_cap(p);
    if (events) {
        *events |= PD_PROTOCOL_EVENT_SRC_CAP;
    }
}

static void handler_vender_def(PD_protocol_t * p, uint16_t header, uint32_t * obj, PD_protocol_event_t * events)
{
    // TODO: implement VDM parsing
}

static bool responder_get_sink_cap(PD_protocol_t * p, uint16_t * header, uint32_t * obj)
{
    /* Reference: USB_PD_R2_0 V1.3 - 20170112   Page 157   6.4.1.2.3 Sink Fixed Supply Power Data Object */
    uint32_t data = ((uint32_t)100 << 0) |                        /* B9...0     Operational Current in 10mA units */
                    ((uint32_t)100 << 10) |                       /* B19...10   Voltage in 50mV units */
                    ((uint32_t)1 << 26) |                         /* B26        USB Communications Capable */
                    ((uint32_t)1 << 28) |                         /* B28        Higher Capability */
                    ((uint32_t)PD_PDO_TYPE_FIXED_SUPPLY << 30);   /* B31...30   Fixed supply */
    *obj = data;
    *header = generate_header(p, PD_DATA_MSG_TYPE_SINK_CAP, 1);
    return true;
}

static bool responder_reject(PD_protocol_t * p, uint16_t * header, uint32_t * obj)
{
    *header = generate_header(p, PD_CONTROL_MSG_TYPE_REJECT, 0);
    return true;
}

static bool responder_soft_reset(PD_protocol_t * p, uint16_t * header, uint32_t * obj)
{
    *header = generate_header(p, PD_CONTROL_MSG_TYPE_ACCEPT, 0);
    return true;
}

static bool responder_source_cap(PD_protocol_t * p, uint16_t * header, uint32_t * obj)
{
    PD_power_info_t info;
    PD_protocol_get_power_info(p, p->power_data_obj_selected, &info);
    /* Reference: USB_PD_R2_0 V1.3 - 20170112   Page 159   6.4.2 Request Message */
    uint32_t req = info.max_i ? info.max_i : info.max_p;
    uint32_t pos = p->power_data_obj_selected + 1;
    uint32_t data = ((uint32_t)req << 0) |    /*  B 9...0   Max Operating Current 10mA units / Max Operating Power in 250mW units */
                    ((uint32_t)req << 10) |   /* B19...10   Operating Current 10mA units / Operating Power in 250mW units */
                    ((uint32_t)1 << 25) |     /* B25        USB Communication Capable */
                    ((uint32_t)pos << 28);    /* B30...28   Object position (000b is Reserved and Shall Not be used) */
    *obj = data;
    *header = generate_header(p, PD_DATA_MSG_TYPE_REQUEST, 1);
    return true;
}

static bool responder_vender_def(PD_protocol_t * p, uint16_t * header, uint32_t * obj)
{
    // TODO: implement VDM respond
    return false;
}

static bool PD_protocol_get_msg_info(const char * name, uint16_t header, PD_msg_info_t * msg_info)
{
    PD_msg_header_info_t h;
    parse_header(&h, header);
    if (msg_info) {
        msg_info->name = name;
        msg_info->id = h.id;
        msg_info->spec_rev = h.spec_rev;
        msg_info->num_of_obj = h.num_of_obj;
        return true;
    }
    return false;
}

void PD_protocol_handle_msg(PD_protocol_t * p, uint16_t header, uint32_t * obj, PD_protocol_event_t * events)
{
    PD_msg_header_info_t h;
    parse_header(&h, header);
    p->msg_state = h.num_of_obj ? &data_msg_list[h.type] : &ctrl_msg_list[h.type];
    p->rx_msg_header = header;
    if (p->msg_state->handler) {
        p->msg_state->handler(p, header, obj, events);
    }
}

bool PD_protocol_respond(PD_protocol_t * p, uint16_t * header, uint32_t * obj)
{
    p->tx_msg_name = "";
    if (p && p->msg_state && p->msg_state->responder && header && obj) {
        return p->msg_state->responder(p, (uint16_t *)header, obj);
    }
    return false;
}

void PD_protocol_create_get_src_cap(PD_protocol_t * p, uint16_t * header)
{
    *header = generate_header(p, PD_DATA_MSG_TYPE_GET_SRC_CAP, 0);
}

void PD_protocol_create_request(PD_protocol_t * p, uint16_t * h, uint32_t * obj)
{
    responder_source_cap(p, h, obj);
}

bool PD_protocol_get_power_info(PD_protocol_t * p, uint8_t index, PD_power_info_t * power_info)
{
    if (p && index < p->power_data_obj_count && power_info) {
        uint32_t obj = p->power_data_obj[index];
        switch (obj >> 30) {
            case PD_PDO_TYPE_FIXED_SUPPLY:
                /* Reference: USB_PD_R2_0 V1.3 - 20170112   Page 154   6.4.1.2.3 Source Fixed Supply Power Data Object */
                power_info->min_v = 0;
                power_info->max_v = (obj >> 10) & 0x3FF;    /*  B19...10  Voltage in 50mV units */
                power_info->max_i = (obj >>  0) & 0x3FF;    /*  B9...0    Max Current in 10mA units */
                power_info->max_p = 0;
                break;
            case PD_PDO_TYPE_BATTERY:
                /* Reference: USB_PD_R2_0 V1.3 - 20170112   Page 156   6.4.1.2.5 Battery Supply Power Data Object */
                power_info->min_v = (obj >> 10) & 0x3FF;    /*  B19...10  Min Voltage in 50mV units */
                power_info->max_v = (obj >> 20) & 0x3FF;    /*  B29...20  Max Voltage in 50mV units */
                power_info->max_i = 0;
                power_info->max_p = (obj >>  0) & 0x3FF;    /*  B9...0    Max Allowable Power in 250mW units */
                break;
            case PD_PDO_TYPE_VARIABLE_SUPPLY:
                /* Reference: USB_PD_R2_0 V1.3 - 20170112   Page 156   6.4.1.2.4 Variable Supply (non-Battery) Power Data Object */
                power_info->min_v = (obj >> 10) & 0x3FF;    /*  B19...10  Min Voltage in 50mV units */
                power_info->max_v = (obj >> 20) & 0x3FF;    /*  B29...20  Max Voltage in 50mV units */
                power_info->max_i = (obj >>  0) & 0x3FF;    /*  B9...0    Max Current in 10mA units */
                power_info->max_p = 0;
                break;
        }
        return true;
    }
    return false;
}

bool PD_protocol_get_tx_msg_info(PD_protocol_t * p, PD_msg_info_t * msg_info)
{
    return p && PD_protocol_get_msg_info(p->tx_msg_name, p->tx_msg_header, msg_info);
}

bool PD_protocol_get_rx_msg_info(PD_protocol_t * p, PD_msg_info_t * msg_info)
{
    return p && PD_protocol_get_msg_info(p->msg_state->name, p->rx_msg_header, msg_info);
}

bool PD_protocol_set_power_option(PD_protocol_t * p, enum PD_power_option_t option)
{
    p->power_option = option;
    if (p->power_data_obj_count > 0) {
        evaluate_src_cap(p);
        return true;    /* need to re-send request */
    }
    return false;
}

bool PD_protocol_select_power(PD_protocol_t * p, uint8_t index)
{
    if (index < p->power_data_obj_count) {
        p->power_data_obj_selected = index;
        return true;    /* need to re-send request */
    }
    return false;
}

void PD_protocol_reset(PD_protocol_t * p)
{
    p->msg_state = &ctrl_msg_list[0];
    p->message_id = 0;
}

void PD_protocol_init(PD_protocol_t * p)
{
    memset(p, 0, sizeof(PD_protocol_t));
    p->msg_state = &ctrl_msg_list[0];
}
