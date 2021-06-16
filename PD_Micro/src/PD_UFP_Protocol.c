
/**
 * PD_UFP_Protocol.c
 *
 *  Updated on: Jan 25, 2021
 *      Author: Ryan Ma
 *
 * Minimalist USB PD implement with only UFP(device) sink only functionality
 * Requires PD PHY to do automatic GoodCRC response on valid SOP messages.
 * Requires only stdint.h, stdbool.h and string.h
 * No use of bit-field for better cross-platform compatibility
 *
 * Support PD3.0 PPS
 * Do not support extended message. Not necessary for PD trigger and PPS.
 * 
 * Reference: USB_PD_R2_0 V1.3 - 20170112
 *            USB_PD_R3_0 V2.0 20190829 + ECNs 2020-12-10
 *            - Chapter 6. Protocol Layer
 *
 */
 
#include <string.h>
#include "PD_UFP_Protocol.h"

#define PD_SPECIFICATION_REVISION           0x2

#define PD_CONTROL_MSG_TYPE_ACCEPT          0x3
#define PD_CONTROL_MSG_TYPE_REJECT          0x4
#define PD_CONTROL_MSG_TYPE_GET_SRC_CAP     0x7
#define PD_CONTROL_MSG_TYPE_NOT_SUPPORT     0x10
#define PD_CONTROL_MSG_TYPE_GET_PPS_STATUS  0x14

#define PD_DATA_MSG_TYPE_REQUEST            0x2
#define PD_DATA_MSG_TYPE_SINK_CAP           0x4
#define PD_DATA_MSG_TYPE_VENDOR_DEFINED     0xF

#define PD_EXT_MSG_TYPE_SINK_CAP_EXT        0xF

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

/* Optimize RAM usage on AVR MCU by allocate const in PROGMEM */
#if defined(__AVR__)
#include <avr/pgmspace.h>
#define SET_MSG_STAGE(d, s) do { static struct PD_msg_state_t m; memcpy_P(&m, s, sizeof(struct PD_msg_state_t)); d = &m; } while (0)
#define SET_MSG_NAME(d, s)  do { static char n[16]; strncpy_P(n, s, 15); d = n; } while (0)
#define COPY_PDO(d, s)      do { memcpy_P(&d, &s, 4); } while (0)
#else
#define PROGMEM
#define SET_MSG_STAGE(d, s) do { d = s; } while (0)
#define SET_MSG_NAME(d, s)  do { d = s; } while (0)
#define COPY_PDO(d, s)      do { d = s; } while (0)
#endif

#define T(name) static const char str_ ## name [] PROGMEM = #name

static void handler_good_crc   (PD_protocol_t * p, uint16_t header, uint32_t * obj, PD_protocol_event_t * events);
static void handler_goto_min   (PD_protocol_t * p, uint16_t header, uint32_t * obj, PD_protocol_event_t * events);
static void handler_accept     (PD_protocol_t * p, uint16_t header, uint32_t * obj, PD_protocol_event_t * events);
static void handler_reject     (PD_protocol_t * p, uint16_t header, uint32_t * obj, PD_protocol_event_t * events);
static void handler_ps_rdy     (PD_protocol_t * p, uint16_t header, uint32_t * obj, PD_protocol_event_t * events);
static void handler_source_cap (PD_protocol_t * p, uint16_t header, uint32_t * obj, PD_protocol_event_t * events);
static void handler_BIST       (PD_protocol_t * p, uint16_t header, uint32_t * obj, PD_protocol_event_t * events);
static void handler_alert      (PD_protocol_t * p, uint16_t header, uint32_t * obj, PD_protocol_event_t * events);
static void handler_vender_def (PD_protocol_t * p, uint16_t header, uint32_t * obj, PD_protocol_event_t * events);
static void handler_PPS_Status (PD_protocol_t * p, uint16_t header, uint32_t * obj, PD_protocol_event_t * events);

static bool responder_get_sink_cap  (PD_protocol_t * p, uint16_t * header, uint32_t * obj);
static bool responder_reject        (PD_protocol_t * p, uint16_t * header, uint32_t * obj);
static bool responder_soft_reset    (PD_protocol_t * p, uint16_t * header, uint32_t * obj);
static bool responder_source_cap    (PD_protocol_t * p, uint16_t * header, uint32_t * obj);
static bool responder_vender_def    (PD_protocol_t * p, uint16_t * header, uint32_t * obj);
static bool responder_sink_cap_ext  (PD_protocol_t * p, uint16_t * header, uint32_t * obj);
static bool responder_not_support   (PD_protocol_t * p, uint16_t * header, uint32_t * obj);

T(C0); T(GoodCRC); T(GotoMin); T(Accept); T(Reject); T(Ping); T(PS_RDY); T(Get_Src_Cap);
T(Get_Sink_Cap); T(DR_Swap); T(PR_Swap); T(VCONN_Swap); T(Wait); T(Soft_Rst); T(Dat_Rst); T(Dat_Rst_Cpt);
T(NS); T(Get_Src_Ext); T(Get_Stat); T(FR_Swap); T(Get_PPS_Stat); T(Get_CC); T(Get_Sink_Ext); T(C_R);

static const struct PD_msg_state_t ctrl_msg_list[] PROGMEM = {
    {.name = str_C0,            .handler = 0,                   .responder = 0},
    {.name = str_GoodCRC,       .handler = handler_good_crc,    .responder = 0},
    {.name = str_GotoMin,       .handler = handler_goto_min,    .responder = 0},
    {.name = str_Accept,        .handler = handler_accept,      .responder = 0},
    {.name = str_Reject,        .handler = handler_reject,      .responder = 0},
    {.name = str_Ping,          .handler = 0,                   .responder = 0},
    {.name = str_PS_RDY,        .handler = handler_ps_rdy,      .responder = 0},
    {.name = str_Get_Src_Cap,   .handler = 0,                   .responder = responder_not_support},
    {.name = str_Get_Sink_Cap,  .handler = 0,                   .responder = responder_get_sink_cap},
    {.name = str_DR_Swap,       .handler = 0,                   .responder = responder_reject},
    {.name = str_PR_Swap,       .handler = 0,                   .responder = responder_not_support},
    {.name = str_VCONN_Swap,    .handler = 0,                   .responder = responder_reject},
    {.name = str_Wait,          .handler = 0,                   .responder = 0},
    {.name = str_Soft_Rst,      .handler = 0,                   .responder = responder_soft_reset},
    {.name = str_Dat_Rst,       .handler = 0,                   .responder = 0},
    {.name = str_Dat_Rst_Cpt,   .handler = 0,                   .responder = 0},
    
    {.name = str_NS,            .handler = 0,                   .responder = 0},
    {.name = str_Get_Src_Ext,   .handler = 0,                   .responder = responder_not_support},
    {.name = str_Get_Stat,      .handler = 0,                   .responder = responder_not_support},
    {.name = str_FR_Swap,       .handler = 0,                   .responder = responder_not_support},
    {.name = str_Get_PPS_Stat,  .handler = 0,                   .responder = responder_not_support},
    {.name = str_Get_CC,        .handler = 0,                   .responder = responder_not_support},
    {.name = str_Get_Sink_Ext,  .handler = 0,                   .responder = responder_sink_cap_ext},
    {.name = str_C_R,           .handler = 0,                   .responder = responder_not_support},
};

T(D0); T(Src_Cap); T(Request); T(BIST); T(Sink_Cap); T(Bat_Stat); T(Alert); T(Get_CI);
T(Enter_USB); T(D9); T(D10); T(D11); T(D12); T(D13); T(D14); T(VDM);
T(D_R); 

static const struct PD_msg_state_t data_msg_list[] PROGMEM = {
    {.name = str_D0,            .handler = 0,                   .responder = 0},
    {.name = str_Src_Cap,       .handler = handler_source_cap,  .responder = responder_source_cap},
    {.name = str_Request,       .handler = 0,                   .responder = responder_not_support},
    {.name = str_BIST,          .handler = handler_BIST,        .responder = 0},
    {.name = str_Sink_Cap,      .handler = 0,                   .responder = responder_not_support},
    {.name = str_Bat_Stat,      .handler = 0,                   .responder = responder_not_support},
    {.name = str_Alert,         .handler = handler_alert,       .responder = 0},
    {.name = str_Get_CI,        .handler = 0,                   .responder = responder_not_support},
    {.name = str_Enter_USB,     .handler = 0,                   .responder = 0},
    {.name = str_D9,            .handler = 0,                   .responder = 0},
    {.name = str_D10,           .handler = 0,                   .responder = 0},
    {.name = str_D11,           .handler = 0,                   .responder = 0},
    {.name = str_D12,           .handler = 0,                   .responder = 0},
    {.name = str_D13,           .handler = 0,                   .responder = 0},
    {.name = str_D14,           .handler = 0,                   .responder = 0},
    {.name = str_VDM,           .handler = handler_vender_def,  .responder = responder_vender_def},

    {.name = str_D_R,           .handler = 0,                   .responder = responder_not_support},
};

T(E0); T(Src_Cap_Ext); T(Status); T(Get_Bat_cap); T(Get_Bat_Stat); T(Bat_Cap); T(Get_Mfg_Info); T(Mfg_Info);
T(Sec_Request); T(Sec_Response); T(FU_request); T(FU_Response); T(PPS_Stat); T(Country_Info); T(Country_Code); T(Sink_Cap_Ext);
T(E_R);

static const struct PD_msg_state_t ext_msg_list[] PROGMEM = {
    {.name = str_E0,            .handler = 0,                   .responder = responder_not_support},
    {.name = str_Src_Cap_Ext,   .handler = 0,                   .responder = 0},
    {.name = str_Status,        .handler = 0,                   .responder = 0},
    {.name = str_Get_Bat_cap,   .handler = 0,                   .responder = responder_not_support},
    {.name = str_Get_Bat_Stat,  .handler = 0,                   .responder = responder_not_support},
    {.name = str_Bat_Cap,       .handler = 0,                   .responder = 0},
    {.name = str_Get_Mfg_Info,  .handler = 0,                   .responder = responder_not_support},
    {.name = str_Mfg_Info,      .handler = 0,                   .responder = 0},
    {.name = str_Sec_Request,   .handler = 0,                   .responder = responder_not_support},
    {.name = str_Sec_Response,  .handler = 0,                   .responder = 0},
    {.name = str_FU_request,    .handler = 0,                   .responder = responder_not_support},
    {.name = str_FU_Response,   .handler = 0,                   .responder = 0},
    {.name = str_PPS_Stat,      .handler = handler_PPS_Status,  .responder = 0},
    {.name = str_Country_Info,  .handler = 0,                   .responder = 0},
    {.name = str_Country_Code,  .handler = 0,                   .responder = 0},
    {.name = str_Sink_Cap_Ext,  .handler = 0,                   .responder = responder_not_support},

    {.name = str_E_R,           .handler = 0,                   .responder = responder_not_support},
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

static uint8_t evaluate_src_cap(PD_protocol_t * p, uint16_t PPS_voltage, uint8_t PPS_current)
{
    const PD_power_option_setting_t * setting;
    PD_power_info_t info;
    uint8_t option = p->power_option;
    uint8_t selected = 0;

    /* If selected option is not available, use first PDO. Reference: 6.4.1 Capabilities Message
       The vSafe5V Fixed Supply Object Shall always be the first object. */
    if (option >= sizeof(power_option_setting) / sizeof(power_option_setting[0])) {
        return 0;
    }

    setting = &power_option_setting[option];
    for (uint8_t n = 0; PD_protocol_get_power_info(p, n, &info); n++) {
        if (info.type == PD_PDO_TYPE_AUGMENTED_PDO) {
            uint16_t pps_v = PPS_voltage * 2;    /* Voltage in 20mV units */
            uint16_t pps_i = PPS_current * 5;    /* Current in 50mA units */
            /* PD_power_info_t: Voltage in 50mV units, Current in 10mA units */
            if (info.min_v * 5 <= pps_v && pps_v <= info.max_v * 5 && pps_i <= info.max_i) {
                return n;
            }
        } else {
            uint8_t v = setting->use_voltage ? info.max_v >> 2 : 1;
            uint8_t i = setting->use_current ? info.max_i >> 2 : 1;
            uint16_t power = (uint16_t)v * i;  /* reduce 10-bit power info to 8-bit and use 8-bit x 8-bit multiplication */
            if (power <= setting->limit) {
                selected = n;
            }
        }
    }
    return selected;
}

static void parse_header(PD_msg_header_info_t * info, uint16_t header)
{
    /* Reference: 6.2.1.1 Message Header */ 
    info->type = (header >> 0) & 0x1F;                  /*   4...0  Message Type */
    info->spec_rev = (header >> 6) & 0x3;               /*   7...6  Specification Revision */
    info->id = (header >> 9) & 0x7;                     /*  11...9  MessageID */
    info->num_of_obj = (header >> 12) & 0x7;            /* 14...12  Number of Data Objects */
}

static uint16_t generate_header(PD_protocol_t * p, uint8_t type, uint8_t obj_count)
{
    /* Reference: 6.2.1.1 Message Header */ 
    uint16_t h = ((uint16_t)type << 0) |                      /*   4...0  Message Type */
                 ((uint16_t)PD_SPECIFICATION_REVISION << 6) | /*   7...6  Specification Revision */
                 ((uint16_t)p->message_id << 9) |             /*  11...9  MessageID */
                 ((uint16_t)obj_count << 12);                 /* 14...12  Number of Data Objects */
    p->tx_msg_header = h;
    return h;
}

static uint16_t generate_header_ext(PD_protocol_t * p, uint8_t type, uint8_t data_size, uint32_t * obj)
{
    uint16_t h = generate_header(p, type, (data_size + 5) >> 2); /* set obj_count to fit ext header and data */
    h |= (uint16_t)1 << 15;     /* Set extended field */
    /* Reference: 6.2.1.2 Extended Message Headerr */ 
    obj[0] |= ((uint16_t)data_size << 0) |  /*   8...0  ata Size */
              /* Assume short message, set Chunk Number and Request Chunk to 0 */
              ((uint16_t)1 << 15);          /*      15  Chunked */
    p->tx_msg_header = h;
    return h;
}

static void handler_good_crc(PD_protocol_t * p, uint16_t header, uint32_t * obj, PD_protocol_event_t * events)
{
    /* Reference: 6.2.1.3 Message ID 
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

static void handler_accept(PD_protocol_t * p, uint16_t header, uint32_t * obj, PD_protocol_event_t * events)
{
    if (events) {
        *events |= PD_PROTOCOL_EVENT_ACCEPT;
    }
}

static void handler_reject(PD_protocol_t * p, uint16_t header, uint32_t * obj, PD_protocol_event_t * events)
{
    if (events) {
        *events |= PD_PROTOCOL_EVENT_PS_RDY;
    }
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
    p->power_data_obj_selected = evaluate_src_cap(p, p->PPS_voltage, p->PPS_current);
    if (events) {
        *events |= PD_PROTOCOL_EVENT_SRC_CAP;
    }
}

static void handler_BIST(PD_protocol_t * p, uint16_t header, uint32_t * obj, PD_protocol_event_t * events)
{
    // TODO: implement BIST
}

static void handler_alert(PD_protocol_t * p, uint16_t header, uint32_t * obj, PD_protocol_event_t * events)
{
    // TODO: implement alert
}

static void handler_vender_def(PD_protocol_t * p, uint16_t header, uint32_t * obj, PD_protocol_event_t * events)
{
    // TODO: implement VDM parsing
}

static void handler_PPS_Status(PD_protocol_t * p, uint16_t header, uint32_t * obj, PD_protocol_event_t * events)
{
    /* Handle chunked Extended message,  Offset 2 byte for Extended Message Header */
    p->PPSSDB[0] = (obj[0] >> 16) & 0xFF;
    p->PPSSDB[1] = (obj[0] >> 24) & 0xFF;
    p->PPSSDB[2] = (obj[1] >>  0) & 0xFF;
    p->PPSSDB[3] = (obj[1] >>  8) & 0xFF;
    if (events) {
        *events |= PD_PROTOCOL_EVENT_PPS_STATUS;
    }
}

static bool responder_get_sink_cap(PD_protocol_t * p, uint16_t * header, uint32_t * obj)
{
    /* Reference: 6.4.1.2.3 Sink Fixed Supply Power Data Object */
    uint32_t data = ((uint32_t)100 << 0) |                        /* B9...0     Operational Current in 10mA units */
                    ((uint32_t)100 << 10) |                       /* B19...10   Voltage in 50mV units */
                    ((uint32_t)1 << 26) |                         /* B26        USB Communications Capable */
                    ((uint32_t)1 << 28) |                         /* B28        Higher Capability */
                    ((uint32_t)PD_PDO_TYPE_FIXED_SUPPLY << 30);   /* B31...30   Fixed supply */
    *obj = data; /* Only implement 5V 1A Fix supply PDO. Source rarely request sink cap */
    *header = generate_header(p, PD_DATA_MSG_TYPE_SINK_CAP, 1);
    return true;
}

static bool responder_sink_cap_ext(PD_protocol_t * p, uint16_t * header, uint32_t * obj)
{
    /* Reference: 6.5.13 Sink_Capabilities_Extended Message 
                  6.12.3 Applicability of Extended Messages  (Normative; Shall be supported) */
    #define SINK_CAP_VID                0
    #define SINK_CAP_PID                0
    #define SINK_CAP_XID                0       /* If the vendor does not have an XID, then it Shall return zero */
    #define SINK_CAP_FW_Version         1
    #define SINK_CAP_HW_Version         1
    #define SINK_CAP_SKEDB_Version      1
    #define SINK_CAP_SINK_MODE          0x3     /* Bit 0: PPS charging supported, Bit 1: VBUS powered */
    #define SINK_CAP_SINK_MIN_PDP       5       /* Minimum     PD Power in Watt */
    #define SINK_CAP_SINK_OP_PDP        5       /* Operational PD Power in Watt */
    #define SINK_CAP_SINK_MAX_PDP       100     /* Maximum     PD Power in Watt */
    static const uint32_t SKEDB[6] PROGMEM = { /* 2-byte header + 21-byte data, chunked to 6 PDO */
    /* PDO[0], data byte  0...1  */
        /* 16-bit LSB is reserved for Extended Message Header */
        ((uint32_t)SINK_CAP_VID << 16),             /* Byte  0...1  VID */
    /* PDO[1], data byte  2...5  */
        ((uint32_t)SINK_CAP_PID << 0) |             /* Byte  2...3  PID */
        (((uint32_t)SINK_CAP_XID & 0xFF) << 16),    /* Byte  4...5  XID */
    /* PDO[2], data byte  6...9  */
        (((uint32_t)SINK_CAP_XID >> 16) << 0) |     /* Byte  6...7  XID */
        ((uint32_t)SINK_CAP_FW_Version << 16) |     /* Byte      8  FW Version */
        ((uint32_t)SINK_CAP_HW_Version << 24),      /* Byte      9  HW Version */
    /* PDO[3], data byte 10...13 */
        ((uint32_t)SINK_CAP_SKEDB_Version << 0),    /* Byte     10  SKEDB Version */
        /* Not set Byte 11 Load Step, Byte 13..12 Sink Load Characteristics */
    /* PDO[4], data byte 14...17 */
        /* Not set Byte 14 Compliance, Byte 15 Touch Temp, Byte 16 Battery Info */
        ((uint32_t)SINK_CAP_SINK_MODE << 24),       /* Byte     17  Sink Modes */
    /* PDO[5], data byte 18...20 */
        ((uint32_t)SINK_CAP_SINK_MIN_PDP << 0) |    /* Byte     18  Minimum PDP */
        ((uint32_t)SINK_CAP_SINK_OP_PDP << 8) |     /* Byte     19  Operational PDP */
        ((uint32_t)SINK_CAP_SINK_MAX_PDP << 16)     /* Byte     20  Maximum PDP */
    };
    uint8_t i;
    for (i = 0; i < 6; i++) {
        COPY_PDO(obj[i], SKEDB[i]);
    }
    *header = generate_header_ext(p, PD_EXT_MSG_TYPE_SINK_CAP_EXT, 21, obj);
    return false;
}

static bool responder_reject(PD_protocol_t * p, uint16_t * header, uint32_t * obj)
{
    *header = generate_header(p, PD_CONTROL_MSG_TYPE_REJECT, 0);
    return true;
}

static bool responder_not_support(PD_protocol_t * p, uint16_t * header, uint32_t * obj)
{
    *header = generate_header(p, PD_CONTROL_MSG_TYPE_NOT_SUPPORT, 0);
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
    uint32_t data, pos = p->power_data_obj_selected + 1;
    PD_protocol_get_power_info(p, p->power_data_obj_selected, &info);
    /* Reference: 6.4.2 Request Message */
    if (info.type == PD_PDO_TYPE_AUGMENTED_PDO) {
        /* NOTE: To compatible PD2.0 PHY, do not set Unchunked Extended Messages Supported */
        data = ((uint32_t)p->PPS_current << 0) |    /* B6 ...0    Operating Current 50mA units */
               ((uint32_t)p->PPS_voltage << 9) |    /* B19...9    Output Voltage in 20mV units */
               ((uint32_t)1 << 25) |                /* B25        USB Communication Capable */
               ((uint32_t)pos << 28);               /* B30...28   Object position (000b is Reserved and Shall Not be used) */
    } else {
        uint32_t req = info.max_i ? info.max_i : info.max_p;
        data = ((uint32_t)req << 0) |    /* B9 ...0    Max Operating Current 10mA units / Max Operating Power in 250mW units */
               ((uint32_t)req << 10) |   /* B19...10   Operating Current 10mA units / Operating Power in 250mW units */
               ((uint32_t)1 << 25) |     /* B25        USB Communication Capable */
               ((uint32_t)pos << 28);    /* B30...28   Object position (000b is Reserved and Shall Not be used) */
    }
    *obj = data;
    *header = generate_header(p, PD_DATA_MSG_TYPE_REQUEST, 1);
    return true;
}

static bool responder_vender_def(PD_protocol_t * p, uint16_t * header, uint32_t * obj)
{
    // TODO: implement VDM respond
    return false;
}

void PD_protocol_handle_msg(PD_protocol_t * p, uint16_t header, uint32_t * obj, PD_protocol_event_t * events)
{
    #define EXT_MSG_LIMIT   (sizeof(ext_msg_list) / sizeof(ext_msg_list[0]) - 1)
    #define DATA_MSG_LIMIT  (sizeof(data_msg_list) / sizeof(data_msg_list[0]) - 1)
    #define CTRL_MSG_LIMIT  (sizeof(ctrl_msg_list) / sizeof(ctrl_msg_list[0]) - 1)

    const struct PD_msg_state_t * state;
    PD_msg_header_info_t h;
    parse_header(&h, header);
    p->rx_msg_header = header;
    if ((header >> 15) & 0x1) {
        state = &ext_msg_list[h.type > EXT_MSG_LIMIT ? EXT_MSG_LIMIT : h.type];
    } else if (h.num_of_obj) {
        state = &data_msg_list[h.type > DATA_MSG_LIMIT ? DATA_MSG_LIMIT : h.type];
    } else {
        state =&ctrl_msg_list[h.type > CTRL_MSG_LIMIT ? CTRL_MSG_LIMIT : h.type];
    }
    SET_MSG_STAGE(p->msg_state, state);
    if (p->msg_state->handler) {
        p->msg_state->handler(p, header, obj, events);
    }
}

bool PD_protocol_respond(PD_protocol_t * p, uint16_t * header, uint32_t * obj)
{
    if (p && p->msg_state && p->msg_state->responder && header && obj) {
        return p->msg_state->responder(p, (uint16_t *)header, obj);
    }
    return false;
}

void PD_protocol_create_get_src_cap(PD_protocol_t * p, uint16_t * header)
{
    *header = generate_header(p, PD_CONTROL_MSG_TYPE_GET_SRC_CAP, 0);
}

void PD_protocol_create_get_PPS_status(PD_protocol_t *p, uint16_t *header)
{
    *header = generate_header(p, PD_CONTROL_MSG_TYPE_GET_PPS_STATUS, 0);
}

void PD_protocol_create_request(PD_protocol_t * p, uint16_t * header, uint32_t * obj)
{
    responder_source_cap(p, header, obj);
}

bool PD_protocol_get_power_info(PD_protocol_t * p, uint8_t index, PD_power_info_t * power_info)
{
    if (p && index < p->power_data_obj_count && power_info) {
        uint32_t obj = p->power_data_obj[index];
        power_info->type = obj >> 30;
        switch (power_info->type) {
        case PD_PDO_TYPE_FIXED_SUPPLY:
            /* Reference: 6.4.1.2.3 Source Fixed Supply Power Data Object */
            power_info->min_v = 0;
            power_info->max_v = (obj >> 10) & 0x3FF;    /*  B19...10  Voltage in 50mV units */
            power_info->max_i = (obj >>  0) & 0x3FF;    /*  B9 ...0   Max Current in 10mA units */
            power_info->max_p = 0;
            break;
        case PD_PDO_TYPE_BATTERY:
            /* Reference: 6.4.1.2.5 Battery Supply Power Data Object */
            power_info->min_v = (obj >> 10) & 0x3FF;    /*  B19...10  Min Voltage in 50mV units */
            power_info->max_v = (obj >> 20) & 0x3FF;    /*  B29...20  Max Voltage in 50mV units */
            power_info->max_i = 0;
            power_info->max_p = (obj >>  0) & 0x3FF;    /*  B9 ...0   Max Allowable Power in 250mW units */
            break;
        case PD_PDO_TYPE_VARIABLE_SUPPLY:
            /* Reference: 6.4.1.2.4 Variable Supply (non-Battery) Power Data Object */
            power_info->min_v = (obj >> 10) & 0x3FF;    /*  B19...10  Min Voltage in 50mV units */
            power_info->max_v = (obj >> 20) & 0x3FF;    /*  B29...20  Max Voltage in 50mV units */
            power_info->max_i = (obj >>  0) & 0x3FF;    /*  B9 ...0   Max Current in 10mA units */
            power_info->max_p = 0;
            break;
        case PD_PDO_TYPE_AUGMENTED_PDO:
            /* Reference: 6.4.1.3.4 Programmable Power Supply Augmented Power Data Object */
            power_info->max_v = ((obj >> 17) & 0xFF) * 2;   /*  B24...17  Max Voltage in 100mV units */
            power_info->min_v = ((obj >>  8) & 0xFF) * 2;   /*  B15...8   Min Voltage in 100mV units */
            power_info->max_i = ((obj >>  0) & 0x7F) * 5;   /*  B6 ...0   Max Current in 50mA units */
            power_info->max_p = 0;
            break;
        }
        return true;
    }
    return false;
}

bool PD_protocol_get_msg_info(uint16_t header, PD_msg_info_t * msg_info)
{
    PD_msg_header_info_t h;
    parse_header(&h, header);
    if (msg_info) {
        const char * name;
        const struct PD_msg_state_t * state;
        uint8_t type = h.type;
        SET_MSG_STAGE(state, header & 0x8000 ? &ext_msg_list[type] :
                        h.num_of_obj ? &data_msg_list[type] : &ctrl_msg_list[type]);
        SET_MSG_NAME(name, state->name);
        msg_info->name = name;
        msg_info->id = h.id;
        msg_info->spec_rev = h.spec_rev;
        msg_info->num_of_obj = h.num_of_obj;
        msg_info->extended = header >> 15;
        return true;
    }
    return false;
}

bool PD_protocol_get_PPS_status(PD_protocol_t *p, PPS_status_t * PPS_status)
{
    if (p && PPS_status) {
        /* Reference: 6.5.10 PPS_Status Message */
        PPS_status->output_voltage = ((uint16_t)p->PPSSDB[1] << 8) | p->PPSSDB[0];
        PPS_status->output_current = p->PPSSDB[2];
        PPS_status->flag_PTF = (p->PPSSDB[3] >> 1) & 0x3;   /* Bit 1 ... 2 */
        PPS_status->flag_OMF = (p->PPSSDB[3] >> 3) & 0x1;   /* Bit 3 */
        return true;
    }
    return false;
}

bool PD_protocol_set_power_option(PD_protocol_t * p, enum PD_power_option_t option)
{
    p->power_option = option;
    p->PPS_voltage = 0;
    p->PPS_current = 0;
    if (p->power_data_obj_count > 0) {
        p->power_data_obj_selected = evaluate_src_cap(p, p->PPS_voltage, p->PPS_current);
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

bool PD_protocol_set_PPS(PD_protocol_t * p, uint16_t PPS_voltage, uint8_t PPS_current, bool strict)
{
    if (p->PPS_voltage != PPS_voltage || p->PPS_current != PPS_current) {
        uint8_t selected = evaluate_src_cap(p, PPS_voltage, PPS_current);
        if (selected || !strict) {
            p->PPS_voltage = PPS_voltage;
            p->PPS_current = PPS_current;
            p->power_data_obj_selected = selected;
            return true;    /* need to re-send request */            
        }
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
