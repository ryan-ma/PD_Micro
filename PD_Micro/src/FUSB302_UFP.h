
/**
 * FUSB302_UFP.h
 *
 *  Updated on: Jan 4, 2021
 *      Author: Ryan Ma
 *
 * Minimalist USB PD implement with only UFP(device) functionality
 * Requires only stdint.h and string.h
 * No use of bit-field for better cross-platform compatibility
 *
 * FUSB302 can support PD3.0 with limitations and workarounds
 * - Do not have enough FIFO for unchunked message, use chunked message instead
 * - VBUS sense low threshold at 4V, disable vbus_sense if request PPS below 4V
 * 
 */

#ifndef FUSB302_UFP_H
#define FUSB302_UFP_H

#include <stdint.h>

enum {
    FUSB302_SUCCESS             = 0,
    FUSB302_BUSY                = (1 << 0),
    FUSB302_ERR_PARAM           = (1 << 1),
    FUSB302_ERR_DEVICE_ID       = (1 << 2),
    FUSB302_ERR_READ_DEVICE     = (1 << 3),
    FUSB302_ERR_WRITE_DEVICE    = (1 << 4)
};
typedef uint8_t FUSB302_ret_t;

#define FUSB302_EVENT_ATTACHED          (1 << 0)
#define FUSB302_EVENT_DETACHED          (1 << 1)
#define FUSB302_EVENT_RX_SOP            (1 << 2)
#define FUSB302_EVENT_GOOD_CRC_SENT     (1 << 3)
typedef uint8_t FUSB302_event_t;

typedef struct {
    /* setup by user */
    uint8_t i2c_address;
    FUSB302_ret_t (*i2c_read)(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint8_t count);
    FUSB302_ret_t (*i2c_write)(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint8_t count);
    FUSB302_ret_t (*delay_ms)(uint32_t t);

    /* used by this library */
    const char * err_msg;
    uint16_t rx_header;
    uint8_t rx_buffer[32];
    uint8_t reg_control[15];
    uint8_t reg_status[7];
    
    uint8_t interrupta;
    uint8_t interruptb;
    uint8_t cc1;
    uint8_t cc2;
    uint8_t state;
    uint8_t vbus_sense;
} FUSB302_dev_t;

static inline const char * FUSB302_get_last_err_msg(FUSB302_dev_t *dev) { return dev->err_msg; }

FUSB302_ret_t FUSB302_init            (FUSB302_dev_t *dev);
FUSB302_ret_t FUSB302_pd_reset        (FUSB302_dev_t *dev);
FUSB302_ret_t FUSB302_pdwn_cc         (FUSB302_dev_t *dev, uint8_t enable);
FUSB302_ret_t FUSB302_set_vbus_sense  (FUSB302_dev_t *dev, uint8_t enable);
FUSB302_ret_t FUSB302_get_ID          (FUSB302_dev_t *dev, uint8_t *version_ID, uint8_t *revision_ID);
FUSB302_ret_t FUSB302_get_cc          (FUSB302_dev_t *dev, uint8_t *cc1, uint8_t *cc2);
FUSB302_ret_t FUSB302_get_vbus_level  (FUSB302_dev_t *dev, uint8_t *vbus);
FUSB302_ret_t FUSB302_get_message     (FUSB302_dev_t *dev, uint16_t *header, uint32_t *data);
FUSB302_ret_t FUSB302_tx_sop          (FUSB302_dev_t *dev, uint16_t header, const uint32_t *data);
FUSB302_ret_t FUSB302_tx_hard_reset   (FUSB302_dev_t *dev);
FUSB302_ret_t FUSB302_alert           (FUSB302_dev_t *dev, FUSB302_event_t *events);

#endif /* FUSB302_H */

