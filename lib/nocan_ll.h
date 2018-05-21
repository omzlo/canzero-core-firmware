#ifndef _NOCAN_LL_H_
#define _NOCAN_LL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define LL_SYS_BIT                      2

#define LL_SYS_ANY                          0
#define LL_SYS_ADDRESS_REQUEST              1
#define LL_SYS_ADDRESS_CONFIGURE            2
#define LL_SYS_ADDRESS_CONFIGURE_ACK        3
#define LL_SYS_ADDRESS_LOOKUP               4
#define LL_SYS_ADDRESS_LOOKUP_ACK           5
#define LL_SYS_NODE_BOOT_REQUEST            6
#define LL_SYS_NODE_BOOT_ACK                7
#define LL_SYS_NODE_PING                    8
#define LL_SYS_NODE_PING_ACK                9

#define LL_SYS_CHANNEL_REGISTER             10
#define LL_SYS_CHANNEL_REGISTER_ACK         11
#define LL_SYS_CHANNEL_UNREGISTER           12
#define LL_SYS_CHANNEL_UNREGISTER_ACK       13

#define LL_SYS_CHANNEL_SUBSCRIBE            14
#define LL_SYS_CHANNEL_UNSUBSCRIBE          15
#define LL_SYS_CHANNEL_LOOKUP               16
#define LL_SYS_CHANNEL_LOOKUP_ACK           17

#define LL_SYS_BOOTLOADER_GET_SIGNATURE		18
#define LL_SYS_BOOTLOADER_GET_SIGNATURE_ACK	19
#define LL_SYS_BOOTLOADER_SET_ADDRESS		20
#define LL_SYS_BOOTLOADER_SET_ADDRESS_ACK	21
#define LL_SYS_BOOTLOADER_WRITE			    22
#define LL_SYS_BOOTLOADER_WRITE_ACK		    23
#define LL_SYS_BOOTLOADER_READ			    24
#define LL_SYS_BOOTLOADER_READ_ACK		    25
#define LL_SYS_BOOTLOADER_LEAVE			    26
#define LL_SYS_BOOTLOADER_LEAVE_ACK		    27
#define LL_SYS_BOOTLOADER_ERASE			    28
#define LL_SYS_BOOTLOADER_ERASE_ACK		    29

#define LL_SYS_DEBUG_MESSAGE                31


#define NOCAN_LL_OK               0
#define NOCAN_LL_ERROR          (-1)
#define NOCAN_LL_RX_TIMEOUT     (-2)
#define NOCAN_LL_TX_TIMEOUT     (-3)
#define NOCAN_LL_NO_FILTER      (-4)

#ifdef ARDUINO
    #include "sam.h"
#else
    #include <samd21.h>
#endif

int nocan_ll_init(uint8_t reset_type);
#define NOCAN_INIT_SOFT_RESET (0)
#define NOCAN_INIT_HARD_RESET (1)

inline int nocan_ll_tx_pending() __attribute__((always_inline));
inline int nocan_ll_tx_pending()
{
    return (PORT->Group[0].IN.reg & PORT_PA27)==0;
}

inline int nocan_ll_rx_pending() __attribute__((always_inline));
inline int nocan_ll_rx_pending()
{
    return (PORT->Group[0].IN.reg & PORT_PA28)==0;
}

int nocan_ll_request_node_id(void);

int nocan_ll_send(uint32_t eid, uint8_t dlen, const uint8_t *data);

int nocan_ll_recv(uint32_t *eid, uint8_t *dlen, uint8_t *data);

int nocan_ll_scan(uint32_t mask, uint32_t value, uint32_t *eid, uint8_t *dlen, uint8_t *data);

int nocan_ll_set_node_id_filter(int8_t node_id);

int nocan_ll_add_channel_filter(uint16_t channel_id);

int nocan_ll_remove_channel_filter(uint16_t channel_id);

uint8_t nocan_ll_get_status(void);

uint8_t nocan_ll_get_error(void);

int nocan_ll_get_udid(uint8_t *dest);

void nocan_ll_set_led(int on);

int nocan_ll_read_registers(uint8_t len, uint8_t *dest);

int nocan_ll_send_virtual_serial(int8_t node_id, const char *msg);

#define EID_SYS_BIT               (1<<18)
#define EID_SYS(node_id,fn,param) (((uint32_t)(node_id)<<21)|((uint32_t)(fn)<<8)|((uint32_t)(param))|EID_SYS_BIT)
#define EID_PUB(node_id,chan_id)  (((uint32_t)(node_id)<<21)|((uint32_t)(chan_id)))
#define EID_SYS_MASK              ((0xFF00)|EID_SYS_BIT)
#define EID_SYS_FUNC(fn)          (((uint32_t)(fn)<<8)|EID_SYS_BIT)
#define EID_GET_SYS_FUNC(eid)     ((uint8_t)(eid>>8))
#define EID_GET_SYS_PARAM(eid)    ((int8_t)(eid))
#define EID_GET_CHANNEL_ID(eid)   ((uint16_t)((eid)&0xFFFF))
#define EID_GET_NODE_ID(eid)      (((eid)>>21&0x7F))

#define nocan_ll_scan_sys(fn, eid, dlen, data) nocan_ll_scan(EID_SYS_MASK, EID_SYS_FUNC(fn), eid, dlen, data);


// STATUS

#define NOCAN_STATUS_RX_PENDING     0x01 
#define NOCAN_STATUS_TX_PENDING     0x02 
#define NOCAN_STATUS_LED            0x04
#define NOCAN_STATUS_ERROR_TX_OVERFLOW  0x10
#define NOCAN_STATUS_ERROR_RX_OVERFLOW  0x20
#define NOCAN_STATUS_ERROR_RX_MESSAGE   0x40

#ifdef __cplusplus
}
#endif


#endif
