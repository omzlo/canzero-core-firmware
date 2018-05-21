#include "nocan_ll.h"
#include "spi.h"

#define SPI_NOCAN_NOP               0x00            
#define SPI_NOCAN_GET_UDID          0x01
#define SPI_NOCAN_GET_SIGNATURE     0x02
#define SPI_NOCAN_LED               0x03
#define SPI_NOCAN_RESET             0x04
#define SPI_NOCAN_GET_ERROR         0x05
#define SPI_NOCAN_GET_STATUS        0x06
#define SPI_NOCAN_STORE_DATA        0x07
#define SPI_NOCAN_STORE_SEND        0x08
#define SPI_NOCAN_FETCH_DATA        0x09
#define SPI_NOCAN_FETCH_ACK         0x0A
#define SPI_NOCAN_FILTER_WRITE      0x0B
#define SPI_NOCAN_FILTER_COMMIT     0x0C
#define SPI_NOCAN_FILTER_READ       0x0D   
#define SPI_NOCAN_NODE_ID           0x0E            

#ifdef ARDUINO
    #define delay_ms(n) delay(n)
    #define now_ms() millis()
#else
    #include "systick.h"
    #define delay_ms(n) systick_delay_ms(n)
    #define now_ms() systick_now_ms()
#endif
/* NOTE: severall functions have a 100ms timeout (approx.), done by cycling 50 times a 2ms timeout. 
 * Normally these timeouts will not be used because these functions are only called if there is data.
 */


/* There are two PINS defined and used here:
 * CAN_RECV_INT is PB27
 * CAN_BUSY_INT is PB28
 */

extern inline int nocan_ll_tx_pending();
extern inline int nocan_ll_rx_pending();

#ifdef DEBUG
int serial_printf(const char *format, ...);
static int retcode(const char *s, int code)
{
    if (code!=NOCAN_LL_OK)
    {
        serial_printf("# %s => %i\r\n",s,code);
    }
    return code;
}
#define RETCODE(c) retcode(__FUNCTION__,c)
#else 
#define RETCODE(c) (c)
#endif

static uint16_t channel_filters[12];

int nocan_ll_init(uint8_t reset_type)
{
    uint8_t buf[4];

    // Configure GPIOs for CAN_RECV_INT and CAN_BUSY_INT as inputs
    PORT->Group[0].DIRCLR.reg = PORT_PA27;
    PORT->Group[0].PINCFG[27].reg = PORT_PINCFG_INEN;
    PORT->Group[0].DIRCLR.reg = PORT_PA28;
    PORT->Group[0].PINCFG[28].reg = PORT_PINCFG_INEN;

    // init SPI and reset
    spi_init(500000/2);

    spi_begin();
    spi_transfer(SPI_NOCAN_RESET);
    spi_transfer(reset_type);
    spi_end();
    
    // BUG: this should not be neeeded.
    delay_ms(250);


    nocan_ll_read_registers(4,buf);
    if (buf[0]!='N' || buf[1]!='C' || buf[2]!='A' || buf[3]!='N')
        return RETCODE(NOCAN_LL_ERROR);

    nocan_ll_set_node_id_filter(0);

    for (int i=0;i<12;i++)
        channel_filters[i] = 0xFFFF;

    return RETCODE(NOCAN_LL_OK);
}

int nocan_ll_request_node_id(void)
{
    int8_t status;
    int8_t node_id;
    uint8_t udid_send[8]; 
    uint8_t udid_recv[8];
    uint8_t attempts,i,rlen;
    uint32_t eid;

    if (nocan_ll_get_udid(udid_send)<0)
        return RETCODE(NOCAN_LL_ERROR);

    for (attempts=0;attempts<3;attempts++) 
    {
        status = nocan_ll_send(EID_SYS(0,LL_SYS_ADDRESS_REQUEST,0),8,udid_send);
    
        if (status!=0)
            return RETCODE(status);

        for (;;)
        {
            status = nocan_ll_scan(EID_SYS_MASK, EID_SYS_FUNC(LL_SYS_ADDRESS_CONFIGURE), &eid, &rlen, udid_recv);

            // status is either NOCAN_LL_RX_TIMEOUT or NOCAN_LL_OK
            // If NOCAN_LL_OK, we still need to check that the response was not destined to another node.
            // If NOCAN_LL_RX_TIMEOUT, we break out of the loop. 
            
            if (status==NOCAN_LL_OK)                // case 1: got the right message, check if it for our device 
            {
                if (rlen==8) {
                    node_id = EID_GET_SYS_PARAM(eid);

                    for (i=0;i<8;i++)
                        if (udid_send[i]!=udid_recv[i]) break;

                    if (i==8) 
                    {
                        nocan_ll_send(EID_SYS(node_id,LL_SYS_ADDRESS_CONFIGURE_ACK,node_id),0,0);
                        return node_id;
                    }
                }
            }
            else if (status==NOCAN_LL_RX_TIMEOUT)   // case 2: we didn't get the right message after 500ms so we exit the loop
            {
                break; // out of for (;;) loop
            }
            else
            {
                return RETCODE(status);             // case 3: something went really wrong
            } 
        }
    }

    // we failed after 3 attempts.
    return RETCODE(NOCAN_LL_ERROR);
}

int nocan_ll_read_registers(uint8_t len, uint8_t *dest)
{
    spi_begin();
    spi_transfer(SPI_NOCAN_GET_SIGNATURE);
    for (int i=0;i<len;i++)
        dest[i] = spi_transfer(0);
    spi_end();

    return RETCODE(NOCAN_LL_OK); 
}

static int _ll_wait_for_rx_pending(void)
{
    uint32_t ref = now_ms();
    uint8_t status;

    do {
        if (nocan_ll_rx_pending())
        {
            status = nocan_ll_get_status();
            if ((status & NOCAN_STATUS_RX_PENDING)!=0)
                return 1;
        }
        delay_ms(5);
    } while ((now_ms()-ref)<250);

    return 0;
}

static int _ll_wait_for_tx_empty(void)
{
    uint32_t ref = now_ms();
    uint8_t status;

    do {
        if (!nocan_ll_tx_pending()) 
        {
            status = nocan_ll_get_status();
            if ((status & NOCAN_STATUS_TX_PENDING)==0)
                return 1;
        }
        delay_ms(5);
    } while ((now_ms()-ref)<250); 

    return 0;
}

int nocan_ll_send(uint32_t eid, uint8_t dlen, const uint8_t *data)
{
    if (!_ll_wait_for_tx_empty())
        return RETCODE(NOCAN_LL_TX_TIMEOUT);

    spi_begin();
    spi_transfer(SPI_NOCAN_STORE_DATA);
    spi_transfer(eid>>24);
    spi_transfer(eid>>16);
    spi_transfer(eid>>8);
    spi_transfer(eid);
    spi_transfer(dlen);
    for (int i=0;i<dlen;i++)
        spi_transfer(data[i]);
    spi_end();

    spi_begin();
    spi_transfer(SPI_NOCAN_STORE_SEND);
    spi_end();

    return RETCODE(NOCAN_LL_OK);
}

int nocan_ll_recv(uint32_t *eid, uint8_t *dlen, uint8_t *data)
{
    uint8_t i;
    uint8_t rlen;
    uint32_t eid0, eid1, eid2, eid3;

    if (!_ll_wait_for_rx_pending()) 
        return RETCODE(NOCAN_LL_RX_TIMEOUT);

    spi_begin();
    spi_transfer(SPI_NOCAN_FETCH_DATA);   
    eid0 = spi_transfer(0);         // 0
    eid1 = spi_transfer(0);         // 1
    eid2 = spi_transfer(0);         // 2
    eid3 = spi_transfer(0);         // 3
    rlen = spi_transfer(0);         // 4

    if (data)
    {
        for (i=0;i<rlen;i++)
            data[i] = spi_transfer(0);
    }
    spi_end();

    if (dlen) *dlen = rlen;

    if (eid) *eid = (eid0<<24)|(eid1<<16)|(eid2<<8)|eid3;

    spi_begin();
    spi_transfer(SPI_NOCAN_FETCH_ACK);
    spi_end();

    return RETCODE(NOCAN_LL_OK);
}

int nocan_ll_scan(uint32_t mask, uint32_t value, uint32_t *eid, uint8_t *len, uint8_t *data)
{
   uint32_t reid;
   int retval;
   uint32_t ref = now_ms();
      
   //serial_printf("scan:\r\n");
   do {    
       retval = nocan_ll_recv(&reid, len, data);

       if (retval!=NOCAN_LL_OK)
           return retval;

       if ((reid&mask) == value)
       {
           //serial_printf("+ (%x & %x) == %x\r\n", reid, mask, value);
           if (eid) *eid = reid;
           return RETCODE(NOCAN_LL_OK);
       }
       //serial_printf("! (%x & %x) != %x\r\n", reid, mask, value);

   } while ((now_ms()-ref)<500);
   //serial_printf("! Timeout\r\n");
   
   return RETCODE(NOCAN_LL_RX_TIMEOUT);
}

int nocan_ll_add_channel_filter(uint16_t channel_id)
{
    uint8_t filter;

    for (filter=0;filter<12;filter++)
    {
        if (channel_filters[filter] == channel_id)
            return RETCODE(NOCAN_LL_OK);
    }

    for (filter=0;filter<12;filter++)
    {
        if (channel_filters[filter] == 0xFFFF)
        {
            spi_begin();
            spi_transfer(SPI_NOCAN_FILTER_WRITE);
            spi_transfer(filter);
            spi_transfer(0xFF);
            spi_transfer(channel_id>>8);
            spi_transfer(channel_id&0xFF);
            spi_end();

            spi_begin();
            spi_transfer(SPI_NOCAN_FILTER_COMMIT);
            spi_transfer(filter);
            spi_end();

            // TODO: read back
            return RETCODE(NOCAN_LL_OK);
        }
    }

    return RETCODE(NOCAN_LL_NO_FILTER);
}

int nocan_ll_remove_channel_filter(uint16_t channel_id)
{
    uint8_t filter;

    for (filter=0;filter<12;filter++)
    {
        if (channel_filters[filter] == channel_id)
        {
            spi_begin();
            spi_transfer(SPI_NOCAN_FILTER_WRITE);
            spi_transfer(filter);
            spi_transfer(0xFF);
            spi_transfer(0xFF);
            spi_transfer(0xFF);
            spi_end();

            spi_begin();
            spi_transfer(SPI_NOCAN_FILTER_COMMIT);
            spi_transfer(filter);
            spi_end();

            // TODO: read back
            return RETCODE(NOCAN_LL_OK);
        }
    }

    return RETCODE(NOCAN_LL_NO_FILTER);
}

int nocan_ll_set_node_id_filter(int8_t node_id)
{
    spi_begin();
    spi_transfer(SPI_NOCAN_NODE_ID);
    spi_transfer(node_id);
    spi_end();

    return RETCODE(NOCAN_LL_OK);
}

uint8_t nocan_ll_get_status(void)
{
    uint8_t ss;

    spi_begin();
    spi_transfer(SPI_NOCAN_GET_STATUS);
    ss = spi_transfer(0);
    spi_end();

    return ss;
}

int nocan_ll_get_udid(uint8_t *dest)
{
    uint8_t i;

    spi_begin();
    spi_transfer(SPI_NOCAN_GET_UDID);
    for (i=0;i<8;i++)
        dest[i]=spi_transfer(0);
    spi_end();

    return RETCODE(NOCAN_LL_OK);
}

void nocan_ll_set_led(int on)
{
    spi_begin();
    spi_transfer(SPI_NOCAN_LED);
    spi_transfer(on);
    spi_end();
}

uint8_t nocan_ll_get_error(void)
{
    uint8_t ee;
    
    spi_begin();
    spi_transfer(SPI_NOCAN_GET_ERROR);
    ee = spi_transfer(0);
    spi_end();

    return ee;
}

int nocan_ll_send_virtual_serial(int8_t node_id, const char *msg)
{
    uint8_t len = 0;
    while (msg[len]!=0 && len<64) len++;
    return nocan_ll_send(EID_SYS(node_id, LL_SYS_DEBUG_MESSAGE, 0), len, (const uint8_t *)msg);   
}
