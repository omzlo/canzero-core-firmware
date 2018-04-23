#ifndef _NOCAN_H_
#define _NOCAN_H_

#include <stdint.h>
#include <nocan_ll.h>

/*
    EID format used:
    pos size    description
    ------------------------
    0   1       first_packet_flag   (0=>not the fist packet in a block, 1=> is the first packet)
    1   7       node_id      
    8   1       last_packet_flag    (0=>not the last packet in a block, 1=> is not the last packet)
    9   1       reserved 
    10  1       sys_flag

    -- if sys_flag == 0 then
    11  2       reserved           
    13  16      channel_id

    -- if sys_flag == 1 then
    11  2       reserved
    13  8       function
    21  8       parameter
       
    ------------------------
    29  -       total length
*/ 


typedef int16_t NocanChannelId;
typedef int8_t NocanNodeId;
typedef struct {
    NocanNodeId node_id;
    NocanChannelId channel_id;
    uint8_t data_len;
    uint8_t data[64];
} NocanMessage;

class NocanClass {
    public:
        enum {  
            OK = 0,
            ERROR = -1,
            TIMEOUT = -2,
            NO_DATA = -3,
            NO_BUFFER_AVAILABLE = -4,
            SYSTEM_MESSAGE = -32,
            WORKING = -33
        };

        NocanNodeId open(void); 

        int8_t close(void) {            
            led(false);
            /*  TODO: send close sys message */
            return NocanClass::OK;
        }

        int8_t lookupChannel(const char *channel, NocanChannelId *channel_id) const;

        int8_t registerChannel(const char *channel, NocanChannelId *channel_id) const;

        int8_t unregisterChannel(NocanChannelId channel_id) const;

        int8_t subscribeChannel(NocanChannelId channel_id) const;

        int8_t lookupAndSubscribeChannel(const char *channel, NocanChannelId *channel_id) const { 
            uint8_t status;
            status = lookupChannel(channel,channel_id);
            if (status<0)
                return NocanClass::ERROR;
            return subscribeChannel(*channel_id);
        }

        int8_t unsubscribeChannel(NocanChannelId channel_id) const;

        int8_t publishMessage(const NocanMessage &msg) const;
        
        int8_t publishMessage(NocanChannelId cid, const char *str) const;

        int8_t receiveMessage(NocanMessage *msg) const;
        
        int8_t getUniqueDeviceIdentifier(uint8_t *dest) const { 
            return nocan_ll_get_udid(dest); 
        }

        uint8_t status(void) const { 
            return nocan_ll_get_status(); 
        }

        bool receivePending(void) const {
            return (bool)nocan_ll_rx_pending();
        }
        bool transmitPending(void) const {
            return (bool)nocan_ll_tx_pending();
        }
        void led(bool on) {
            nocan_ll_set_led((int)on);
        }

    private:
        NocanNodeId _node_id;
};

extern NocanClass Nocan;

#endif
