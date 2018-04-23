#include "nocan.h"

NocanClass Nocan;

NocanNodeId NocanClass::open(void)
{
    if (nocan_ll_init(NOCAN_INIT_SOFT_RESET)<0) return NocanClass::ERROR;

    _node_id = nocan_ll_request_node_id();

    if (_node_id<=0)
        return 0;

    if (nocan_ll_set_node_id_filter(_node_id)<0)
    {
        return NocanClass::ERROR;
    }

    led(true);
    return _node_id;
}

int8_t NocanClass::lookupChannel(const char *channel, NocanChannelId *channel_id) const
{
    int8_t status;
    uint32_t eid;
    uint8_t channel_len = 0;
    uint8_t rlen;
    uint8_t c[8];

    while (channel_len<64 && channel[channel_len]!=0) channel_len++;
	
    status = nocan_ll_send(EID_SYS(_node_id,LL_SYS_CHANNEL_LOOKUP,0),channel_len,(const uint8_t *)channel);
    if (status<0)      
        return status;

    status = nocan_ll_scan_sys(LL_SYS_CHANNEL_LOOKUP_ACK,&eid,&rlen,c);
    if (status<0)
        return status;

    if (EID_GET_SYS_PARAM(eid)<0 || rlen!=2)
        return NocanClass::ERROR;

    *channel_id = ((uint16_t)c[0])<<8 | c[1];

    return NocanClass::OK;

}

int8_t NocanClass::registerChannel(const char *channel, NocanChannelId *channel_id) const
{
    int8_t status;
    uint8_t channel_len = 0;
    uint32_t eid;
    uint8_t rlen;
    uint8_t c[8];

    while (channel_len<64 && channel[channel_len]!=0) channel_len++;
	
    status = nocan_ll_send(EID_SYS(_node_id,LL_SYS_CHANNEL_REGISTER,0),channel_len,(const uint8_t *)channel);
    
    if (status<0)      
        return status;

    status = nocan_ll_scan_sys(LL_SYS_CHANNEL_REGISTER_ACK,&eid,&rlen,c);

    if (status<0)
        return status;

    if (EID_GET_SYS_PARAM(eid)<0 || rlen!=2)
        return NocanClass::ERROR;

    *channel_id = ((uint16_t)c[0])<<8 | c[1];

    return NocanClass::OK;
}

int8_t NocanClass::unregisterChannel(NocanChannelId channel_id) const
{
    int8_t status;
    uint8_t c[2];

    c[0] = channel_id >> 8;
    c[1] = channel_id & 0xFF;

    status = nocan_ll_send(EID_SYS(_node_id,LL_SYS_CHANNEL_UNREGISTER,0),2,c);
    if (status<0)
        return status;

    status = nocan_ll_scan_sys(LL_SYS_CHANNEL_UNREGISTER_ACK,0,0,0);
    if (status<0)
        return status;

    return NocanClass::OK;
}

int8_t NocanClass::subscribeChannel(NocanChannelId channel_id) const
{
    int8_t status;
    uint8_t c[2];

    status = nocan_ll_add_channel_filter(channel_id);

    if (status<0)
        return status;

    c[0] = channel_id >> 8;
    c[1] = channel_id & 0xFF;

    return nocan_ll_send(EID_SYS(_node_id,LL_SYS_CHANNEL_SUBSCRIBE,0),2,c);
}

int8_t NocanClass::unsubscribeChannel(NocanChannelId channel_id) const
{
    int8_t status;
    uint8_t c[2];

    status = nocan_ll_remove_channel_filter(channel_id);

    if (status<0)
        return status;

    c[0] = channel_id >> 8;
    c[1] = channel_id & 0xFF;

    return nocan_ll_send(EID_SYS(_node_id,LL_SYS_CHANNEL_UNSUBSCRIBE,0),2,c);
}

int8_t NocanClass::publishMessage(const NocanMessage &msg) const
{
    return nocan_ll_send(EID_PUB(_node_id, msg.channel_id), msg.data_len, msg.data);
}

int8_t NocanClass::publishMessage(NocanChannelId cid, const char *str) const
{
    NocanMessage m;
    m.channel_id = cid;
    m.data_len = 0;
    while (m.data_len<64 && *str) m.data[m.data_len++] = *str++;
    return publishMessage(m);
}

int8_t NocanClass::receiveMessage(NocanMessage *msg) const
{
    uint32_t eid;
    int8_t status; 
        
    for (;;) {
        status = nocan_ll_recv(&eid, &msg->data_len, msg->data);

        if (status<0)
            return status;

        if ((eid&EID_SYS_BIT)!=0)
        {
            if (EID_GET_SYS_FUNC(eid)==LL_SYS_NODE_PING)
            {
                nocan_ll_send(EID_SYS(EID_GET_NODE_ID(eid),LL_SYS_NODE_PING_ACK,0),0,0);
            }
            // else ignore the message
        }
        else 
        {
            msg->node_id = EID_GET_NODE_ID(eid);
            msg->channel_id = EID_GET_CHANNEL_ID(eid);
            break;
        }
    }
    return status;
}

