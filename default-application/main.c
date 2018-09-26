#include <samd21.h>
#include "system_samd21.h"
#include "serial.h"
#include "systick.h"
#include <nocan_ll.h>

#define LED_PIN 8

int main(void)
{
    uint8_t udid[8];

    SystemInit();
    systick_init();

    serial_open(115200);

    REG_PORT_DIR1 |= (1<<LED_PIN);

    serial_printf("Started\r\n");

    for (;;)
    {
        if (nocan_ll_init(NOCAN_INIT_SOFT_RESET)<0)
        {
            serial_printf("nocan_init failed.\r\n");
            goto fail;
        }

        if (nocan_ll_get_udid(udid)<0)
        {
            serial_printf("nocan_get_udid failed.\r\n");
            goto fail;
        }

        serial_printf("UDID:");
        for (uint8_t k=0;k<8;k++)
            serial_printf(" %x",udid[k]);
        serial_printf("\r\n");


        int8_t node_id = nocan_ll_request_node_id();

        if (node_id<=0)
        {
            serial_printf("request node id failed (%x)\r\n", node_id);
            goto fail;
        }
        //else
        //{
        //   serial_printf("Got node id %x\r\n",node_id);
        //}

        if (nocan_ll_set_node_id_filter(node_id)<0)
        {
            serial_printf("set sys filter failed\r\n");
            goto fail;
        }
       
        nocan_ll_set_led(1);

        int8_t status;
        
        status = nocan_ll_send(EID_SYS(node_id,LL_SYS_CHANNEL_REGISTER,0), 8, (uint8_t *)"led$(ID)");

        if (status<0)
        {
            serial_printf("LL_SYS_CHANNEL_REGISTER failed\r\n");
            goto fail;
        }
        //else
        //{
        //    serial_printf("channel register succeeded\r\n");
        //}

        uint8_t c[2];
        int8_t register_success;
        uint8_t rlen;
        uint16_t channel_id;
        uint32_t eid;

        status = nocan_ll_scan(EID_SYS_MASK, EID_SYS_FUNC(LL_SYS_CHANNEL_REGISTER_ACK),&eid,&rlen,c);

        register_success = EID_GET_SYS_PARAM(eid);

        if (status<0 || register_success<0 || rlen!=2)
        {
            serial_printf("LL_SYS_CHANNEL_REGISTER_ACK failed, status=%x, param=%x, rlen=%x\r\n", status, register_success, rlen);
            goto fail;
        }
        //else
        //{
        //    serial_printf("Channel register ACK succeeded\r\n");
       // }

        channel_id = ((uint16_t)c[0])<<8 | c[1];

        status = nocan_ll_send(EID_SYS(node_id,LL_SYS_CHANNEL_SUBSCRIBE,0),2,c);

        if (status<0)
        {
            serial_printf("LL_SYS_CHANNEL_SUBSCRIBE failed for channel %x\r\n",channel_id);
            goto fail;
        }
        //else
        // {
        //    serial_printf("channel subscribe succeeded\r\n");
       // }

        status = nocan_ll_add_channel_filter(channel_id);

        if (status<0)
        {
            serial_printf("add channel filter failed\r\n");
            goto fail;
        }

        serial_printf("Nocan initialization is successful.\r\n");

        nocan_ll_set_led(1);

        for (;;)
        {
            if (nocan_ll_rx_pending())
            {
                uint8_t src_data[8];
                uint8_t src_data_len;
                int8_t src_node_id;
                uint16_t src_channel_id;

                status = nocan_ll_recv(&eid, &src_data_len, src_data);

                if ((eid&EID_SYS_BIT)==0)
                {
                    src_node_id = EID_GET_NODE_ID(eid);
                    src_channel_id = EID_GET_CHANNEL_ID(eid);

                    if (status<0) 
                    {
                        serial_printf("recv failed\r\n");
                        goto fail;
                    }

                    serial_printf("Got message from node %x on channel %x\r\n", src_node_id, src_channel_id);

                    if (src_data_len==1) 
                    {
                        if (src_data[0]=='1')
                        {
                            REG_PORT_OUT1 |= (1<<LED_PIN);
                            nocan_ll_send_virtual_serial(node_id,"ON");
                        }
                        if (src_data[0]=='0')
                        {
                            REG_PORT_OUT1 &= ~(1<<LED_PIN);
                            nocan_ll_send_virtual_serial(node_id,"OFF");
                        }
                    }
                } 
                else 
                {
                    serial_printf("Received unexpected system message from node 0x%x, with fn=0x%x, param=0x%x\r\n", 
                            EID_GET_NODE_ID(eid), EID_GET_SYS_FUNC(eid), EID_GET_SYS_PARAM(eid));
                }
            }
        }

fail:
        serial_printf("Failure\r\n");
        serial_printf("Pausing 60 seconds...");
        systick_delay_ms(60000);
    }
}

