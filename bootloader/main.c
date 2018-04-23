#include <samd21.h>
#include "system_samd21.h"
#include "serial.h"
#include "systick.h"
#include "spi.h"
#include "nocan_ll.h"

#define ASSERT_OR_FAIL(expression) do { if (!(expression)) goto general_failure; } while (0)
#ifndef NULL
    #define NULL ((void *)0)
#endif
#ifdef DEBUG
    #define dprintf(...) serial_printf(__VA_ARGS__)
    #define dsysmsg(n,x) nocan_ll_send_virtual_serial(n,x)
#else
    #define dprintf(...) 
    #define dsysmsg(n,x) 
#endif

/* A few lines of code were borrowed with love from Arduino Zero bootloader. */

extern uint32_t __sketch_vectors_ptr;

uint32_t PAGE_SIZE, PAGES, MAX_FLASH;

void try_start_app(void);
void write_page(uint32_t u_addr, uint8_t *u_data, uint32_t u_size);
void read_mem(uint8_t *u_dst, uint32_t u_src, uint8_t u_size);


int main(void)
{
    uint32_t pageSizes[] = { 8, 16, 32, 64, 128, 256, 512, 1024 };
    uint32_t tstamp; // systick_now_ms()
    uint32_t retry_delay;
    int led_on;
    int8_t nid;
    uint32_t eid;
    uint8_t eid_param;
    uint8_t packet_len;
    uint8_t packet_data[8];
    uint32_t addr, cur_addr, page_offset;
    uint8_t page[1024] __attribute__ ((aligned (4))); // we use the size of the biggest page.
    uint8_t udid[8];

    SystemInit();
    systick_init();

#ifdef DEBUG
    serial_open(115200); 
#endif

    dprintf("\r\n\r\n*** START ***\r\n\r\n");

    /* get mem+page size info 
     * These could be hardcoded, but this is more flexible.
     */
    PAGE_SIZE = pageSizes[NVMCTRL->PARAM.bit.PSZ];
    PAGES = NVMCTRL->PARAM.bit.NVMP;
    MAX_FLASH = PAGE_SIZE * PAGES;
    addr = 0x2000; // 8K
    page_offset = 0;

    for (;;)
    {
        dprintf("ll_init\r\n");
        ASSERT_OR_FAIL(nocan_ll_init(NOCAN_INIT_HARD_RESET)>=0);

        dprintf("ll_get_udid\r\n");
        ASSERT_OR_FAIL(nocan_ll_get_udid(udid)>=0);

        retry_delay = 125;
        for (;;)
        {
            dprintf("ll_request_node_id\r\n");
            nid = nocan_ll_request_node_id();

            if (nid>0) break; // though 0 is valid, it should not be assigned to a node.

            nocan_ll_set_led(1);
            systick_delay_ms(250);
            nocan_ll_set_led(0);
            systick_delay_ms(250);
            nocan_ll_set_led(1);
            systick_delay_ms(250);
            nocan_ll_set_led(0);

            systick_delay_ms(retry_delay);
            if (retry_delay<8000) retry_delay<<=1;
        }
        
        ASSERT_OR_FAIL(nocan_ll_set_node_id_filter(nid)>=0);

        if (nocan_ll_send(EID_SYS(nid, LL_SYS_NODE_BOOT_ACK, 0), 8, udid)<0)
            dprintf("ll_send boot ack failed\r\n");
        else
            dprintf("ll_send book ack ok\r\n");

        led_on = 0;
        for (int delay = 0; delay<20; delay++)
        {
            led_on ^= 1;
            nocan_ll_set_led(led_on);

            tstamp = systick_now_ms();

            while (systick_now_ms()-tstamp<500)
            {
                /* NOTE: we will never recieve anything else than a system message because no subscription exists in the filters. */
                if (nocan_ll_recv(&eid, &packet_len, packet_data)==NOCAN_LL_OK)
                {
                    dprintf("ll_recv %x\r\n",EID_GET_SYS_FUNC(eid));
                    eid_param = EID_GET_SYS_PARAM(eid);

                    switch (EID_GET_SYS_FUNC(eid)) {

                        case LL_SYS_BOOTLOADER_GET_SIGNATURE:
                            ASSERT_OR_FAIL(packet_len==0);
                            packet_data[0] = DSU->DID.reg>>24;
                            packet_data[1] = DSU->DID.reg>>16;
                            packet_data[2] = DSU->DID.reg>>8;
                            packet_data[4] = DSU->DID.reg;
                            nocan_ll_send(EID_SYS(nid, LL_SYS_BOOTLOADER_GET_SIGNATURE_ACK, 0),4,packet_data);
                            break;

                        case LL_SYS_BOOTLOADER_SET_ADDRESS:
                            ASSERT_OR_FAIL(packet_len==4);
                            addr = ((uint32_t)packet_data[0]<<24) |
                                ((uint32_t)packet_data[1]<<16) | 
                                ((uint32_t)packet_data[2]<<8) |
                                ((uint32_t)packet_data[3]); 
                            page_offset = 0;
                            nocan_ll_send(EID_SYS(nid,LL_SYS_BOOTLOADER_SET_ADDRESS_ACK,0),0,NULL);
                            dprintf("Set address to %x (page size=%x)\r\n",addr,PAGE_SIZE);
                            break;

                        case LL_SYS_BOOTLOADER_ERASE:
                            cur_addr = addr;
                            while (cur_addr < MAX_FLASH)
                            {
                                // dprintf("[Erase %x]", cur_addr);
                                NVMCTRL->ADDR.reg = cur_addr>>1;
                                NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMDEX_KEY | NVMCTRL_CTRLA_CMD_ER;
                                while (NVMCTRL->INTFLAG.bit.READY == 0);
                                cur_addr += PAGE_SIZE * 4; // Skip a ROW
                            }
                            nocan_ll_send(EID_SYS(nid,LL_SYS_BOOTLOADER_ERASE_ACK,0),0,NULL);
                            break;

                        case LL_SYS_BOOTLOADER_WRITE:
                            // is it the final packet in the page ?
                            if (eid_param==1) // final packet ?
                            {
                                // dprintf("[write page]\r\n");
                                // todo: check CRC32 
                                write_page(addr,page,page_offset);
                                nocan_ll_send(EID_SYS(nid,LL_SYS_BOOTLOADER_WRITE_ACK,1),0,NULL);
                            }
                            else
                            {
                                if (page_offset+packet_len>PAGE_SIZE)
                                {
                                    // fail
                                    nocan_ll_send(EID_SYS(nid,LL_SYS_BOOTLOADER_WRITE_ACK,-1),0,NULL);
                                }
                                else
                                {
                                    //dprintf("[D]");
                                    for (uint32_t i=0;i<packet_len;i++) page[page_offset++] = packet_data[i];
                                    nocan_ll_send(EID_SYS(nid,LL_SYS_BOOTLOADER_WRITE_ACK,0),0,NULL);
                                }
                            }
                            break;

                        case LL_SYS_BOOTLOADER_READ:
                            ASSERT_OR_FAIL(packet_len==0);
                            ASSERT_OR_FAIL(eid_param<=8);

                            read_mem(packet_data,addr+page_offset,eid_param);
                            page_offset+=eid_param;
                            nocan_ll_send(EID_SYS(nid,LL_SYS_BOOTLOADER_READ_ACK,0),eid_param,packet_data);
                            break;

                        case LL_SYS_BOOTLOADER_LEAVE:
                            nocan_ll_send(EID_SYS(nid,LL_SYS_BOOTLOADER_LEAVE_ACK,0),0,NULL);
                            try_start_app();
                            break;

                        case LL_SYS_NODE_PING:
                            nocan_ll_send(EID_SYS(nid, LL_SYS_NODE_PING_ACK, eid_param), packet_len, packet_data);
                            break;
                    } // switch

                    // reset 10 second timer.
                    delay = 0;

                } // if sys message
            } // while 500ms
        } // for (delay=0 ...) 

        try_start_app();
        systick_delay_ms(3000);
    }

general_failure:
    dprintf("\r\nGeneral failure. Reset in 60 seconds.\r\n");

    for (int delay=0; delay<20; delay++)
    {
        nocan_ll_set_led(1);
        systick_delay_ms(250);
        nocan_ll_set_led(0);
        systick_delay_ms(250);

        nocan_ll_set_led(1);
        systick_delay_ms(250);
        nocan_ll_set_led(0);
        systick_delay_ms(250);

        nocan_ll_set_led(1);
        systick_delay_ms(250);
        nocan_ll_set_led(0);
        systick_delay_ms(250);

        systick_delay_ms(1500);
    }
    NVIC_SystemReset();
}

void try_start_app(void)
{
    uint32_t* pulSketch_Start_Address;

    if (__sketch_vectors_ptr == 0xFFFFFFFF)
    {
        dprintf("Failed to launch app: no code.\r\n");
        /* Stay in bootloader */
        return;
    }

    /*
     * Load the sketch Reset Handler address
     * __sketch_vectors_ptr is exported from linker script and point on first 32b word of sketch vector table
     * First 32b word is sketch stack
     * Second 32b word is sketch entry point: Reset_Handler()
     */
    pulSketch_Start_Address = &__sketch_vectors_ptr ;
    pulSketch_Start_Address++ ;

    /*
     * Test vector table address of sketch @ &__sketch_vectors_ptr
     * Stay in bootloader if this function is not aligned enough, ie not valid
     */
    if ( ((uint32_t)(&__sketch_vectors_ptr) & ~SCB_VTOR_TBLOFF_Msk) != 0x00)
    {
        dprintf("Failed to launch app: bad vector table.\r\n");
        /* Stay in bootloader */
        return;
    }

    nocan_ll_set_led(1);

    __set_MSP( (uint32_t)(__sketch_vectors_ptr) );

    /* Rebase the vector table base address */
    SCB->VTOR = ((uint32_t)(&__sketch_vectors_ptr) & SCB_VTOR_TBLOFF_Msk);

    /* Jump to application Reset Handler in the application */
    asm("bx %0"::"r"(*pulSketch_Start_Address));
}

void write_page(uint32_t u_addr, uint8_t *u_data, uint32_t u_size)
{
    uint32_t word_size = u_size>>2;
    uint32_t *src_addr = (uint32_t *)u_data;
    uint32_t *dst_addr = (uint32_t *)u_addr;
    
    // Manual write (see 21.6.5.3 in SAMD21 datasheet):
    // 1) Clear page buffer, putting 0xFF all over page buffer.
    // 2) Write to the page buffer by addressing the NVM main address space directly, copying u_data to u_addr
    // 3) Commit changes made


    // 1) clear the page buffer
    NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMDEX_KEY | NVMCTRL_CTRLA_CMD_PBC;
    while (NVMCTRL->INTFLAG.bit.READY == 0);

    // 2) Fill page buffer
    // This does not actually write to dst_addr, but to the page buffer.
    // dst_addr must be addressed in 2 or 4 byte chunks, 1 byte access causes exception.
    for (uint32_t i=0;i<(PAGE_SIZE>>2) && i<word_size; i++)
        dst_addr[i] = src_addr[i];

    // 3) commit page buffer to memory.
    NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMDEX_KEY | NVMCTRL_CTRLA_CMD_WP;
    while (NVMCTRL->INTFLAG.bit.READY == 0);

}

void read_mem(uint8_t *u_dst, uint32_t u_addr, uint8_t u_len)
{
    uint8_t *u_src = (uint8_t *)u_addr;
    while (u_len--) *u_dst++ = *u_src++;
}

