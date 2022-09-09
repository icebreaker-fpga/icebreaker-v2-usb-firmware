/* 
 * The MIT License (MIT)
 *
 * Copyright (c) 2022 Greg Davill 
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * This file is part of the TinyUSB stack.
 */

#include "tusb_option.h"

#if CFG_TUD_ENABLED && (CFG_TUSB_MCU == OPT_MCU_CH32VF307)
#include "device/dcd.h"

#include "usb_ch32_usbhs_reg.h"
#include "core_riscv.h"

// Max number of bi-directional endpoints including EP0
#define EP_MAX 16

typedef struct {
    uint8_t *buffer;
    // tu_fifo_t * ff; // TODO support dcd_edpt_xfer_fifo API
    uint16_t total_len;
    uint16_t queued_len;
    uint16_t max_size;
    bool short_packet;
} xfer_ctl_t;

#define XFER_CTL_BASE(_ep, _dir) &xfer_status[_ep][_dir]
static xfer_ctl_t xfer_status[EP_MAX][2];

#define EP_TX_LEN(ep) *(volatile uint16_t *)((volatile uint16_t *)&(USBHSD->UEP0_TX_LEN) + (ep)*2)
#define EP_TX_CTRL(ep) *(volatile uint8_t *)((volatile uint8_t *)&(USBHSD->UEP0_TX_CTRL) + (ep)*4)
#define EP_RX_CTRL(ep) *(volatile uint8_t *)((volatile uint8_t *)&(USBHSD->UEP0_RX_CTRL) + (ep)*4)
#define EP_RX_MAX_LEN(ep) *(volatile uint16_t *)((volatile uint16_t *)&(USBHSD->UEP0_MAX_LEN) + (ep)*2)

#define EP_TX_DMA_ADDR(ep) *(volatile uint32_t *)((volatile uint32_t *)&(USBHSD->UEP1_TX_DMA) + (ep - 1))
#define EP_RX_DMA_ADDR(ep) *(volatile uint32_t *)((volatile uint32_t *)&(USBHSD->UEP1_RX_DMA) + (ep - 1))

/* Endpoint Buffer */
__attribute__((aligned(4))) uint8_t ep0_databuf[64];  // ep0(64)

volatile uint8_t USBHS_Dev_Endp0_Tog = 0x01;

void dcd_init(uint8_t rhport) {
    (void)rhport;

    memset(&xfer_status, 0, sizeof(xfer_status));

    // usb_dc_low_level_init();

    USBHSD->HOST_CTRL = 0x00;
    USBHSD->HOST_CTRL = USBHS_PHY_SUSPENDM;

    USBHSD->CONTROL = 0;
    USBHSD->CONTROL = USBHS_DMA_EN | USBHS_INT_BUSY_EN | USBHS_HIGH_SPEED;

    USBHSD->INT_EN = 0;
    USBHSD->INT_EN = USBHS_SETUP_ACT_EN | USBHS_TRANSFER_EN | USBHS_DETECT_EN | USBHS_SUSPEND_EN;

    /* EP0 Enable */
    USBHSD->ENDP_CONFIG = USBHS_EP0_T_EN | USBHS_EP0_R_EN;
    USBHSD->ENDP_TYPE = 0x00;
    USBHSD->BUF_MODE = 0x00;

    USBHSD->UEP0_MAX_LEN = 64;

    USBHSD->UEP0_DMA = (uint32_t)ep0_databuf;

    USBHSD->UEP0_TX_LEN = 0;
    USBHSD->UEP0_TX_CTRL = USBHS_EP_T_RES_NAK;
    USBHSD->UEP0_RX_CTRL = USBHS_EP_R_RES_NAK;

    for (int ep = 1; ep < EP_MAX; ep++) {
        EP_RX_MAX_LEN(ep) = 512;

        EP_TX_LEN(ep) = 0;
        EP_TX_CTRL(ep) = USBHS_EP_T_AUTOTOG | USBHS_EP_T_RES_NAK;
        EP_RX_CTRL(ep) = USBHS_EP_R_AUTOTOG | USBHS_EP_R_RES_NAK;
    }

    USBHSD->DEV_AD = 0;
    USBHSD->CONTROL |= USBHS_DEV_PU_EN;
}

void dcd_int_enable(uint8_t rhport) {
    (void)rhport;

    NVIC_EnableIRQ(USBHS_IRQn);
}

void dcd_int_disable(uint8_t rhport) {
    (void)rhport;

    NVIC_DisableIRQ(USBHS_IRQn);
}

void dcd_edpt_close_all(uint8_t rhport) {
    (void)rhport;
}

void dcd_set_address(uint8_t rhport, uint8_t dev_addr) {
    (void)dev_addr;

    // Response with zlp status
    dcd_edpt_xfer(rhport, 0x80, NULL, 0);
}

void dcd_edpt0_status_complete(uint8_t rhport, tusb_control_request_t const *request) {
    (void)rhport;

    if (request->bmRequestType_bit.recipient == TUSB_REQ_RCPT_DEVICE &&
        request->bmRequestType_bit.type == TUSB_REQ_TYPE_STANDARD &&
        request->bRequest == TUSB_REQ_SET_ADDRESS) {
        USBHSD->DEV_AD = (uint8_t)request->wValue;
    }

    EP_TX_CTRL(0) = USBHS_EP_T_RES_NAK;
    EP_RX_CTRL(0) = USBHS_EP_R_RES_NAK;
}

bool dcd_edpt_open(uint8_t rhport, tusb_desc_endpoint_t const *desc_edpt) {
    (void)rhport;

    uint8_t const epnum = tu_edpt_number(desc_edpt->bEndpointAddress);
    uint8_t const dir = tu_edpt_dir(desc_edpt->bEndpointAddress);

    TU_ASSERT(epnum < EP_MAX);

    xfer_ctl_t *xfer = XFER_CTL_BASE(epnum, dir);
    xfer->max_size = tu_edpt_packet_size(desc_edpt);
    xfer->total_len = 0;
    xfer->queued_len = 0;
    xfer->buffer = 0;
    xfer->short_packet = false;

    if (epnum != 0) {
        if (tu_edpt_dir(desc_edpt->bEndpointAddress) == TUSB_DIR_OUT) {
            USBHSD->ENDP_CONFIG |= (USBHS_EP0_R_EN << epnum);
            EP_RX_CTRL(epnum) = USBHS_EP_R_AUTOTOG | USBHS_EP_R_RES_NAK;
        } else {
            EP_TX_LEN(epnum) = 0;
            EP_TX_CTRL(epnum) = USBHS_EP_T_AUTOTOG | USBHS_EP_T_RES_NAK | USBHS_EP_T_TOG_0;
        }
    }

    return true;
}

int usbd_ep_close(const uint8_t ep) {
    (void)ep;

    return 0;
}
void dcd_edpt_stall(uint8_t rhport, uint8_t ep_addr) {
    (void)rhport;

    uint8_t const epnum = tu_edpt_number(ep_addr);
    uint8_t const dir = tu_edpt_dir(ep_addr);

    if (epnum == 0) {
        if (dir == TUSB_DIR_OUT) {
            USBHSD->UEP0_RX_CTRL = USBHS_EP_R_RES_STALL;
        } else {
            USBHSD->UEP0_TX_LEN = 0;
            USBHSD->UEP0_TX_CTRL = USBHS_EP_T_RES_STALL;
        }
    } else {
        if (dir == TUSB_DIR_OUT) {
            EP_RX_CTRL(epnum) = (EP_RX_CTRL(epnum) & ~USBHS_EP_R_RES_MASK) | USBHS_EP_R_RES_STALL;

        } else {
            EP_TX_CTRL(epnum) = (EP_TX_CTRL(epnum) & ~USBHS_EP_T_RES_MASK) | USBHS_EP_T_RES_STALL;
        }
    }
}

void dcd_edpt_clear_stall(uint8_t rhport, uint8_t ep_addr) {
    (void)rhport;

    uint8_t const epnum = tu_edpt_number(ep_addr);
    uint8_t const dir = tu_edpt_dir(ep_addr);

    if (epnum == 0) {
        if (dir == TUSB_DIR_OUT) {
            USBHSD->UEP0_RX_CTRL = USBHS_EP_R_RES_ACK;
        } else {
        }
    } else {
        if (dir == TUSB_DIR_OUT) {
            EP_RX_CTRL(epnum) = (EP_RX_CTRL(epnum) & ~(USBHS_EP_R_RES_MASK | USBHS_EP_T_TOG_MASK)) | USBHS_EP_T_RES_ACK;

        } else {
            EP_TX_CTRL(epnum) = (EP_TX_CTRL(epnum) & ~(USBHS_EP_T_RES_MASK | USBHS_EP_T_TOG_MASK)) | USBHS_EP_T_RES_NAK;
        }
    }
}

bool dcd_edpt_xfer(uint8_t rhport, uint8_t ep_addr, uint8_t *buffer, uint16_t total_bytes) {
    (void)rhport;
    uint8_t const epnum = tu_edpt_number(ep_addr);
    uint8_t const dir = tu_edpt_dir(ep_addr);
    
    __disable_irq();

    xfer_ctl_t *xfer = XFER_CTL_BASE(epnum, dir);
    xfer->buffer = buffer;
    // xfer->ff           = NULL; // TODO support dcd_edpt_xfer_fifo API
    xfer->total_len = total_bytes;
    xfer->queued_len = 0;
    xfer->short_packet = false;

    // uint16_t num_packets = (total_bytes / xfer->max_size);
    uint16_t short_packet_size = total_bytes % (xfer->max_size + 1);

    // Zero-size packet is special case.
    if (short_packet_size == 0 || (total_bytes == 0)) {
        xfer->short_packet = true;
    }

    if (dir == TUSB_DIR_IN) {
        if (!total_bytes) {
            //xfer->short_packet = true;
            if (epnum == 0) {
                USBHSD->UEP0_TX_LEN = 0;
                USBHSD->UEP0_TX_CTRL = USBHS_EP_T_RES_ACK | (USBHS_Dev_Endp0_Tog ? USBHS_EP_T_TOG_1 : USBHS_EP_T_TOG_0);
                USBHS_Dev_Endp0_Tog ^= 1;
            } else {
                EP_TX_LEN(epnum) = 0;
                EP_TX_CTRL(epnum) = (EP_TX_CTRL(epnum) & ~(USBHS_EP_T_RES_MASK)) | USBHS_EP_T_RES_ACK;
            }
        } else {
            xfer->queued_len += short_packet_size;

            if (epnum == 0) {
                memcpy(ep0_databuf, buffer, short_packet_size);

                USBHSD->UEP0_TX_LEN = short_packet_size;
                USBHSD->UEP0_TX_CTRL = USBHS_EP_T_RES_ACK | (USBHS_Dev_Endp0_Tog ? USBHS_EP_T_TOG_1 : USBHS_EP_T_TOG_0);
                USBHS_Dev_Endp0_Tog ^= 1;
            } else {

                EP_TX_DMA_ADDR(epnum) = (uint32_t)buffer;
                USBHSD->ENDP_CONFIG |= (USBHS_EP0_T_EN << epnum);
                EP_TX_LEN(epnum) = short_packet_size;
                EP_TX_CTRL(epnum) = (EP_TX_CTRL(epnum) & ~(USBHS_EP_T_RES_MASK)) | USBHS_EP_T_RES_ACK;
            }
        }
    } else { /* TUSB_DIR_OUT */
        if (epnum == 0) {
            USBHSD->UEP0_RX_CTRL = (USBHSD->UEP0_RX_CTRL & ~(USBHS_EP_R_RES_MASK)) | USBHS_EP_R_RES_ACK;
        } else {
            EP_RX_DMA_ADDR(epnum) = (uint32_t)xfer->buffer;
            USBHSD->ENDP_CONFIG |= (USBHS_EP0_R_EN << epnum);
            EP_RX_CTRL(epnum) = (EP_RX_CTRL(epnum) & ~(USBHS_EP_R_RES_MASK)) | USBHS_EP_R_RES_ACK;
        }
    }

    __enable_irq();
    return true;
}


static void receive_packet(xfer_ctl_t *xfer, uint16_t xfer_size) {
    // xfer->queued_len = xfer->total_len - remaining;

    uint16_t remaining = xfer->total_len - xfer->queued_len;
    uint16_t to_recv_size;

    if (remaining <= xfer->max_size) {
        // Avoid buffer overflow.
        to_recv_size = (xfer_size > remaining) ? remaining : xfer_size;
    } else {
        // Room for full packet, choose recv_size based on what the microcontroller
        // claims.
        to_recv_size = (xfer_size > xfer->max_size) ? xfer->max_size : xfer_size;
    }

    if (to_recv_size) {
    }

    xfer->queued_len += xfer_size;

    // Per USB spec, a short OUT packet (including length 0) is always
    // indicative of the end of a transfer (at least for ctl, bulk, int).
    xfer->short_packet = (xfer_size < xfer->max_size);
}

void dcd_int_handler(uint8_t rhport) {
    (void)rhport;

    uint32_t end_num, rx_token;
    uint8_t intflag = 0;

    intflag = USBHSD->INT_FG;

    if (intflag & USBHS_TRANSFER_FLAG) {
        USBHSD->INT_FG = USBHS_TRANSFER_FLAG; /* Clear flag */

        end_num = (USBHSD->INT_ST) & MASK_UIS_ENDP;
        rx_token = (((USBHSD->INT_ST) & MASK_UIS_TOKEN) >> 4) & 0x03;

        uint8_t endp = end_num | (rx_token == PID_IN ? TUSB_DIR_IN_MASK : 0);

        xfer_ctl_t *xfer = XFER_CTL_BASE(end_num, tu_edpt_dir(endp));

        if (rx_token == PID_OUT) {
            uint16_t rx_len = USBHSD->RX_LEN;

            receive_packet(xfer, rx_len);

            if (end_num == 0) {
                USBHSD->UEP0_RX_CTRL = (USBHSD->UEP0_RX_CTRL & ~(USBHS_EP_R_RES_MASK)) | USBHS_EP_R_RES_NAK;
                memcpy(xfer->buffer, ep0_databuf, rx_len);
            }

            if (xfer->short_packet || (xfer->queued_len == xfer->total_len)) {
                xfer->short_packet = false;
                dcd_event_xfer_complete(0, endp, xfer->queued_len, XFER_RESULT_SUCCESS, true);
            }

        } else if (rx_token == PID_IN) {
            if (xfer->short_packet || (xfer->queued_len == xfer->total_len)) {
                xfer->short_packet = false;
                xfer->total_len = 0;
                dcd_event_xfer_complete(0, endp, xfer->queued_len, XFER_RESULT_SUCCESS, true);

                EP_TX_CTRL(end_num) = (EP_TX_CTRL(end_num) & ~(USBHS_EP_T_RES_MASK)) | USBHS_EP_T_RES_NAK;

                if (end_num == 0) {
                }
            } else {
                dcd_edpt_xfer(0, endp, xfer->buffer + xfer->queued_len, xfer->total_len - xfer->queued_len);
            }
        }

    } else if (intflag & USBHS_SETUP_FLAG) {
        USBHS_Dev_Endp0_Tog = 1;
        dcd_event_setup_received(0, ep0_databuf, true);
        
        USBHSD->INT_FG = USBHS_SETUP_FLAG; /* Clear flag */
    } else if (intflag & USBHS_DETECT_FLAG) {
        USBHS_Dev_Endp0_Tog = 1;

        xfer_status[0][TUSB_DIR_OUT].max_size = 64;
        xfer_status[0][TUSB_DIR_IN].max_size = 64;

        dcd_event_bus_reset(0, TUSB_SPEED_HIGH, true);

        USBHSD->DEV_AD = 0;
        USBHSD->UEP0_RX_CTRL = USBHS_EP_R_RES_ACK | USBHS_EP_R_TOG_0;

        USBHSD->INT_FG = USBHS_DETECT_FLAG; /* Clear flag */
    } else if (intflag & USBHS_SUSPEND_FLAG) {
        dcd_event_t event = { .rhport = rhport, .event_id = DCD_EVENT_SUSPEND };
        dcd_event_handler(&event, true);

        USBHSD->INT_FG = USBHS_SUSPEND_FLAG; /* Clear flag */
    }

}

#endif