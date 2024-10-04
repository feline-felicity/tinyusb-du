/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2024
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

/*
 * DCD for AVR DU family with one FS Device peripheral (USB0).
 * The controller is DMA capable and does not require allocation of dedicated
 * packet memory. Still, endpoint description table and transaction complete
 * FIFO have to be placed in SRAM, which is done in this file according to
 * `CFG_TUD_ENDPPOINT_MAX`.
 */

#include "tusb_option.h"

#if CFG_TUD_ENABLED && (CFG_TUSB_MCU == OPT_MCU_AVRDU)

#include <avr/io.h>

#include "device/dcd.h"

#define TU_AVRDU_EP0SIZECONST(_size) ((_size)>=64 ? USB_BUFSIZE_DEFAULT_BUF64_gc : ((_size)>=32 ? USB_BUFSIZE_DEFAULT_BUF32_gc : ((_size)>=16 ? USB_BUFSIZE_DEFAULT_BUF16_gc : USB_BUFSIZE_DEFAULT_BUF8_gc)))
TU_VERIFY_STATIC(CFG_TUD_ENDPOINT0_SIZE == 64 || CFG_TUD_ENDPOINT0_SIZE == 32 || CFG_TUD_ENDPOINT0_SIZE == 16 || CFG_TUD_ENDPOINT0_SIZE == 8);

// Transaction-Complete FIFO and EP Descriptor Table
typedef struct TU_ATTR_PACKED  {
    register8_t fifo[2 * CFG_TUD_ENDPPOINT_MAX];
    USB_EP_PAIR_t eps[CFG_TUD_ENDPPOINT_MAX];
} avrdu_desctable_t;

static avrdu_desctable_t TU_ATTR_ALIGNED(2) fifo_desc;
static uint8_t TU_ATTR_ALIGNED(2) setup_buffer[8]; // Buffer for endpoint 0 setup
static bool sof_int_enabled = false;

static void disable_all_eps(void) {
    for (uint8_t i = 0; i < CFG_TUD_ENDPPOINT_MAX; i++) {
        fifo_desc.eps[i].IN.CTRL = USB_TYPE_DISABLE_gc;
        fifo_desc.eps[i].OUT.CTRL = USB_TYPE_DISABLE_gc;
    }
}

static void configure_ep0_for_setup(void) {
    // `OUT` buffer is used for SETUP stage (8 B);
    // `IN` buffer is used for DATA(IN/OUT) stage.
    fifo_desc.eps[0].IN.STATUS = USB_BUSNAK_bm;
    fifo_desc.eps[0].IN.CTRL = USB_TYPE_CONTROL_gc | USB_MULTIPKT_bm | TU_AVRDU_EP0SIZECONST(CFG_TUD_ENDPOINT0_SIZE);
    fifo_desc.eps[0].OUT.DATAPTR = (uint16_t)setup_buffer;
    fifo_desc.eps[0].OUT.STATUS = USB_BUSNAK_bm;
    fifo_desc.eps[0].OUT.CTRL = USB_TYPE_CONTROL_gc | USB_MULTIPKT_bm | TU_AVRDU_EP0SIZECONST(CFG_TUD_ENDPOINT0_SIZE);
}

static inline void TU_ATTR_ALWAYS_INLINE poll_for_rmw(void) {
    // Needed as per datasheet before using USB0.STATUS RMW regs.
    // In most cases, previous RMW ops should have been completed,
    // so it is unlikely that this results in actual delay.
    // Even if it does, the delay should be minimal...
    while(USB0.INTFLAGSB & USB_RMWBUSY_bm)
        ;
}

/*------------------------------------------------------------------*/
/* Device API
 *------------------------------------------------------------------*/
void dcd_init(uint8_t rhport) {
    // Assumptions:
    // - Clocks are configured (CLK_PER>=12 MHz)
    // - Vusb is provided (external 3.3 V available or SYSCFG.USBVREG==1)
    USB0.ADDR = 0;
    USB0.FIFORP = 0; // This resets both pointers
    USB0.EPPTR = (uint16_t)fifo_desc.eps;
    // Clear all bus and EP interrupt flags
    USB0.INTFLAGSA = USB_SOF_bm | USB_SUSPEND_bm | USB_RESUME_bm | USB_RESET_bm | USB_STALLED_bm | USB_UNF_bm | USB_OVF_bm;
    USB0.INTFLAGSB = USB_TRNCOMPL_bm | USB_GNDONE_bm | USB_SETUP_bm;
    disable_all_eps();
    configure_ep0_for_setup();
    USB0.CTRLB = 0 /* | USB_GNAUTO_bm */; // Is it helpful to NAK all EPs on SETUP?
    USB0.CTRLA = USB_ENABLE_bm | USB_FIFOEN_bm | ((CFG_TUD_ENDPPOINT_MAX - 1) & 0x0F);

    dcd_connect(rhport);
}

void dcd_int_enable(uint8_t rhport) {
    // AVR doesn't have per-peripheral interrupt mask
    USB0.INTCTRLA |= USB_SUSPEND_bm | USB_RESUME_bm | USB_RESET_bm | (sof_int_enabled ? USB_SOF_bm : 0) ;
    USB0.INTCTRLB |= USB_TRNCOMPL_bm | USB_SETUP_bm;
    (void) rhport;
}

void dcd_int_disable(uint8_t rhport) {
    USB0.INTCTRLA = 0;
    USB0.INTCTRLB = 0;
    (void) rhport;
}

void dcd_set_address(uint8_t rhport, uint8_t dev_addr) {
    (void) dev_addr;
    /* Response with status first before changing device address */
    dcd_edpt_xfer(rhport, tu_edpt_addr(0, TUSB_DIR_IN), NULL, 0);
    /* Nothing to do here. Actually updated after status stage */
}

void dcd_remote_wakeup(uint8_t rhport) {
    (void) rhport;
    // Do nothing if bus is not suspended for 5 ms as per datasheet
    if (!(USB0.BUSSTATE & USB_WTRSM_bm))
        return;
    USB0.CTRLB |= USB_URESUME_bm;
}

void dcd_connect(uint8_t rhport) {
    (void)rhport;
    USB0.CTRLB |= USB_ATTACH_bm;
}

void dcd_disconnect(uint8_t rhport) {
    (void)rhport;
    USB0.CTRLB &= ~USB_ATTACH_bm;
}

void dcd_sof_enable(uint8_t rhport, bool en) {
    (void)rhport;
    sof_int_enabled = en;
    if(en) {
        USB0.CTRLA |= USB_SOF_bm;
    } else {
        USB0.CTRLA &= ~USB_SOF_bm;
    }
}

//--------------------------------------------------------------------+
// Endpoint API
//--------------------------------------------------------------------+
bool dcd_edpt_open(uint8_t rhport, tusb_desc_endpoint_t const *ep_desc) {
    (void) rhport;
    uint8_t const ep_addr = ep_desc->bEndpointAddress;
    uint8_t const ep_num = tu_edpt_number(ep_addr);
    tusb_dir_t const dir = tu_edpt_dir(ep_addr);
    const uint16_t packet_size = tu_edpt_packet_size(ep_desc);
    uint8_t xfertype = ep_desc->bmAttributes.xfer;

    // this API is not used for establishing control transfers
    uint8_t ctrl = (xfertype == TUSB_XFER_ISOCHRONOUS ? USB_TYPE_ISO_gc : USB_TYPE_BULKINT_gc) | USB_MULTIPKT_bm;
    if(xfertype == TUSB_XFER_ISOCHRONOUS) {
        if(packet_size == 1023) ctrl |= USB_BUFSIZE_ISO_BUF1023_gc;
        else if(packet_size == 512) ctrl |= USB_BUFSIZE_ISO_BUF512_gc;
        else if(packet_size == 256) ctrl |= USB_BUFSIZE_ISO_BUF256_gc;
        else if(packet_size == 128) ctrl |= USB_BUFSIZE_ISO_BUF128_gc;
        else if(packet_size == 64) ctrl |= USB_BUFSIZE_ISO_BUF64_gc;
        else if(packet_size == 32) ctrl |= USB_BUFSIZE_ISO_BUF32_gc;
        else if(packet_size == 16) ctrl |= USB_BUFSIZE_ISO_BUF16_gc;
        else if(packet_size == 8) ctrl |= USB_BUFSIZE_ISO_BUF8_gc;
        else TU_ASSERT(false);
    } else {
        if(packet_size == 64) ctrl |= USB_BUFSIZE_DEFAULT_BUF64_gc;
        else if(packet_size == 32) ctrl |= USB_BUFSIZE_DEFAULT_BUF32_gc;
        else if(packet_size == 16) ctrl |= USB_BUFSIZE_DEFAULT_BUF16_gc;
        else if(packet_size == 8) ctrl |= USB_BUFSIZE_DEFAULT_BUF8_gc;
        else TU_ASSERT(false);
    }

    if(dir == TUSB_DIR_OUT) {
        fifo_desc.eps[ep_num].OUT.STATUS = USB_BUSNAK_bm;
        fifo_desc.eps[ep_num].OUT.CTRL = ctrl;
    } else {
        fifo_desc.eps[ep_num].IN.STATUS = USB_BUSNAK_bm;
        fifo_desc.eps[ep_num].IN.CTRL = ctrl;
    }
    return true;
}

void dcd_edpt_close_all(uint8_t rhport) {
    (void) rhport;
    disable_all_eps();
    configure_ep0_for_setup();
}

void dcd_edpt_close(uint8_t rhport, uint8_t ep_addr) {
    (void) rhport;
    (void) ep_addr;
    uint8_t const ep_num = tu_edpt_number(ep_addr);
    tusb_dir_t const dir = tu_edpt_dir(ep_addr);
    if(dir == TUSB_DIR_OUT) {
        fifo_desc.eps[ep_num].OUT.CTRL = USB_TYPE_DISABLE_gc;
    } else {
        fifo_desc.eps[ep_num].IN.CTRL = USB_TYPE_DISABLE_gc;
    }
}

bool dcd_edpt_xfer(uint8_t rhport, uint8_t ep_addr, uint8_t *buffer, uint16_t total_bytes) {
    (void) rhport;
    uint8_t const ep_num = tu_edpt_number(ep_addr);
    tusb_dir_t const dir = tu_edpt_dir(ep_addr);

    if(dir != TUSB_DIR_OUT || ep_num == 0) {
        // IN transaction or control transfer data stage in either direction
        fifo_desc.eps[ep_num].IN.DATAPTR = (uint16_t)buffer;
    } else {
        // OUT transaction
        fifo_desc.eps[ep_num].OUT.DATAPTR = (uint16_t)buffer;
    }

    if(dir != TUSB_DIR_OUT) {
        // IN transaction
        fifo_desc.eps[ep_num].IN.CNT = total_bytes;
        fifo_desc.eps[ep_num].IN.MCNT = 0;
        poll_for_rmw();
        USB0.STATUS[ep_num].INCLR = USB_CRC_bm | USB_UNFOVF_bm | USB_TRNCOMPL_bm | USB_EPSETUP_bm | USB_STALLED_bm | USB_BUSNAK_bm;
    } else {
        // OUT transaction
        fifo_desc.eps[ep_num].OUT.MCNT = total_bytes;
        fifo_desc.eps[ep_num].OUT.CNT = 0;
        poll_for_rmw();
        USB0.STATUS[ep_num].OUTCLR = USB_CRC_bm | USB_UNFOVF_bm | USB_TRNCOMPL_bm | USB_EPSETUP_bm | USB_STALLED_bm | USB_BUSNAK_bm;
    }
    return true;
}

void dcd_edpt_stall(uint8_t rhport, uint8_t ep_addr) {
    (void)rhport;
    uint8_t const ep_num = tu_edpt_number(ep_addr);
    tusb_dir_t const dir = tu_edpt_dir(ep_addr);
    if(dir == TUSB_DIR_OUT) {
        fifo_desc.eps[ep_num].OUT.CTRL |= USB_DOSTALL_bm;
    } else {
        fifo_desc.eps[ep_num].IN.CTRL |= USB_DOSTALL_bm;
    }
}

void dcd_edpt_clear_stall(uint8_t rhport, uint8_t ep_addr) {
    (void) rhport;
    uint8_t const ep_num = tu_edpt_number(ep_addr);
    tusb_dir_t const dir = tu_edpt_dir(ep_addr);
    if(dir == TUSB_DIR_OUT) {
        poll_for_rmw();
        USB0.STATUS[ep_num].OUTCLR = USB_TOGGLE_bm;
        fifo_desc.eps[ep_num].OUT.CTRL &= ~USB_DOSTALL_bm;
    } else {
        poll_for_rmw();
        USB0.STATUS[ep_num].INCLR = USB_TOGGLE_bm;
        fifo_desc.eps[ep_num].IN.CTRL &= ~USB_DOSTALL_bm;
    }
}

void dcd_edpt0_status_complete(uint8_t /* rhport */, tusb_control_request_t const * request) {
    // Set address here if the completed transaction is SET_ADDRESS
    if (request->bmRequestType_bit.recipient == TUSB_REQ_RCPT_DEVICE
        && request->bmRequestType_bit.type == TUSB_REQ_TYPE_STANDARD
        && request->bRequest == TUSB_REQ_SET_ADDRESS) {
            USB0.ADDR = (uint8_t) request->wValue;
    }
}

//--------------------------------------------------------------------+
// ISR
//--------------------------------------------------------------------+

void dcd_bus_int_handler(void) {
    uint8_t intflagsa = USB0.INTFLAGSA;
    if (intflagsa & USB_SOF_bm) {
        // SOF; notify tusb
        USB0.INTFLAGSA = USB_SOF_bm;
        dcd_event_bus_signal(0, DCD_EVENT_SOF, true);
    }
    if (intflagsa & USB_RESET_bm) {
        // RESET; reset peripheral and notify tusb
        USB0.ADDR = 0;
        USB0.FIFORP = 0;
        USB0.INTFLAGSA = USB_SOF_bm | USB_SUSPEND_bm | USB_RESUME_bm | USB_RESET_bm | USB_STALLED_bm | USB_UNF_bm | USB_OVF_bm;
        USB0.INTFLAGSB = USB_TRNCOMPL_bm | USB_GNDONE_bm | USB_SETUP_bm;
        disable_all_eps();
        configure_ep0_for_setup();
        // USB0.CTRLB = 0;
        USB0.CTRLA = USB_ENABLE_bm | USB_FIFOEN_bm | ((CFG_TUD_ENDPPOINT_MAX - 1) & 0x0F);
        dcd_event_bus_signal(0, DCD_EVENT_BUS_RESET, true);
    }
    if (intflagsa & USB_SUSPEND_bm) {
        // SUSPEND; just notify tusb
        USB0.INTFLAGSA = USB_SUSPEND_bm;
        dcd_event_bus_signal(0, DCD_EVENT_SUSPEND, true);
    }
    if (intflagsa & USB_RESUME_bm) {
        // RESUME; just notify tusb
        USB0.INTFLAGSA = USB_RESUME_bm;
        dcd_event_bus_signal(0, DCD_EVENT_RESUME, true);
    }
}

static void handle_trancompl(void) {
    while (USB0.INTFLAGSB & USB_TRNCOMPL_bm) {
        // The FIFO is poorly documented in the current provisional version of DS;
        // FIFORP seems to be a signed negative integer corresponding to the offset
        // from the endpoint table base address
        uint8_t const fifon = fifo_desc.fifo[2 * CFG_TUD_ENDPPOINT_MAX + (int8_t)USB0.FIFORP];

        uint8_t ep_num = fifon >> 4;
        bool dir_in = fifon & USB_DIR_bm;
        uint8_t ep_addr = tu_edpt_addr(ep_num, dir_in);
        uint16_t bytes_transferred;
        if(dir_in) {
            bytes_transferred = fifo_desc.eps[ep_num].IN.CNT;
            poll_for_rmw();
            USB0.STATUS[0].INCLR = USB_TRNCOMPL_bm;
        } else {
            bytes_transferred = fifo_desc.eps[ep_num].OUT.CNT;
            poll_for_rmw();
            USB0.STATUS[0].OUTCLR = USB_TRNCOMPL_bm;
        }
        dcd_event_xfer_complete(0, ep_addr, bytes_transferred, XFER_RESULT_SUCCESS, true);
    }
}

void dcd_xfer_int_handler(void) {
    // TRNCOMPL is cleared when FIFO has been emptied
    handle_trancompl();

    uint8_t intflagsb = USB0.INTFLAGSB;
    if(intflagsb & USB_SETUP_bm) {
        // SETUP transaction (assumed to be on EP0)
        USB0.INTFLAGSB = USB_SETUP_bm;

        dcd_event_setup_received(0, setup_buffer, true);
        // Other flags in STATUS are handled by hardware
        // DS demands EPSETUP cleared before parsing SETUP packet
        poll_for_rmw();
        USB0.STATUS[0].OUTCLR = USB_EPSETUP_bm;
        poll_for_rmw();
        USB0.STATUS[0].INCLR = USB_EPSETUP_bm;
        // EP0 stall is cleared on a new SETUP.
        fifo_desc.eps[0].IN.CTRL &= ~USB_DOSTALL_bm;
        fifo_desc.eps[0].OUT.CTRL &= ~USB_DOSTALL_bm;
    }
    if(intflagsb & USB_GNDONE_bm) {
        // We dont't care
        USB0.INTFLAGSB = USB_GNDONE_bm;
    }
}


void dcd_int_handler(uint8_t rhport) {
    (void) rhport;
    // DU has separate interrupt lines for bus event and transfer complete event.
    // These can be directly called instead of this unified handler for reduced latency.
    // if (USB0.INTFLAGSB & ~(USB_RMWBUSY_bm | USB_GNDONE_bm)) {
        dcd_xfer_int_handler();
    // } else {
        dcd_bus_int_handler();
    // }
}

#endif
