/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2023 Ha Thach (tinyusb.org)
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

#if CFG_TUH_ENABLED && CFG_TUSB_MCU == OPT_MCU_NONE

#include "host/hcd.h"
#include "ux_hcd_xhci_api.h"
//#include "dcd_ensemble_def.h" //for gobal USB registers

#include "clk.h"
#include "power.h"

// Max number of endpoints application can open, can be larger than DWC2_CHANNEL_COUNT_MAX
#ifndef CFG_TUH_DWC2_ENDPOINT_MAX
#define CFG_TUH_DWC2_ENDPOINT_MAX 16
#endif

#define DWC2_CHANNEL_COUNT_MAX    16 // absolute max channel count
TU_VERIFY_STATIC(CFG_TUH_DWC2_ENDPOINT_MAX <= 255, "currently only use 8-bit for index");

enum {
  HCD_XFER_ERROR_MAX = 3
};

enum {
  HCD_XFER_PERIOD_SPLIT_NYET_MAX = 3
};

//--------------------------------------------------------------------
//
//--------------------------------------------------------------------

typedef union {
  uint32_t value;
  struct TU_ATTR_PACKED {
    uint32_t ep_size         : 11; // 0..10 Maximum packet size
    uint32_t ep_num          :  4; // 11..14 Endpoint number
    uint32_t ep_dir          :  1; // 15 Endpoint direction
    uint32_t rsv16           :  1; // 16 Reserved
    uint32_t low_speed_dev   :  1; // 17 Low-speed device
    uint32_t ep_type         :  2; // 18..19 Endpoint type
    uint32_t err_multi_count :  2; // 20..21 Error (splitEn = 1) / Multi (SplitEn = 0)  count
    uint32_t dev_addr        :  7; // 22..28 Device address
    uint32_t odd_frame       :  1; // 29 Odd frame
    uint32_t disable         :  1; // 30 Channel disable
    uint32_t enable          :  1; // 31 Channel enable
  };
} dwc2_channel_char_t;
TU_VERIFY_STATIC(sizeof(dwc2_channel_char_t) == 4, "incorrect size");

typedef union {
  uint32_t value;
  struct TU_ATTR_PACKED {
    uint32_t hub_port    :  7; // 0..6 Hub port number
    uint32_t hub_addr    :  7; // 7..13 Hub address
    uint32_t xact_pos    :  2; // 14..15 Transaction position
    uint32_t split_compl :  1; // 16 Split completion
    uint32_t rsv17_30    : 14; // 17..30 Reserved
    uint32_t split_en    :  1; // 31 Split enable
  };
} dwc2_channel_split_t;
TU_VERIFY_STATIC(sizeof(dwc2_channel_split_t) == 4, "incorrect size");

// Host driver struct for each opened endpoint
typedef struct {
  union {
    uint32_t hcchar;
    dwc2_channel_char_t hcchar_bm;
  };
  union {
    uint32_t hcsplt;
    dwc2_channel_split_t hcsplt_bm;
  };

  struct TU_ATTR_PACKED {
    uint32_t uframe_interval : 18; // micro-frame interval
    uint32_t speed           : 2;
    uint32_t next_pid        : 2; // PID for next transfer
    uint32_t next_do_ping    : 1; // Do PING for next transfer if possible (highspeed OUT)
    // uint32_t : 9;
  };

  uint32_t uframe_countdown; // micro-frame count down to transfer for periodic, only need 18-bit

  uint8_t* buffer;
  uint16_t buflen;
} hcd_endpoint_t;

// Additional info for each channel when it is active
typedef struct {
  volatile bool allocated;
  uint8_t ep_id;
  struct TU_ATTR_PACKED {
    uint8_t err_count : 3;
    uint8_t period_split_nyet_count : 3;
    uint8_t halted_nyet : 1;
  };
  uint8_t result;

  uint16_t xferred_bytes;  // bytes that accumulate transferred though USB bus for the whole hcd_edpt_xfer(), which can
                           // be composed of multiple channel_xfer_start() (retry with NAK/NYET)
  uint16_t fifo_bytes;     // bytes written/read from/to FIFO (may not be transferred on USB bus).
} hcd_xfer_t;

typedef struct {
  hcd_xfer_t xfer[DWC2_CHANNEL_COUNT_MAX];
  hcd_endpoint_t edpt[CFG_TUH_DWC2_ENDPOINT_MAX];
} hcd_data_t;

hcd_data_t _hcd_data;

#if 1
// USB Registers Access Types
#define _rw volatile uint32_t
#define __w volatile uint32_t
#define __r volatile const uint32_t

volatile struct {
    union {
        _rw gctl;                       // Global Core Control Register
        struct {
            _rw dsblclkgtng      : 1;   // Disable clock gating
            __r gblhibernationen : 1;   // Hibernation enable status at the global level
            __r : 1;                    // Reserved
            _rw disscramble      : 1;   // Disable scrambling
            _rw scaledown        : 2;   // Scale-down mode
            _rw ramclksel        : 2;   // RAM clock select
            __r : 2;                    // Reserved
            _rw sofitpsync       : 1;   // Reserved
            _rw coresoftreset    : 1;   // Core soft reset
            _rw prtcapdir        : 2;   // Port capability direction
            _rw frmscldwn        : 2;   // Frame scales down
            __r : 1;                    // Reserved
            _rw bypssetaddr      : 1;   // Bypass set address in Device mode
            __r : 14;                   // Reserved
        } gctl_b;
    };
} *host_ugbl = (void *) (USB_BASE + 0xC110);
#endif


// Allocate a new endpoint
TU_ATTR_ALWAYS_INLINE static inline uint8_t edpt_alloc(void) {
  for (uint32_t i = 0; i < CFG_TUH_DWC2_ENDPOINT_MAX; i++) {
    hcd_endpoint_t* edpt = &_hcd_data.edpt[i];
    if (edpt->hcchar_bm.enable == 0) {
      tu_memclr(edpt, sizeof(hcd_endpoint_t));
      edpt->hcchar_bm.enable = 1;
      return i;
    }
  }
  return TUSB_INDEX_INVALID_8;
}

// Find a endpoint that is opened previously with hcd_edpt_open()
// Note: EP0 is bidirectional
TU_ATTR_ALWAYS_INLINE static inline uint8_t edpt_find_opened(uint8_t dev_addr, uint8_t ep_num, uint8_t ep_dir) {
  for (uint8_t i = 0; i < (uint8_t)CFG_TUH_DWC2_ENDPOINT_MAX; i++) {
    const dwc2_channel_char_t* hcchar_bm = &_hcd_data.edpt[i].hcchar_bm;
    if (hcchar_bm->enable && hcchar_bm->dev_addr == dev_addr &&
        hcchar_bm->ep_num == ep_num && (ep_num == 0 || hcchar_bm->ep_dir == ep_dir)) {
      return i;
    }
  }
  return TUSB_INDEX_INVALID_8;
}

//--------------------------------------------------------------------+
// Controller API
//--------------------------------------------------------------------+

#define ONE_KB                            1024
#define UX_DEMO_STACK_SIZE                (4*ONE_KB)
#define UX_DEMO_NS_SIZE                   (128 * ONE_KB)
#define UX_REGULAR_MEMORY_SIZE            (79 * ONE_KB)
#define UX_CACHE_SAFE_MEMORY_SIZE         (20 * ONE_KB)

static uint8_t dma_buf[UX_DEMO_NS_SIZE]__attribute__((section("usb_dma_buf")));

static volatile bool _set_address_requested = false;

// optional hcd configuration, called by tuh_configure()
bool hcd_configure(uint8_t rhport, uint32_t cfg_id, const void* cfg_param) {
  (void) rhport;
  (void) cfg_id;
  (void) cfg_param;

  printf("Called %s(%u %u %p)\n", __FUNCTION__, rhport, cfg_id, cfg_param);
  return true;
}

UX_HCD *_hcd;

// Initialize controller to host mode
bool hcd_init(uint8_t rhport, const tusb_rhport_init_t* rh_init) {
  (void) rhport;
  (void) rh_init;

  static UX_HCD hcd __attribute__((section("usb_dma_buf")));
  _hcd = &hcd;
  uint32_t ret;

  ret = ux_system_initialize(dma_buf, UX_REGULAR_MEMORY_SIZE, dma_buf + UX_REGULAR_MEMORY_SIZE, UX_CACHE_SAFE_MEMORY_SIZE);
  /* Check for error.  */
  if (ret != UX_SUCCESS)
  {
      return false;
  }

  // enable 20mhz clock
  enable_cgu_clk20m();
  // enable usb peripheral clock
  enable_usb_periph_clk();
  // power up usb phy
  enable_usb_phy_power();
  // disable usb phy isolation
  disable_usb_phy_isolation();
  // clear usb phy power-on-reset signal
  CLKCTL_PER_MST->USB_CTRL2 &= ~(1 << 8);

  sys_busy_loop_us(50000);
  host_ugbl->gctl_b.coresoftreset = 1;
  sys_busy_loop_us(50000);
  host_ugbl->gctl_b.prtcapdir = 0x1; //Host mode
  sys_busy_loop_us(50000);
  host_ugbl->gctl_b.coresoftreset = 0;
  sys_busy_loop_us(50000);


  hcd.ux_hcd_io = (void*)USB_BASE;

  ret = _ux_hcd_xhci_initialize(&hcd);

  printf("%010u _ux_hcd_xhci_initialize() returned %d\r\n", DWT->CYCCNT, ret);

  return ret == UX_SUCCESS;
}


// Interrupt Handler
void hcd_int_handler(uint8_t rhport, bool in_isr) {
  (void) rhport;
  (void) in_isr;
  uint32_t port_status = hcd_xhci->op_regs->PORTSC;
#ifdef DEBUG
  printf("%010u Called %s(%u %u), port_status=%#x\r\n", DWT->CYCCNT, __FUNCTION__, rhport, in_isr, port_status);
#endif
#if 0
  printf("PS: %s%s%s%s%s%sPLS%u %sROS%u RWS%u %s%s%s%s%s%s%s%s%s%s%s%s%s%s\r\n",
         port_status & UX_PS_CCS ? "CSS " : "",
         port_status & UX_PS_PES ? "PES " : "",
         port_status & UX_PS_PSS ? "PSS " : "",
         port_status & UX_PS_POCI? "POCI " : "",
         port_status & UX_PS_PRS ? "PRS " : "",
         port_status & UX_PS_PPS ? "PPS " : "",
         (port_status >> 5) & 0x0f,
         port_status & (1 << 9) ? "PP " : "",
         (port_status >> 10) & 0x0f,
         (port_status >> 14) & 0x03,
         port_status & (1 << 16) ? "LWS " : "",
         port_status & (1 << 17) ? "CSC " : "",
         port_status & (1 << 18) ? "PEC " : "",
         port_status & (1 << 19) ? "WRC " : "",
         port_status & (1 << 20) ? "OCC " : "",
         port_status & (1 << 21) ? "PRC " : "",
         port_status & (1 << 22) ? "PLC " : "",
         port_status & (1 << 23) ? "CEC " : "",
         port_status & (1 << 24) ? "CAS " : "",
         port_status & (1 << 25) ? "WCE " : "",
         port_status & (1 << 26) ? "WDE " : "",
         port_status & (1 << 27) ? "WOE " : "",
         port_status & (1 << 30) ? "DR " : "",
         port_status & (1 << 31) ? "WPR " : ""
        );
#endif

  UX_HCD_XHCI *xhci = hcd_xhci;

  // Check if port status changed, handle device attach/remove event
  if (_ux_hcd_xhci_port_current_status_get(hcd_xhci, 0))
  {
    #ifdef DEBUG
    printf("hcd_xhci_port_status_changed\r\n");
    #endif
    UX_HCD * hcd = hcd_xhci -> ux_hcd_xhci_hcd_owner;
    // Is this HCD operational?
    if (hcd -> ux_hcd_status == UX_HCD_STATUS_OPERATIONAL)
    {
      // Call HCD for port status
      uint32_t port_status =  hcd -> ux_hcd_entry_function(hcd,
              UX_HCD_GET_PORT_STATUS, (void *)((ALIGN_TYPE)rhport));
      // Check return status
      if (port_status != UX_PORT_INDEX_UNKNOWN)
      {
          // The port_status value is valid and will tell us if there is
          // a device attached\detached on the downstream port.
          if (port_status & UX_PS_CCS) {
              hcd_event_device_attach(rhport, true);
          } else {
              hcd_event_device_remove(rhport, true);
          }
      }
    }
  }

#if 1
  UX_XHCI_TRB *event_ring_deq;
  uint64_t reg_64;
  uint32_t reg;
  uint32_t irq_pending;
  if (xhci == NULL)
      return;
  /* Check if the xHC generated the interrupt, or the irq is shared */
  reg = xhci->op_regs->USBSTS;
  if (reg == ~(uint32_t)0)
  {
      _ux_hcd_xhci_hc_died(xhci);
      return;
  }
  if (!(reg & STS_EINT))
      return;
  if (reg & STS_FATAL)
  {
#ifdef DEBUG
      printf("WARNING: Host System Error\n");
#endif
      _ux_hcd_xhci_halt(xhci);
      return;
  }
  /*
   * Clear the op reg interrupt status first,
   * so we can receive interrupts from other MSI-X interrupters.
   * Write 1 to clear the interrupt status.
   */
  reg |= STS_EINT;
  xhci->op_regs->USBSTS = reg;
  /* Read the interrupter Set Register  */
  irq_pending = xhci->run_regs->ir_set[0].IMAN;
  irq_pending |= IMAN_IP;
  xhci->run_regs->ir_set[0].IMAN = irq_pending;
#endif

#if 1
  if ((xhci->xhc_state & UX_XHCI_STATE_DYING) || (xhci->xhc_state & UX_XHCI_STATE_HALTED))
  {
#ifdef DEBUG
      printf("xHCI dying, ignoring interrupt. ""Shouldn't IRQs be disabled?\n");
#endif
      /* Clear the event handler busy flag (RW1C);
       * the event ring should be empty.
       */
      reg_64 = xhci->run_regs->ir_set[0].ERDP;
      xhci->run_regs->ir_set[0].ERDP = reg_64 | ERST_EHB;
      return;
  }

#endif
  event_ring_deq = xhci->event_ring->dequeue;


#if 1 //TODO
  UX_XHCI_TRB *event;
  event = xhci->event_ring->dequeue;
//  RTSS_InvalidateDCache_by_Addr(&event->event_cmd, sizeof(event->event_cmd));

  uint32_t trb_comp_code = GET_COMP_CODE((event->trans_event.transfer_len));
  uint32_t slot_id = TRB_TO_SLOT_ID((event->trans_event.flags));
  int32_t ep_index = TRB_TO_EP_ID((event->trans_event.flags)) - 1;
  uint32_t trb_type = (((event->event_cmd.flags) & TRB_TYPE_BITMASK) >> 10);
  xfer_result_t xfer_result = trb_comp_code + XFER_RESULT_SUCCESS - COMP_SUCCESS;
  printf("trb_comp_code=%d, xfer_result=%d, TRB_TYPE=%u, ep_index=%d\r\n", trb_comp_code, xfer_result, trb_type, ep_index);

  //FIXME: force assert to prevent board crash later
  TU_ASSERT(trb_comp_code == 1,);

  if (trb_type == TRB_TRANSFER)
  {
#ifdef DEBUG
      printf("%s() TRB_TRANSFER \r\n", __FUNCTION__);
#endif
//      _ux_utility_event_flags_set(&CONTROL_EP_FLAG, UX_XHCI_CONTROL_EP_EVENT, TX_OR);

      printf("len=%d, flags=0x%lx, slot_id=%u\r\n",
             EVENT_TRB_LEN(event->trans_event.transfer_len),
             event->trans_event.flags, slot_id);


#if 1
      TU_ASSERT(ep_index < CFG_TUH_DWC2_ENDPOINT_MAX,);
      hcd_endpoint_t* edpt = &_hcd_data.edpt[ep_index];
      dwc2_channel_char_t* hcchar_bm = &edpt->hcchar_bm;

      //TODO: for now ep_index set by USBX code. Need to set it in hcd_edpt_xfer() <- currently this variant
      //      or use ep_index directly without tu_edpt_addr()
      //TODO: also get dev_addr from slot_id
      uint8_t ep_addr = tu_edpt_addr(hcchar_bm->ep_num, hcchar_bm->ep_dir);
      printf("dev_addr=%d, ep_addr=%d\r\n", hcchar_bm->dev_addr, ep_addr);
#endif
      //hcd_event_xfer_complete(hcchar.dev_addr, ep_addr, xfer->xferred_bytes, (xfer_result_t)xfer->result, in_isr);
      //hcd_event_xfer_complete(0, 0, EVENT_TRB_LEN(event->trans_event.transfer_len), xfer_result, true);
      //hcd_event_xfer_complete(hcchar_bm->dev_addr, ep_addr, EVENT_TRB_LEN(event->trans_event.transfer_len), xfer_result, true);
      //FIXME: may be we shall pass original_length - EVENT_TRB_LEN(event->trans_event.transfer_len) ?
      hcd_event_xfer_complete(0, ep_index, EVENT_TRB_LEN(event->trans_event.transfer_len), xfer_result, true);

#if 0
    int32_t status = -1;
    UX_XHCI_VIRT_DEVICE  *xdev;
    UX_XHCI_VIRT_EP   *ep;
    UX_XHCI_TD   *td = NULL;

    xdev = xhci->devs[slot_id];
    if (!xdev)
    {
#ifdef DEBUG
        printf("ERROR Transfer event pointed to bad slot %u\n",slot_id);
#endif
        return;
    }
    ep = &xdev->eps[ep_index];

      //TODO: call to
      //process_ctrl_td(xhci, td, ep_trb, event, ep, &status);
      //process_ctrl_td(xhci, td, NULL, &event->trans_event, ep, &status);
      finish_td(xhci, td, event, ep, &status);
#endif
  }
  else if (trb_type == TRB_COMPLETION)
  {
#ifdef DEBUG
      printf("%s() TRB_COMPLETION\r\n", __FUNCTION__);
#endif
    UX_XHCI_COMMAND *cmd = _ux_hcd_xhci_list_first_entry(&xhci->cmd_list, UX_XHCI_COMMAND, cmd_list);
    UX_XHCI_TRB * cmd_trb = xhci->cmd_ring->dequeue;
    uint32_t cmd_type = TRB_FIELD_TO_TYPE((cmd_trb->generic.field[3]));

    printf("command_trb = %p, cmd_trb=%p, event->cmd_trb=0x%"PRIx64", cmd_type=%lu, status=0x%lx, flags=0x%lx\r\n",
           cmd->command_trb, cmd_trb, event->event_cmd.cmd_trb, cmd_type, event->event_cmd.status, event->event_cmd.flags);

    if ((cmd_type == TRB_ADDR_DEV) && _set_address_requested)
    {
#ifdef DEBUG
        printf("%s() TRB_ADDR_DEV\r\n", __FUNCTION__);
#endif
        _set_address_requested = false;

      uint8_t dev_addr = 0;
      hcd_event_xfer_complete(dev_addr, 0, EVENT_TRB_LEN(event->trans_event.transfer_len), xfer_result, true);
    }

  }
  /* TODO: handle
    TRB_PORT_STATUS
    TRB_ENABLE_SLOT
    TRB_ADDR_DEV
  */
  //else //FIXME: this else removed for now because _ux_hcd_xhci_control_transfer_request() waits for complete flag
#endif

#if 1
  {
  //_ux_xhci_event_irq_handler(hcd_xhci);
      if(_ux_hcd_xhci_handle_events(xhci))
          _ux_hcd_xhci_update_erst_dequeue(xhci, event_ring_deq);

  }
#else
  {


      //TODO: _ux_hcd_xhci_update_erst_dequeue(xhci, event_ring_deq);
  }
#endif
}

// Enable USB interrupt
void hcd_int_enable (uint8_t rhport) {
  (void) rhport;
//    printf("Called %s(%u)\n", __FUNCTION__, rhport);
    NVIC_EnableIRQ(USB_IRQ_IRQn);
}

// Disable USB interrupt
void hcd_int_disable(uint8_t rhport) {
  (void) rhport;
//  printf("Called %s(%u)\n", __FUNCTION__, rhport);
    NVIC_DisableIRQ(USB_IRQ_IRQn);
}

// Get frame number (1ms)
uint32_t hcd_frame_number(uint8_t rhport) {
  (void) rhport;
  printf("Called %s(%u)\n", __FUNCTION__, rhport);
  return 0;
}

//--------------------------------------------------------------------+
// Port API
//--------------------------------------------------------------------+

// Get the current connect status of roothub port
bool hcd_port_connect_status(uint8_t rhport) {
  UX_HCD * hcd = hcd_xhci -> ux_hcd_xhci_hcd_owner;
  uint32_t port_status = hcd -> ux_hcd_entry_function(hcd, UX_HCD_GET_PORT_STATUS, (void *)((ALIGN_TYPE)rhport));
  return (port_status & UX_PS_CCS);
}

// Reset USB bus on the port. Return immediately, bus reset sequence may not be complete.
// Some port would require hcd_port_reset_end() to be invoked after 10ms to complete the reset sequence.
void hcd_port_reset(uint8_t rhport) {
  UX_HCD * hcd = hcd_xhci -> ux_hcd_xhci_hcd_owner;
  uint32_t port_status =  hcd -> ux_hcd_entry_function(hcd, UX_HCD_RESET_PORT, (void *)((ALIGN_TYPE)rhport));
  if (port_status != UX_SUCCESS) {
    printf("ERROR: HCD port reset has failed\r\n");
  } else {
    printf("DEBUG: HCD port reset success\r\n");
  }
}

// Complete bus reset sequence, may be required by some controllers
void hcd_port_reset_end(uint8_t rhport) {
  (void) rhport;
}

// Get port link speed
tusb_speed_t hcd_port_speed_get(uint8_t rhport) {
  (void) rhport;
  uint32_t port_sts_ctrl = hcd_xhci->op_regs->PORTSC;
  if (DEV_LOWSPEED(port_sts_ctrl)) {
    printf("DEBUG: low speed device\r\n");
    return TUSB_SPEED_LOW;
  } else if (DEV_FULLSPEED(port_sts_ctrl)) {
    printf("DEBUG: full speed device\r\n");
    return TUSB_SPEED_FULL;
  } else if (DEV_HIGHSPEED(port_sts_ctrl)) {
    printf("DEBUG: high speed device\r\n");
    return TUSB_SPEED_HIGH;
  } else {
    printf("ERROR: invalid device speed (%x)\r\n", DEV_PORT_SPEED(port_sts_ctrl));
    return TUSB_SPEED_INVALID;
  }
}

// HCD closes all opened endpoints belong to this device
void hcd_device_close(uint8_t rhport, uint8_t dev_addr) {
  (void) rhport;
  (void) dev_addr;
  printf("Called %s(%u %u)\n", __FUNCTION__, rhport, dev_addr);
}

//--------------------------------------------------------------------+
// Endpoints API
//--------------------------------------------------------------------+
#include "host/usbh.h"

unsigned int tx_timer_activate(UX_TIMER *timer)
{
  // Dummy function
  //printf("Called %s(%p)\n\r", __FUNCTION__, timer);
  // NOTE: TinyUSB assumes each USB event triggers interrupt,
  // dosen't require status change polling.
  return 0;
}

UX_DEVICE  *_ux_host_stack_new_device_get(void)
{

#if UX_MAX_DEVICES > 1
ULONG           container_index;
#endif
UX_DEVICE       *device;

    /* Start with the first device.  */
    void *memory =  _ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY, sizeof(UX_DEVICE));
    device =  (UX_DEVICE *) memory;

#if UX_MAX_DEVICES > 1
    /* Reset the container index.  */
    container_index =  0;

    /* Search the list until the end.  */
    while (container_index++ < _ux_system_host -> ux_system_host_max_devices)
#endif
    {

        /* Until we have found an unused entry.  */
        if (device -> ux_device_handle == UX_UNUSED)
        {

            /* Reset the entire entry.  */
            _ux_utility_memory_set(device, 0, sizeof(UX_DEVICE)); /* Use case of memset is verified. */

            /* This entry is now used.  */
            device -> ux_device_handle =  UX_USED;

            /* Return the device pointer.  */
            return(device);
        }
#if UX_MAX_DEVICES > 1

        /* Move to the next device entry.  */
        device++;
#endif
    }

    /* No unused devices, return NULL.  */
    return(UX_NULL);
}

unsigned int  _ux_host_stack_transfer_request(UX_TRANSFER *transfer_request)
{

//UX_INTERRUPT_SAVE_AREA

UX_ENDPOINT     *endpoint;
UX_DEVICE       *device;
UX_HCD          *hcd;
unsigned int            status;

#ifdef DEBUG
    printf("_ux_host_stack_transfer_request()\r\n");
#endif

    /* Get the endpoint container from the transfer_request */
    endpoint =  transfer_request -> ux_transfer_request_endpoint;

    /* Get the device container from the endpoint.  */
    device =  endpoint -> ux_endpoint_device;

    /* Ensure we are not preempted by the enum thread while we check the device
       state and set the transfer status.  */
    //UX_DISABLE

    /* We can only transfer when the device is ATTACHED, ADDRESSED OR CONFIGURED.  */
    if ((device -> ux_device_state == UX_DEVICE_ATTACHED) || (device -> ux_device_state == UX_DEVICE_ADDRESSED)
            || (device -> ux_device_state == UX_DEVICE_CONFIGURED))
    {

        /* Set the transfer to pending.  */
        transfer_request -> ux_transfer_request_completion_code =  UX_TRANSFER_STATUS_PENDING;

        /* Save the thread making this transfer. If we're under interrupt, this
           will be null.  */
        //transfer_request -> ux_transfer_request_thread_pending =  _ux_utility_thread_identify();
    }
    else
    {

        /* The device is in an invalid state. Restore interrupts and return error.  */
        //UX_RESTORE

        /* Check if this is endpoint 0.  */
        //if ((endpoint -> ux_endpoint_descriptor.bEndpointAddress & (unsigned int)~UX_ENDPOINT_DIRECTION) == 0)
        //{

        //    /* Check if the class has already protected it.  */
        //    if (device -> ux_device_protection_semaphore.tx_semaphore_count == 0)
        //    {

        //        /* Class is using endpoint 0. Unprotect semaphore.  */
        //        _ux_utility_semaphore_put(&device -> ux_device_protection_semaphore);
        //    }
        //}

        return(UX_TRANSFER_NOT_READY);
    }

    /* Restore interrupts.  */
    //UX_RESTORE

    /* If trace is enabled, insert this event into the trace buffer.  */
    //UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_STACK_TRANSFER_REQUEST, device, endpoint, transfer_request, 0, UX_TRACE_HOST_STACK_EVENTS, 0, 0)

    /* With the device we have the pointer to the HCD.  */
    //hcd = UX_DEVICE_HCD_GET(device);
    hcd = hcd_xhci -> ux_hcd_xhci_hcd_owner;

    /* If this is endpoint 0, we protect the endpoint from a possible re-entry.  */
    // if ((endpoint -> ux_endpoint_descriptor.bEndpointAddress & (unsigned int)~UX_ENDPOINT_DIRECTION) == 0)
    // {

    //     /* Check if the class has already protected it.  */
    //     if (device -> ux_device_protection_semaphore.tx_semaphore_count != 0)
    //     {

    //         /* We are using endpoint 0. Protect with semaphore.  */
    //         status =  _ux_utility_semaphore_get(&device -> ux_device_protection_semaphore, UX_WAIT_FOREVER);

    //         /* Check for status.  */
    //         if (status != UX_SUCCESS)

    //             /* Something went wrong. */
    //             return(status);
    //     }
    // }

    /* Send the command to the controller.  */
    status =  hcd -> ux_hcd_entry_function(hcd, UX_HCD_TRANSFER_REQUEST, transfer_request);

    /* If this is endpoint 0, we unprotect the endpoint. */
    // if ((endpoint -> ux_endpoint_descriptor.bEndpointAddress & (UINT)~UX_ENDPOINT_DIRECTION) == 0)

    //     /* We are using endpoint 0. Unprotect with semaphore.  */
    //     _ux_utility_semaphore_put(&device -> ux_device_protection_semaphore);

    /* And return the status.  */
    return(status);
}

unsigned int  _ux_host_stack_device_address_set(UX_DEVICE *device)
{
unsigned int            status = UX_ERROR;
UX_TRANSFER     *transfer_request;
UX_ENDPOINT     *control_endpoint;
unsigned short          device_address;

#ifdef DEBUG
    printf("_ux_host_stack_device_address_set()\r\n");
#endif
    UX_HCD * hcd = hcd_xhci -> ux_hcd_xhci_hcd_owner;
    UX_HCD_XHCI *xhci =  (UX_HCD_XHCI *) hcd -> ux_hcd_controller_hardware;

    /* Retrieve the pointer to the control endpoint.  */
    control_endpoint =  &device -> ux_device_control_endpoint;

    /* Retrieve the transfer request pointer.  */
    transfer_request =  &control_endpoint -> ux_endpoint_transfer_request;

    /* Initialize device address to 1.  */
    device_address =  1;

    /* If trace is enabled, insert this event into the trace buffer.  */
    //UX_TRACE_IN_LINE_INSERT(UX_TRACE_HOST_STACK_DEVICE_ADDRESS_SET, device, device_address, 0, 0, UX_TRACE_HOST_STACK_EVENTS, 0, 0)

    /* Create a transfer request for the SET_ADDRESS request.  */
    transfer_request -> ux_transfer_request_data_pointer =      UX_NULL;
    transfer_request -> ux_transfer_request_requested_length =  0;
    transfer_request -> ux_transfer_request_function =          UX_SET_ADDRESS;
    transfer_request -> ux_transfer_request_type =              UX_REQUEST_OUT | UX_REQUEST_TYPE_STANDARD | UX_REQUEST_TARGET_DEVICE;
    transfer_request -> ux_transfer_request_value =             device_address;
    transfer_request -> ux_transfer_request_index =             0;

    /* Send request to HCD layer.  */
    status =  _ux_host_stack_transfer_request(transfer_request);

    /* Now, this address will be the one used in future transfers.  The transfer may have failed and therefore
        all the device resources including the new address will be free.*/
    device -> ux_device_address =  (unsigned long) device_address;

    /* Check completion status.  */
    if (status == UX_SUCCESS)
    {

        /* Some devices need some time to accept this address.  */
        tusb_time_delay_ms_api(UX_DEVICE_ADDRESS_SET_WAIT);

        /* Return successful status.  */
        return(status);
    }
    else
    {

        /* We have an error at the first device transaction. This is mostly
            due to the device having failed on the reset after power up.
            we will try again either at the root hub or regular hub. */
        return(status);
    }
}

static UX_DEVICE  *_created_device = NULL;

// Open an endpoint
bool hcd_edpt_open(uint8_t rhport, uint8_t dev_addr, tusb_desc_endpoint_t const * ep_desc) {
  (void) rhport;
  (void) dev_addr;
  (void) ep_desc;
  printf("Called %s(%u %u %p)\r\n", __FUNCTION__, rhport, dev_addr, ep_desc);

  // NOTE: ep_desc is allocated on the stack when called from usbh_edpt_control_open()
  // You need to copy the data into a local variable who maintains the state of the endpoint and transfer.
  // Check _hcd_data in hcd_dwc2.c for example.

  UX_HCD * hcd = hcd_xhci -> ux_hcd_xhci_hcd_owner;
  UX_HCD_XHCI *xhci =  (UX_HCD_XHCI *) hcd -> ux_hcd_controller_hardware;

  tuh_bus_info_t bus_info;
  tuh_bus_info_get(dev_addr, &bus_info);

#if 1
  // find a free endpoint
  const uint8_t ep_id = edpt_alloc();
  TU_ASSERT(ep_id < CFG_TUH_DWC2_ENDPOINT_MAX);
  hcd_endpoint_t* edpt = &_hcd_data.edpt[ep_id];

  dwc2_channel_char_t* hcchar_bm = &edpt->hcchar_bm;
  hcchar_bm->ep_size         = tu_edpt_packet_size(ep_desc);
  hcchar_bm->ep_num          = tu_edpt_number(ep_desc->bEndpointAddress);
  hcchar_bm->ep_dir          = tu_edpt_dir(ep_desc->bEndpointAddress);
  hcchar_bm->low_speed_dev   = (bus_info.speed == TUSB_SPEED_LOW) ? 1 : 0;
  hcchar_bm->ep_type         = ep_desc->bmAttributes.xfer; // ep_type matches TUSB_XFER_*
  hcchar_bm->err_multi_count = 0;
  hcchar_bm->dev_addr        = dev_addr;
  hcchar_bm->odd_frame       = 0;
  hcchar_bm->disable         = 0;
  hcchar_bm->enable          = 1;

  dwc2_channel_split_t* hcsplt_bm = &edpt->hcsplt_bm;
  hcsplt_bm->hub_port        = bus_info.hub_port;
  hcsplt_bm->hub_addr        = bus_info.hub_addr;
  hcsplt_bm->xact_pos        = 0;
  hcsplt_bm->split_compl     = 0;
//  hcsplt_bm->split_en        = (rh_speed == TUSB_SPEED_HIGH && bus_info.speed != TUSB_SPEED_HIGH) ? 1 : 0;

  edpt->speed = bus_info.speed;
//  edpt->next_pid = HCTSIZ_PID_DATA0;
  if (ep_desc->bmAttributes.xfer == TUSB_XFER_ISOCHRONOUS) {
      edpt->uframe_interval = 1 << (ep_desc->bInterval - 1);
      if (bus_info.speed == TUSB_SPEED_FULL) {
          edpt->uframe_interval <<= 3;
      }
  } else if (ep_desc->bmAttributes.xfer == TUSB_XFER_INTERRUPT) {
      if (bus_info.speed == TUSB_SPEED_HIGH) {
          edpt->uframe_interval = 1 << (ep_desc->bInterval - 1);
      } else {
          edpt->uframe_interval = ep_desc->bInterval << 3;
      }
  }

  //TODO: if this is new device, need to send TRB_ENABLE_SLOT command
  //      to allocate slot_id
#endif

#if 1
  if (ep_desc->bEndpointAddress == 0) {
    printf("_ux_host_stack_new_device_get() \r\n");
    UX_DEVICE  *device = _ux_host_stack_new_device_get();
    if (device == UX_NULL)
      // UX_TOO_MANY_DEVICES
      return false;

    // Store the device instance.
    _created_device = device;

    // At this stage the device is attached but not configured.
    //   we don't have to worry about power consumption yet.
    //   Initialize the device structure.  */
    device -> ux_device_handle =         (uint32_t) (ALIGN_TYPE) device;
    device -> ux_device_state =          UX_DEVICE_ATTACHED;
    UX_DEVICE_MAX_POWER_SET(device, UX_MAX_SELF_POWER);
    UX_DEVICE_PARENT_SET(device, UX_NULL);
    UX_DEVICE_HCD_SET(device, hcd);
    UX_DEVICE_PORT_LOCATION_SET(device, rhport);
    if (bus_info.speed == TUSB_SPEED_HIGH) {
      device -> ux_device_speed = UX_HIGH_SPEED_DEVICE;
    } else if (bus_info.speed == TUSB_SPEED_FULL) {
      device -> ux_device_speed = UX_FULL_SPEED_DEVICE;
    } else {
      device -> ux_device_speed = UX_LOW_SPEED_DEVICE;
    }

    UX_ENDPOINT  *control_endpoint = &device -> ux_device_control_endpoint;
    control_endpoint -> ux_endpoint =  (unsigned long) (ALIGN_TYPE) control_endpoint;
    //control_endpoint -> ux_endpoint_next_endpoint =  UX_NULL;
    //control_endpoint -> ux_endpoint_interface =      UX_NULL;
    control_endpoint -> ux_endpoint_device = device;
    control_endpoint -> ux_endpoint_transfer_request.ux_transfer_request_endpoint = control_endpoint;

    // If the device is running in high speed the default max packet size for the control endpoint is 64.
    // All other speeds the size is 8.
    if (bus_info.speed == TUSB_SPEED_HIGH) {
      control_endpoint -> ux_endpoint_descriptor.wMaxPacketSize =  UX_DEFAULT_HS_MPS;
    } else {
      control_endpoint -> ux_endpoint_descriptor.wMaxPacketSize =  UX_DEFAULT_MPS;
    }

    // Create the default control endpoint at the HCD level.
    //printf("UX_HCD_CREATE_ENDPOINT\r\n");
    unsigned int status =  hcd -> ux_hcd_entry_function(hcd, UX_HCD_CREATE_ENDPOINT, (void *) control_endpoint);
    if (status == UX_SUCCESS) {
      // Now control endpoint is ready, set state to running
      control_endpoint -> ux_endpoint_state = UX_ENDPOINT_RUNNING;

#if 0
      tusb_time_delay_ms_api(UX_RH_ENUMERATION_RETRY_DELAY);
        /* Set the address of the device. The first time a USB device is
           accessed, it responds to the address 0. We need to change the address
           to a free device address between 1 and 127 ASAP.  */
        status =  _ux_host_stack_device_address_set(device);
        if (status == UX_SUCCESS)
#endif
          return true;
    }
  }

  return false;
#else
  return true;
#endif
}

bool hcd_edpt_close(uint8_t rhport, uint8_t daddr, uint8_t ep_addr) {
  (void) rhport;
  (void) daddr;
  (void) ep_addr;
  printf("Called %s(%u %u %u)\n", __FUNCTION__, rhport, daddr, ep_addr);
  return false; // TODO not implemented yet
}

// Submit a transfer, when complete hcd_event_xfer_complete() must be invoked
bool hcd_edpt_xfer(uint8_t rhport, uint8_t dev_addr, uint8_t ep_addr, uint8_t * buffer, uint16_t buflen) {
  (void) rhport;
  (void) dev_addr;
  (void) ep_addr;
  (void) buffer;
  (void) buflen;

  //TODO: get it from dev_addr
  UX_HCD_XHCI *xhci = hcd_xhci;
  const int32_t slot_id = xhci->slot_id;
  const uint32_t ep_index = ((ep_addr == TUSB_DIR_IN_MASK) ? 0x00 : ep_addr);
  //FIXME: in the USBX there is one ep_index = 0 for all 3 messages in the get_descriptor request
  //       but in the tinyUSB second call is IN request with addr 0x80


  //dwc2_regs_t* dwc2 = DWC2_REG(rhport);
  const uint8_t ep_num = tu_edpt_number(ep_addr);
  const uint8_t ep_dir = tu_edpt_dir(ep_addr);

  uint8_t ep_id = edpt_find_opened(dev_addr, ep_num, ep_dir);

  printf("Called %s(%u %u 0x%x %p %u) ep_id=%u, slot_id=%ld", __FUNCTION__, rhport, dev_addr, ep_addr, buffer, buflen, ep_id, slot_id);
  for (int i = 0; i < buflen; i++)
  {
      printf(" %02x", buffer[i]);
  }
  printf("\n\r");

#if 1
    //FIXME: Fake event to simulate tusb flow
    hcd_event_xfer_complete(0, ep_index, buflen, XFER_RESULT_SUCCESS, false);
#endif

#if 0
  TU_ASSERT(ep_id < CFG_TUH_DWC2_ENDPOINT_MAX);
  hcd_endpoint_t* edpt = &_hcd_data.edpt[ep_id];

  edpt->buffer = buffer;
  edpt->buflen = buflen;

  if (ep_num == 0) {
      // update ep_dir since control endpoint can switch direction
      edpt->hcchar_bm.ep_dir = ep_dir;
  }

//TODO: transfer data
// fill TRB and set DOORBELL

/*
    Required data:
    1. ring for current slot_id and current endpoint.
    2. slot_id, should be allocated in hcd_edpt_open(), then we can get it using dev_addr
       Number of slots is 256 in USBX and 64 in the alif E7 device.
    3. USBX code has fixed array of 31 endpoints for each device
    4. Each enpoint contains the ring which consist from one or more segments (default 2)
       Number of segments can be incremented if required, see _ux_hcd_xhci_ring_expansion()
    5. Each segment has 256 TRBs. The last TRB in each segment has a link to the first TRB
       of the next segment, see _ux_hcd_xhci_link_segments()
    6. ?What about TDs? (Transfer Descriptors)
*/

  UX_XHCI_RING *ep_ring;
  UX_URB_PRIV *urb_priv;
  UX_XHCI_TD *td;
  UX_XHCI_TRB_INFO  trb_info;
  UX_XHCI_GENERIC_TRB *start_trb;
  int32_t num_trbs;
  int32_t start_cycle;
  int32_t ret;
  uint32_t field;
  UX_XHCI_VIRT_EP *ep;
  ep = &xhci->devs[slot_id]->eps[ep_index];
  ep_ring = ep->ring;
  if (!ep_ring)
  {
      return -1;
  }

  num_trbs = 1;
  printf("ep_ring=%p, enqueue=%p, dequeue=%p\r\n", ep_ring, ep_ring->enqueue, ep_ring->dequeue);

  // Retrieve the pointer to the control endpoint.
  UX_DEVICE       *device = _created_device;
  UX_ENDPOINT     *control_endpoint =  &device -> ux_device_control_endpoint;
  UX_TRANSFER     *urb =  &control_endpoint -> ux_endpoint_transfer_request;
  uint32_t        num_tds = 1;

  urb_priv = _ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY, sizeof(*urb_priv) + (sizeof(UX_XHCI_TD) * num_tds));
  if (!urb_priv)
  {
#ifdef DEBUG
      printf("urb_priv alloc failed %d \n",sizeof(UX_XHCI_TD) * num_tds);
#endif
      return -1;
  }
  urb_priv->num_tds = num_tds;
  urb_priv->num_tds_done = 0;
  urb->hcpriv = urb_priv;

  ret = prepare_transfer(xhci, xhci->devs[slot_id], ep_index, xhci->stream_id, num_trbs, urb, 0);
  if (ret < 0)
  {
      return false;
  }
//  urb_priv = urb->hcpriv;
  td = &urb_priv->td[0];
  /*
   * Don't give the first TRB to the hardware (by toggling the cycle bit)
   * until we've finished creating all the other TRBs.  The ring's cycle
   * state may change as we enqueue the other TRBs, so save it too.
   */
  start_trb = &ep_ring->enqueue->generic;
  start_cycle = ep_ring->cycle_state;
  field = 0;
  /* Immediate Data (IDT).bit and SETUP TRB  */
  field |= TRB_IDT | TRB_TYPE(TRB_SETUP);
  if (start_cycle == 0)
      field |= TRB_CYCLE;

  /* xHCI 1.0/1.1 6.4.1.2.1: Transfer Type field */
  if (xhci->hci_version >= 0x100)
  {
      if (urb->ux_transfer_request_requested_length > 0)
      {
          if ((urb -> ux_transfer_request_type & UX_REQUEST_DIRECTION) == UX_REQUEST_IN)
              field |= TRB_TX_TYPE(TRB_DATA_IN);
          else
              field |= TRB_TX_TYPE(TRB_DATA_OUT);
      }
  }

#if 1
  if ((buflen <= 8) && (tu_edpt_dir(ep_addr) == TUSB_DIR_OUT))
  {
    memcpy(&trb_info, buffer, buflen);
  }
  else
  {
    trb_info.low_address = LocalToGlobal(buffer);
    trb_info.high_address = 0;
  }
#else
  trb_info.low_address = urb -> ux_transfer_request_type | urb -> ux_transfer_request_function << 8
                         | urb -> ux_transfer_request_value << 16;
  trb_info.high_address = urb -> ux_transfer_request_index | urb -> ux_transfer_request_requested_length << 16;
#endif
  trb_info.size =  TRB_LEN(buflen) | TRB_INTR_TARGET(0);
  trb_info.cntrl_field = field | TRB_IOC;
  /* Queue the SETUP Stage TRB   */
  queue_trb(xhci, ep_ring, false, &trb_info);

  giveback_first_trb(xhci, slot_id, ep_index, 0, start_cycle, start_trb);
#endif
  return true;
}

// Abort a queued transfer. Note: it can only abort transfer that has not been started
// Return true if a queued transfer is aborted, false if there is no transfer to abort
bool hcd_edpt_abort_xfer(uint8_t rhport, uint8_t dev_addr, uint8_t ep_addr) {
  (void) rhport;
  (void) dev_addr;
  (void) ep_addr;

  printf("Called %s(%u %u %u)\n", __FUNCTION__, rhport, dev_addr, ep_addr);
  return false;
}

#define UX_DEVICE_DESCRIPTOR_LENGTH                                     18
#define UX_REQUEST_TYPE_STANDARD                                        0x00u
#define UX_REQUEST_TARGET_DEVICE                                        0x00u
#define UX_DEVICE_DESCRIPTOR_ITEM                                       1u

// Submit a special transfer to send 8-byte Setup Packet, when complete hcd_event_xfer_complete() must be invoked
bool hcd_setup_send(uint8_t rhport, uint8_t dev_addr, uint8_t const setup_packet[8]) {
  (void) rhport;
  (void) dev_addr;
  (void) setup_packet;

  bool ret = false;
  unsigned int status;

  printf("Called %s(%u %u %p)", __FUNCTION__, rhport, dev_addr, setup_packet);

  for (int i = 0; i < 8; i++)
  {
      printf(" %02x", setup_packet[i]);
  }
  printf("\r\n");

#if 0
  ret = hcd_edpt_xfer(rhport, dev_addr, 0, (uint8_t*)(uintptr_t) setup_packet, 8);
#endif

#if 1
  tusb_time_delay_ms_api(UX_RH_ENUMERATION_RETRY_DELAY);

  // Retrieve the pointer to the control endpoint.
  UX_DEVICE       *device = _created_device;
  UX_ENDPOINT     *control_endpoint =  &device -> ux_device_control_endpoint;
  UX_TRANSFER     *transfer_request =  &control_endpoint -> ux_endpoint_transfer_request;
  unsigned int request_length = setup_packet[6] | (setup_packet[7] << 8); //8;

  if (setup_packet[1] == UX_SET_ADDRESS)
  {
      //This is command TRB so need the different way to process
      int device_address = setup_packet[2] | (setup_packet[3] << 8);;

//      printf("device=%p\r\n", device);
      _set_address_requested = true;
#if 1
//    status = _ux_hcd_xhci_address_device(xhci, (((UX_TRANSFER*) parameter)->ux_transfer_request_endpoint->ux_endpoint_device));
      status = _ux_hcd_xhci_address_device(hcd_xhci, device);
#else
      //FIXME: workaround for testing
      status =  _ux_host_stack_device_address_set(device);
#endif

      /* Now, this address will be the one used in future transfers.  The transfer may have failed and therefore
          all the device resources including the new address will be free.*/
      device -> ux_device_address =  (unsigned long) device_address;

      if (status == UX_SUCCESS)
          return true;

      _set_address_requested = false;
  }
  else
  {
      // Need to allocate memory for the descriptor
      unsigned char * descriptor = UX_NULL;
      descriptor = _ux_utility_memory_allocate(UX_SAFE_ALIGN, UX_CACHE_SAFE_MEMORY,
                   8);
      if (descriptor == UX_NULL)
          return(UX_MEMORY_INSUFFICIENT);

      // Create a transfer_request for the GET_DESCRIPTOR request. The first transfer_request asks
      // for the first 8 bytes only. This way we will know the real MaxPacketSize
      // value for the control endpoint.
      transfer_request -> ux_transfer_request_data_pointer =      descriptor;
      transfer_request -> ux_transfer_request_requested_length =  request_length; //8;
      transfer_request -> ux_transfer_request_function =          setup_packet[1]; //UX_GET_DESCRIPTOR;
      transfer_request -> ux_transfer_request_type =              setup_packet[0]; //UX_REQUEST_IN | UX_REQUEST_TYPE_STANDARD | UX_REQUEST_TARGET_DEVICE;
      transfer_request -> ux_transfer_request_value =             setup_packet[2] | (setup_packet[3] << 8); //UX_DEVICE_DESCRIPTOR_ITEM << 8;
      transfer_request -> ux_transfer_request_index =             setup_packet[4] | (setup_packet[5] << 8); //0;

      // Send request to HCD layer.
      //unsigned int status =  _ux_host_stack_transfer_request(transfer_request);
      // We can only transfer when the device is ATTACHED, ADDRESSED OR CONFIGURED.
      if ((device -> ux_device_state == UX_DEVICE_ATTACHED) || (device -> ux_device_state == UX_DEVICE_ADDRESSED)
              || (device -> ux_device_state == UX_DEVICE_CONFIGURED))
      {
        // Set the transfer to pending.
    //    transfer_request -> ux_transfer_request_completion_code =  UX_TRANSFER_STATUS_COMPLETED;//UX_TRANSFER_STATUS_PENDING;
        transfer_request -> ux_transfer_request_completion_code =  UX_TRANSFER_STATUS_PENDING;

        // Pointer to the HCD.
        UX_HCD * hcd = hcd_xhci -> ux_hcd_xhci_hcd_owner;
        // Send the command to the controller.

        status =  _ux_hcd_xhci_transfer_request(hcd_xhci, transfer_request);
        //unsigned int status =  hcd -> ux_hcd_entry_function(hcd, UX_HCD_TRANSFER_REQUEST, transfer_request);

        //device -> ux_device_address =  (unsigned long) device_address;

        // Check for correct transfer and entire descriptor returned.
        if ((status == UX_SUCCESS) &&
            (transfer_request -> ux_transfer_request_actual_length == request_length))
        {
          // Print descriptor
            printf("response:");

            for (int i = 0; i < 8; i++)
            {
                printf(" %02x", descriptor[i]);
            }
            printf("\r\n");

          return true;
        }
      }

      // Free all used resources.
      _ux_utility_memory_free(descriptor);
  }

  return false;
#else
  return ret;
#endif
}

// clear stall, data toggle is also reset to DATA0
bool hcd_edpt_clear_stall(uint8_t rhport, uint8_t dev_addr, uint8_t ep_addr) {
  (void) rhport;
  (void) dev_addr;
  (void) ep_addr;

  printf("Called %s(%u %u %u)\n", __FUNCTION__, rhport, dev_addr, ep_addr);
  return false;
}

#endif
