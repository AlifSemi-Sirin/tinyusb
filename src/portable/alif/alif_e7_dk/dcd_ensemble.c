/* Copyright (C) 2024 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

#include "tusb_option.h"

#if CFG_TUD_ENABLED

#include "device/dcd.h"
#include "dcd_ensemble_def.h"

#include <zephyr/cache.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>

#define LOG_PRINT_INFO(fmt, ...) printf("[%lld ms] INFO  %s: " fmt "\n", k_uptime_get(), __func__, ##__VA_ARGS__)
#define LOG_PRINT_ERROR(fmt, ...) printf("[%lld ms] ERROR %s: " fmt "\n", k_uptime_get(), __func__, ##__VA_ARGS__)
#define LOG_PRINT_SHORT(fmt, ...) printf(fmt "\n", ##__VA_ARGS__)

#define LOG_NO_INFO(fmt, ...) ((void) 0)
#define LOG_NO_ERROR(fmt, ...) ((void) 0)
#define LOG_NO_SHORT(fmt, ...) ((void) 0)

#define LOG_ALIF_INFO LOG_PRINT_INFO
#define LOG_ALIF_ERROR LOG_PRINT_ERROR
#define LOG_ALIF_SHORT LOG_PRINT_SHORT

#define USB_NODE DT_NODELABEL(usb0)
#define USB_CTRL_BASE DT_REG_ADDR(USB_NODE)
#define USB_ALIF_IRQ DT_IRQN(USB_NODE)

#define DEPEVT_STS_NOTREADY_MASK 0x0B
#define DEPEVT_STS_NOTREADY_CTRL_DATA_RQ 0x01
#define DEPEVT_STS_NOTREADY_CTRL_STAT_RQ 0x02
#define DEPEVT_STS_XFER_ACTIVE   (1U << 3)

#define CONFIG_USB_DEVICE_HIGH_SPEED// ALIF USB HIGH-speed mode activated

#if defined(CONFIG_USB_DEVICE_HIGH_SPEED)
  #define ALIF_DEVSPD_SETTING 0x0 /* High-speed */
  #define ALIF_TUSB_SPEED TUSB_SPEED_HIGH
  #define ALIF_MAX_PCK_SIZE 512
#elif defined(CONFIG_USB_DEVICE_FULL_SPEED)
  #define ALIF_DEVSPD_SETTING 0x1 /* Full-speed */
  #define ALIF_TUSB_SPEED TUSB_SPEED_FULL
  #define ALIF_MAX_PCK_SIZE 64
#else
  #error "You must select CONFIG_USB_DEVICE_HIGH_SPEED or _FULL_SPEED"
#endif

#define XHC_REG_RD(addr) sys_read32((addr))
#define XHC_REG_WR(addr, val) sys_write32((val), (addr))

  // Structs and Buffers --------------------------------------------------------
  #define EVT_BUF_SIZE 1024
  #define CTRL_BUF_SIZE 64

__aligned(4096) CFG_TUSB_MEM_SECTION
    static uint32_t _evnt_buf[EVT_BUF_SIZE];

__aligned(32) CFG_TUSB_MEM_SECTION
    static uint8_t _ctrl_buf[CTRL_BUF_SIZE];// [TODO] runtime alloc

__aligned(32) CFG_TUSB_MEM_SECTION
    static uint32_t _xfer_trb[TUP_DCD_ENDPOINT_MAX][4];// [TODO] runtime alloc

#define ISO_CHAIN_MAX 16
__aligned(32) CFG_TUSB_MEM_SECTION 
    static uint32_t _xfer_trb_iso[ISO_CHAIN_MAX][4];       // isochronous transfer TRBs

__aligned(32) CFG_TUSB_MEM_SECTION  
static uint8_t iso_test_data[ISO_CHAIN_MAX * ALIF_MAX_PCK_SIZE];


static uint16_t _xfer_bytes[TUP_DCD_ENDPOINT_MAX];
static uint16_t _iso_mps[TUP_DCD_ENDPOINT_MAX]; // ISOCHRONOUS endpoint max packet size
static bool _xfer_isochron[TUP_DCD_ENDPOINT_MAX];       // ISOCHRONOUS endpoint status (4 physical endpoints)

static uint8_t * buff_test;
static uint8_t buff_test_size;

/// API Extension --------------------------------------------------------------
static uint8_t usb_dc_alif_send_ep_cmd(uint8_t ep, uint8_t cmd_type, uint16_t param);

static volatile uint32_t *_evnt_tail;
static bool _ctrl_long_data = false;
static bool _xfer_cfgd = false;
static uint32_t _sts_stage = 0;

/// Private Functions ----------------------------------------------------------

static uint8_t _dcd_start_xfer(uint8_t ep, void *buf, uint32_t size, uint8_t type);
static uint8_t _dcd_start_xfer_iso(uint8_t ep, void *buf, uint32_t size, uint16_t start_frame);
static void _dcd_handle_depevt(uint8_t rhport, const evt_t *evt);
static void _dcd_handle_devt(uint8_t rhport, uint8_t evt, uint16_t info);

void dcd_uninit(void);

#if CFG_TUSB_OS == OPT_OS_ZEPHYR
// helpers
static inline uint32_t _get_transfered_bytes(uint8_t ep) {
  return _xfer_bytes[ep] - (_xfer_trb[ep][2] & 0x00FFFFFF);
}

static inline void enable_usb_periph_clk(void) {
  sys_set_bits(EXPMST_PERIPH_CLK_EN, PERIPH_CLK_ENA_USB_CKEN);
}

static inline void disable_usb_periph_clk(void) {
  sys_clear_bits(EXPMST_PERIPH_CLK_EN, PERIPH_CLK_ENA_USB_CKEN);
}

static inline void enable_cgu_clk20m(void) {
  sys_set_bits(CGU_CLK_ENA, CLK_ENA_CLK20M);
}

static inline void disable_cgu_clk20m(void) {
  sys_clear_bits(CGU_CLK_ENA, CLK_ENA_CLK20M);
}

static inline void enable_usb_phy_power(void) {
  sys_clear_bits(VBAT_PWR_CTRL, PWR_CTRL_UPHY_PWR_MASK);
}

static inline void disable_usb_phy_power(void) {
  sys_set_bits(VBAT_PWR_CTRL, PWR_CTRL_UPHY_PWR_MASK);
}

static inline void enable_usb_phy_isolation(void) {
  sys_set_bits(VBAT_PWR_CTRL, PWR_CTRL_UPHY_ISO);
}

static inline void disable_usb_phy_isolation(void) {
  sys_clear_bits(VBAT_PWR_CTRL, PWR_CTRL_UPHY_ISO);
}

static inline void usb_ctrl2_phy_power_on_reset_set(void) {
  sys_set_bits(EXPMST_USB_CTRL2, USB_CTRL2_POR_RST_MASK);
}

static inline void usb_ctrl2_phy_power_on_reset_clear(void) {
  sys_clear_bits(EXPMST_USB_CTRL2, USB_CTRL2_POR_RST_MASK);
}
#endif



/// Device Setup ---------------------------------------------------------------

// Initializes the USB peripheral for device mode and enables it.
// This function should enable internal D+/D- pull-up for enumeration.
bool dcd_init(uint8_t rhport, const tusb_rhport_init_t* rh_init)
{
  (void) rh_init;

  // fill the test buffer with some data
  for (int i = 0; i < sizeof(iso_test_data); i++) {
    iso_test_data[i] = 0xA5;
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
  usb_ctrl2_phy_power_on_reset_clear();

  // force stop/disconnect
  dcd_disconnect(rhport);// ToDO: check if this is needed

  // Device soft reset
  sys_set_bits(DCTL_REG, DCTL_CSFTRST);
  while ((*(volatile uint32_t *) DCTL_REG & DCTL_CSFTRST) != 0) {
    k_busy_wait(1 * 1000);// 1000 μs = 1 ms
  }

  // Core + PHY soft reset
  k_busy_wait(50 * 1000);// 50 ms
  sys_set_bits(GCTL_REG, GCTL_CORESOFTRESET);
  sys_set_bits(GUSB2PHYCFG0_REG, GUSB2PHYCFG0_PHYSOFTRST);
  k_busy_wait(50 * 1000);// 50 ms
  sys_clear_bits(GUSB2PHYCFG0_REG, GUSB2PHYCFG0_PHYSOFTRST);
  k_busy_wait(50 * 1000);// 50 ms
  sys_clear_bits(GCTL_REG, GCTL_CORESOFTRESET);
  k_busy_wait(50 * 1000);// 50 ms

  // Check controller ID
  uint32_t rd_val = XHC_REG_RD(GSNPSID_REG);
  if ((rd_val & 0xFFFF0000) != 0x55330000) {
    LOG_ALIF_ERROR("Invalid USB controller ID! Expected 0x5533xxxx, got 0x%08x", rd_val);
    return false;
  }
//   LOG_ALIF_INFO("Controller ID: 0x%08x", rd_val);

  // Global bus config
  XHC_REG_WR(GSBUSCFG0_REG, GSBUSCFG0_INCRBRSTENA | GSBUSCFG0_INCR16BRSTENA);

  // Configure USB2 PHY
  rd_val = XHC_REG_RD(GUSB2PHYCFG0_REG);
  rd_val &= ~(GUSB2PHYCFG0_PHYIF | GUSB2PHYCFG0_ULPI_UTMI_SEL | GUSB2PHYCFG0_USBTRDTIM_MASK);
  rd_val |= GUSB2PHYCFG0_PHYIF | GUSB2PHYCFG0_USBTRDTIM(5);// UTMI+, 8-bit, HS
  XHC_REG_WR(GUSB2PHYCFG0_REG, rd_val);

  // Set Device Speed (USBHS only)
  sys_clear_bits(DCFG_REG, DCFG_SPEED_MASK);              // ToDo: FULL/HIGH Speed
  sys_set_bits(DCFG_REG, DCFG_SPEED(ALIF_DEVSPD_SETTING));// 0x1: Full-speed 12MBit/s

  // allocate ring buffer for events
  memset(_evnt_buf, 0, sizeof(_evnt_buf));
  sys_cache_data_flush_range(_evnt_buf, sizeof(_evnt_buf));// zephyr equ for RTSS_CleanDCache_by_Addr(..)
  _evnt_tail = _evnt_buf;

  XHC_REG_WR(GEVNTADDRL0_REG, (uint32_t) local_to_global(_evnt_buf));

//   LOG_ALIF_INFO("Address loc->glob: %p -> %x", _evnt_buf, XHC_REG_RD(GEVNTADDRL0_REG));

  // Set Event Buffer Size
  sys_clear_bits(GEVNTSIZ0_REG, GEVNTSIZ0_EVENTSIZ_MASK);
  sys_set_bits(GEVNTSIZ0_REG, GEVNTSIZ0_EVENTSIZ(sizeof(_evnt_buf)));
  // Clear Event Buffer counter
  sys_clear_bits(GEVNTCOUNT0_REG, GEVNTCOUNT0_EVNTCOUNT_MASK);

  // Enable USB events
  sys_set_bits(DEVTEN_REG, DEVTEN_DISSCONEVTEN);
  sys_set_bits(DEVTEN_REG, DEVTEN_USBRSTEVTEN);
  sys_set_bits(DEVTEN_REG, DEVTEN_CONNECTDONEEVTEN);
  sys_set_bits(DEVTEN_REG, DEVTEN_ULSTCNGEN);

  // Endpoint Configuration - Start config phase
  (void) usb_dc_alif_send_ep_cmd(0, CMDTYP_DEPSTARTCFG, 0);

  // Endpoint Configuration - EP0 OUT (ep = 0)
  uint8_t ep_idx = 0;
  XHC_REG_WR(DEPCMDPAR1N(ep_idx), (0 << 25) | (1 << 10) | (1 << 8));
  XHC_REG_WR(DEPCMDPAR0N(ep_idx), (0 << 22) | (0 << 17) | (ALIF_MAX_PCK_SIZE << 3) | (0 << 1));
  (void) usb_dc_alif_send_ep_cmd(ep_idx, CMDTYP_DEPCFG, 0);

  // Endpoint Configuration - EP0 IN (ep = 1)
  ep_idx = 1;
  XHC_REG_WR(DEPCMDPAR1N(ep_idx), (1 << 25) | (1 << 10) | (1 << 8));
  XHC_REG_WR(DEPCMDPAR0N(ep_idx), (0 << 22) | (0 << 17) | (ALIF_MAX_PCK_SIZE << 3) | (0 << 1));
  (void) usb_dc_alif_send_ep_cmd(ep_idx, CMDTYP_DEPCFG, 0);

  // set initial xfer configuration for CONTROL eps
  XHC_REG_WR(DEPCMDPAR0N(0), 1);
  (void) usb_dc_alif_send_ep_cmd(0, CMDTYP_DEPXFERCFG, 0);

  XHC_REG_WR(DEPCMDPAR0N(1), 1);
  (void) usb_dc_alif_send_ep_cmd(1, CMDTYP_DEPXFERCFG, 0);

  // prepare trb for the first setup packet
  uint8_t ep = 0;
  memset(_ctrl_buf, 0, sizeof(_ctrl_buf));
  _xfer_trb[ep][0] = (uint32_t) local_to_global(_ctrl_buf);
  _xfer_trb[ep][1] = 0;
  _xfer_trb[ep][2] = 8;
  _xfer_trb[ep][3] = (1 << 11) | (1 << 10) | (TRBCTL_CTL_SETUP << 4) | (1 << 1) | (1 << 0);
  sys_cache_data_flush_range(_xfer_trb[ep], sizeof(_xfer_trb[0]));

  // send trb to the usb dma
  XHC_REG_WR(DEPCMDPAR1N(ep), (uint32_t) local_to_global(_xfer_trb[ep]));
  XHC_REG_WR(DEPCMDPAR0N(ep), 0);
  (void) usb_dc_alif_send_ep_cmd(ep, CMDTYP_DEPSTRTXFER, 0);

  sys_set_bits(DALEPENA_REG, (1 << ep));      // enable ep0 OUT
  sys_set_bits(DALEPENA_REG, (1 << (ep + 1)));// enable ep0 IN

  // enable pull-ups
  dcd_connect(rhport);

  // 7. Enable IRQ
  NVIC_ClearPendingIRQ(USB_ALIF_IRQ);
  NVIC_SetPriority(USB_ALIF_IRQ, 5);
  IRQ_CONNECT(USB_ALIF_IRQ, 5, dcd_int_handler, NULL, 0);


return true;
}

// Processes all the hardware generated events e.g bus reset, new data packet
// from host... It will be called by application in the MCU USB interrupt handler.
void dcd_int_handler(uint8_t rhport)
{
  (void) rhport;

  // process failures first
  uint32_t gsts = XHC_REG_RD(GSTS_REG);
  if (gsts & GSTS_DEVICE_IP) {
    if ((gsts & GSTS_CSRTIMEOUT) || (gsts & GSTS_BUSERRADDRVLD)) {
      LOG_ALIF_ERROR("USB controller bus error: CSRTimeout=%d, BusErrAddrVld=%d",
                     (gsts & GSTS_CSRTIMEOUT) != 0, (gsts & GSTS_BUSERRADDRVLD) != 0);
      __BKPT(0);
    }
  }

  // cycle through event queue
  // evaluate on every iteration to prevent unnecessary isr exit/reentry
  while (XHC_REG_RD(GEVNTCOUNT0_REG) & GEVNTCOUNT0_EVNTCOUNT_MASK) {
    sys_cache_data_invd_range(_evnt_buf, sizeof(_evnt_buf));
    volatile evt_t e = {.val = *_evnt_tail++};
    //  LOG_ALIF_INFO("%010u IRQ loop, evnt %08x", DWT->CYCCNT, e.val);

    // wrap around
    if (_evnt_tail >= (_evnt_buf + 1024)) _evnt_tail = _evnt_buf;

    // dispatch the right handler for the event type
    if (e.depevt.is_devt == 0) {// DEPEVT
    //   _dcd_handle_depevt(rhport, e.depevt.ep, e.depevt.evt, e.depevt.sts);
      _dcd_handle_depevt(rhport, &e);
    } else {// DEVT
      _dcd_handle_devt(rhport, e.devt.evt, e.devt.info);
    }

    // consume one event
    XHC_REG_WR(GEVNTCOUNT0_REG, 4);
  }

}

void dcd_int_enable (uint8_t rhport)
{
  (void) rhport;
  NVIC_EnableIRQ(USB_ALIF_IRQ);
}

// Disables the USB device interrupt.
// May be used to prevent concurrency issues when mutating data structures
// shared between main code and the interrupt handler.
void dcd_int_disable(uint8_t rhport)
{
  (void) rhport;
  
#if CFG_TUSB_OS == OPT_OS_ZEPHYR 
  NVIC_DisableIRQ(USB_ALIF_IRQ);
#else
  NVIC_DisableIRQ(USB_IRQ_IRQn);
#endif
}

// Receive Set Address request, mcu port must also include status IN response.
// If your peripheral automatically changes address during enumeration you may
// leave this empty and also no queue an event for the corresponding SETUP packet.
void dcd_set_address(uint8_t rhport, uint8_t dev_addr)
{
  uint32_t rd_val = XHC_REG_RD(DCFG_REG);
  rd_val &= ~DCFG_DEVADDR_MASK;
  rd_val |= DCFG_DEVADDR(dev_addr);
  XHC_REG_WR(DCFG_REG, rd_val);

  dcd_edpt_xfer(rhport, tu_edpt_addr(0, TUSB_DIR_IN), NULL, 0);
}

// Called to remote wake up host when suspended (e.g hid keyboard)
void dcd_remote_wakeup(uint8_t rhport)
{
  (void) rhport;

#if CFG_TUSB_OS == OPT_OS_ZEPHYR 
  LOG_ALIF_INFO("%010u >%s", DWT->CYCCNT, __func__);
#else
  LOG("%010u >%s", DWT->CYCCNT, __func__);
#endif
}

// Connect by enabling internal pull-up resistor on D+/D-
void dcd_connect(uint8_t rhport)
{
  (void) rhport;
#if CFG_TUSB_OS == OPT_OS_ZEPHYR 
  sys_set_bits(DCTL_REG, DCTL_RUN_STOP);
#else
  udev->dctl_b.run_stop = 1;
#endif
}

// Disconnect by disabling internal pull-up resistor on D+/D-
void dcd_disconnect(uint8_t rhport)
{
  (void) rhport;
  // [TODO] clear all xfers and eps first
#if CFG_TUSB_OS == OPT_OS_ZEPHYR 
  sys_clear_bits(DCTL_REG, DCTL_RUN_STOP);
#else
  udev->dctl_b.run_stop = 0;
#endif
}

// Enable/Disable Start-of-frame interrupt. Default is disabled
void dcd_sof_enable(uint8_t rhport, bool en)
{
  (void) rhport;
  (void) en;
#if CFG_TUSB_OS == OPT_OS_ZEPHYR 
  LOG_ALIF_INFO("%010u >%s", DWT->CYCCNT, __func__);
#else
  LOG("%010u >%s", DWT->CYCCNT, __func__);
#endif
}

/// Endpoint Management --------------------------------------------------------

// Invoked when a control transfer's status stage is complete.
// May help DCD to prepare for next control transfer, this API is optional.
void dcd_edpt0_status_complete(uint8_t rhport, tusb_control_request_t const * request)
{
    (void) rhport;
    (void) request;

    _ctrl_long_data = false;
    _dcd_start_xfer(TUSB_DIR_OUT, _ctrl_buf, 8, TRBCTL_CTL_SETUP);
}

// Opening an endpoint is done for all non-control endpoints once the host picks
// a configuration that the device should use.
// At this point, the endpoint should be enabled in the peripheral and
// configured to match the endpoint descriptor.
// Pay special attention to the direction of the endpoint you can get from the
// helper methods above. It will likely change what registers you are setting.
// Also make sure to enable endpoint specific interrupts.
bool dcd_edpt_open(uint8_t rhport, tusb_desc_endpoint_t const * desc_ep)
{
    (void) rhport;
    uint32_t cfg0, cfg1;

    uint8_t ep = (tu_edpt_number(desc_ep->bEndpointAddress) << 1) |
                tu_edpt_dir(desc_ep->bEndpointAddress);

    // LOG_ALIF_SHORT("-->> +: %x", desc_ep->bEndpointAddress);

  if (false == _xfer_cfgd) {
    // Endpoint Configuration - Start config phase
    (void) usb_dc_alif_send_ep_cmd(0, CMDTYP_DEPSTARTCFG, 2);
    _xfer_cfgd = true;
  }

    uint8_t fifo_num = TUSB_DIR_IN == tu_edpt_dir(desc_ep->bEndpointAddress) ?
                      tu_edpt_number(desc_ep->bEndpointAddress) : 0;
    
    bool is_high_speed = (XHC_REG_RD(DSTS_REG) & DCFG_SPEED_MASK) == 0x0; // 0x00/0x01/0x04 - High/Full/SuperSpeed

    uint8_t bInterval_m1; // bInterval value minus 1. The valid values for this field are 0x0 through 0xD
    if (is_high_speed) {
      bInterval_m1 = desc_ep->bInterval > 0
                    ? desc_ep->bInterval - 1
                    : 0;
    } else {
      bInterval_m1 = 0;
    }

    cfg1 =  (ep << 25)          // USB Endpoint Number
            | (bInterval_m1 << 16)  // Bit 16-23: (bInterval - 1)
            | (1U << 10)        // Bit 10: XferNotReady Enable (XferNRdyEn)
            | (1U << 8);        // Bit 8: XferComplete Enable (XferCmplEn)
    
    if ( TUSB_XFER_ISOCHRONOUS == desc_ep->bmAttributes.xfer ) {
        // cfg1 |= (1U << 31);     // FIFO-based. Set to 1 if this isochronous endpoint
        _xfer_isochron[ep] = true;
        _iso_mps[ep] = desc_ep->wMaxPacketSize & 0x7FF; // store ISO MPS for later use
        LOG_ALIF_SHORT("-->>+ iso: %x, interval: %u", ep, bInterval_m1);
    }
    
    cfg0 =  (fifo_num << 17)  // FIFO Number (FIFONum) ?? for isochronous ep?
            | ((desc_ep->wMaxPacketSize & 0x7FF) << 3)
            | (desc_ep->bmAttributes.xfer << 1);

    XHC_REG_WR(DEPCMDPAR1N(ep), cfg1);
    XHC_REG_WR(DEPCMDPAR0N(ep), cfg0);
    (void) usb_dc_alif_send_ep_cmd(ep, CMDTYP_DEPCFG, 0);

    XHC_REG_WR(DEPCMDPAR0N(ep), 1);
    (void) usb_dc_alif_send_ep_cmd(ep, CMDTYP_DEPXFERCFG, 0);

    sys_set_bits(DALEPENA_REG, 1U << ep);


  return true;
}

// Close all non-control endpoints, cancel all pending transfers if any.
// Invoked when switching from a non-zero Configuration by SET_CONFIGURE therefore
// required for multiple configuration support.
void dcd_edpt_close_all(uint8_t rhport)
{
    (void) rhport;
    dcd_int_disable(rhport);
    dcd_int_enable(rhport);
}

// Close an endpoint. his function is used for implementing alternate settings.
// After calling this, the device should not respond to any packets directed
// towards this endpoint. When called, this function must abort any transfers in
// progress through this endpoint, before returning.
// Implementation is optional. Must be called from the USB task.
// Interrupts could be disabled or enabled during the call.
/**
 * @brief Close (disable) the specified endpoint and abort any in-flight transfers.
 *
 * After this call, the USB controller will no longer accept or send packets
 * on this endpoint. Any pending TRB is canceled, and driver state is reset
 * so no further callbacks occur.
 *
 * Based on Alif E7 USB controller behavior (DALEPENA bit disables the endpoint,
 * DEPENDXFER command aborts an active TRB).
 *
 * @param rhport   Root hub port (unused for single-port controllers)
 * @param ep_addr  TinyUSB endpoint address (logical endpoint + direction)
 */
void dcd_edpt_close(uint8_t rhport, uint8_t ep_addr) {
  (void) rhport;

  LOG_ALIF_SHORT("-->> -: %x", ep_addr);

  // Convert TinyUSB endpoint address to physical index:
  // physical_index = (endpoint_number << 1) | direction
  uint8_t ep_index = (tu_edpt_number(ep_addr) << 1) | tu_edpt_dir(ep_addr);

  // 1) Disable endpoint so hardware ignores further packets
  sys_clear_bits(DALEPENA_REG, (1U << ep_index));

  // 2) If a TRB is still owned by hardware (HWO bit set), abort it:
  if (_xfer_trb[ep_index][3] & (1U << 0)) {
    // Temporarily allow CMDTYP_DEPENDXFER to take effect
    sys_set_bits(GUCTL2_REG, GUCTL2_RST_ACTBITLATER);
    (void) usb_dc_alif_send_ep_cmd(ep_index, CMDTYP_DEPENDXFER, 0);
    sys_clear_bits(GUCTL2_REG, GUCTL2_RST_ACTBITLATER);
  }

  // 3) Clear TRB descriptor and reset bookkeeping
  _xfer_trb[ep_index][3] = 0;// clear control bits (incl. HWO/IOC)
  _xfer_bytes[ep_index] = 0; // no bytes pending

//   LOG_ALIF_INFO("Endpoint %u %s closed",
//                 (unsigned) (ep_index >> 1),
//                 (ep_index & 1) ? "IN" : "OUT");
}

// Submit a transfer, When complete dcd_event_xfer_complete() is invoked to
// notify the stack
bool dcd_edpt_xfer(uint8_t rhport, uint8_t ep_addr, uint8_t * buffer, uint16_t total_bytes)
{
  (void) rhport;

  uint8_t ep = (tu_edpt_number(ep_addr) << 1) | tu_edpt_dir(ep_addr);

  // ISOCHRONOUS
  if (_xfer_isochron[ep]) {
    LOG_ALIF_SHORT("-->>xf iso: %x, len=%u", ep_addr, total_bytes);
    // _dcd_start_xfer_iso(ep, buffer, total_bytes, 0);
    _dcd_start_xfer_iso(ep, iso_test_data, sizeof(iso_test_data), 0);

    buff_test = buffer;
    buff_test_size = total_bytes;

    return true;
  }    

   LOG_ALIF_SHORT("-->>xf: %x, len=%u", ep_addr, total_bytes);
  switch (ep) {
    case 0: {// CONTROL OUT
      if (total_bytes > 0) {
        // DATA OUT stage
        _xfer_bytes[ep] = total_bytes;
        // TRBCTL_CTL_STAT3: DATA stage for control OUT
        _dcd_start_xfer(ep, buffer, total_bytes, TRBCTL_CTL_STAT3);
      } else {
        // STATUS OUT stage: after DATA IN
        if (++_sts_stage == 2) {
          _sts_stage = 0;
          dcd_event_xfer_complete(rhport,
                                  tu_edpt_addr(0, TUSB_DIR_OUT),
                                  0,
                                  XFER_RESULT_SUCCESS,
                                  true);
        }
      }
    } break;

    case 1: {// CONTROL IN
      _xfer_bytes[ep] = total_bytes;

      if (total_bytes > 0) {
        // DATA IN stage
        sys_cache_data_flush_range(buffer, total_bytes);// zephyr equ for RTSS_CleanDCache_by_Addr(..)
        // choose TRB type based on long-data flag
        uint8_t trb_type = _ctrl_long_data ? TRBCTL_NORMAL : TRBCTL_CTL_DATA;
        if (total_bytes == 64) {
          // if exactly max EP0 size, mark long-data for next chunk
          _ctrl_long_data = true;
        }
        _dcd_start_xfer(ep, buffer, total_bytes, trb_type);
      } else {
        // STATUS IN stage
        if (++_sts_stage == 2) {
          _sts_stage = 0;
          dcd_event_xfer_complete(rhport,
                                  tu_edpt_addr(0, TUSB_DIR_IN),
                                  0,
                                  XFER_RESULT_SUCCESS,
                                  true);
        }
      }
    } break;

    default: {
        uint8_t type;

        // BULK & INTERRUPT endpoints
        _xfer_bytes[ep] = total_bytes;
        if (tu_edpt_dir(ep_addr) == TUSB_DIR_IN) {
            // cache clean before IN transfer
            sys_cache_data_flush_range(buffer, total_bytes);// zephyr equ for RTSS_CleanDCache_by_Addr(..)
        } else {
            // for OUT endpoints controller may require full-size requests
            // you can adjust this if needed or remove hack
            // total_bytes = MIN(total_bytes, /* your max packet size */ 512);
            total_bytes = ALIF_MAX_PCK_SIZE;
        }

        // start transfer: NORMAL for data, NORMAL_ZLP for zero-length
        type = (total_bytes > 0) ? TRBCTL_NORMAL : TRBCTL_NORMAL_ZLP;
        (void) _dcd_start_xfer(ep, buffer, total_bytes, type);
    }
  }

  return true;
}

// Submit a transfer using fifo, When complete dcd_event_xfer_complete() is invoked to notify the stack
// This API is optional, may be useful for register-based for transferring data.
TU_ATTR_WEAK bool dcd_edpt_xfer_fifo(uint8_t rhport, uint8_t ep_addr, tu_fifo_t * ff, uint16_t total_bytes) {
    (void) rhport;
    (void) ep_addr;
    (void) ff;
    (void) total_bytes;
    return true;
}

// Stall endpoint, any queuing transfer should be removed from endpoint
void dcd_edpt_stall(uint8_t rhport, uint8_t ep_addr)
{
  (void) rhport;

  uint8_t ep = (tu_edpt_number(ep_addr) << 1) | tu_edpt_dir(ep_addr);

  (void) usb_dc_alif_send_ep_cmd(ep, CMDTYP_DEPSSTALL, 0);

  if (0 == tu_edpt_number(ep_addr)) {
    _ctrl_long_data = false;
    _dcd_start_xfer(TUSB_DIR_OUT, _ctrl_buf, 8, TRBCTL_CTL_SETUP);
  }

}

// clear stall, data toggle is also reset to DATA0
// This API never calls with control endpoints, since it is auto cleared when
// receiving setup packet
void dcd_edpt_clear_stall(uint8_t rhport, uint8_t ep_addr)
{
  (void) rhport;
  uint8_t ep = (tu_edpt_number(ep_addr) << 1) | tu_edpt_dir(ep_addr);

  (void) usb_dc_alif_send_ep_cmd(ep, CMDTYP_DEPCSTALL, 0);
}

void dcd_uninit(void)
{

  usb_ctrl2_phy_power_on_reset_set();// set usb phy power-on-reset signal

  enable_usb_phy_isolation(); // enable usb phy isolation
  disable_usb_phy_power(); // power down usb phy
  disable_usb_periph_clk(); // disable usb peripheral clock
}

static uint8_t usb_dc_alif_send_ep_cmd(uint8_t ep, uint8_t typ, uint16_t param) 
{
  // Store PHY configuration
  uint32_t phycfg = XHC_REG_RD(GUSB2PHYCFG0_REG);
  sys_clear_bits(GUSB2PHYCFG0_REG, GUSB2PHYCFG0_ENBLSLPM | GUSB2PHYCFG0_SUSPENDUSB20);

  sys_clear_bits(DEPCMDN(ep), DEPCMD_CMDTYP_MASK);
  sys_set_bits(DEPCMDN(ep), DEPCMD_CMDTYP(typ));

  sys_clear_bits(DEPCMDN(ep), DEPCMD_CMDIOC);

  sys_clear_bits(DEPCMDN(ep), DEPCMD_PARAM_MASK);
  sys_set_bits(DEPCMDN(ep), DEPCMD_PARAM(param));

  // dispatch command and wait for completion
  sys_set_bits(DEPCMDN(ep), DEPCMD_CMDACT);
  while (*((volatile uint32_t *) DEPCMDN(ep)) & DEPCMD_CMDACT) {
    __NOP();
  }

  // Restore PHY configuration
  XHC_REG_WR(GUSB2PHYCFG0_REG, phycfg);
  return (XHC_REG_RD(DEPCMDN(ep)) >> 12) & 0x0F;// Get command status;
}


/**
 * \brief Handle a DEPEVT (Device Endpoint Event) from the USB event ring.
 *
 * \param ep        Physical endpoint index (0 = EP0 OUT, 1 = EP0 IN, 2+ = other endpoints)
 * \param evt       DEPEVT event type (XFERCOMPLETE, XFERINPROGRESS, XFERNOTREADY, etc.)
 * \param sts       DEPEVT status field (used for “Not Ready” codes or other flags)
 * \param par       DEPEVT parameter field (typically unused for IN/OUT transfers)
 */
static void _dcd_handle_depevt(uint8_t rhport, const evt_t *ep_evt)
{
    if (ep_evt->depevt.is_devt) return; // Not a DEPEVT, skip

  const uint8_t ep = ep_evt->depevt.ep;
  const uint8_t evt = ep_evt->depevt.evt;
  const uint8_t sts = ep_evt->depevt.sts;
  const uint16_t par = ep_evt->depevt.par;

  // LOG_ALIF_INFO("DEPEVT: ep=%u, evt=%u, sts=%02x, par=%04x",
  //               (unsigned) ep, (unsigned) evt, (unsigned) sts, depevt.par);

  // Validate endpoint index

  if (!(ep < TUP_DCD_ENDPOINT_MAX)) {
    TU_MESS_FAILED();
    TU_BREAKPOINT();
    return;
  }

  switch (evt) {
    case DEPEVT_XFERCOMPLETE: {
      // Invalidate the TRB entry so we can safely read updated fields
      sys_cache_data_invd_range(_xfer_trb[ep], sizeof(_xfer_trb[0]));
      // Transfer completed: extract TRBCTL type from the TRB
      uint8_t trbctl = (_xfer_trb[ep][3] >> 4) & 0x3F;

      if (ep == 0) {
        // EP0 OUT (SETUP or STATUS OUT)
        if (trbctl == TRBCTL_CTL_SETUP) {
          // SETUP stage finished: invalidate control buffer
          sys_cache_data_invd_range(_ctrl_buf, sizeof(_ctrl_buf));
          dcd_event_setup_received(rhport, _ctrl_buf, true);
          break;
        }

        if (trbctl == TRBCTL_CTL_STAT3) {
          if (0 < _xfer_bytes[0]) {
            sys_cache_data_invd_range((void *) _xfer_trb[0][0], _xfer_bytes[0]);
            dcd_event_xfer_complete(rhport, tu_edpt_addr(0, TUSB_DIR_OUT),
                                    _get_transfered_bytes(0),
                                    XFER_RESULT_SUCCESS, true);
          } else {
            if (2 == ++_sts_stage) {
              _sts_stage = 0;
              dcd_event_xfer_complete(rhport, tu_edpt_addr(0, TUSB_DIR_OUT),
                                      0, XFER_RESULT_SUCCESS, true);

              // *(volatile uint32_t*) 0x4900C000 ^= 8; // [TEMP]
            }
          }
        } else {
          // invalid TRBCTL value
          __BKPT(0);
        }
      } else if (ep == 1) {
        if (trbctl != TRBCTL_CTL_STAT2) {// STATUS IN notification is done at xfer request
          dcd_event_xfer_complete(rhport, tu_edpt_addr(0, TUSB_DIR_IN),
                                  _get_transfered_bytes(1),
                                  XFER_RESULT_SUCCESS, true);

        } else {
          if (2 == ++_sts_stage) {
            _sts_stage = 0;
            dcd_event_xfer_complete(rhport, tu_edpt_addr(0, TUSB_DIR_IN),
                                    0, XFER_RESULT_SUCCESS, true);

            // *(volatile uint32_t*) 0x4900C000 ^= 8; // [TEMP]
          }
        }
      } else {
        // [TODO] check if ep is open
        if (TUSB_DIR_OUT == tu_edpt_dir(tu_edpt_addr(ep >> 1, ep & 1))) {
          sys_cache_data_invd_range((void *) _xfer_trb[ep][0],
                                    ALIF_MAX_PCK_SIZE - _xfer_trb[ep][2]);
          //   _xfer_bytes[ep] - _xfer_trb[ep][2]);
          dcd_event_xfer_complete(rhport, tu_edpt_addr(ep >> 1, ep & 1),
                                  ALIF_MAX_PCK_SIZE - _xfer_trb[ep][2],
                                  XFER_RESULT_SUCCESS, true);
        } else {
          dcd_event_xfer_complete(rhport, tu_edpt_addr(ep >> 1, ep & 1),
                                  _get_transfered_bytes(ep),
                                  XFER_RESULT_SUCCESS, true);
        }
      }
    } break;

    case DEPEVT_XFERINPROGRESS: {
      LOG_ALIF_INFO("Transfer in progress");
    } break;

    case DEPEVT_XFERNOTREADY: {
      // Transfer-not-ready indicates endpoint needs re-arming

        if (_xfer_isochron[ep]) {
            // ISOCHRONOUS endpoint
            bool active = sts & DEPEVT_STS_XFER_ACTIVE;
            if(!active)
            {
                // Host initiated a transfer, but the requested transfer is not present in the hardware.
                LOG_ALIF_SHORT("X par: %x, sts=%x", par, sts);
                // _dcd_start_xfer_iso(ep, buff_test, buff_test_size, par);
                _dcd_start_xfer_iso(ep,
                          iso_test_data,
                          sizeof(iso_test_data),
                          par /*par=StartFrame*/);
            }
            else {
               // Host initiated a transfer, the transfer is present, but no valid TRBs are available.
                usb_dc_alif_send_ep_cmd(ep, CMDTYP_DEPSTRTXFER, par);
            }
            break;
        }

        // LOG_ALIF_SHORT("-->>nr: %x, sts=%x", ep, sts);
        // EP0 OUT and IN are special cases
        if ((0 == ep) && (DEPEVT_STS_NOTREADY_CTRL_STAT_RQ == (sts & DEPEVT_STS_NOTREADY_MASK))) {
            _xfer_bytes[ep] = 0;
            _dcd_start_xfer(ep, _ctrl_buf, 64, TRBCTL_CTL_STAT3);
            break;
        }

      if ((1 == ep) && (DEPEVT_STS_NOTREADY_CTRL_STAT_RQ == (sts & DEPEVT_STS_NOTREADY_MASK))) {
        _dcd_start_xfer(ep, NULL, 0, TRBCTL_CTL_STAT2);
        break;
      }

      if ((0 == ep) && (sts & DEPEVT_STS_XFER_ACTIVE)) {
        if (_xfer_trb[ep][3] & (1 << 0)) {// transfer was configured
          // dependxfer can only block when actbitlater is set
          sys_set_bits(GUCTL2_REG, GUCTL2_RST_ACTBITLATER);
          (void) usb_dc_alif_send_ep_cmd(ep, CMDTYP_DEPENDXFER, 0);
          sys_clear_bits(GUCTL2_REG, GUCTL2_RST_ACTBITLATER);

          // reset the trb byte count and clean the cache
          sys_cache_data_invd_range(_xfer_trb[ep], sizeof(_xfer_trb[0]));
          _xfer_trb[ep][2] = _xfer_bytes[ep];
          sys_cache_data_flush_range(_xfer_trb[ep], sizeof(_xfer_trb[0]));// zephyr equ for RTSS_CleanDCache_by_Addr(..)

          // prepare ep command
          XHC_REG_WR(DEPCMDPAR1N(ep), (uint32_t) local_to_global(_xfer_trb[ep]));
          XHC_REG_WR(DEPCMDPAR0N(ep), 0);
          (void) usb_dc_alif_send_ep_cmd(ep, CMDTYP_DEPSTRTXFER, 0);
        }
      }
      
    } break;

    case DEPEVT_EPCMDCMPLT: {
      // redundant, currently no commands are issued with IOC bit set
    } break;
  }

}

static void _dcd_handle_devt(uint8_t rhport, uint8_t evt, uint16_t info)
{
  switch (evt) {
    case DEVT_USBRST: {
      _xfer_cfgd = false;

      // [TODO] issue depcstall for any ep in stall mode
      sys_clear_bits(DCFG_REG, DCFG_DEVADDR_MASK);// reset address

      // ToDo : FULL/HIGH Speed
      sys_set_bits(DCFG_REG, DCFG_SPEED(ALIF_DEVSPD_SETTING));// 0x1: Full-speed (USB 2.0 PHY clock is 30 MHz or 60 MHz)
      dcd_event_bus_reset(rhport, ALIF_TUSB_SPEED, true);     // [TODO] actual speed
    } break;
    case DEVT_CONNECTDONE: {
      // read conn speed from dsts
      // program ramclksel in gctl if needed
      //  LOG_ALIF_INFO("Connect done");

      uint8_t ep_idx = 0;
      XHC_REG_WR(DEPCMDPAR1N(ep_idx), (0 << 25) | (1 << 10) | (1 << 8));
      XHC_REG_WR(DEPCMDPAR0N(ep_idx), (2 << 30) | (0 << 22) | (0 << 17) | (64 << 3) | (0 << 1));
      (void) usb_dc_alif_send_ep_cmd(ep_idx, CMDTYP_DEPCFG, 0);

      ep_idx = 1;
      XHC_REG_WR(DEPCMDPAR1N(ep_idx), (1 << 25) | (1 << 10) | (1 << 8));
      XHC_REG_WR(DEPCMDPAR0N(ep_idx), (2 << 30) | (0 << 22) | (0 << 17) | (64 << 3) | (0 << 1));
      (void) usb_dc_alif_send_ep_cmd(ep_idx, CMDTYP_DEPCFG, 0);
    } break;
    case DEVT_ULSTCHNG: {
      // LOG_ALIF_INFO("Link status change");
      switch (info) {
        case 0x3: {// suspend (L2)
          dcd_event_bus_signal(rhport, DCD_EVENT_SUSPEND, true);
        } break;
        case 0x4: {// disconnected
          dcd_event_bus_signal(rhport, DCD_EVENT_UNPLUGGED, true);
        } break;
        case 0xF: {// resume
          dcd_event_bus_signal(rhport, DCD_EVENT_RESUME, true);
        } break;
        default: {
        }
      }
      // 0x0: ON state
      // 0x2: L1 state (sleep)
      // 0x3: L2 state (suspend)
      // 0x4: disconnected state
      // 0x5: early suspend
      // 0xE: reset
      // 0xF: resume
    } break;
    case DEVT_ERRTICERR: {
      __BKPT(0);
    } break;
    default: {
      LOG_ALIF_INFO("Unknown DEVT event");
    }
  }
}

/**
 * \brief Program a Transfer Request Block (TRB) and kick off a transfer.
 * \param ep    Physical endpoint index (0–31)
 * \param buf   Pointer to the data buffer (local address)
 * \param size  Number of bytes to transfer
 * \param type  TRBCTL_* type (e.g. TRBCTL_NORMAL, TRBCTL_CTL_DATA, etc.)
 * \return      Status returned by _dcd_cmd_wait (0 = success)
 */
static uint8_t _dcd_start_xfer(uint8_t ep, void* buf, uint32_t size, uint8_t type)
{
  /* Prevent races between programming TRB and ISR handling */
  NVIC_DisableIRQ(USB_ALIF_IRQ);

  /* Populate the TRB fields */
  _xfer_trb[ep][0] = buf ? (uint32_t) local_to_global(buf) : 0U; // Buffer Pointer Low (BPTRL)
  _xfer_trb[ep][1] = 0U;                                         // Buffer Pointer High (BPTRH)
  _xfer_trb[ep][2] = size;                                       // Buffer Size (BUFSIZ)
  _xfer_trb[ep][3] = (1U << 11)              // Interrupt on Complete
                    | (1U << 10)            // Interrupt on Short Packet / Interrupt on MissedIsoc (ISP/IMI)
                    | ((uint32_t) type << 4)// Indicates the type of TRB
                    | (1U << 1)             // Last TRB (LST)
                    | (1U << 0);            // Hardware Owner of Descriptor (HWO)

  /* Clean D-cache so USB controller sees updated TRB */
  sys_cache_data_flush_range(_xfer_trb[ep], sizeof(_xfer_trb[0]));// zephyr equ for RTSS_CleanDCache_by_Addr(..)

  // EP command
  XHC_REG_WR(DEPCMDPAR1N(ep), (uint32_t) local_to_global(_xfer_trb[ep]));
  XHC_REG_WR(DEPCMDPAR0N(ep), 0);

  /* Re-enable USB interrupt before issuing command */
  NVIC_EnableIRQ(USB_ALIF_IRQ);

  return usb_dc_alif_send_ep_cmd(ep, CMDTYP_DEPSTRTXFER, 0);
}

static uint8_t _dcd_start_xfer_iso(uint8_t ep, void* buf, uint32_t size, uint16_t start_frame)
{
    uint32_t mps = _iso_mps[ep];
    uint8_t num_trb = (size + mps - 1) / mps;
    if (num_trb > ISO_CHAIN_MAX) num_trb = ISO_CHAIN_MAX;

    _xfer_bytes[ep] = size;

    // 2) Сброс D-cache для всего буфера данных
    sys_cache_data_flush_range(buf, size);

    // 3) Заполняем цепочку TRB
    for (uint8_t i = 0; i < num_trb; i++) {
        uint32_t ptr = (uint32_t)local_to_global((uint8_t*)buf + i * mps);
        uint32_t len = MIN(mps, size - i * mps);
        uint32_t type = (i == 0) ? TRBCTL_ISO_FIRST : TRBCTL_ISO;
        uint32_t chain = (i < num_trb - 1) ? (1U << 2) : 0U; // CHN
        uint32_t last  = (i == num_trb - 1) ? (1U << 1) : 0U; // LST

        _xfer_trb_iso[i][0] = ptr;      // Buffer Pointer Low
        _xfer_trb_iso[i][1] = 0;        // Buffer Pointer High
        _xfer_trb_iso[i][2] = len;      // Transfer Length
        _xfer_trb_iso[i][3] =
             (1U << 11)                 // IOC: interrupt on complete
           | (1U << 10)                 // IMI: interrupt on missed isoc
           | (type   <<  4)             // TRB type (ISO_FIRST / ISO)
           | (1U     <<  3)             // CSP: continue on short pkt
           | chain                      // CHN: chain bit
           | last                       // LST: last TRB
           | (1U     <<  0);            // HWO: hardware owner
    }

    // 4) Сброс D-cache для всей цепочки TRB сразу
    sys_cache_data_flush_range(_xfer_trb_iso,
                              num_trb * sizeof(_xfer_trb_iso[0]));

    // 5) Записываем адрес TRB-ring в контроллер
    NVIC_DisableIRQ(USB_ALIF_IRQ);
    XHC_REG_WR(DEPCMDPAR1N(ep),
               (uint32_t)local_to_global(_xfer_trb_iso[0]));
    XHC_REG_WR(DEPCMDPAR0N(ep), 0);
    NVIC_EnableIRQ(USB_ALIF_IRQ);

    // 6) Запускаем передачу на заданный microframe
    return usb_dc_alif_send_ep_cmd(ep,
                                   CMDTYP_DEPSTRTXFER,
                                   start_frame);    
}


#endif
