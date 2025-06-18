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

  #include "platform_def.h"
  #include "dcd_ensemble.h"

  #pragma message("Building for Zephyr RTOS")
  #include <zephyr/kernel.h>
  #include <zephyr/cache.h>
  #include <zephyr/devicetree.h>
  
  #define LOG_PRINT_INFO(fmt, ...)  printf("[%lld ms] INFO  %s: " fmt "\n", k_uptime_get(), __func__, ##__VA_ARGS__)
  #define LOG_PRINT_ERROR(fmt, ...) printf("[%lld ms] ERROR %s: " fmt "\n", k_uptime_get(), __func__, ##__VA_ARGS__)
  #define LOG_PRINT_SHORT(fmt, ...) printf( fmt "\n", ##__VA_ARGS__)

  #define LOG_NO_INFO(fmt, ...)  ((void)0)
  #define LOG_NO_ERROR(fmt, ...) ((void)0)
  #define LOG_NO_SHORT(fmt, ...) ((void)0)
  
  #define LOG_ALIF_INFO  LOG_NO_INFO
  #define LOG_ALIF_ERROR LOG_PRINT_ERROR
  #define LOG_ALIF_SHORT LOG_NO_SHORT
      
  #define USB_NODE        DT_NODELABEL(usb0)
  #define USB_CTRL_BASE   DT_REG_ADDR(USB_NODE)
  #define USB_ALIF_IRQ    DT_IRQN(USB_NODE)

  #define DEPEVT_STS_NOTREADY_MASK   0x0B
  #define DEPEVT_STS_NOTREADY_CODE   0x02

#if defined(CONFIG_USB_DEVICE_HIGH_SPEED)
  #define ALIF_DEVSPD_SETTING   0x0  /* High-speed */
  #define ALIF_TUSB_SPEED       TUSB_SPEED_HIGH
  #define ALIF_MAX_PCK_SIZE     512
#elif defined(CONFIG_USB_DEVICE_FULL_SPEED)
  #define ALIF_DEVSPD_SETTING   0x1  /* Full-speed */
  #define ALIF_TUSB_SPEED       TUSB_SPEED_FULL
  #define ALIF_MAX_PCK_SIZE     64
#else
  #error "You must select CONFIG_USB_DEVICE_HIGH_SPEED or _FULL_SPEED"
#endif


  // Direct register access for compatibility
  #define XHC_REG_RD(addr)      sys_read32((addr))
  #define XHC_REG_WR(addr, val) sys_write32((val), (addr))

  // prototype for the functions
  static uint8_t usb_dc_alif_send_ep_cmd(uint8_t ep, uint8_t cmd_type, uint16_t param);
 
 #include "device/dcd.h"
 
#define MAX_TRB_NUM      TUP_DCD_ENDPOINT_MAX
 
// Structs and Buffers --------------------------------------------------------
#define EVT_BUF_SIZE  1024
#define CTRL_BUF_SIZE 64

__aligned(4096) CFG_TUSB_MEM_SECTION \
static uint32_t _evnt_buf[EVT_BUF_SIZE];

__aligned(32) CFG_TUSB_MEM_SECTION \
static uint8_t _ctrl_buf[CTRL_BUF_SIZE];    // [TODO] runtime alloc

__aligned(32) CFG_TUSB_MEM_SECTION \
static uint32_t _xfer_trb[MAX_TRB_NUM][4];  // [TODO] runtime alloc

static uint16_t _xfer_bytes[MAX_TRB_NUM];

 static volatile uint32_t* _evnt_tail;
 static bool     _ctrl_long_data = false;
 static bool     _xfer_cfgd = false;
 static uint32_t _sts_stage = 0;
 
 /// Private Functions ----------------------------------------------------------
 
 static uint8_t _dcd_start_xfer(uint8_t ep, void* buf, uint32_t size, uint8_t type);
 static void _dcd_handle_depevt(uint8_t rhport, uint8_t ep_index, uint8_t evt, uint8_t sts, uint16_t par);
 static void _dcd_handle_devt(uint8_t rhport, uint8_t evt, uint16_t info);
  
 /// API Extension --------------------------------------------------------------
 
 void dcd_uninit(void);
 
 // helpers
 static inline uint32_t _get_transfered_bytes(uint8_t ep) {
    return _xfer_bytes[ep] - (_xfer_trb[ep][2] & 0x00FFFFFF);
}

static inline void enable_usb_periph_clk(void)
{
	sys_set_bits(EXPMST_PERIPH_CLK_EN, PERIPH_CLK_ENA_USB_CKEN);
}

static inline void disable_usb_periph_clk(void)
{
	sys_clear_bits(EXPMST_PERIPH_CLK_EN, PERIPH_CLK_ENA_USB_CKEN);
}

static inline void enable_cgu_clk20m(void)
{
	sys_set_bits(CGU_CLK_ENA, CLK_ENA_CLK20M);
}

static inline void disable_cgu_clk20m(void)
{
	sys_clear_bits(CGU_CLK_ENA, CLK_ENA_CLK20M);
}

static inline void enable_usb_phy_power(void)
{
	sys_clear_bits(VBAT_PWR_CTRL, PWR_CTRL_UPHY_PWR_MASK);
}

static inline void disable_usb_phy_power(void)
{
	sys_set_bits(VBAT_PWR_CTRL, PWR_CTRL_UPHY_PWR_MASK);
}

static inline void enable_usb_phy_isolation(void)
{
	sys_set_bits(VBAT_PWR_CTRL, PWR_CTRL_UPHY_ISO);
}

static inline void disable_usb_phy_isolation(void)
{
	sys_clear_bits(VBAT_PWR_CTRL, PWR_CTRL_UPHY_ISO);
}

static inline void usb_ctrl2_phy_power_on_reset_set()
{
	sys_set_bits(EXPMST_USB_CTRL2, USB_CTRL2_POR_RST_MASK);
}

static inline void usb_ctrl2_phy_power_on_reset_clear()
{
	sys_clear_bits(EXPMST_USB_CTRL2, USB_CTRL2_POR_RST_MASK);
}
 
 /// Device Setup ---------------------------------------------------------------
 
 // Initializes the USB peripheral for device mode and enables it.
 // This function should enable internal D+/D- pull-up for enumeration.
bool dcd_init(uint8_t rhport, const tusb_rhport_init_t* rh_init) 
{
    (void) rh_init;

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
    dcd_disconnect(rhport);     // ToDO: check if this is needed

	// Device soft reset
	sys_set_bits(DCTL_REG, DCTL_CSFTRST);
    while ((*(volatile uint32_t *)DCTL_REG & DCTL_CSFTRST) != 0) {
        k_busy_wait(1 *1000);  // 1000 μs = 1 ms
    }

	// Core + PHY soft reset
    k_busy_wait(50 * 1000); // 50 ms
	sys_set_bits(GCTL_REG, GCTL_CORESOFTRESET);
	sys_set_bits(GUSB2PHYCFG0_REG, GUSB2PHYCFG0_PHYSOFTRST);
	k_busy_wait(50 * 1000); // 50 ms
	sys_clear_bits(GUSB2PHYCFG0_REG, GUSB2PHYCFG0_PHYSOFTRST);
    k_busy_wait(50 * 1000); // 50 ms
	sys_clear_bits(GCTL_REG, GCTL_CORESOFTRESET);
    k_busy_wait(50 * 1000); // 50 ms

	// Check controller ID
	uint32_t rd_val = XHC_REG_RD(GSNPSID_REG);
	if ((rd_val & 0xFFFF0000) != 0x55330000) {
		LOG_ALIF_ERROR("Invalid USB controller ID! Expected 0x5533xxxx, got 0x%08x", rd_val);
		return false;
	}
    LOG_ALIF_INFO("Controller ID: 0x%08x", rd_val);

	// Global bus config
	XHC_REG_WR(GSBUSCFG0_REG, GSBUSCFG0_INCRBRSTENA | GSBUSCFG0_INCR16BRSTENA);

	// Configure USB2 PHY
	rd_val = XHC_REG_RD(GUSB2PHYCFG0_REG);
	rd_val &= ~(GUSB2PHYCFG0_PHYIF | GUSB2PHYCFG0_ULPI_UTMI_SEL | GUSB2PHYCFG0_USBTRDTIM_MASK);
	rd_val |= GUSB2PHYCFG0_PHYIF | GUSB2PHYCFG0_USBTRDTIM(5); // UTMI+, 8-bit, HS
	XHC_REG_WR(GUSB2PHYCFG0_REG, rd_val);

	// Set Device Speed (USBHS only)
	sys_clear_bits(DCFG_REG, DCFG_SPEED_MASK); // ToDo: FULL/HIGH Speed
    sys_set_bits(DCFG_REG, DCFG_SPEED(ALIF_DEVSPD_SETTING));  // 0x1: Full-speed 12MBit/s

    // allocate ring buffer for events
    memset(_evnt_buf, 0, sizeof(_evnt_buf));
    sys_cache_data_flush_range(_evnt_buf, sizeof(_evnt_buf)); // zephyr equ for RTSS_CleanDCache_by_Addr(..)
    _evnt_tail = _evnt_buf;

	XHC_REG_WR(GEVNTADDRL0_REG, (uint32_t)local_to_global(_evnt_buf));
    
    LOG_ALIF_INFO("Address loc->glob: %p -> %x", _evnt_buf, XHC_REG_RD(GEVNTADDRL0_REG));

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
	(void)usb_dc_alif_send_ep_cmd(0, DEPCMD_DEPSTARTCFG, 0);

	// Endpoint Configuration - EP0 OUT (ep = 0)
	uint8_t ep_idx = 0;
	XHC_REG_WR(DEPCMDPAR1N(ep_idx), (0 << 25) | (1 << 10) | (1 << 8));
	XHC_REG_WR(DEPCMDPAR0N(ep_idx), (0 << 22) | (0 << 17) | (ALIF_MAX_PCK_SIZE << 3) | (0 << 1));
	(void)usb_dc_alif_send_ep_cmd(ep_idx, DEPCMD_DEPCFG, 0);

	// Endpoint Configuration - EP0 IN (ep = 1)
	ep_idx = 1;
	XHC_REG_WR(DEPCMDPAR1N(ep_idx), (1 << 25) | (1 << 10) | (1 << 8));
	XHC_REG_WR(DEPCMDPAR0N(ep_idx), (0 << 22) | (0 << 17) | (ALIF_MAX_PCK_SIZE << 3) | (0 << 1));
	(void)usb_dc_alif_send_ep_cmd(ep_idx, DEPCMD_DEPCFG, 0);

	// set initial xfer configuration for CONTROL eps
    XHC_REG_WR(DEPCMDPAR0N(0), 1);
	(void)usb_dc_alif_send_ep_cmd(0, DEPCMD_DEPXFERCFG, 0);

    XHC_REG_WR(DEPCMDPAR0N(1), 1);
	(void)usb_dc_alif_send_ep_cmd(1, DEPCMD_DEPXFERCFG, 0);

    // prepare trb for the first setup packet
    uint8_t ep = 0;
    memset(_ctrl_buf, 0, sizeof(_ctrl_buf));
    _xfer_trb[ep][0] = (uint32_t) local_to_global(_ctrl_buf);
    _xfer_trb[ep][1] = 0;
    _xfer_trb[ep][2] = 8;
    _xfer_trb[ep][3] = (1 << 11) | (1 << 10) | (TRBCTL_CTL_SETUP << 4) | (1 << 1) | (1 << 0);
    sys_cache_data_flush_range(_xfer_trb[ep], sizeof(_xfer_trb[ep]));

    // send trb to the usb dma
    XHC_REG_WR(DEPCMDPAR1N(ep), (uint32_t)local_to_global(_xfer_trb[ep]));
	XHC_REG_WR(DEPCMDPAR0N(ep), 0);
	(void)usb_dc_alif_send_ep_cmd(ep, DEPCMD_DEPSTRTXFER, 0);

    sys_set_bits(DALEPENA_REG, (1 << ep)); // enable ep0 OUT
    sys_set_bits(DALEPENA_REG, (1 << (ep + 1))); // enable ep0 IN

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
    while (XHC_REG_RD(GEVNTCOUNT0_REG) & GEVNTCOUNT0_EVNTCOUNT_MASK)  {
         sys_cache_data_invd_range(_evnt_buf, sizeof(_evnt_buf)); 
         volatile evt_t e = {.val = *_evnt_tail++};
        //  LOG_ALIF_INFO("%010u IRQ loop, evnt %08x", DWT->CYCCNT, e.val);

          // wrap around
         if (_evnt_tail >= (_evnt_buf + 1024)) _evnt_tail = _evnt_buf;
 
         // dispatch the right handler for the event type
         if (e.depevt.is_devt == 0) {// DEPEVT
             _dcd_handle_depevt(rhport, e.depevt.ep, e.depevt.evt, e.depevt.sts, e.depevt.par);
         } else { // DEVT
             _dcd_handle_devt(rhport, e.devt.evt, e.devt.info);
         } 
 
         // consume one event
         XHC_REG_WR(GEVNTCOUNT0_REG, 4);
    }
 }
  
 // Enables the USB device interrupt.
 // May be used to prevent concurrency issues when mutating data structures
 // shared between main code and the interrupt handler.
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
    NVIC_DisableIRQ(USB_ALIF_IRQ);
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
    LOG_ALIF_INFO("%010u >%s", DWT->CYCCNT, __func__);
 }
 
 // Connect by enabling internal pull-up resistor on D+/D-
 void dcd_connect(uint8_t rhport)
 {
     // [TODO] clear all xfers and eps first
    sys_set_bits(DCTL_REG, DCTL_RUN_STOP);
     (void) rhport;
 }
 
 // Disconnect by disabling internal pull-up resistor on D+/D-
 void dcd_disconnect(uint8_t rhport)
 {
     // [TODO] clear all xfers and eps first
    sys_clear_bits(DCTL_REG, DCTL_RUN_STOP);

     (void) rhport;
 }
 
 // Enable/Disable Start-of-frame interrupt. Default is disabled
 void dcd_sof_enable(uint8_t rhport, bool en)
 {
    LOG_ALIF_INFO("%010u >%s", DWT->CYCCNT, __func__);
 }
  
 /// Endpoint Management --------------------------------------------------------
 
 // Invoked when a control transfer's status stage is complete.
 // May help DCD to prepare for next control transfer, this API is optional.
 void dcd_edpt0_status_complete(uint8_t rhport, tusb_control_request_t const * request)
 {
    // LOG_ALIF_INFO("%010u >%s", DWT->CYCCNT, __func__);
 
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
    // LOG_ALIF_INFO(" ep %02x, max packet size %d, xfer type %d",
    //        desc_ep->bEndpointAddress, desc_ep->wMaxPacketSize, desc_ep->bmAttributes.xfer);

     if (TUSB_XFER_ISOCHRONOUS == desc_ep->bmAttributes.xfer)
         return false;
 
     uint8_t ep_index = (tu_edpt_number(desc_ep->bEndpointAddress) << 1) |
                  tu_edpt_dir(desc_ep->bEndpointAddress);
 
     // [TODO] verify that the num doesn't exceed hw max
 
     if (false == _xfer_cfgd) {
        // Endpoint Configuration - Start config phase
         (void)usb_dc_alif_send_ep_cmd(0, DEPCMD_DEPSTARTCFG, 2);     // check here!! - usb_dc_alif_send_ep_cmd(ep, DEPCMD_DEPSTARTCFG, 2) ??
         _xfer_cfgd = true;
     }
 
     uint8_t fifo_num = TUSB_DIR_IN == tu_edpt_dir(desc_ep->bEndpointAddress) ?
                        tu_edpt_number(desc_ep->bEndpointAddress) : 0;
     uint8_t interval = 0 < desc_ep->bInterval ? (desc_ep->bInterval - 1) : 0;
 
	XHC_REG_WR(DEPCMDPAR1N(ep_index), (ep_index << 25) | (interval << 16) | (1 << 10) | (1 << 8));
	XHC_REG_WR(DEPCMDPAR0N(ep_index), (0 << 30) | (0 << 22) | (fifo_num<< 17) | ((desc_ep->wMaxPacketSize & 0x7FF) << 3) | (desc_ep->bmAttributes.xfer << 1));
	(void)usb_dc_alif_send_ep_cmd(ep_index, DEPCMD_DEPCFG, 0);

    XHC_REG_WR(DEPCMDPAR0N(ep_index), 1);
	(void)usb_dc_alif_send_ep_cmd(ep_index, DEPCMD_DEPXFERCFG, 0);
 
    sys_set_bits(DALEPENA_REG, (1 << ep_index)); 
 
    return true;
 }
 
 // Close all non-control endpoints, cancel all pending transfers if any.
 // Invoked when switching from a non-zero Configuration by SET_CONFIGURE therefore
 // required for multiple configuration support.
 void dcd_edpt_close_all(uint8_t rhport)
 {
    // disable interrupt to prevent race condition
    dcd_int_disable(rhport);

    // LOG_ALIF_INFO("%010u >%s", DWT->CYCCNT, __func__);

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
 void dcd_edpt_close(uint8_t rhport, uint8_t ep_addr) 
{
    (void)rhport;

    // Convert TinyUSB endpoint address to physical index:
    // physical_index = (endpoint_number << 1) | direction
    uint8_t ep_index = (tu_edpt_number(ep_addr) << 1) | tu_edpt_dir(ep_addr);

    // 1) Disable endpoint so hardware ignores further packets
    sys_clear_bits(DALEPENA_REG, (1U << ep_index));

    // 2) If a TRB is still owned by hardware (HWO bit set), abort it:
    if (_xfer_trb[ep_index][3] & (1U << 0)) {
        // Temporarily allow DEPCMD_DEPENDXFER to take effect
        sys_set_bits(GUCTL2_REG, GUCTL2_RST_ACTBITLATER);
        (void)usb_dc_alif_send_ep_cmd(ep_index, DEPCMD_DEPENDXFER, 0);
        sys_clear_bits(GUCTL2_REG, GUCTL2_RST_ACTBITLATER);
    }

    // 3) Clear TRB descriptor and reset bookkeeping
    _xfer_trb[ep_index][3] = 0;          // clear control bits (incl. HWO/IOC)
    _xfer_bytes[ep_index] = 0;           // no bytes pending

    LOG_ALIF_INFO("Endpoint %u %s closed",
                  (unsigned)(ep_index >> 1),
                  (ep_index & 1) ? "IN" : "OUT");
}
 
 // Submit a transfer, When complete dcd_event_xfer_complete() is invoked to
 // notify the stack
bool dcd_edpt_xfer(uint8_t rhport, uint8_t ep_addr, uint8_t *buffer, uint16_t total_bytes)
{
    // Compute physical endpoint index: (number << 1) | dir
    uint8_t ep = (tu_edpt_number(ep_addr) << 1) | tu_edpt_dir(ep_addr);

    switch (ep) {
        case 0: {  // CONTROL OUT
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

        case 1: {  // CONTROL IN
            _xfer_bytes[ep] = total_bytes;

            if (total_bytes > 0) {
                // DATA IN stage
                sys_cache_data_flush_range(buffer, total_bytes); // zephyr equ for RTSS_CleanDCache_by_Addr(..)
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
                    dcd_event_xfer_complete( rhport,
                                             tu_edpt_addr(0, TUSB_DIR_IN),
                                             0,
                                             XFER_RESULT_SUCCESS,
                                             true);
                }
            }
        } break;

        default: {  // BULK & INTERRUPT endpoints
            // LOG_ALIF_SHORT("-> ep=%u len=%u", ep, total_bytes);
            _xfer_bytes[ep] = total_bytes;
            if (tu_edpt_dir(ep_addr) == TUSB_DIR_IN) {
                // cache clean before IN transfer
                sys_cache_data_flush_range(buffer, total_bytes); // zephyr equ for RTSS_CleanDCache_by_Addr(..)
            } else {
                // for OUT endpoints controller may require full-size requests
                // you can adjust this if needed or remove hack
                // total_bytes = MIN(total_bytes, /* your max packet size */ 512);
                total_bytes = ALIF_MAX_PCK_SIZE;
            }

            // start transfer: NORMAL for data, NORMAL_ZLP for zero-length
            (void) _dcd_start_xfer(ep,
                                             buffer,
                                             total_bytes,
                                             total_bytes ? TRBCTL_NORMAL : TRBCTL_NORMAL_ZLP);
        }
    }

    return true;
}
 
 // Submit a transfer using fifo, When complete dcd_event_xfer_complete() is invoked to notify the stack
 // This API is optional, may be useful for register-based for transferring data.
 bool dcd_edpt_xfer_fifo(uint8_t rhport, uint8_t ep_addr, tu_fifo_t * ff, uint16_t total_bytes) TU_ATTR_WEAK;
 
 // Stall endpoint, any queuing transfer should be removed from endpoint
 void dcd_edpt_stall(uint8_t rhport, uint8_t ep_addr)
 {
     // DEPSSTALL command
     uint8_t ep = (tu_edpt_number(ep_addr) << 1) | tu_edpt_dir(ep_addr);
 
    (void)usb_dc_alif_send_ep_cmd(ep, DEPCMD_DEPSSTALL, 0);

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
     // DEPCSTALL command
 
     uint8_t ep = (tu_edpt_number(ep_addr) << 1) | tu_edpt_dir(ep_addr);
 
    (void)usb_dc_alif_send_ep_cmd(ep, DEPCMD_DEPCSTALL, 0);
 }
 
 void dcd_uninit(void)
 {
     // disable usb device mode
     usb_ctrl2_phy_power_on_reset_set();// set usb phy power-on-reset signal
     enable_usb_phy_isolation(); // enable usb phy isolation
     disable_usb_phy_power(); // power down usb phy
     disable_usb_periph_clk(); // disable usb peripheral clock
 }
 
  /**
 * \brief Handle a DEPEVT (Device Endpoint Event) from the USB event ring.
 *
 * \param ep_index  Physical endpoint index (0 = EP0 OUT, 1 = EP0 IN, 2+ = other endpoints)
 * \param evt       DEPEVT event type (XFERCOMPLETE, XFERINPROGRESS, XFERNOTREADY, etc.)
 * \param sts       DEPEVT status field (used for “Not Ready” codes or other flags)
 * \param par       DEPEVT parameter field (typically unused for IN/OUT transfers)
 */
static void _dcd_handle_depevt(uint8_t rhport, uint8_t ep_index, uint8_t evt, uint8_t sts, uint16_t par)
 {
    if (!(ep_index < MAX_TRB_NUM)) {
        TU_MESS_FAILED();
        TU_BREAKPOINT();
        return;
    }    

    switch (evt) {
        case DEPEVT_XFERCOMPLETE: 
        {
            // Invalidate the TRB entry so we can safely read updated fields
            sys_cache_data_invd_range(_xfer_trb[ep_index], sizeof(_xfer_trb[0]));
            // Transfer completed: extract TRBCTL type from the TRB
            uint8_t trbctl = (_xfer_trb[ep_index][3] >> 4) & 0x3F;

            if (ep_index == 0)
            {
                // EP0 OUT (SETUP or STATUS OUT)
                if (trbctl == TRBCTL_CTL_SETUP)
                {
                    // SETUP stage finished: invalidate control buffer 
                    sys_cache_data_invd_range(_ctrl_buf, sizeof(_ctrl_buf));
                    dcd_event_setup_received(rhport, _ctrl_buf, true);
                    break;
                } 

                if (trbctl == TRBCTL_CTL_STAT3) 
                {
                     if (0 < _xfer_bytes[0]) {
                         sys_cache_data_invd_range((void*) _xfer_trb[0][0], _xfer_bytes[0]);
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
                 } 
                 else 
                 {
                     // invalid TRBCTL value
                     __BKPT(0);
                 }
             } 
             else if (ep_index == 1) 
             {
                 if (trbctl != TRBCTL_CTL_STAT2) { // STATUS IN notification is done at xfer request
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
             } 
             else 
             {
                 // [TODO] check if ep is open
                if (TUSB_DIR_OUT == tu_edpt_dir(tu_edpt_addr(ep_index >> 1, ep_index & 1)))
                {
                     sys_cache_data_invd_range((void*) _xfer_trb[ep_index][0],
                                                   ALIF_MAX_PCK_SIZE - _xfer_trb[ep_index][2]);
                                                 //   _xfer_bytes[ep] - _xfer_trb[ep][2]);
                 dcd_event_xfer_complete(rhport, tu_edpt_addr(ep_index >> 1, ep_index & 1),
                                         ALIF_MAX_PCK_SIZE - _xfer_trb[ep_index][2],
                                         XFER_RESULT_SUCCESS, true);
                 } 
                 else
                 {
                    dcd_event_xfer_complete(rhport, tu_edpt_addr(ep_index >> 1, ep_index & 1),
                                         _get_transfered_bytes(ep_index),
                                         XFER_RESULT_SUCCESS, true);
                 }
             }
         } break;
         
         case DEPEVT_XFERINPROGRESS: 
         {
            LOG_ALIF_INFO("Transfer in progress");
         } break;
         
         case DEPEVT_XFERNOTREADY: 
         {
            // Transfer-not-ready indicates endpoint needs re-arming

            // LOG_ALIF_INFO("Transfer not ready: %s", sts & 8 ? "no TRB" : "no XFER");
            if ((1 == ep_index) && (DEPEVT_STS_NOTREADY_CODE == (sts & DEPEVT_STS_NOTREADY_MASK))) 
            {
                 _dcd_start_xfer(1, NULL, 0, TRBCTL_CTL_STAT2);
                 break;
            }
 
             if ((0 == ep_index) && (DEPEVT_STS_NOTREADY_CODE == (sts & DEPEVT_STS_NOTREADY_MASK))) {
                 _xfer_bytes[0] = 0;
                 _dcd_start_xfer(0, _ctrl_buf, 64, TRBCTL_CTL_STAT3);
                 break;
             }
 
             if ((1 > ep_index) && (sts & (1 << 3))) {
                 if (_xfer_trb[ep_index][3] & (1 << 0)) { // transfer was configured
                     // dependxfer can only block when actbitlater is set
                     sys_set_bits(GUCTL2_REG, GUCTL2_RST_ACTBITLATER);
                     (void)usb_dc_alif_send_ep_cmd(ep_index, DEPCMD_DEPENDXFER, 0);
                     sys_clear_bits(GUCTL2_REG, GUCTL2_RST_ACTBITLATER);
 
                     // reset the trb byte count and clean the cache
                     sys_cache_data_invd_range(_xfer_trb[ep_index], sizeof(_xfer_trb[0]));
                     _xfer_trb[ep_index][2] = _xfer_bytes[ep_index];
                     sys_cache_data_flush_range(_xfer_trb[ep_index], sizeof(_xfer_trb[0])); // zephyr equ for RTSS_CleanDCache_by_Addr(..)
 
                     // prepare ep command
                    XHC_REG_WR(DEPCMDPAR1N(ep_index), (uint32_t)local_to_global(_xfer_trb[ep_index]));
	                XHC_REG_WR(DEPCMDPAR0N(ep_index), 0);
	                (void)usb_dc_alif_send_ep_cmd(ep_index, DEPCMD_DEPSTRTXFER, 0);
 
                     *(volatile uint32_t*) 0x49007000 ^= 16; // [TEMP]
                 }
             }
         } break;

         case DEPEVT_EPCMDCMPLT: 
         {
             // redundant, currently no commands are issued with IOC bit set
         } break;
     }
 }
 
 static void _dcd_handle_devt(uint8_t rhport, uint8_t evt, uint16_t info)
 {
    // LOG_ALIF_INFO("DEVT evt%u info %u", evt, info);

     switch (evt) {
         case DEVT_USBRST: {
            _xfer_cfgd = false;
 
            // [TODO] issue depcstall for any ep in stall mode
            sys_clear_bits(DCFG_REG, DCFG_DEVADDR_MASK); // reset address
            
            // ToDo : FULL/HIGH Speed
            sys_set_bits(DCFG_REG, DCFG_SPEED(ALIF_DEVSPD_SETTING));  // 0x1: Full-speed (USB 2.0 PHY clock is 30 MHz or 60 MHz)
            dcd_event_bus_reset(rhport, ALIF_TUSB_SPEED, true); // [TODO] actual speed
         } break;
         case DEVT_CONNECTDONE: {
             // read conn speed from dsts
             // program ramclksel in gctl if needed
            //  LOG_ALIF_INFO("Connect done");
 
            uint8_t ep_idx = 0;
            XHC_REG_WR(DEPCMDPAR1N(ep_idx), (0 << 25) | (1 << 10) | (1 << 8));
	        XHC_REG_WR(DEPCMDPAR0N(ep_idx), (2 << 30) | (0 << 22) | (0 << 17) | (64 << 3) | (0 << 1));
	        (void)usb_dc_alif_send_ep_cmd(ep_idx, DEPCMD_DEPCFG, 0);
             
            ep_idx = 1;
            XHC_REG_WR(DEPCMDPAR1N(ep_idx), (1 << 25) | (1 << 10) | (1 << 8));
	        XHC_REG_WR(DEPCMDPAR0N(ep_idx), (2 << 30) | (0 << 22) | (0 << 17) | (64 << 3) | (0 << 1));
	        (void)usb_dc_alif_send_ep_cmd(ep_idx, DEPCMD_DEPCFG, 0);
        } break;
         case DEVT_ULSTCHNG: {
            // LOG_ALIF_INFO("Link status change");
             switch (info) {
                 case 0x3: { // suspend (L2)
                     dcd_event_bus_signal(rhport, DCD_EVENT_SUSPEND, true);
                 } break;
                 case 0x4: { // disconnected
                     dcd_event_bus_signal(rhport, DCD_EVENT_UNPLUGGED, true);
                 } break;
                 case 0xF: { // resume
                     dcd_event_bus_signal(rhport, DCD_EVENT_RESUME, true);
                 } break;
                 default: {}
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
static uint8_t _dcd_start_xfer(uint8_t ep, void *buf, uint32_t size, uint8_t type)
{
    // if(ep >= MAX_TRB_NUM) {
    //     LOG_ALIF_ERROR("Invalid ep %u", ep);
    //     return 1;
    // }
    // LOG_ALIF_SHORT("-->> ep=%u, size=%u, type=%u", ep, size, type);

    /* Prevent races between programming TRB and ISR handling */
    NVIC_DisableIRQ(USB_ALIF_IRQ);

    /* Populate the TRB fields */
    _xfer_trb[ep][0] = buf ? (uint32_t)local_to_global(buf) : 0U;
    _xfer_trb[ep][1] = 0U;                             // reserved
    _xfer_trb[ep][2] = size;                           // transfer length
    _xfer_trb[ep][3] = (1U << 11)    // Interrupt on Complete
                    | (1U << 10)    // Interrupt on Short Packet / Interrupt on MissedIsoc (ISP/IMI)
                    | ((uint32_t)type << 4)  // Indicates the type of TRB
                    | (1U << 1)     // ISP (Immediate Start)
                    | (1U << 0);    // Hardware Owner of Descriptor (HWO)

    /* Clean D-cache so USB controller sees updated TRB */
    sys_cache_data_flush_range(_xfer_trb[ep], sizeof(_xfer_trb[0])); // zephyr equ for RTSS_CleanDCache_by_Addr(..) 

	// EP command
	XHC_REG_WR(DEPCMDPAR1N(ep), (uint32_t)local_to_global(_xfer_trb[ep]));
	XHC_REG_WR(DEPCMDPAR0N(ep), 0);

    /* Re-enable USB interrupt before issuing command */
    NVIC_EnableIRQ(USB_ALIF_IRQ);

	(void)usb_dc_alif_send_ep_cmd(ep, DEPCMD_DEPSTRTXFER, 0);

    return 0; 
}

static uint8_t usb_dc_alif_send_ep_cmd(uint8_t ep, uint8_t cmd_type, uint16_t param)
{
	// Store PHY configuration
	uint32_t phycfg = XHC_REG_RD(GUSB2PHYCFG0_REG);
	sys_clear_bits(GUSB2PHYCFG0_REG, GUSB2PHYCFG0_ENBLSLPM | GUSB2PHYCFG0_SUSPENDUSB20);

    sys_clear_bits(DEPCMDN(ep), DEPCMD_CMDTYP_MASK);
    sys_set_bits(DEPCMDN(ep), DEPCMD_CMDTYP(cmd_type));

    sys_clear_bits(DEPCMDN(ep), DEPCMD_CMDIOC);

    sys_clear_bits(DEPCMDN(ep), DEPCMD_PARAM_MASK);
    sys_set_bits(DEPCMDN(ep), DEPCMD_PARAM(param));

    // dispatch command and wait for completion
    sys_set_bits(DEPCMDN(ep), DEPCMD_CMDACT);
    while(*((volatile uint32_t *)DEPCMDN(ep)) & DEPCMD_CMDACT) 
    { 
        __NOP(); 
    }

	// Restore PHY configuration
	XHC_REG_WR(GUSB2PHYCFG0_REG, phycfg);
	return (XHC_REG_RD(DEPCMDN(ep)) >> 12) & 0x0F; // Get command status;
}
 
#endif // CFG_TUD_ENABLED