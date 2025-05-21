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
 
#if defined(__ZEPHYR__)
  #pragma message("Building for Zephyr RTOS")
  #include <zephyr/kernel.h>
  #include <zephyr/cache.h>
  
  #include <soc_memory_map.h>     // for local_to_global() function
    #define LOG_MEM_INFO(fmt, ...)  log_mem_append("[I] %s: " fmt "\n", __func__, ##__VA_ARGS__)
    #define LOG_MEM_ERROR(fmt, ...) log_mem_append("[E] %s: " fmt "\n", __func__, ##__VA_ARGS__)

    #define LOG_PRINT_INFO(fmt, ...)  printf("[%lld ms] INFO  %s: " fmt "\n", k_uptime_get(), __func__, ##__VA_ARGS__)
    #define LOG_PRINT_ERROR(fmt, ...) printf("[%lld ms] ERROR %s: " fmt "\n", k_uptime_get(), __func__, ##__VA_ARGS__)

    #define LOG_ALIF_INFO  LOG_MEM_INFO
    #define LOG_ALIF_ERROR LOG_MEM_ERROR
        
    #define LOG_ALIF_HEXDUMP log_buffer_hex

    // ToDO: remove this when USB_CTRL_BASE is defined in device tree
    #define USB_CTRL_BASE   0x48200000
    #define USB_ALIF_IRQ    101
//   #define USB_CTRL_BASE DT_REG_ADDR(DT_INST(0, DT_DRV_COMPAT))
//   #define USB_ALIF_IRQ  DT_IRQN(DT_INST(0, DT_DRV_COMPAT))

  #include "platform_def.h"
  #include "dcd_ensemble.h"


  // Direct register access for compatibility
  #define XHC_REG_RD(addr)      sys_read32((addr))
  #define XHC_REG_WR(addr, val) sys_write32((val), (addr))

  // prototype for the functions
  bool RTSS_IsCacheClean_Required_by_Addr(volatile void *addr, int32_t size);
  static uint8_t usb_dc_alif_send_ep_cmd(uint8_t ep, uint8_t cmd_type, uint16_t param);


#else
  #include "clk.h"
  #include "power.h"
  #include "alif_dcd_reg.h"

  #if defined(CORE_M55_HE)
    #pragma message("Building for CORE_M55_HE (baremetal)")
    #include "M55_HE.h"
    #include "M55_HE_Config.h"
  #elif defined(CORE_M55_HP)
    #pragma message("Building for CORE_M55_HP (baremetal)")
    #include "M55_HP.h"
    #include "M55_HP_Config.h"
  #else
    #error "Unsupported core!"
  #endif
#endif
 
 #include "device/dcd.h"

#define HEX_DUMP_BYTES_PER_LINE 16

 #define TUSB_ALIF_DEBUG
 #define TUSB_ALIF_DEBUG_DEPTH (2048)
 
 #if defined(TUSB_ALIF_DEBUG)
 #if (1 < TUSB_ALIF_DEBUG_DEPTH)
 #define LOG(...)      memset(logbuf[bi % TUSB_ALIF_DEBUG_DEPTH], ' ', 48);\
                           snprintf(logbuf[(bi++) % TUSB_ALIF_DEBUG_DEPTH], 48, __VA_ARGS__);
 char logbuf[TUSB_ALIF_DEBUG_DEPTH][48];
 int bi = 0;
 #else
 #define LOG(...)      memset(logbuf, ' ', 48);\
                           snprintf(logbuf, 48, __VA_ARGS__)
 char logbuf[48];
 #endif
 #else
 #define LOG(...)
 #endif
 
#define MAX_EP_NUM 8            // max number of endpoints
#define MAX_TRB_NUM MAX_EP_NUM  // max number of trbs 

/// Structs and Buffers --------------------------------------------------------
#define EVT_BUF_SIZE  1024
#define CTRL_BUF_SIZE 64
__aligned(4096) CFG_TUSB_MEM_SECTION static uint32_t _evnt_buf[EVT_BUF_SIZE];
__aligned(32) CFG_TUSB_MEM_SECTION static uint8_t _ctrl_buf[CTRL_BUF_SIZE]; // [TODO] runtime alloc
__aligned(32) CFG_TUSB_MEM_SECTION static uint32_t _xfer_trb[MAX_TRB_NUM][4];         // [TODO] runtime alloc

//  static uint32_t  _evnt_buf[1024] CFG_TUSB_MEM_SECTION __attribute__((aligned(4096))); // [TODO] runtime alloc
 static volatile uint32_t* _evnt_tail;
 static uint16_t _xfer_bytes[8];
 static bool     _ctrl_long_data = false;
 static bool     _xfer_cfgd = false;
 // static bool     _addr_req = false;
 static uint32_t _sts_stage = 0;
 
 /// Private Functions ----------------------------------------------------------
 
 static uint8_t _dcd_start_xfer(uint8_t ep, void* buf, uint32_t size, uint8_t type);
 
 static void _dcd_handle_depevt(uint8_t ep, uint8_t evt, uint8_t sts, uint16_t par);
 static void _dcd_handle_devt(uint8_t evt, uint16_t info);
 static void log_buffer_hex(const uint8_t *buf, size_t len);
 static void log_mem_append(const char *fmt, ...);
 void flush_prev_session_log(void);
 static void init_log_mem(void);
 
 /// API Extension --------------------------------------------------------------
 
 void dcd_uninit(void);
 
 /// Device Setup ---------------------------------------------------------------
 
 // Initializes the USB peripheral for device mode and enables it.
 // This function should enable internal D+/D- pull-up for enumeration.
bool dcd_init(uint8_t rhport, const tusb_rhport_init_t* rh_init) 
{
    (void) rh_init;

    init_log_mem();
    flush_prev_session_log();

    LOG_ALIF_INFO("-->");

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
    dcd_disconnect(-1);     // ToDO: check if this is needed

	// Device soft reset
	sys_set_bits(DCTL_REG, DCTL_CSFTRST);
	while (sys_test_bit(DCTL_REG, DCTL_CSFTRST)) {
        k_busy_wait(1 * 1000);  // 1000 μs = 1 ms
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
		return -ENODEV;
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
	sys_clear_bits(DCFG_REG, DCFG_SPEED_MASK); // ToDo: Full Speed

    // allocate ring buffer for events
    memset(_evnt_buf, 0, sizeof(_evnt_buf));
    RTSS_CleanDCache_by_Addr((uint32_t *)_evnt_buf, sizeof(_evnt_buf));
    _evnt_tail = _evnt_buf;

	XHC_REG_WR(GEVNTADDRL0_REG, (uint32_t)local_to_global(_evnt_buf));
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
	uint8_t st = usb_dc_alif_send_ep_cmd(0, DEPCMD_DEPSTARTCFG, 0);
	if (st != 0) {
		LOG_ALIF_ERROR("EP0 DEPSTARTCFG failed, status=0x%02x", st);
	}

	// Endpoint Configuration - EP0 OUT (ep = 0)
	uint8_t ep_idx = 0;
	XHC_REG_WR(DEPCMDPAR1N(ep_idx), (0 << 25) | (1 << 10) | (1 << 8));
	XHC_REG_WR(DEPCMDPAR0N(ep_idx), (0 << 22) | (0 << 17) | (512 << 3) | (0 << 1));
	st = usb_dc_alif_send_ep_cmd(ep_idx, DEPCMD_DEPCFG, 0);
	if (st != 0) {
		LOG_ALIF_ERROR("EP%d DEPCFG failed, status=0x%02x", ep_idx, st);
	}

	// Endpoint Configuration - EP0 IN (ep = 1)
	ep_idx = 1;
	XHC_REG_WR(DEPCMDPAR1N(ep_idx), (1 << 25) | (1 << 10) | (1 << 8));
	XHC_REG_WR(DEPCMDPAR0N(ep_idx), (0 << 22) | (0 << 17) | (512 << 3) | (0 << 1));
	st = usb_dc_alif_send_ep_cmd(ep_idx, DEPCMD_DEPCFG, 0);
	if (st != 0) {
		LOG_ALIF_ERROR("EP%d DEPCFG failed, status=0x%02x", ep_idx, st);
	}

	// set initial xfer configuration for CONTROL eps
    XHC_REG_WR(DEPCMDPAR0N(0), 1);
	st = usb_dc_alif_send_ep_cmd(0, DEPCMD_DEPXFERCFG, 0);
	if (st != 0) {
		LOG_ALIF_ERROR("EP%d DEPXFERCFG failed, status=0x%02x", 0, st);
	}
	XHC_REG_WR(DEPCMDPAR0N(1), 1);
	st = usb_dc_alif_send_ep_cmd(1, DEPCMD_DEPXFERCFG, 0);
	if (st != 0) {
		LOG_ALIF_ERROR("EP%d DEPXFERCFG failed, status=0x%02x", 1, st);
	}

    // prepare trb for the first setup packet
    uint8_t ep = 0;
    memset(_ctrl_buf, 0, sizeof(_ctrl_buf));
    _xfer_trb[ep][0] = (uint32_t) local_to_global(_ctrl_buf);
    _xfer_trb[ep][1] = 0;
    _xfer_trb[ep][2] = 8;
    _xfer_trb[ep][3] = (1 << 11) | (1 << 10) | (TRBCTL_CTL_SETUP << 4) | (1 << 1) | (1 << 0);
    RTSS_CleanDCache_by_Addr(_xfer_trb[ep], sizeof(_xfer_trb[ep]));

    // send trb to the usb dma
    XHC_REG_WR(DEPCMDPAR1N(ep), (uint32_t)local_to_global(_xfer_trb[ep]));
	XHC_REG_WR(DEPCMDPAR0N(ep), 0);
	if (usb_dc_alif_send_ep_cmd(ep, DEPCMD_DEPSTRTXFER, 0) != 0) {
		LOG_ALIF_ERROR("tx err!");
	}

    sys_set_bits(DALEPENA_REG, (1 << ep)); // enable ep0 OUT
    sys_set_bits(DALEPENA_REG, (1 << (ep + 1))); // enable ep0 IN

    // enable pull-ups
    dcd_connect(rhport);

	// 7. Enable IRQ
	IRQ_CONNECT(USB_ALIF_IRQ, 5, dcd_int_handler, NULL, 0);
    
	return true;
} 
 
 // Processes all the hardware generated events e.g bus reset, new data packet
 // from host... It will be called by application in the MCU USB interrupt handler.
 void dcd_int_handler(uint8_t rhport)
 {
    //  LOG_ALIF_INFO("-->>");

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
        //  RTSS_InvalidateDCache_by_Addr(_evnt_buf, sizeof(_evnt_buf));
         sys_cache_data_invd_range(_evnt_buf, sizeof(_evnt_buf)); // zephyr equ for RTSS_InvalidateDCache_by_Addr(..)
         volatile evt_t e = {.val = *_evnt_tail++};
        //  LOG_ALIF_INFO("%010u IRQ loop, evnt %08x", DWT->CYCCNT, e.val);

          // wrap around
         if (_evnt_tail >= (_evnt_buf + 1024)) _evnt_tail = _evnt_buf;
 
         // dispatch the right handler for the event type
         if (e.depevt.is_devt == 0) {// DEPEVT
             _dcd_handle_depevt(e.depevt.ep, e.depevt.evt, e.depevt.sts, e.depevt.par);
         } else { // DEVT
             _dcd_handle_devt(e.devt.evt, e.devt.info);
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
    // LOG_ALIF_INFO("-->>");
    sb_dc_alif_int_enable();
    (void) rhport;
 }
 
 // Disables the USB device interrupt.
 // May be used to prevent concurrency issues when mutating data structures
 // shared between main code and the interrupt handler.
 void dcd_int_disable(uint8_t rhport)
 {
    // LOG_ALIF_INFO("-->>");
    sb_dc_alif_int_disable();
    (void) rhport;
 }
 
 // Receive Set Address request, mcu port must also include status IN response.
 // If your peripheral automatically changes address during enumeration you may
 // leave this empty and also no queue an event for the corresponding SETUP packet.
 void dcd_set_address(uint8_t rhport, uint8_t dev_addr)
 {
    LOG_ALIF_INFO("-->>");

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
    LOG_ALIF_INFO("-->>");  
     // [TODO] clear all xfers and eps first
    sys_set_bits(DCTL_REG, DCTL_RUN_STOP);
     (void) rhport;
 }
 
 // Disconnect by disabling internal pull-up resistor on D+/D-
 void dcd_disconnect(uint8_t rhport)
 {
    LOG_ALIF_INFO("-->>");  
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
    LOG_ALIF_INFO("%010u >%s", DWT->CYCCNT, __func__);
 
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
    LOG_ALIF_INFO("%010u >%s %u %s %u %u", DWT->CYCCNT, __func__, desc_ep->bEndpointAddress,
         desc_ep->bmAttributes.xfer == TUSB_XFER_BULK ? "bulk" : "int",
         desc_ep->wMaxPacketSize, desc_ep->bInterval);
 
     if (TUSB_XFER_ISOCHRONOUS == desc_ep->bmAttributes.xfer)
         return false;
 
     uint8_t ep = (tu_edpt_number(desc_ep->bEndpointAddress) << 1) |
                  tu_edpt_dir(desc_ep->bEndpointAddress);
 
     // [TODO] verify that the num doesn't exceed hw max
 
     if (false == _xfer_cfgd) {
        // Endpoint Configuration - Start config phase
         usb_dc_alif_send_ep_cmd(0, DEPCMD_DEPSTARTCFG, 2);     // check here!! - usb_dc_alif_send_ep_cmd(ep, DEPCMD_DEPSTARTCFG, 2) ??
         _xfer_cfgd = true;
     }
 
     uint8_t fifo_num = TUSB_DIR_IN == tu_edpt_dir(desc_ep->bEndpointAddress) ?
                        tu_edpt_number(desc_ep->bEndpointAddress) : 0;
     uint8_t interval = 0 < desc_ep->bInterval ? (desc_ep->bInterval - 1) : 0;
 
	XHC_REG_WR(DEPCMDPAR1N(ep), (ep << 25) | (interval << 16) | (1 << 10) | (1 << 8));
	XHC_REG_WR(DEPCMDPAR0N(ep), (0 << 30) | (0 << 22) | (fifo_num<< 17) | ((desc_ep->wMaxPacketSize & 0x7FF) << 3) | (desc_ep->bmAttributes.xfer << 1));
	if (usb_dc_alif_send_ep_cmd(ep, DEPCMD_DEPCFG, 0) != 0) {
		LOG_ALIF_ERROR("EP%d DEPCFG failed", ep);
        return false;
	}

    XHC_REG_WR(DEPCMDPAR0N(0), 1);
	if (usb_dc_alif_send_ep_cmd(0, DEPCMD_DEPXFERCFG, 0) != 0) {
		LOG_ALIF_ERROR("EP%d DEPXFERCFG failed", 0);
        return false;
	}
 
     sys_set_bits(DALEPENA_REG, (1 << ep)); 
 
     return true;
 }
 
 // Close all non-control endpoints, cancel all pending transfers if any.
 // Invoked when switching from a non-zero Configuration by SET_CONFIGURE therefore
 // required for multiple configuration support.
 void dcd_edpt_close_all(uint8_t rhport)
 {
    LOG_ALIF_INFO("%010u >%s", DWT->CYCCNT, __func__);
 }
 
 // Close an endpoint. his function is used for implementing alternate settings.
 // After calling this, the device should not respond to any packets directed
 // towards this endpoint. When called, this function must abort any transfers in
 // progress through this endpoint, before returning.
 // Implementation is optional. Must be called from the USB task.
 // Interrupts could be disabled or enabled during the call.
 void dcd_edpt_close(uint8_t rhport, uint8_t ep_addr) 
{
    // TODO: implement this function
    (void)rhport;
    (void)ep_addr;
    TU_ASSERT(0, "dcd_edpt_close() not implemented");
}
 
 // Submit a transfer, When complete dcd_event_xfer_complete() is invoked to
 // notify the stack
bool dcd_edpt_xfer(uint8_t rhport, uint8_t ep_addr, uint8_t *buffer, uint16_t total_bytes)
{
    (void) rhport;
    LOG_ALIF_INFO("ep_addr=%u buf=%p len=%u",
            ep_addr, (void*)buffer, total_bytes);
    // LOG_ALIF_HEXDUMP(buffer, total_bytes);

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
                RTSS_CleanDCache_by_Addr(buffer, total_bytes);
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
                    dcd_event_xfer_complete(TUD_OPT_RHPORT,
                                             tu_edpt_addr(0, TUSB_DIR_IN),
                                             0,
                                             XFER_RESULT_SUCCESS,
                                             true);
                }
            }
        } break;

        default: {  // BULK & INTERRUPT endpoints
            _xfer_bytes[ep] = total_bytes;
            if (tu_edpt_dir(ep_addr) == TUSB_DIR_IN) {
                // cache clean before IN transfer
                RTSS_CleanDCache_by_Addr(buffer, total_bytes);
            } else {
                // for OUT endpoints controller may require full-size requests
                // you can adjust this if needed or remove hack
                // total_bytes = MIN(total_bytes, /* your max packet size */ 512);
                total_bytes = 512;
            }

            // start transfer: NORMAL for data, NORMAL_ZLP for zero-length
            uint8_t result = _dcd_start_xfer(ep,
                                             buffer,
                                             total_bytes,
                                             total_bytes ? TRBCTL_NORMAL : TRBCTL_NORMAL_ZLP);
            LOG_ALIF_INFO("start xfer returned %u", result);
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
     LOG_ALIF_INFO("-->>");
 
     uint8_t ep = (tu_edpt_number(ep_addr) << 1) | tu_edpt_dir(ep_addr);
 
    if(usb_dc_alif_send_ep_cmd(ep, DEPCMD_DEPSSTALL, 0) != 0) {
        LOG_ALIF_ERROR("EP%d DEPSSTALL failed", ep);
        return;
    }

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
     LOG_ALIF_INFO("-->>");
 
     uint8_t ep = (tu_edpt_number(ep_addr) << 1) | tu_edpt_dir(ep_addr);
 
    usb_dc_alif_send_ep_cmd(ep, DEPCMD_DEPCSTALL, 0);
 }
 
 void dcd_uninit(void)
 {
    LOG_ALIF_INFO("-->>");
 
     // disable usb device mode
     usb_ctrl2_phy_power_on_reset_set();// set usb phy power-on-reset signal
     enable_usb_phy_isolation(); // enable usb phy isolation
     disable_usb_phy_power(); // power down usb phy
     disable_usb_periph_clk(); // disable usb peripheral clock
 }
 
 static void _dcd_handle_depevt(uint8_t ep, uint8_t evt, uint8_t sts, uint16_t par)
 {
    // LOG_ALIF_INFO("%010u DEPEVT ep%u evt%u sts%u par%u", DWT->CYCCNT, ep, evt, sts, par);
 
     switch (evt) {
         case DEPEVT_XFERCOMPLETE: {
             LOG_ALIF_INFO("Transfer complete");
             sys_cache_data_invd_range(_xfer_trb[ep], sizeof(_xfer_trb[0]));// zephyr equ for RTSS_InvalidateDCache_by_Addr(..)
             if (0 == ep) {
                 uint8_t trbctl = (_xfer_trb[0][3] >> 4) & 0x3F;
                 if (TRBCTL_CTL_SETUP == trbctl) {
                     sys_cache_data_invd_range(_ctrl_buf, sizeof(_ctrl_buf));

                    //  LOG_ALIF_HEXDUMP(_ctrl_buf, 8);

                     dcd_event_setup_received(TUD_OPT_RHPORT, _ctrl_buf, true);
                 } else if (TRBCTL_CTL_STAT3 == trbctl) {
                     if (0 < _xfer_bytes[0]) {
                         sys_cache_data_invd_range((void*) _xfer_trb[0][0], _xfer_bytes[0]);
                         dcd_event_xfer_complete(TUD_OPT_RHPORT, tu_edpt_addr(0, TUSB_DIR_OUT),
                                                 _xfer_bytes[0] - (_xfer_trb[0][2] & 0xFFFFFF),
                                                 XFER_RESULT_SUCCESS, true);
                     } else {
                         if (2 == ++_sts_stage) {
                             _sts_stage = 0;
                             dcd_event_xfer_complete(TUD_OPT_RHPORT, tu_edpt_addr(0, TUSB_DIR_OUT),
                                                     0, XFER_RESULT_SUCCESS, true);
 
                             // *(volatile uint32_t*) 0x4900C000 ^= 8; // [TEMP]
                         }
                     }
                 } else {
                     // invalid TRBCTL value
                     __BKPT(0);
                 }
             } else if (1 == ep) {
                 uint8_t trbctl = (_xfer_trb[1][3] >> 4) & 0x3F;
                 LOG_ALIF_INFO("ep1 xfer trb3 = %08x trb2 = %08x", _xfer_trb[1][3], _xfer_trb[1][2]);
                 if (TRBCTL_CTL_STAT2 != trbctl) { // STATUS IN notification is done at xfer request
                     dcd_event_xfer_complete(TUD_OPT_RHPORT, tu_edpt_addr(0, TUSB_DIR_IN),
                                             _xfer_bytes[1] - (_xfer_trb[1][2] & 0xFFFFFF),
                                             XFER_RESULT_SUCCESS, true);
                 } else {
                     if (2 == ++_sts_stage) {
                         _sts_stage = 0;
                         dcd_event_xfer_complete(TUD_OPT_RHPORT, tu_edpt_addr(0, TUSB_DIR_IN),
                                                 0, XFER_RESULT_SUCCESS, true);
 
                         // *(volatile uint32_t*) 0x4900C000 ^= 8; // [TEMP]
                     }
                 }
             } else {
                 // [TODO] check if ep is open
                 LOG_ALIF_INFO("ep%u xfer trb3 = %08x trb2 = %08x", ep, _xfer_trb[ep][3], _xfer_trb[ep][2]);
                 
                 if (TUSB_DIR_OUT == tu_edpt_dir(tu_edpt_addr(ep >> 1, ep & 1))) {
                     sys_cache_data_invd_range((void*) _xfer_trb[ep][0],
                                                   512 - _xfer_trb[ep][2]);
                                                 //   _xfer_bytes[ep] - _xfer_trb[ep][2]);
                 dcd_event_xfer_complete(TUD_OPT_RHPORT, tu_edpt_addr(ep >> 1, ep & 1),
                                         512 - _xfer_trb[ep][2],
                                         XFER_RESULT_SUCCESS, true);
                 } else
                 dcd_event_xfer_complete(TUD_OPT_RHPORT, tu_edpt_addr(ep >> 1, ep & 1),
                                         _xfer_bytes[ep] - _xfer_trb[ep][2],
                                         XFER_RESULT_SUCCESS, true);

             }
         } break;
         case DEPEVT_XFERINPROGRESS: {
            LOG_ALIF_INFO("Transfer in progress");
         } break;
         case DEPEVT_XFERNOTREADY: {
            LOG_ALIF_INFO("Transfer not ready: %s", sts & 8 ? "no TRB" : "no XFER");
 
             // XferNotReady NotActive for status stage
             if ((1 == ep) && (0b0010 == (sts & 0b1011))) {
                 _dcd_start_xfer(1, NULL, 0, TRBCTL_CTL_STAT2);
                 break;
             }
 
             if ((0 == ep) && (0b0010 == (sts & 0b1011))) {
                 _xfer_bytes[0] = 0;
                 _dcd_start_xfer(0, _ctrl_buf, 64, TRBCTL_CTL_STAT3);
                 break;
             }
 
             if ((1 > ep) && (sts & (1 << 3))) {
                 if (_xfer_trb[ep][3] & (1 << 0)) { // transfer was configured
                     // dependxfer can only block when actbitlater is set
                     sys_set_bits(GUCTL2_REG, GUCTL2_RST_ACTBITLATER);
                     usb_dc_alif_send_ep_cmd(ep, DEPCMD_DEPENDXFER, 0);
                     sys_clear_bits(GUCTL2_REG, GUCTL2_RST_ACTBITLATER);
 
                     // reset the trb byte count and clean the cache
                     sys_cache_data_invd_range(_xfer_trb[ep], sizeof(_xfer_trb[0]));
                     _xfer_trb[ep][2] = _xfer_bytes[ep];
                     RTSS_CleanDCache_by_Addr(_xfer_trb[ep], sizeof(_xfer_trb[ep]));
 
                     // prepare ep command
                    XHC_REG_WR(DEPCMDPAR1N(ep), (uint32_t)local_to_global(_xfer_trb[ep]));
	                XHC_REG_WR(DEPCMDPAR0N(ep), 0);
	                if (usb_dc_alif_send_ep_cmd(ep, DEPCMD_DEPSTRTXFER, 0) != 0) {
		                LOG_ALIF_ERROR("tx err!");
	                }
 
                     *(volatile uint32_t*) 0x49007000 ^= 16; // [TEMP]
                 }
             }
         } break;
         case DEPEVT_EPCMDCMPLT: {
             // redundant, currently no commands are issued with IOC bit set
         } break;
     }
 }
 
 static void _dcd_handle_devt(uint8_t evt, uint16_t info)
 {
    // LOG_ALIF_INFO("DEVT evt%u info %u", evt, info);

     switch (evt) {
         case DEVT_USBRST: {
             _xfer_cfgd = false;
 
             // [TODO] issue depcstall for any ep in stall mode
            sys_clear_bits(DCFG_REG, DCFG_DEVADDR_MASK); // reset address
             LOG_ALIF_INFO("USB reset");
             dcd_event_bus_reset(TUD_OPT_RHPORT, TUSB_SPEED_HIGH, true); // [TODO] actual speed
         } break;
         case DEVT_CONNECTDONE: {
             // read conn speed from dsts
             // program ramclksel in gctl if needed
             LOG_ALIF_INFO("Connect done");
 
            uint8_t ep_idx = 0;
            XHC_REG_WR(DEPCMDPAR1N(ep_idx), (0 << 25) | (1 << 10) | (1 << 8));
	        XHC_REG_WR(DEPCMDPAR0N(ep_idx), (2 << 30) | (0 << 22) | (0 << 17) | (64 << 3) | (0 << 1));
	        if (usb_dc_alif_send_ep_cmd(ep_idx, DEPCMD_DEPCFG, 0) != 0) {
		        LOG_ALIF_ERROR("EP%d DEPCFG failed", ep_idx);
	        }
             
            ep_idx = 1;
            XHC_REG_WR(DEPCMDPAR1N(ep_idx), (1 << 25) | (1 << 10) | (1 << 8));
	        XHC_REG_WR(DEPCMDPAR0N(ep_idx), (2 << 30) | (0 << 22) | (0 << 17) | (64 << 3) | (0 << 1));
	        if (usb_dc_alif_send_ep_cmd(ep_idx, DEPCMD_DEPCFG, 0) != 0) {
		        LOG_ALIF_ERROR("EP%d DEPCFG failed", ep_idx);
	        }
         } break;
         case DEVT_ULSTCHNG: {
            LOG_ALIF_INFO("Link status change");
             switch (info) {
                 case 0x3: { // suspend (L2)
                     dcd_event_bus_signal(TUD_OPT_RHPORT, DCD_EVENT_SUSPEND, true);
                 } break;
                 case 0x4: { // disconnected
                     dcd_event_bus_signal(TUD_OPT_RHPORT, DCD_EVENT_UNPLUGGED, true);
                 } break;
                 case 0xF: { // resume
                     dcd_event_bus_signal(TUD_OPT_RHPORT, DCD_EVENT_RESUME, true);
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
    if(ep >= MAX_TRB_NUM) {
        LOG_ALIF_ERROR("Invalid ep %u", ep);
        return 1;
    }
    // LOG_ALIF_INFO("-->>");

    /* Prevent races between programming TRB and ISR handling */
    sb_dc_alif_int_disable();  

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
    RTSS_CleanDCache_by_Addr(_xfer_trb[ep], sizeof(_xfer_trb[ep]));  

	// EP command
	XHC_REG_WR(DEPCMDPAR1N(ep), (uint32_t)local_to_global(_xfer_trb[ep]));
	XHC_REG_WR(DEPCMDPAR0N(ep), 0);

  /* Re-enable USB interrupt before issuing command */
    sb_dc_alif_int_enable();

    // Issue DEPSTRTXFER command to start transfer
	if (usb_dc_alif_send_ep_cmd(ep, DEPCMD_DEPSTRTXFER, 0) != 0) {
		LOG_ALIF_ERROR("tx err!");
        return 1; 
	}

    return 0; 
}


 // Only for Zephyr specific functions -----------------------------------
#if defined(__ZEPHYR__)

 __STATIC_FORCEINLINE
bool RTSS_Is_TCM_Addr(const volatile void *local_addr)
{
	uint32_t addr = (uint32_t)local_addr;

	return ((addr < (ITCM_BASE + ITCM_SIZE)) ||
		((addr > DTCM_BASE) && (addr < (DTCM_BASE + DTCM_SIZE))));
}

bool RTSS_IsCacheClean_Required_by_Addr(volatile void *addr, int32_t size)
{
	(void)size;
	/*
	 * This is a hook, where user can redefine its implementation in application.
	 *
	 * For some scenarios, User do not need to do anything apart from DSB for
	 * un-cached or shared regions, and do not need to clean write-through regions.
	 * This particular API is introduced to reduce the overhead in Cache operation
	 * function for the above scenarios mentioned.
	 *
	 * User can define the range of memories for the cache operations can be skipped.
	 * Return True if cache operation is required else return False.
	 *
	 */

	/*
	 * If the provided address is in TCM, then no cache operation is required
	 */
	if (RTSS_Is_TCM_Addr(addr)) {
		return false;
	}

	return true;
}

static uint8_t usb_dc_alif_send_ep_cmd(uint8_t ep, uint8_t cmd_type, uint16_t param)
{
	// LOG_ALIF_INFO(" ep=0x%02x, cmd_type=0x%02x, param=0x%04x", ep, cmd_type, param);

	// Store PHY configuration
	uint32_t phycfg = XHC_REG_RD(GUSB2PHYCFG0_REG);
	sys_clear_bits(GUSB2PHYCFG0_REG, GUSB2PHYCFG0_ENBLSLPM);
	sys_clear_bits(GUSB2PHYCFG0_REG, GUSB2PHYCFG0_SUSPENDUSB20);

	// Set up command in DEPCMD register
	uint32_t cmd = XHC_REG_RD(DEPCMDN(ep));
	cmd &= ~DEPCMD_CMDTYP_MASK;
	cmd |= DEPCMD_CMDTYP(cmd_type);

	cmd &= ~DEPCMD_CMDIOC; // Clear IOC bit

	cmd &= ~DEPCMD_PARAM_MASK;
	cmd |= DEPCMD_PARAM(param);

	cmd |= DEPCMD_CMDACT; // Set ACT bit
	XHC_REG_WR(DEPCMDN(ep), cmd);

	// Wait for command completion
	while (XHC_REG_RD(DEPCMDN(ep)) & DEPCMD_CMDACT) {
        // k_busy_wait(1);
	}

	// Restore PHY configuration
	XHC_REG_WR(GUSB2PHYCFG0_REG, phycfg);

	uint8_t status = (XHC_REG_RD(DEPCMDN(ep)) >> 12) & 0x0F; // Get command status
	// LOG_DBG("DEPCMD complete: ep=%02x, cmd=0x%02x, result=0x%02x", ep, cmd_type, status);

	return status;
}

#endif 

static void log_buffer_hex(const uint8_t *buf, size_t len)
{
    char line[HEX_DUMP_BYTES_PER_LINE * 3 + 1]; // "FF " = 3 symbols + \0
    size_t offset = 0;

    for (size_t i = 0; i < len; i++) {
        if (i % HEX_DUMP_BYTES_PER_LINE == 0) {
            offset = 0;
        }

        offset += snprintf(line + offset,
                           sizeof(line) - offset,
                           "%02x ",
                           buf[i]);

        if ((i % HEX_DUMP_BYTES_PER_LINE) == (HEX_DUMP_BYTES_PER_LINE - 1) ||
            i == len - 1) {
            line[offset] = '\0';
            printf("%04zx: %s\n",  // %04zx — offset in buffer
                          i - (i % HEX_DUMP_BYTES_PER_LINE),
                          line);
        }
    }
}
 
#define LOG_MEM_SIZE 1024 * 2
#define LOG_MEM_MAGIC 0xA5A5A5A5
__attribute__((section(".noinit"))) static char log_mem_buf[LOG_MEM_SIZE];
__attribute__((section(".noinit"))) static size_t log_mem_pos = 0; 
__attribute__((section(".noinit"))) static uint32_t log_mem_magic;

static void init_log_mem(void)
{
    if (log_mem_magic != LOG_MEM_MAGIC) {
        log_mem_pos    = 0;
        log_mem_magic  = LOG_MEM_MAGIC;
    }
}
static void log_mem_append(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    // оставляем 1 байт под '\0'
    int rem = LOG_MEM_SIZE - log_mem_pos - 1;
    if (rem <= 0) {
        va_end(ap);
        return; // buffer overflow
    }

    int written = vsnprintf(&log_mem_buf[log_mem_pos], rem, fmt, ap);
    va_end(ap);
    if (written > 0) {
        log_mem_pos += (written < rem ? written : rem);
    }
}

// Burst log to stdout
void flush_prev_session_log(void)
{
    if (log_mem_pos > LOG_MEM_SIZE) {
        log_mem_pos = LOG_MEM_SIZE;
    }

    if (log_mem_pos > 0) {
        /* Print the log buffer */
        size_t offset = 0;
        while (offset < log_mem_pos) {
            size_t chunk = (log_mem_pos - offset > 64) ? 64 : (log_mem_pos - offset);
            printf("%.*s", (int)chunk, &log_mem_buf[offset]);
            offset += chunk;
        }
        printf("\n--- End of previous USB-log session ---\n");
        
        /* Reset the log buffer */
        log_mem_pos = 0;
    }
}

#endif // CFG_TUD_ENABLED