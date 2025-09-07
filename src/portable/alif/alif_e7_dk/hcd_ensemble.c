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

//--------------------------------------------------------------------+
// Controller API
//--------------------------------------------------------------------+

#define ONE_KB                            1024
#define UX_DEMO_STACK_SIZE                (4*ONE_KB)
#define UX_DEMO_NS_SIZE                   (128 * ONE_KB)
#define UX_REGULAR_MEMORY_SIZE            (79 * ONE_KB)
#define UX_CACHE_SAFE_MEMORY_SIZE         (20 * ONE_KB)

static uint8_t dma_buf[UX_DEMO_NS_SIZE]__attribute__((section("usb_dma_buf")));

// optional hcd configuration, called by tuh_configure()
bool hcd_configure(uint8_t rhport, uint32_t cfg_id, const void* cfg_param) {
  (void) rhport;
  (void) cfg_id;
  (void) cfg_param;

  printf("Called %s(%u %u %p)\n", __FUNCTION__, rhport, cfg_id, cfg_param);

  return false;
}

UX_HCD *_hcd;

// Initialize controller to host mode
bool hcd_init(uint8_t rhport, const tusb_rhport_init_t* rh_init) {
  (void) rhport;
  (void) rh_init;

  static UX_HCD hcd;
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

  printf("_ux_hcd_xhci_initialize() returned %d\n\r", ret);

  return ret == UX_SUCCESS;
}


// Interrupt Handler
void hcd_int_handler(uint8_t rhport, bool in_isr) {
  (void) rhport;
  (void) in_isr;
  uint32_t port_status = hcd_xhci->op_regs->PORTSC;
  printf("Called %s(%u %u), port_status=%#x\n\r", __FUNCTION__, rhport, in_isr, port_status);

  _ux_xhci_event_irq_handler(hcd_xhci);
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
    printf("ERROR: HCD port reset has failed\n\r");
  } else {
    printf("DEBUG: HCD port reset success\n\r");
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
    printf("DEBUG: low speed device\n\r");
    return TUSB_SPEED_LOW;
  } else if (DEV_FULLSPEED(port_sts_ctrl)) {
    printf("DEBUG: full speed device\n\r");
    return TUSB_SPEED_FULL;
  } else if (DEV_HIGHSPEED(port_sts_ctrl)) {
    printf("DEBUG: high speed device\n\r");
    return TUSB_SPEED_HIGH;
  } else {
    printf("ERROR: invalid device speed (%x)\n\r", DEV_PORT_SPEED(port_sts_ctrl));
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

    printf("_ux_host_stack_device_address_set()\n\r");
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
  printf("Called %s(%u %u %p)\n\r", __FUNCTION__, rhport, dev_addr, ep_desc);

  // NOTE: ep_desc is allocated on the stack when called from usbh_edpt_control_open()
  // You need to copy the data into a local variable who maintains the state of the endpoint and transfer.
  // Check _hcd_data in hcd_dwc2.c for example.

  UX_HCD * hcd = hcd_xhci -> ux_hcd_xhci_hcd_owner;
  UX_HCD_XHCI *xhci =  (UX_HCD_XHCI *) hcd -> ux_hcd_controller_hardware;

  tuh_bus_info_t bus_info;
  tuh_bus_info_get(dev_addr, &bus_info);

  if (ep_desc->bEndpointAddress == 0) {
    printf("_ux_host_stack_new_device_get() \n\r");
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
    printf("UX_HCD_CREATE_ENDPOINT\n\r");
    unsigned int status =  hcd -> ux_hcd_entry_function(hcd, UX_HCD_CREATE_ENDPOINT, (void *) control_endpoint);
    if (status == UX_SUCCESS) {
      // Now control endpoint is ready, set state to running
      control_endpoint -> ux_endpoint_state = UX_ENDPOINT_RUNNING;

      tusb_time_delay_ms_api(UX_RH_ENUMERATION_RETRY_DELAY);

        /* Set the address of the device. The first time a USB device is
           accessed, it responds to the address 0. We need to change the address
           to a free device address between 1 and 127 ASAP.  */
        status =  _ux_host_stack_device_address_set(device);
        if (status == UX_SUCCESS)
          return true;
    }
  }

  return false;
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

  printf("Called %s(%u %u %u %p %u)\n", __FUNCTION__, rhport, dev_addr, ep_addr, buffer, buflen);

  return false;
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

  printf("Called %s(%u %u %p)\n\r", __FUNCTION__, rhport, dev_addr, setup_packet);
  
  // Retrieve the pointer to the control endpoint.
  UX_DEVICE       *device = _created_device; 
  UX_ENDPOINT     *control_endpoint =  &device -> ux_device_control_endpoint;
  UX_TRANSFER     *transfer_request =  &control_endpoint -> ux_endpoint_transfer_request;

  // Need to allocate memory for the descriptor
  unsigned char * descriptor =  _ux_utility_memory_allocate(UX_SAFE_ALIGN, UX_CACHE_SAFE_MEMORY,
                                UX_DEVICE_DESCRIPTOR_LENGTH);
  if (descriptor == UX_NULL)
    return(UX_MEMORY_INSUFFICIENT);

  // Create a transfer_request for the GET_DESCRIPTOR request. The first transfer_request asks 
  // for the first 8 bytes only. This way we will know the real MaxPacketSize
  // value for the control endpoint.
  transfer_request -> ux_transfer_request_data_pointer =      descriptor;
  transfer_request -> ux_transfer_request_requested_length =  8;
  transfer_request -> ux_transfer_request_function =          UX_GET_DESCRIPTOR;
  transfer_request -> ux_transfer_request_type =              UX_REQUEST_IN | UX_REQUEST_TYPE_STANDARD | UX_REQUEST_TARGET_DEVICE;
  transfer_request -> ux_transfer_request_value =             UX_DEVICE_DESCRIPTOR_ITEM << 8;
  transfer_request -> ux_transfer_request_index =             0;

  // Send request to HCD layer.
  //unsigned int status =  _ux_host_stack_transfer_request(transfer_request);
  // We can only transfer when the device is ATTACHED, ADDRESSED OR CONFIGURED.
  if ((device -> ux_device_state == UX_DEVICE_ATTACHED) || (device -> ux_device_state == UX_DEVICE_ADDRESSED)
          || (device -> ux_device_state == UX_DEVICE_CONFIGURED))
  {
    // Set the transfer to pending.
    transfer_request -> ux_transfer_request_completion_code =  UX_TRANSFER_STATUS_COMPLETED;//UX_TRANSFER_STATUS_PENDING;

    // Pointer to the HCD.
    UX_HCD * hcd = hcd_xhci -> ux_hcd_xhci_hcd_owner;
    // Send the command to the controller.
    unsigned int status =  hcd -> ux_hcd_entry_function(hcd, UX_HCD_TRANSFER_REQUEST, transfer_request);

    // Check for correct transfer and entire descriptor returned.
    if ((status == UX_SUCCESS) && (transfer_request -> ux_transfer_request_actual_length == 8)) {
      // Print descriptor
      printf("%2.x %2.x %2.x %2.x %2.x %2.x %2.x %2.x",
      descriptor[0],descriptor[1],descriptor[2],descriptor[3],
      descriptor[4],descriptor[5],descriptor[6],descriptor[7]);

      return true;
    }
  }

  // Free all used resources.
  _ux_utility_memory_free(descriptor);

  return false;
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
