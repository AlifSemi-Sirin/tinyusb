/* Copyright (C) 2023 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */
/**************************************************************************//**
 * @file     ux_hcd_xhci_api.h
 * @author   Mahesh Avula
 * @email    mahesh.avula@alifsemi.com
 * @version  V1.0.0
 * @date     12-Dec-2023
 * @brief    header file for xhci driver
 * @bug      None.
 * @Note     None
 ******************************************************************************/

#ifndef _UX_HCD_XHCI_API_H_
#define _UX_HCD_XHCI_API_H_

#include "ux_hcd_xhci.h"
#include "system_utils.h"

#ifdef   __cplusplus

/* Yes, C++ compiler is present.  Use standard C.  */
extern   "C" {

#endif

/* Define XHCI function prototypes.  */

uint32_t _ux_hcd_xhci_mem_init(UX_HCD_XHCI *pHcd_xhci);
void    _ux_hcd_xhci_endpoint_reset(UX_HCD_XHCI *hcd_xhci, UX_ENDPOINT *endpoint);
uint32_t    _ux_hcd_xhci_entry(UX_HCD *hcd, uint32_t function, void *parameter);
uint32_t  _ux_hcd_xhci_frame_number_get(UX_HCD_XHCI *xhci, uint32_t *frame_number);
void  _ux_hcd_xhci_frame_number_set(UX_HCD_XHCI *xhci, uint32_t frame_number);
uint32_t    _ux_hcd_xhci_initialize(UX_HCD *hcd);
void    _ux_hcd_xhci_interrupt_handler(void);
uint32_t   _ux_hcd_xhci_port_status_get(UX_HCD_XHCI *hcd_xhci, uint32_t port_index);
uint32_t  _ux_hcd_xhci_port_enable(UX_HCD_XHCI *xhci, uint32_t port_index);
uint32_t _ux_hcd_xhci_halt(UX_HCD_XHCI *xhci);
void _ux_hcd_xhci_hc_died(UX_HCD_XHCI *xhci);
uint32_t _ux_hcd_xhci_reset(UX_HCD_XHCI *xhci);
UX_XHCI_SEGMENT *trb_in_td(
   UX_HCD_XHCI   *xhci,
   UX_XHCI_SEGMENT  *start_seg,
   UX_XHCI_TRB  *start_trb,
   UX_XHCI_TRB  *end_trb,
   uint64_t  suspect_dma,
   bool debug);

UX_XHCI_RING *_ux_hcd_xhci_triad_to_transfer_ring(
   UX_HCD_XHCI *xhci,
   uint32_t slot_id,
   uint32_t  ep_index,
   uint32_t stream_id);
void _ux_hcd_xhci_queue_new_dequeue_state(
   UX_HCD_XHCI *xhci,
   uint32_t slot_id,
   uint32_t ep_index,
   UX_XHCI_DEQUEUE_STATE *deq_state);

int32_t _ux_hcd_xhci_check_maxpacket(UX_HCD_XHCI *xhci,uint32_t slot_id,uint32_t ep_index,
                                          UX_TRANSFER *urb);


uint32_t _ux_hcd_xhci_get_endpoint_index(UX_ENDPOINT_DESCRIPTOR *desc);
void _ux_hcd_xhci_cleanup_command_queue(UX_HCD_XHCI *xhci);

UX_XHCI_RING *xhci_stream_id_to_ring(
   UX_XHCI_VIRT_DEVICE *dev,
   uint32_t ep_index,
   uint32_t stream_id);

int32_t finish_td(
   UX_HCD_XHCI *xhci,
   UX_XHCI_TD *td,
   UX_XHCI_TRANSFER_EVENT *event,
   UX_XHCI_VIRT_EP *ep,
   int32_t *status);

int32_t process_isoc_td(
   UX_HCD_XHCI *xhci,
   UX_XHCI_TD *td,
   UX_XHCI_TRB *ep_trb,
   UX_XHCI_TRANSFER_EVENT *event,
   UX_XHCI_VIRT_EP *ep,
   int32_t *status);

int32_t process_bulk_intr_td(
   UX_HCD_XHCI *xhci,
   UX_XHCI_TD *td,
   UX_XHCI_TRB *ep_trb,
   UX_XHCI_TRANSFER_EVENT *event,
   UX_XHCI_VIRT_EP *ep,
   int32_t *status);

int32_t process_ctrl_td(
   UX_HCD_XHCI *xhci,
   UX_XHCI_TD *td,
   UX_XHCI_TRB *ep_trb,
   UX_XHCI_TRANSFER_EVENT *event,
   UX_XHCI_VIRT_EP *ep,
   int32_t *status);

int32_t handle_transfer_event(UX_HCD_XHCI *xhci, UX_XHCI_TRANSFER_EVENT *event);
void handle_cmd_completion(UX_HCD_XHCI *xhci, UX_XHCI_EVENT_CMD *event);
void handle_port_status(UX_HCD_XHCI *xhci, UX_XHCI_TRB *event);
void _ux_hcd_xhci_update_erst_dequeue(
   UX_HCD_XHCI *xhci,
   UX_XHCI_TRB *event_ring_deq);
int32_t _ux_hcd_xhci_handle_events(UX_HCD_XHCI *xhci);
void handle_vendor_event(
   UX_HCD_XHCI *xhci,
   UX_XHCI_TRB *event);
void handle_device_notification(
   UX_HCD_XHCI *xhci,
   UX_XHCI_TRB *event);
void _ux_hcd_xhci_urb_free_priv(UX_URB_PRIV *urb_priv);
void inc_deq(UX_HCD_XHCI *xhci, UX_XHCI_RING *ring);
bool trb_is_link(UX_XHCI_TRB *trb);
bool trb_is_noop(UX_XHCI_TRB *trb);

int32_t _ux_hcd_xhci_align_td(
   UX_HCD_XHCI *xhci,
   UX_TRANSFER *urb,
   uint32_t enqd_len,
   uint32_t *trb_buff_len,
   UX_XHCI_SEGMENT *seg);

int32_t _ux_hcd_xhci_transfer_request(UX_HCD_XHCI *xhci, UX_TRANSFER *urb);
int32_t _ux_hcd_xhci_check_bandwidth( UX_HCD_XHCI *xhci, UX_DEVICE *udev);

UX_XHCI_COMMAND *_ux_hcd_xhci_alloc_command_with_ctx_sz(UX_HCD_XHCI *xhci);

void _ux_xhci_event_irq_handler(UX_HCD_XHCI *xhci);

int32_t _ux_hcd_xhci_is_vendor_info_code(UX_HCD_XHCI *xhci, uint32_t trb_comp_code);
int32_t skip_isoc_td(UX_HCD_XHCI *xhci, UX_XHCI_TD *td, UX_XHCI_TRANSFER_EVENT *event, UX_XHCI_VIRT_EP *ep, int32_t *status);


void _ux_hcd_xhci_cleanup_halted_endpoint(
   UX_HCD_XHCI *xhci,
   uint32_t slot_id,
   uint32_t ep_index,
   uint32_t stream_id,
   UX_XHCI_TD *td,
   UX_XHCI_EP_RESET_TYPE reset_type);
int32_t sum_trb_lengths(
   UX_HCD_XHCI *xhci,
   UX_XHCI_RING *ring,
   UX_XHCI_TRB *stop_trb);

void td_to_noop(
  UX_HCD_XHCI *xhci,
  UX_XHCI_RING *ep_ring,
  UX_XHCI_TD *td,
  bool flip_cycle);

void giveback_first_trb(
   UX_HCD_XHCI *xhci,
   int32_t slot_id,
   uint32_t ep_index,
   uint32_t stream_id,
   uint32_t start_cycle,
   UX_XHCI_GENERIC_TRB *start_trb);

int32_t _ux_hcd_xhci_find_next_ext_cap(
   UX_HCD_XHCI  *xhci , uint32_t start, int32_t id);

void queue_trb(
   UX_HCD_XHCI *xhci,
   UX_XHCI_RING *ring,
   bool more_trbs_coming,
   UX_XHCI_TRB_INFO *trb_info);

uint32_t _ux_hcd_xhci_td_remainder(UX_HCD_XHCI *xhci, int32_t transferred,
         int32_t trb_buff_len, uint32_t td_total_len, UX_TRANSFER *urb,bool more_trbs_coming);

int32_t prepare_ring(UX_HCD_XHCI *xhci, UX_XHCI_RING *ep_ring, uint32_t ep_state, uint32_t num_trbs);
void check_interval(UX_HCD_XHCI *xhci,UX_TRANSFER *urb,UX_XHCI_EP_CTX *ep_ctx);
int32_t _ux_hcd_xhci_requires_manual_halt_cleanup(
   UX_HCD_XHCI *xhci,
   UX_XHCI_EP_CTX *ep_ctx,
   uint32_t trb_comp_code);

UX_XHCI_RING *_ux_hcd_xhci_stream_id_to_ring(UX_XHCI_VIRT_DEVICE *dev,uint32_t ep_index, uint32_t stream_id);
/* Queue a configure endpoint command TRB */
int32_t _ux_hcd_xhci_queue_configure_endpoint(
   UX_HCD_XHCI *xhci,
   UX_XHCI_COMMAND *cmd,
   uint64_t in_ctx_ptr,
   uint32_t slot_id,
   bool command_must_succeed);

void _ux_hcd_xhci_free_device_endpoint_resources(
   UX_HCD_XHCI *xhci,
   UX_XHCI_VIRT_DEVICE *virt_dev,
   bool drop_control_ep);

void _ux_hcd_xhci_free_dev(UX_HCD_XHCI *xhci, UX_DEVICE *udev);

uint64_t _ux_hcd_xhci_trb_virt_to_dma(UX_XHCI_SEGMENT *seg, UX_XHCI_TRB *trb);

UX_XHCI_COMMAND *xhci_alloc_command(UX_HCD_XHCI *xhci, bool allocate_completion);

UX_XHCI_INPUT_CONTROL_CTX *xhci_get_input_control_ctx(UX_XHCI_CONTAINER_CTX *ctx);

int32_t _ux_hcd_xhci_configure_endpoint(
   UX_HCD_XHCI *xhci,
   UX_DEVICE *udev,
   UX_XHCI_COMMAND *command,
   bool ctx_change,
   bool must_succeed);

void _ux_hcd_xhci_update_tt_active_eps(
   UX_HCD_XHCI *xhci,
   UX_XHCI_VIRT_DEVICE *virt_dev,
   int32_t old_active_eps);

void _ux_hcd_xhci_free_virt_device(
   UX_HCD_XHCI *xhci, uint32_t slot_id);

int32_t _ux_hcd_xhci_queue_reset_ep(UX_HCD_XHCI *xhci, UX_XHCI_COMMAND *cmd, uint32_t slot_id,
       uint32_t ep_index, UX_XHCI_EP_RESET_TYPE reset_type);

void _ux_hcd_xhci_cleanup_stalled_ring(
   UX_HCD_XHCI *xhci,
   uint32_t ep_index,
   uint32_t stream_id,
   UX_XHCI_TD *td);

void _ux_hcd_xhci_free_command(
   UX_HCD_XHCI *xhci,
   UX_XHCI_COMMAND *command);

void _ux_hcd_xhci_slot_copy(
   UX_HCD_XHCI *xhci,
   UX_XHCI_CONTAINER_CTX *in_ctx,
   UX_XHCI_CONTAINER_CTX *out_ctx);

int32_t queue_command(UX_HCD_XHCI *xhci, UX_XHCI_COMMAND *cmd, UX_XHCI_TRB_INFO *trb_info,
                        bool command_must_succeed);

uint32_t _ux_hcd_xhci_get_endpoint_flag_from_index(uint32_t ep_index);

uint32_t _ux_hcd_xhci_get_endpoint_flag(UX_ENDPOINT_DESCRIPTOR *desc);

void _ux_hcd_xhci_endpoint_copy(
   UX_HCD_XHCI *xhci,
   UX_XHCI_CONTAINER_CTX *in_ctx,
   UX_XHCI_CONTAINER_CTX *out_ctx,
   uint32_t ep_index);

uint64_t _ux_hcd_xhci_trb_virt_to_dma(UX_XHCI_SEGMENT *seg, UX_XHCI_TRB *trb);

void _ux_hcd_xhci_ring_device(UX_HCD_XHCI *xhci, uint32_t slot_id);

void _ux_hcd_xhci_test_and_clear_bit(UX_HCD_XHCI *xhci, UX_XHCI_PORT *port, uint32_t port_bit);

void _ux_hcd_xhci_ring_ep_doorbell(
   UX_HCD_XHCI *xhci,
   uint32_t slot_id,
   uint32_t ep_index,
   uint32_t stream_id);

int32_t _ux_hcd_xhci_ring_expansion(
   UX_HCD_XHCI *xhci,
   UX_XHCI_RING *ring,
   uint32_t num_trbs);

void _ux_hcd_xhci_set_link_state(UX_HCD_XHCI *xhci, UX_XHCI_PORT *port, uint32_t link_state);

/*int _ux_hcd_xhci_queue_ctrl_tx(UX_HCD_XHCI *xhci, UX_TRANSFER *urb, int slot_id,
              unsigned int ep_index);*/

int32_t _ux_hcd_xhci_endpoint_init(UX_HCD_XHCI *xhci, UX_XHCI_VIRT_DEVICE *virt_dev, UX_DEVICE *udev, UX_ENDPOINT *ep);

int32_t _ux_hcd_xhci_add_endpoint(UX_HCD_XHCI *xhci, UX_DEVICE *udev, UX_ENDPOINT *ep);

UX_XHCI_SLOT_CTX *xhci_get_slot_ctx(UX_HCD_XHCI *xhci, UX_XHCI_CONTAINER_CTX *ctx);

int32_t _ux_hcd_xhci_alloc_virt_device(UX_HCD_XHCI *xhci,uint32_t slot_id,UX_DEVICE *udev);

int32_t _ux_hcd_xhci_alloc_dev(UX_HCD_XHCI *xhci, UX_DEVICE *udev);

int32_t _ux_hcd_xhci_address_device(UX_HCD_XHCI *xhci, UX_DEVICE *udev);

int32_t _ux_hcd_xhci_enable_device(UX_HCD_XHCI *xhci, UX_DEVICE *udev);

void _ux_hcd_xhci_set_port_power(UX_HCD_XHCI *xhci,  bool ON);

uint32_t _ux_hcd_xhci_disable_port(UX_HCD_XHCI *xhci, uint32_t port_index);

uint32_t _ux_hcd_xhci_reset_port(UX_HCD_XHCI *xhci, uint32_t port_index);

void _ux_hcd_xhci_clear_port_change_bit(
   UX_HCD_XHCI *xhci,
   uint16_t wValue,
   uint16_t wIndex,
   uint32_t port_status);

int32_t _ux_hcd_xhci_reset_device(UX_HCD_XHCI *xhci, UX_DEVICE *udev);

void _ux_hcd_xhci_free_endpoint_ring(UX_HCD_XHCI *xhci, UX_XHCI_VIRT_DEVICE *virt_dev, uint32_t ep_index);

void xhci_endpoint_reset(UX_HCD_XHCI *xhci, UX_ENDPOINT *host_ep);

UX_XHCI_INPUT_CONTROL_CTX *
_ux_hcd_xhci_get_input_control_ctx(UX_XHCI_CONTAINER_CTX *ctx);

UX_XHCI_SLOT_CTX *_ux_hcd_xhci_get_slot_ctx(UX_HCD_XHCI *xhci, UX_XHCI_CONTAINER_CTX *ctx);

void _ux_hcd_xhci_setup_input_ctx_for_config_ep(UX_HCD_XHCI *xhci, UX_XHCI_CONTAINER_CTX *in_ctx,
                        UX_XHCI_CONTAINER_CTX *out_ctx, UX_XHCI_INPUT_CONTROL_CTX *ctrl_ctx, uint32_t add_flags,
                        uint32_t drop_flags);

UX_XHCI_EP_CTX *_ux_hcd_xhci_get_ep_ctx(UX_HCD_XHCI *xhci, UX_XHCI_CONTAINER_CTX *ctx,
                            uint32_t ep_index);

UX_XHCI_COMMAND *_ux_hcd_xhci_alloc_command(UX_HCD_XHCI *xhci);

UX_XHCI_COMMAND *_ux_hcd_xhci_alloc_command_with_ctx(UX_HCD_XHCI *xhci, bool allocate_completion);

void _ux_hcd_xhci_ring_cmd_db(UX_HCD_XHCI *xhci);

void _ux_hcd_xhci_setup_input_ctx_for_config_ep(
   UX_HCD_XHCI *xhci,
   UX_XHCI_CONTAINER_CTX *in_ctx,
   UX_XHCI_CONTAINER_CTX *out_ctx,
   UX_XHCI_INPUT_CONTROL_CTX *ctrl_ctx,
   uint32_t add_flags,
   uint32_t drop_flags);

bool  _ux_hcd_xhci_port_current_status_get(UX_HCD_XHCI *xhci, uint32_t port_index);
int32_t _ux_hcd_xhci_bulk_transfer_request(UX_HCD_XHCI *xhci, UX_TRANSFER *urb, uint32_t slot_id,
                  uint32_t ep_index);

int32_t _ux_hcd_xhci_isoc_transfer_request(UX_HCD_XHCI *xhci, UX_TRANSFER *urb,uint32_t slot_id,
                          uint32_t ep_index);

int32_t _ux_hcd_xhci_control_transfer_request(UX_HCD_XHCI *xhci, UX_TRANSFER *urb,int32_t slot_id,
                              uint32_t ep_index);

int32_t _ux_hcd_xhci_interrupt_transfer_request(UX_HCD_XHCI *xhci, UX_TRANSFER *urb, uint32_t slot_id,
                  uint32_t ep_index);

int32_t prepare_transfer(UX_HCD_XHCI *xhci, UX_XHCI_VIRT_DEVICE *xdev,
     uint32_t ep_index, uint32_t stream_id, uint32_t num_trbs, UX_TRANSFER *urb, uint32_t td_index);


int32_t xhci_isoc_transfer_queue(UX_HCD_XHCI *xhci, UX_TRANSFER *urb,uint32_t slot_id, uint32_t ep_index);
uint32_t _ux_hcd_xhci_get_burst_count(UX_HCD_XHCI *xhci, uint32_t total_packet_count);
uint32_t _ux_hcd_xhci_get_last_burst_packet_count(UX_HCD_XHCI *xhci, uint32_t total_packet_count);
int32_t xhci_get_isoc_frame_id(UX_HCD_XHCI *xhci, UX_TRANSFER *urb, int32_t index);
uint32_t count_isoc_trbs(UX_TRANSFER *urb, int32_t i);
uint32_t count_trbs_needed(UX_TRANSFER *urb);
uint32_t count_trbs(uint64_t addr, uint32_t len);
uint32_t xhci_get_max_esit_payload(UX_DEVICE *udev,UX_ENDPOINT *ep);

int32_t _ux_hcd_xhci_queue_reset_device(
          UX_HCD_XHCI *xhci, UX_XHCI_COMMAND *cmd, uint32_t slot_id);

static inline int32_t ux_endpoint_maxp_mult(const UX_ENDPOINT_DESCRIPTOR *epd)
{
   return (((epd->wMaxPacketSize) & UX_MAX_NUMBER_OF_TRANSACTIONS_MASK)>> UX_MAX_NUMBER_OF_TRANSACTIONS_SHIFT) + 1;
}

static inline int32_t ux_endpoint_dir_in(const UX_ENDPOINT_DESCRIPTOR *epd)
{
   return ((epd->bEndpointAddress & UX_ENDPOINT_DIRECTION) == UX_ENDPOINT_IN);
}

static inline int32_t ux_endpoint_dir_out(const UX_ENDPOINT_DESCRIPTOR *epd)
{
   return ((epd->bEndpointAddress & UX_ENDPOINT_DIRECTION) == UX_ENDPOINT_OUT);
}

static inline int32_t ux_endpoint_is_bulk_out(const UX_ENDPOINT_DESCRIPTOR *epd)
{
   return ux_endpoint_xfer_bulk(epd) && ux_endpoint_dir_out(epd);
}

static inline UX_XHCI_RING *_ux_hcd_xhci_urb_to_transfer_ring(
   UX_HCD_XHCI *xhci, UX_TRANSFER *urb)
{
   return _ux_hcd_xhci_triad_to_transfer_ring(xhci,
                    1 ,
                _ux_hcd_xhci_get_endpoint_index(&urb->ux_transfer_request_endpoint->ux_endpoint_descriptor),
                    0);
}

static inline bool _ux_hcd_xhci_urb_suitable_for_idt(UX_TRANSFER *urb)
{
   if ((!((urb->ux_transfer_request_endpoint->ux_endpoint_descriptor.bmAttributes.xfer) == UX_ISOCHRONOUS_ENDPOINT)) &&
      ((urb->ux_transfer_request_type & UX_REQUEST_DIRECTION) == UX_REQUEST_OUT) &&
         (((urb->ux_transfer_request_endpoint->ux_endpoint_descriptor.wMaxPacketSize) & UX_MAX_PACKET_SIZE_MASK) >= TRB_IDT_MAX_SIZE) &&
         urb->ux_transfer_request_requested_length <= TRB_IDT_MAX_SIZE)
          return true;

   return false;
}

/**
 * fls - find last (most-significant) bit set
 * @x: the word to search
 *
 * This is defined the same way as ffs.
 * Note fls(0) = 0, fls(1) = 1, fls(0x80000000) = 32.
 */

static inline int32_t ux_fls(uint32_t x)
{
   int32_t r = 32;
   if (!x)
       return 0;
   if (!(x & 0xffff0000u))
   {
      x <<= 16;
      r -= 16;
   }
   if (!(x & 0xff000000u))
   {
      x <<= 8;
      r -= 8;
   }
   if (!(x & 0xf0000000u))
   {
      x <<= 4;
      r -= 4;
   }
   if (!(x & 0xc0000000u))
   {
      x <<= 2;
      r -= 2;
   }
   if (!(x & 0x80000000u))
   {
      x <<= 1;
      r -= 1;
   }
   return r;
}
static uint32_t inline _ux_hcd_xhci_last_valid_endpoint(uint32_t added_ctxs)
{
   return ux_fls(added_ctxs) - 1;
}

#ifdef  __cplusplus
}
#endif
#endif /* end of _UX_HCD_XHCI_API_h_ */
