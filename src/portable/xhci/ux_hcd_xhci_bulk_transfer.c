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
 * @file     _ux_hcd_xhci_bulk_transfer.c
 * @author   Mahesh Avula
 * @email    mahesh.avula@alifsemi.com
 * @version  V1.0.0
 * @date     12-Dec-2023
 * @brief    XHCI host bulk transfers.
 * @bug      None
 * @Note     None
 ******************************************************************************/


/* Include necessary system files.  */

#include "tusb_option.h"

#if CFG_TUH_ENABLED && CFG_TUSB_MCU == OPT_MCU_NONE

#include "host/hcd.h"
#include "ux_hcd_xhci_api.h"
//#include "ux_host_stack.h"

/**
  \fn           _ux_hcd_xhci_bulk_transfer_request
  \brief        xhci driver bulk transfers
  \param[in]    xhci global pointer
  \param[in]    urb pointer
  \param[in]    slot id
  \param[in]    endpoint index
  \return       On success 0 remains error
 */
int32_t _ux_hcd_xhci_bulk_transfer_request(UX_HCD_XHCI *xhci, UX_TRANSFER *urb, uint32_t slot_id,
        uint32_t ep_index)
{
    UX_XHCI_RING *ring;
    UX_URB_PRIV *urb_priv;
    UX_XHCI_TD *td;
    UX_XHCI_TRB_INFO  trb_info;
    UX_XHCI_GENERIC_TRB *start_trb;
    bool more_trbs_coming = true;
    bool need_zero_pkt = false;
    bool first_trb = true;
    uint32_t num_trbs;
    uint32_t start_cycle;
    uint32_t enqd_len, block_len, trb_buff_len, full_len;
    int32_t sent_len, ret;
    uint32_t field, length_field, remainder;
    uint64_t addr, send_addr;
    UX_XHCI_VIRT_EP *ep;
    ep = &xhci->devs[slot_id]->eps[ep_index];
    ring = ep->ring;
    if (!ring)
    {
#ifdef DEBUG
        printf("Invalid ring\n");
#endif
        return -1;
    }
    full_len = urb->ux_transfer_request_requested_length;
    num_trbs = count_trbs_needed(urb);
    addr = (uint64_t) urb->ux_transfer_request_data_pointer;
    block_len = full_len;
    ret = prepare_transfer(xhci, xhci->devs[slot_id], ep_index, xhci->stream_id,
            num_trbs, urb, 0);
    if (ret < 0)
        return ret;
    urb_priv = urb->hcpriv;
    td = &urb_priv->td[0];
    /*
     * Don't give the first TRB to the hardware (by toggling the cycle bit)
     * until we've finished creating all the other TRBs.  The ring's cycle
     * state may change as we enqueue the other TRBs, so save it too.
     */
    start_trb = &ring->enqueue->generic;
    start_cycle = ring->cycle_state;
    send_addr = addr;
    /* Queue the TRBs, even if they are zero-length */
    for (enqd_len = 0; first_trb || enqd_len < full_len; enqd_len += trb_buff_len)
    {
        field = TRB_TYPE(TRB_NORMAL);
        /* TRB buffer should not cross 64KB boundaries */
        trb_buff_len = TRB_BUFF_LEN_UP_TO_BOUNDARY(addr);
        trb_buff_len = (uint32_t) trb_buff_len < (uint32_t)block_len ?
            (uint32_t) trb_buff_len: (uint32_t)block_len;

        if (enqd_len + trb_buff_len > full_len)
            trb_buff_len = full_len - enqd_len;

        /* Don't change the cycle bit of the first TRB until later */
        if (first_trb)
        {
            first_trb = false;
            if (start_cycle == 0)
                field |= TRB_CYCLE;
        }
        else
        {
            field |= ring->cycle_state;
        }

        /* Chain all the TRBs together; clear the chain bit in the last
         * TRB to indicate it's the last TRB in the chain.
         */
        if (enqd_len + trb_buff_len < full_len)
        {
            field |= TRB_CHAIN;
            if (trb_is_link(ring->enqueue + 1))
            {
                if (_ux_hcd_xhci_align_td(xhci, urb, enqd_len, &trb_buff_len, ring->enq_seg))
                {
                    send_addr = ring->enq_seg->bounce_dma;
                    /* assuming TD won't span 2 segs */
                    td->bounce_seg = ring->enq_seg;
                }
            }
        }
        if (enqd_len + trb_buff_len >= full_len)
        {
            field &= ~TRB_CHAIN;
            field |= TRB_IOC;
            more_trbs_coming = false;
            td->last_trb = ring->enqueue;
            if (_ux_hcd_xhci_urb_suitable_for_idt(urb))
            {
                field |= TRB_IDT;
            }
        }
        /* Only set interrupt on short packet for IN endpoints */
        if ((urb -> ux_transfer_request_type & UX_REQUEST_DIRECTION ) == UX_REQUEST_IN)
            field |= TRB_ISP;

        /* Set the TRB length, TD size, and interrupter fields. */
        remainder = _ux_hcd_xhci_td_remainder(xhci, enqd_len, trb_buff_len, full_len, urb, more_trbs_coming);
        length_field = TRB_LEN(trb_buff_len) | TRB_TD_SIZE(remainder) | TRB_INTR_TARGET(0);
        RTSS_CleanDCache_by_Addr((void *)send_addr, urb->ux_transfer_request_requested_length);
        trb_info.low_address = LocalToGlobal((void *)lower_32_bits(send_addr));
        trb_info.high_address = LocalToGlobal((void *)upper_32_bits(send_addr));
        trb_info.size =  length_field;
        trb_info.cntrl_field = field;
        queue_trb(xhci, ring, more_trbs_coming | need_zero_pkt, &trb_info);
        sent_len = trb_buff_len;
        block_len -= sent_len;
        send_addr = addr;
    }
    if (need_zero_pkt)
    {
        /* prepare the transfers  */
        ret = prepare_transfer(xhci, xhci->devs[slot_id],ep_index, xhci->stream_id, 1, urb, 1);
        urb_priv->td[1].last_trb = ring->enqueue;
        field = TRB_TYPE(TRB_NORMAL) | ring->cycle_state | TRB_IOC;
        trb_info.low_address = 0;
        trb_info.high_address = 0;
        trb_info.size =  TRB_INTR_TARGET(0);
        trb_info.cntrl_field = field;
        /* Queue the TRB  */
        queue_trb(xhci, ring, 0, &trb_info);
    }
    if(enqd_len != urb->ux_transfer_request_requested_length)
    {
#ifdef DEBUG
        printf("length mismatch\n");
#endif
    }
    /* Give the TRB to the endpoint doorbell */
    giveback_first_trb(xhci, slot_id, ep_index, xhci->stream_id, start_cycle, start_trb);
    return 0;
}
/**
  \fn           count_trbs_needed
  \brief        Calculate the number of TRB's .
  \param[in]    urb pointer
  \return       Number of TRB's
 */
uint32_t count_trbs_needed(UX_TRANSFER *urb)
{
    return count_trbs((uint64_t)urb->ux_transfer_request_data_pointer, urb->ux_transfer_request_requested_length);
}

/**
  \fn           _ux_hcd_xhci_align_td
  \brief        xhci driver bulk endpoint TD packet aligned or not.
  \param[in]    xhci global pointer
  \param[in]    urb pointer
  \param[in]    enqueued length
  \param[in]    TRB buff length
  \param[in]    Segment pointer
  \return       On success 0 remains error
 */

int32_t _ux_hcd_xhci_align_td(
        UX_HCD_XHCI *xhci,
        UX_TRANSFER *urb,
        uint32_t enqd_len,
        uint32_t *trb_buff_len,
        UX_XHCI_SEGMENT *seg)
{

    uint32_t unalign;
    uint32_t max_pkt;
    uint32_t new_buff_len;
    max_pkt = ux_endpoint_maxp(&urb->ux_transfer_request_endpoint->ux_endpoint_descriptor);
    unalign = (enqd_len + *trb_buff_len) % max_pkt;

    /* we got lucky, last normal TRB data on segment is packet aligned */
    if (unalign == 0)
        return 0;
    /* is the last nornal TRB alignable by splitting it */
    if (*trb_buff_len > unalign)
    {
       *trb_buff_len -= unalign;
        return 0;
    }
    return 1;
}

#endif //CFG_TUH_ENABLED
