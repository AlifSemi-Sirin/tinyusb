/* Copyright (C) 2024 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

/**************************************************************************//**
 * @file     ux_hcd_xhci_isoc_transfer.c
 * @author   Mahesh Avula
 * @email    mahesh.avula@alifsemi.com
 * @version  V1.0.0
 * @date     12-Dec-2023
 * @brief    XHCI host Isochronous transfers.
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
  \fn           _ux_hcd_xhci_isoc_transfer_request
  \brief        xhci driver Isochronous transfers
  \param[in]    xhci global pointer
  \param[in]    urb pointer
  \param[in]    slot id
  \param[in]    endpoint index
  \return       On success 0 remains error
 */
int32_t _ux_hcd_xhci_isoc_transfer_request(UX_HCD_XHCI *xhci, UX_TRANSFER *urb,uint32_t slot_id,
        uint32_t ep_index)
{
    UX_XHCI_RING *ep_ring;
    UX_XHCI_VIRT_DEVICE *devs;
    UX_XHCI_VIRT_EP *ep;
    UX_XHCI_EP_CTX *ep_ctx;
    uint32_t num_tds, num_trbs, i;
    int32_t ret;
    int32_t start_frame;
    int32_t ist;
    devs = xhci-> devs[slot_id];
    ep = &xhci->devs[slot_id]->eps[ep_index];
    ep_ring = devs->eps[ep_index].ring;
    ep_ctx = _ux_hcd_xhci_get_ep_ctx(xhci, xhci->devs[slot_id]->out_ctx, ep_index);
    num_trbs = 0;
    num_tds = ((UX_URB_PRIV *)(urb->hcpriv))->num_tds;

    for (i = 0; i < num_tds; i++)
        num_trbs += count_isoc_trbs(urb, i);

    /* Check the ring to guarantee there is enough room for the whole urb.
     * Do not insert any td of the urb to the ring if the check failed.
     */
    ret = prepare_ring(xhci, ep_ring, GET_EP_CTX_STATE(ep_ctx), num_trbs);
    if (ret)
    {
#ifdef DEBUG
        printf("There is no enough room on ring\n");
#endif
        return ret;
    }
    /* * Check interval value. This should be done before we start to
     * calculate the start frame value. */
    check_interval(xhci, urb, ep_ctx);

    /* Calculate the start frame and put it in xhci->start_frame. */
    if (HCC_CFC(xhci->hcc_params1) && !_ux_hcd_xhci_list_empty(&ep_ring->td_list))
    {
        if (GET_EP_CTX_STATE(ep_ctx) == EP_STATE_RUNNING)
        {
            xhci->start_frame = ep->next_frame_id;
            goto skip_start_over;
        }
    }
    /* Read the Microframe Index Register   */
    start_frame = xhci->run_regs->MFINDEX;
    /* check the Microframe Index bits from 0 to 13   */
    start_frame &= 0x3fff;
    /*
     * Round up to the next frame and consider the time before trb really
     * gets scheduled by hardare.
     */
    /* Read the bit [3] of Isochronous schedule threshold  */
    ist = HCS_IST(xhci->hcs_params2) & 0x7;

    /* if bit [3] of IST Set/ cleared   */
    if (HCS_IST(xhci->hcs_params2) & (1 << 3))
    {
        ist <<= 3;
    }
    start_frame += ist + XHCI_CFC_DELAY;
    start_frame = ROUNDUP(start_frame, 8);

    /*
     * Round up to the next ESIT (Endpoint Service Interval Time) if ESIT
     * is greate than 8 microframes. */
    if(xhci->device->ux_device_speed == UX_LOW_SPEED_DEVICE ||
            xhci->device->ux_device_speed == UX_FULL_SPEED_DEVICE)
    {
        start_frame = ROUNDUP(start_frame, (xhci->interval << 3));
        xhci->start_frame = start_frame >> 3;
    }
    else
    {
        start_frame = ROUNDUP(start_frame, xhci->interval);
        xhci->start_frame = start_frame;
    }
skip_start_over :
    ep_ring->num_trbs_free_temp = ep_ring->num_trbs_free;

    return xhci_isoc_transfer_queue(xhci, urb, slot_id, ep_index);
}

/**
  \fn           xhci_isoc_transfer_queue
  \brief        xhci driver Isochronous transfer queuing
  \param[in]    xhci global pointer
  \param[in]    urb pointer
  \param[in]    slot id
  \param[in]    endpoint index
  \return       On success 0 remains error
 */
int32_t xhci_isoc_transfer_queue(UX_HCD_XHCI *xhci, UX_TRANSFER *urb, uint32_t slot_id, uint32_t ep_index)
{
    UX_XHCI_RING *ep_ring;
    UX_URB_PRIV *urb_priv;
    UX_XHCI_TD *td;
    UX_XHCI_TRB_INFO  trb_info;
    UX_XHCI_GENERIC_TRB *start_trb;
    bool more_trbs_coming = true;
    bool first_trb = true;
    uint32_t start_cycle;
    uint32_t  trb_buff_len;
    int32_t ret,num_tds,trbs_per_td, running_total,i,j,td_len,td_remain_len;
    uint32_t requested = 0;
    uint32_t field, length_field;
    uint64_t start_addr, addr;
    UX_XHCI_VIRT_EP *ep;
    int32_t frame_id;

    ep = &xhci->devs[slot_id]->eps[ep_index];
    ep_ring = xhci->devs[slot_id]->eps[ep_index].ring;
    num_tds =  ((UX_URB_PRIV *)(urb->hcpriv))->num_tds;
    if (num_tds < 1)
    {
#ifdef DEBUG
        printf("Isoc URB with zero packets?\n");
#endif
        return 1;
    }
    start_addr = (uint64_t) urb->ux_transfer_request_data_pointer;
    start_trb = &ep_ring->enqueue->generic;
    start_cycle = ep_ring->cycle_state;
    urb_priv = urb->hcpriv;
    requested = urb->ux_transfer_request_requested_length;
    /* Queue the TRBs for each TD, even if they are zero-length */
    for (i = 0; i < num_tds; i++)
    {
        uint32_t total_pkt_count, max_pkt;
        uint32_t burst_count, last_burst_pkt_count;
        uint32_t sia_frame_id;
        first_trb = true;
        running_total = 0;
        if(requested == 0)
        {
            return -1;
        }
        addr = (start_addr + (i*requested));
        td_len = requested ;
        td_remain_len = td_len;
        max_pkt = ux_endpoint_maxp(&urb->ux_transfer_request_endpoint->ux_endpoint_descriptor);
        total_pkt_count = DIV_ROUND_UP(td_len, max_pkt);
        /* A zero-length transfer still involves at least one packet. */
        if (total_pkt_count == 0)
            total_pkt_count++;
        burst_count = _ux_hcd_xhci_get_burst_count(xhci, total_pkt_count);
        last_burst_pkt_count = _ux_hcd_xhci_get_last_burst_packet_count(xhci, total_pkt_count);
        trbs_per_td = count_isoc_trbs(urb, i);
        ret = prepare_transfer(xhci, xhci->devs[slot_id], ep_index,
                xhci->stream_id, trbs_per_td, urb, i);
        if(ret < 0)
        {
#ifdef DEBUG
            printf("prepare transfer failed\n");
#endif
            if (i == 0)
                return ret;
            goto cleanup;
        }
        td = &urb_priv->td[i];

        /* use SIA as default, if frame id is used overwrite it */
        sia_frame_id = TRB_SIA;

        if (!HCC_CFC(xhci->hcc_params1))
        {
            /* Get the Isoc Frame ID  */
            frame_id = xhci_get_isoc_frame_id(xhci, urb, i);
            if (frame_id >= 0)
                sia_frame_id = TRB_FRAME_ID(frame_id);
        }

        /*
         * Set isoc specific data for the first TRB in a TD.
         * Prevent HW from getting the TRBs by keeping the cycle state
         * inverted in the first TDs isoc TRB.
         */
        field = TRB_TYPE(TRB_ISOC) |
            TRB_TLBPC(last_burst_pkt_count) | sia_frame_id | (i ? ep_ring->cycle_state : !start_cycle);

        /* xhci 1.1 with ETE uses TD_Size field for TBC  */
        if (!ep->use_extended_tbc)
            field |= TRB_TBC(burst_count);
        /* fill the rest of the TRB fields, and remaining normal TRBs */
        for (j = 0; j < trbs_per_td; j++)
        {
            uint32_t remainder = 0;
            /* only first TRB is isoc, overwrite otherwise */
            if (!first_trb)
            {
                field = TRB_TYPE(TRB_NORMAL) | ep_ring->cycle_state;
            }
            /* Only set interrupt on short packet for IN EPs */
            if ((urb -> ux_transfer_request_type & UX_REQUEST_DIRECTION ) == UX_REQUEST_IN)
                field |= TRB_ISP;

            /* Set the chain bit for all except the last TRB  */
            if (j < trbs_per_td - 1)
            {
                more_trbs_coming = true;
                field |= TRB_CHAIN;
            }
            else
            {
                more_trbs_coming = false;
                td->last_trb = ep_ring->enqueue;
                field |= TRB_IOC;
                /* set BEI, except for the last TD */
                if (xhci->hci_version >= 0x100 && i < num_tds - 1)
                    field |= TRB_BEI;
            }

            /* Calculate TRB length */
            /* TRB buffer should not cross 64KB boundaries */
            trb_buff_len = TRB_BUFF_LEN_UP_TO_BOUNDARY(addr);
            if (trb_buff_len > td_remain_len)
            {
                trb_buff_len = td_remain_len;
            }
            /* Set the TRB length, TD size, & interrupter fields. */
            remainder = _ux_hcd_xhci_td_remainder(xhci, running_total,
                    trb_buff_len, td_len,
                    urb, more_trbs_coming);

            length_field = TRB_LEN(trb_buff_len) | TRB_INTR_TARGET(0);
            /* xhci 1.1 with ETE uses TD Size field for TBC */
            if (first_trb && ep->use_extended_tbc)
            {
                length_field |= TRB_TD_SIZE_TBC(burst_count);
            }
            else
            {
                length_field |= TRB_TD_SIZE(remainder);
            }
            first_trb = false;
            RTSS_CleanDCache_by_Addr((void *)addr, urb->ux_transfer_request_requested_length);
            trb_info.low_address = LocalToGlobal((void *)lower_32_bits(addr));
            trb_info.high_address = LocalToGlobal((void *)upper_32_bits(addr));
            trb_info.size =  length_field;
            trb_info.cntrl_field = field;
            /* Queue the Isoc TRB */
            queue_trb(xhci, ep_ring, more_trbs_coming, &trb_info);
            running_total += trb_buff_len;
            addr += trb_buff_len;
            td_remain_len -= trb_buff_len;
        }
        /* Check TD length */
        if (running_total != td_len)
        {
#ifdef DEBUG
            printf("ISOC TD length unmatch\n");
#endif
            ret = -1;
            goto cleanup;
        }
    }

    /* store the next frame id */
    if (HCC_CFC(xhci->hcc_params1))
        ep->next_frame_id = xhci->start_frame + num_tds * xhci->interval;
    /* Give the TRB to endpoint doorbell */
    giveback_first_trb(xhci, slot_id, ep_index, xhci->stream_id, start_cycle, start_trb);
    return 0;

cleanup :
    /* Clean up a partially enqueued isoc transfer. */
    for (i--; i >= 0; i--)
        _ux_hcd_xhci_list_del_init(&urb_priv->td[i].td_list);

    /* Use the first TD as a temporary variable to turn the TDs we've queued
     * into No-ops with a software-owned cycle bit. That way the hardware
     * won't accidentally start executing bogus TDs when we partially
     * overwrite them.  td->first_trb and td->start_seg are already set.
     */
#ifdef DEBUG
    printf("cleanup done in isoc_tx_queue\n");
#endif
    urb_priv->td[0].last_trb = ep_ring->enqueue;
    /* Every TRB except the first & last will have its cycle bit flipped. */
    td_to_noop(xhci, ep_ring, &urb_priv->td[0], true);

    /* Reset the ring enqueue back to the first TRB and its cycle bit. */
    ep_ring->enqueue = urb_priv->td[0].first_trb;
    ep_ring->enq_seg = urb_priv->td[0].start_seg;
    ep_ring->cycle_state = start_cycle;
    ep_ring->num_trbs_free = ep_ring->num_trbs_free_temp;
    return ret;
}
/**
  \fn           count_isoc_trbs
  \brief        xhci driver count Isochronous TRB's
  \param[in]    urb pointer
  \param[in]    Transfer descriptor(TD) index
  \return       number of TRB's
 */
uint32_t count_isoc_trbs(UX_TRANSFER *urb, int32_t i)
{
    uint64_t addr, len;
    addr = (uint64_t) (urb->ux_transfer_request_data_pointer + (i * urb->ux_transfer_request_requested_length));
    len = urb->ux_transfer_request_requested_length;
    return count_trbs(addr, len);
}

/*
 * Calculates Frame ID field of the isochronous TRB identifies the
 * target frame that the Interval associated with this Isochronous
 * Transfer Descriptor will start on. Refer to 4.11.2.5.
 *
 * Returns actual frame id on success, negative value on error.
 */
/**
  \fn           xhci_get_isoc_frame_id
  \brief        xhci driver Isochronous frame ID
  \param[in]    xhci global pointer
  \param[in]    urb pointer
  \param[in]    TD index
  \return       Frme ID
 */
int32_t xhci_get_isoc_frame_id(UX_HCD_XHCI *xhci, UX_TRANSFER *urb, int32_t index)
{
    int32_t start_frame, ist, ret = 0;
    int32_t start_frame_id, end_frame_id, current_frame_id;
    if((xhci->device->ux_device_speed == UX_LOW_SPEED_DEVICE) || (xhci->device->ux_device_speed == UX_FULL_SPEED_DEVICE))
    {
        start_frame = xhci->start_frame + index * xhci->interval;

    }
    else
    {
        start_frame = (xhci->start_frame + index * xhci->interval) >> 3;
    }

    /* Isochronous Scheduling Threshold (IST, bits 0~3 in HCSPARAMS2):
     *
     * If bit [3] of IST is cleared to '0', software can add a TRB no
     * later than IST[2:0] Microframes before that TRB is scheduled to
     * be executed.
     * If bit [3] of IST is set to '1', software can add a TRB no later
     * than IST[2:0] Frames before that TRB is scheduled to be executed.
     */
    ist = HCS_IST(xhci->hcs_params2) & 0x7;
    if (HCS_IST(xhci->hcs_params2) & (1 << 3))
        ist <<= 3;

    /* Software shall not schedule an Isoch TD with a Frame ID value that
     * is less than the Start Frame ID or greater than the End Frame ID,
     * where:
     *
     * End Frame ID = (Current MFINDEX register value + 895 ms.) MOD 2048
     * Start Frame ID = (Current MFINDEX register value + IST + 1) MOD 2048
     *
     * Both the End Frame ID and Start Frame ID values are calculated
     * in microframes. When software determines the valid Frame ID value;
     * The End Frame ID value should be rounded down to the nearest Frame
     * boundary, and the Start Frame ID value should be rounded up to the
     * nearest Frame boundary.
     */
    current_frame_id =  xhci->run_regs->MFINDEX;
    start_frame_id = ROUNDUP(current_frame_id + ist + 1, 8);
    end_frame_id = rounddown(current_frame_id + 895 * 8, 8);

    start_frame &= 0x7ff;
    start_frame_id = (start_frame_id >> 3) & 0x7ff;
    end_frame_id = (end_frame_id >> 3) & 0x7ff;

    if (start_frame_id < end_frame_id)
    {
        if (start_frame > end_frame_id || start_frame < start_frame_id)
            ret = -1;
    }
    else if (start_frame_id > end_frame_id)
    {
        if ((start_frame > end_frame_id && start_frame < start_frame_id))
            ret = -1;
    }
    else
    {
        ret = -1;
    }

    if (index == 0)
    {
        if (ret == -1 || start_frame == start_frame_id)
        {
            start_frame = start_frame_id + 1;
            if (xhci->device->ux_device_speed == UX_LOW_SPEED_DEVICE || xhci->device->ux_device_speed == UX_FULL_SPEED_DEVICE)
                xhci->start_frame = start_frame;
            else
                xhci->start_frame = start_frame << 3;
            ret = 0;
        }
    }

    if (ret)
    {
#ifdef DEBUG
        printf("Frame ID %d (reg %d, index %d) beyond range (%d, %d)\n",
                start_frame, current_frame_id, index, start_frame_id, end_frame_id);
        printf("Ignore frame ID field, use SIA bit instead\n");
#endif
        return ret;
    }
    return start_frame;
}
#endif //CFG_TUH_ENABLED
