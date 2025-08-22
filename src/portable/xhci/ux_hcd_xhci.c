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
 * @file     ux_hcd_xhci_ring.c
 * @author   Mahesh Avula
 * @email    mahesh.avula@alifsemi.com
 * @version  V1.0.0
 * @date     12-Dec-2023
 * @brief    XHCI host controller driver.
 * @bug      None
 * @Note     None
 ******************************************************************************/


/* Include necessary system files.  */

#include "tusb_option.h"

#if CFG_TUH_ENABLED && CFG_TUSB_MCU == OPT_MCU_NONE

#include "host/hcd.h"
#include "ux_hcd_xhci_api.h"
//#include "ux_host_stack.h"
//#include "system_utils.h"
//#include "tcm_partition.h"

uint32_t _ux_hcd_xhci_get_endpoint_flag_from_index(uint32_t ep_index)
{
    return BIT((ep_index + 1));
}

uint32_t _ux_hcd_xhci_get_endpoint_flag(UX_ENDPOINT_DESCRIPTOR *desc)
{
    return BIT((_ux_hcd_xhci_get_endpoint_index(desc) + 1));
}


void _ux_hcd_xhci_urb_free_priv(UX_URB_PRIV *urb_priv)
{
    _ux_utility_memory_free(urb_priv);
}

/*
 * Returns zero if the TRB isn't in this segment, otherwise it returns the DMA
 * address of the TRB.
 */
uint64_t _ux_hcd_xhci_trb_virt_to_dma(UX_XHCI_SEGMENT *seg, UX_XHCI_TRB *trb)
{
    uint32_t segment_offset;

    if (!seg || !trb || trb < seg->trbs)
        return 0;
    /* offset in TRBs */
    segment_offset = trb - seg->trbs;
    if (segment_offset >= TRBS_PER_SEGMENT)
    {
#ifdef DEBUG
        printf("segment offset is greater than TRB range\n");
#endif
        return 0;
    }
    return (uint64_t)seg->dma + (segment_offset * sizeof(*trb));
}

bool trb_is_noop(UX_XHCI_TRB *trb)
{
    return TRB_TYPE_NOOP_LE32(trb->generic.field[3]);
}

bool trb_is_link(UX_XHCI_TRB *trb)
{
    return TRB_TYPE_LINK_LE32(trb->link.control);
}

static bool last_trb_on_seg(UX_XHCI_SEGMENT *seg, UX_XHCI_TRB *trb)
{
    return trb == &seg->trbs[TRBS_PER_SEGMENT - 1];
}

static bool last_trb_on_ring(UX_XHCI_RING *ring, UX_XHCI_SEGMENT *seg, UX_XHCI_TRB *trb)
{
    return last_trb_on_seg(seg, trb) && (seg->next == ring->first_seg);
}

static bool link_trb_toggles_cycle(UX_XHCI_TRB *trb)
{
    return (trb->link.control) & LINK_TOGGLE;
}

static bool last_td_in_urb(UX_XHCI_TD *td)
{
    UX_URB_PRIV *urb_priv = td->urb->hcpriv;
    return urb_priv->num_tds_done == urb_priv->num_tds;
}

static void inc_td_cnt(UX_XHCI_TD *td)
{
    UX_URB_PRIV *urb_priv = td->urb->hcpriv;
    urb_priv->num_tds_done++;
}


static void trb_to_noop(UX_XHCI_TRB *trb, unsigned int noop_type)
{
    if (trb_is_link(trb))
    {
        /* unchain chained link TRBs */
        trb->link.control &= (~TRB_CHAIN);
    }
    else
    {
        trb->generic.field[0] = 0;
        trb->generic.field[1] = 0;
        trb->generic.field[2] = 0;
        /* Preserve only the cycle bit of this TRB */
        trb->generic.field[3] &= (TRB_CYCLE);
        trb->generic.field[3] |= (TRB_TYPE(noop_type));
    }
}

/* Updates trb to point to the next TRB in the ring, and updates seg if the next
 * TRB is in a new segment.  This does not skip over link TRBs, and it does not
 * effect the ring dequeue or enqueue pointers.
 */
static void next_trb(
        UX_HCD_XHCI *xhci,
        UX_XHCI_RING *ring,
        UX_XHCI_SEGMENT **seg,
        UX_XHCI_TRB **trb)
{
    if (trb_is_link(*trb))
    {
        *seg = (*seg)->next;
        *trb = ((*seg)->trbs);
    }
    else
    {
        (*trb)++;
    }
}

/*
 * See Cycle bit rules. SW is the consumer for the event ring only.
 * Don't make a ring full of link TRBs.  That would be dumb and this would loop.
 */

void inc_deq(UX_HCD_XHCI *xhci, UX_XHCI_RING *ring)
{
    /* event ring doesn't have link trbs, check for last trb */
    if (ring->type == TYPE_EVENT)
    {
        if (!last_trb_on_seg(ring->deq_seg, ring->dequeue))
        {
            ring->dequeue++;
            return;
        }
        if (last_trb_on_ring(ring, ring->deq_seg, ring->dequeue))
        {
            ring->cycle_state ^= 1;
        }

        ring->deq_seg = ring->deq_seg->next;
        ring->dequeue = ring->deq_seg->trbs;
        return;
    }

    /* All other rings have link trbs */
    if (!trb_is_link(ring->dequeue))
    {
        ring->dequeue++;
        ring->num_trbs_free++;
    }
    while (trb_is_link(ring->dequeue))
    {
        ring->deq_seg = ring->deq_seg->next;
        ring->dequeue = ring->deq_seg->trbs;
    }
}

/*
 * See Cycle bit rules. SW is the consumer for the event ring only.
 * Don't make a ring full of link TRBs.  That would be dumb and this would loop.
 *
 * If we've just enqueued a TRB that is in the middle of a TD (meaning the
 * chain bit is set), then set the chain bit in all the following link TRBs.
 * If we've enqueued the last TRB in a TD, make sure the following link TRBs
 * have their chain bit cleared (so that each Link TRB is a separate TD).
 *
 * Section 6.4.4.1 of the xhci 1.0 spec says link TRBs cannot have the chain bit
 * set, but other sections.
 *
 * @more_trbs_coming:  Will you enqueue more TRBs before calling
 *    prepare_transfer()?
 */
void inc_enq(UX_HCD_XHCI *xhci, UX_XHCI_RING *ring, bool more_trbs_coming)
{
    uint32_t chain;
    UX_XHCI_TRB *next;

    chain = (ring->enqueue->generic.field[3]) & TRB_CHAIN;
    /* If this is not event ring, there is one less usable TRB */
    if (!trb_is_link(ring->enqueue))
        ring->num_trbs_free--;
    next = ++(ring->enqueue);

    /* Update the dequeue pointer further if that was a link TRB */
    while (trb_is_link(next))
    {
        /*
         * If the caller doesn't plan on enqueueing more TDs before
         * ringing the doorbell, then we don't want to give the link TRB
         * to the hardware just yet. We'll give the link TRB back in
         * prepare_ring() just before we enqueue the TD at the top of
         * the ring.
         */
        if (!chain && !more_trbs_coming)
            break;
        if (!(ring->type == TYPE_ISOC /*&& ! _ux_hcd_xhci_link_trb_quirk(xhci))*/))
        {
            next->link.control &= (~TRB_CHAIN);
            next->link.control |= (chain);
        }
        /* Give this link TRB to the hardware */
        next->link.control ^= (TRB_CYCLE);
        /* Toggle the cycle bit after the last ring segment. */
        if (link_trb_toggles_cycle(next))
            ring->cycle_state ^= 1;

        ring->enq_seg = ring->enq_seg->next;
        ring->enqueue = ring->enq_seg->trbs;
        next = ring->enqueue;
    }
}

/*
 * Check to see if there's room to enqueue num_trbs on the ring and make sure
 * enqueue pointer will not advance into dequeue segment. See rules above.
 */
static inline int32_t room_on_ring(UX_HCD_XHCI *xhci, UX_XHCI_RING *ring, uint32_t num_trbs)
{
    int32_t num_trbs_in_deq_seg = 0;
    if (ring->num_trbs_free < num_trbs)
    {
#ifdef DEBUG
        printf("free TRB's less than the required\n");
#endif
        return 0;
    }
    if (ring->type != TYPE_COMMAND && ring->type != TYPE_EVENT)
    {
        num_trbs_in_deq_seg = ring->dequeue - ring->deq_seg->trbs;
        if (ring->num_trbs_free < num_trbs + num_trbs_in_deq_seg)
            return 0;
    }
    return 1;
}

/* Ring the host controller doorbell after placing a command on the ring */
void _ux_hcd_xhci_ring_cmd_db(UX_HCD_XHCI *xhci)
{
    if (!(xhci->cmd_ring_state & CMD_RING_STATE_RUNNING))
        return;
    xhci->dba_regs->DOORBELL[0] = DB_VALUE_HOST;

}

void _ux_hcd_xhci_ring_ep_doorbell(
        UX_HCD_XHCI *xhci,
        uint32_t slot_id,
        uint32_t ep_index,
        uint32_t stream_id)
{
    UX_XHCI_VIRT_EP *ep = &xhci->devs[slot_id]->eps[ep_index];
    uint32_t ep_state = ep->ep_state;

    /* Don't ring the doorbell for this endpoint if there are pending
     * cancellations because we don't want to interrupt processing.
     * We don't want to restart any stream rings if there's a set dequeue
     * pointer command pending because the device can choose to start any
     * stream once the endpoint is on the HW schedule.
     */
    if ((ep_state & EP_STOP_CMD_PENDING) || (ep_state & SET_DEQ_PENDING) ||
            (ep_state & EP_HALTED) || (ep_state & EP_CLEARING_TT))
        return;
    xhci->dba_regs->DOORBELL[slot_id] = DB_VALUE(ep_index, stream_id);
}

/* Ring the doorbell for any rings with pending URBs */
static void ring_doorbell_for_active_rings(UX_HCD_XHCI *xhci,uint32_t slot_id, uint32_t ep_index)
{
    uint32_t stream_id;
    UX_XHCI_VIRT_EP *ep;
    ep = &xhci->devs[slot_id]->eps[ep_index];

    /* A ring has pending URBs if its TD list is not empty */
    if (!(ep->ep_state & EP_HAS_STREAMS))
    {
        if (ep->ring && !(_ux_hcd_xhci_list_empty(&ep->ring->td_list)))
            _ux_hcd_xhci_ring_ep_doorbell(xhci, slot_id, ep_index, 0);
        return;
    }

    for (stream_id = 1; stream_id < ep->stream_info->num_streams;stream_id++)
    {
        UX_XHCI_STREAM_INFO *stream_info = ep->stream_info;
        if (!_ux_hcd_xhci_list_empty(&stream_info->stream_rings[stream_id]->td_list))
            _ux_hcd_xhci_ring_ep_doorbell(xhci, slot_id, ep_index, stream_id);
    }
}

/* Get the right ring for the given slot_id, ep_index and stream_id.
 * If the endpoint supports streams, boundary check the URB's stream ID.
 * If the endpoint doesn't support streams, return the singular endpoint ring.
 */
UX_XHCI_RING *_ux_hcd_xhci_triad_to_transfer_ring(
        UX_HCD_XHCI  *xhci,
        uint32_t slot_id,
        uint32_t ep_index,
        uint32_t stream_id)
{
    UX_XHCI_VIRT_EP *ep;
    ep = &xhci->devs[slot_id]->eps[ep_index];
    /* Common case: no streams */
    if (!(ep->ep_state & EP_HAS_STREAMS))
        return ep->ring;

    if (stream_id == 0)
    {
#ifdef DEBUG
        printf("WARN: Slot ID %u, ep index %u has streams, ""but URB has no stream ID.\n",
                slot_id, ep_index);
#endif
        return NULL;
    }

    if (stream_id < ep->stream_info->num_streams)
        return ep->stream_info->stream_rings[stream_id];
#ifdef DEBUG
    printf( "WARN: Slot ID %u, ep index %u has " "stream IDs 1 to %u allocated, "
            "but stream ID %u is requested.\n", slot_id, ep_index,
            ep->stream_info->num_streams - 1, stream_id);
#endif
    return NULL;
}

int32_t _ux_hcd_xhci_find_next_ext_cap(
        UX_HCD_XHCI  *xhci , uint32_t start, int32_t id)
{
    uint32_t val;
    uint32_t next;
    uint32_t offset;
    offset = start;
    if (!start || start == UX_XHCI_HCC_PARAMS_OFFSET)
    {
        /* Read the Capability Parameters register 1  */
        val = xhci->regs->HCCPARAMS1;
        if (val == ~0)
            return 0;
        /*  Get the xHCI Extended Capabilities Pointer (xECP) value and
         * that value as a relative offset  */
        offset = XHCI_HCC_EXT_CAPS(val) << 2;
        if (!offset)
            return 0;
    }
    do
    {
        val = *(uint32_t *)((uint8_t *)(xhci->regs) + offset);
        if (val == ~0)
            return 0;
        if (offset != start && (id == 0 || UX_XHCI_EXT_CAPS_ID(val) == id))
            return offset;

        next = UX_XHCI_EXT_CAPS_NEXT(val);
        offset += next << 2;
    } while (next);

    return 0;
}


/*
 * Get the hw dequeue pointer xHC stopped on, either directly from the
 * endpoint context, or if streams are in use from the stream context.
 * The returned hw_dequeue contains the lowest four bits with cycle state
 * and possbile stream context type.
 */
static uint64_t _ux_hcd_xhci_get_hw_deq(
        UX_HCD_XHCI   *xhci,
        UX_XHCI_VIRT_DEVICE  *vdev,
        uint32_t ep_index,
        uint32_t stream_id)
{
    UX_XHCI_EP_CTX *ep_ctx;
    UX_XHCI_STREAM_CTX *st_ctx;
    UX_XHCI_VIRT_EP *ep;
    ep = &vdev->eps[ep_index];
    if (ep->ep_state & EP_HAS_STREAMS)
    {
        st_ctx = &ep->stream_info->stream_ctx_array[stream_id];
        return (st_ctx->stream_ring);
    }
    ep_ctx = _ux_hcd_xhci_get_ep_ctx(xhci, vdev->out_ctx, ep_index);
    return (ep_ctx->deq);
}

/*
 * Move the xHC's endpoint ring dequeue pointer past cur_td.
 * Record the new state of the xHC's endpoint ring dequeue segment,
 * dequeue pointer, stream id, and new consumer cycle state in state.
 * Update our internal representation of the ring's dequeue pointer.
 *
 * We do this in three jumps:
 *  - First we update our new ring state to be the same as when the xHC stopped.
 *  - Then we traverse the ring to find the segment that contains
 *    the last TRB in the TD.  We toggle the xHC's new cycle state when we pass
 *    any link TRBs with the toggle cycle bit set.
 *  - Finally we move the dequeue state one TRB further, toggling the cycle bit
 *    if we've moved it past a link TRB with the toggle cycle bit set.
 */
void _ux_hcd_xhci_find_new_dequeue_state(
        UX_HCD_XHCI *xhci,
        uint32_t slot_id,
        uint32_t ep_index,
        uint32_t stream_id,
        UX_XHCI_TD *cur_td,
        UX_XHCI_DEQUEUE_STATE *state)
{
    UX_XHCI_VIRT_DEVICE  *dev = xhci->devs[slot_id];
    UX_XHCI_VIRT_EP    *ep = &dev->eps[ep_index];
    UX_XHCI_RING   *ep_ring;
    UX_XHCI_SEGMENT    *new_seg;
    UX_XHCI_TRB     *new_deq;
    uint64_t       addr;
    uint64_t        hw_dequeue;
    bool           cycle_found = false;
    bool           td_last_trb_found = false;

    ep_ring = _ux_hcd_xhci_triad_to_transfer_ring(xhci, slot_id, ep_index, stream_id);
    if (!ep_ring)
    {
#ifdef DEBUG
        printf("WARN can't find new dequeue state ""for invalid stream ID %u.\n", stream_id);
#endif
        return;
    }
    /* Dig out the cycle state saved by the xHC during the stop ep cmd */
    hw_dequeue = _ux_hcd_xhci_get_hw_deq(xhci, dev, ep_index, stream_id);
    new_seg = ep_ring->deq_seg;
    new_deq = ep_ring->dequeue;
    state->new_cycle_state = hw_dequeue & 0x1;
    state->stream_id = stream_id;

    /*
     * We want to find the pointer, segment and cycle state of the new trb
     * (the one after current TD's last_trb). We know the cycle state at
     * hw_dequeue, so walk the ring until both hw_dequeue and last_trb are
     * found.
     */
    do
    {
        if (!cycle_found && _ux_hcd_xhci_trb_virt_to_dma(new_seg, new_deq) == (hw_dequeue & ~0xf))
        {
            cycle_found = true;
            if (td_last_trb_found)
                break;
        }
        if (new_deq == cur_td->last_trb)
            td_last_trb_found = true;
        if (cycle_found && trb_is_link(new_deq) && link_trb_toggles_cycle(new_deq))
            state->new_cycle_state ^= 0x1;

        next_trb(xhci, ep_ring, &new_seg, &new_deq);

        /* Search wrapped around, bail out */
        if (new_deq == ep->ring->dequeue)
        {
#ifdef DEBUG
            printf("Error: Failed finding new dequeue state\n");
#endif
            state->new_deq_seg = NULL;
            state->new_deq_ptr = NULL;
            return;
        }

    } while (!cycle_found || !td_last_trb_found);

    state->new_deq_seg = new_seg;
    state->new_deq_ptr = new_deq;

    /* Don't update the ring cycle state for the producer (us). */
#ifdef DEBUG
    printf("Cycle state = 0x%x\n ", state->new_cycle_state);
    printf("New dequeue segment = %p (virtual)\n ",state->new_deq_seg);
#endif
    addr = _ux_hcd_xhci_trb_virt_to_dma(state->new_deq_seg, state->new_deq_ptr);
#ifdef DEBUG
    printf("New dequeue pointer = 0x%llx (DMA)\n", addr);
#endif
}

/* flip_cycle means flip the cycle bit of all but the first and last TRB.
 * (The last TRB actually points to the ring enqueue pointer, which is not part
 * of this TD.)  This is used to remove partially enqueued isoc TDs from a ring.
 */
void td_to_noop(
        UX_HCD_XHCI *xhci,
        UX_XHCI_RING *ep_ring,
        UX_XHCI_TD *td,
        bool flip_cycle)
{
    UX_XHCI_SEGMENT *seg = td->start_seg;
    UX_XHCI_TRB    *trb = td->first_trb;

    while (1)
    {
        trb_to_noop(trb, TRB_TR_NOOP);

        /* flip cycle if asked to */
        if (flip_cycle && trb != td->first_trb && trb != td->last_trb)
            trb->generic.field[3] ^= (TRB_CYCLE);

        if (trb == td->last_trb)
            break;

        next_trb(xhci, ep_ring, &seg, &trb);
    }
}


/*
 * Must be called with in interrupt context,
 */
static void _ux_hcd_xhci_giveback_urb_in_irq(
        UX_HCD_XHCI   *xhci,
        UX_XHCI_TD   *cur_td,
        int32_t status)
{
    UX_TRANSFER  *urb   = cur_td->urb;
    UX_URB_PRIV  *urb_priv  = urb->hcpriv;
    //clean_td[clear_td].clear_urb_priv = urb_priv;
    //clean_td[clear_td ++ %100].num_tds = urb_priv->num_tds;
    _ux_hcd_xhci_urb_free_priv(urb_priv);
    urb->hcpriv = NULL;
}

static void _ux_hcd_xhci_unmap_td_bounce_buffer(
        UX_HCD_XHCI *xhci,
        UX_XHCI_RING *ring,
        UX_XHCI_TD *td)
{
    UX_XHCI_SEGMENT *seg = td->bounce_seg;
    UX_TRANSFER *urb = td->urb;
    if (!ring || !seg || !urb)
        return;
    _ux_utility_memory_free(seg->bounce_buf);
    return;
}

/*
 * When we get a command completion for a Stop Endpoint Command, we need to
 * unlink any cancelled TDs from the ring.  There are two ways to do that:
 *
 *  1. If the HW was in the middle of processing the TD that needs to be
 *     cancelled, then we must move the ring's dequeue pointer past the last TRB
 *     in the TD with a Set Dequeue Pointer Command.
 *  2. Otherwise, we turn all the TRBs in the TD into No-op TRBs (with the chain
 *     bit cleared) so that the HW will skip over them.
 */
static void _ux_hcd_xhci_handle_cmd_stop_ep(
        UX_HCD_XHCI *xhci,
        uint32_t slot_id,
        UX_XHCI_TRB *trb,
        UX_XHCI_EVENT_CMD *event)
{
    uint32_t ep_index;
    UX_XHCI_RING *ep_ring;
    UX_XHCI_VIRT_EP *ep;
    UX_XHCI_TD *cur_td = NULL;
    UX_XHCI_TD *last_unlinked_td;
    UX_XHCI_VIRT_DEVICE *vdev;
    uint64_t hw_deq;
    UX_XHCI_DEQUEUE_STATE deq_state;

    if (!(TRB_TO_SUSPEND_PORT((trb->generic.field[3]))))
    {
        if (!xhci->devs[slot_id])
        {
#ifdef DEBUG
            printf("Stop endpoint command " "completion for disabled slot %d\n",slot_id);
#endif
        }
        return;
    }
    memset(&deq_state, 0, sizeof(deq_state));
    ep_index = TRB_TO_EP_INDEX((trb->generic.field[3]));
    vdev = xhci->devs[slot_id];

    ep = &xhci->devs[slot_id]->eps[ep_index];
    last_unlinked_td = _ux_hcd_xhci_list_last_entry(&ep->cancelled_td_list, UX_XHCI_TD, cancelled_td_list);
    if (_ux_hcd_xhci_list_empty(&ep->cancelled_td_list))
    {
        ep->ep_state &= ~EP_STOP_CMD_PENDING;
        ring_doorbell_for_active_rings(xhci, slot_id, ep_index);
        return;
    }

    /* Fix up the ep ring first, so HW stops executing cancelled TDs.
     * We have the xHCI lock, so nothing can modify this list until we drop
     * it.  We're also in the event handler, so we can't get re-interrupted
     * if another Stop Endpoint command completes
     */
    _ux_hcd_xhci_list_for_each_entry(cur_td, &ep->cancelled_td_list, cancelled_td_list, UX_XHCI_TD)
    {
#ifdef DEBUG
        printf("Removing canceled TD's\n");
#endif
        ep_ring = ep->ring;
        if (!ep_ring)
        {
            /* This shouldn't happen unless a driver is mucking
             * with the stream ID after submission.  This will
             * leave the TD on the hardware ring, and the hardware
             * will try to execute it, and may access a buffer
             * that has already been freed.  In the best case, the
             * hardware will execute it, and the event handler will
             * ignore the completion event for that TD, since it was
             * removed from the td_list for that endpoint.  In
             * short, don't muck with the stream ID after
             * submission.
             */
#ifdef DEBUG
            printf("WARN Cancelled URB %p ""has invalid stream ID %u.\n",
                    cur_td->urb, xhci->stream_id);
#endif
            goto remove_finished_td;
        }
        /*
         * If we stopped on the TD we need to cancel, then we have to
         * move the xHC endpoint ring dequeue pointer past this TD.
         */
        hw_deq = _ux_hcd_xhci_get_hw_deq(xhci, vdev, ep_index, xhci->stream_id);
        hw_deq &= ~0xf;

        if (trb_in_td(xhci, cur_td->start_seg, cur_td->first_trb, cur_td->last_trb, hw_deq, false))
        {
            _ux_hcd_xhci_find_new_dequeue_state(xhci, slot_id, ep_index, xhci->stream_id,
                    cur_td, &deq_state);
        }
        else
        {
            td_to_noop(xhci, ep_ring, cur_td, false);
        }

remove_finished_td:
        /*
         * The event handler won't see a completion for this TD anymore,
         * so remove it from the endpoint ring's TD list.  Keep it in
         * the cancelled TD list for URB completion later.
         */
        _ux_hcd_xhci_list_del_init(&cur_td->td_list);
    }
    ep->ep_state &= ~EP_STOP_CMD_PENDING;

    /* If necessary, queue a Set Transfer Ring Dequeue Pointer command */
    if (deq_state.new_deq_ptr && deq_state.new_deq_seg)
    {
        _ux_hcd_xhci_queue_new_dequeue_state(xhci, slot_id, ep_index, &deq_state);
        /* Ring The HC doorbell for command ring  */
        _ux_hcd_xhci_ring_cmd_db(xhci);
    }
    else
    {
        /* Otherwise ring the doorbell(s) to restart queued transfers */
        ring_doorbell_for_active_rings(xhci, slot_id, ep_index);
    }

    /*
     * Drop the lock and complete the URBs in the cancelled TD list.
     * New TDs to be cancelled might be added to the end of the list before
     * we can complete all the URBs for the TDs we already unlinked.
     * So stop when we've completed the URB for the last TD we unlinked.
     */
    do
    {
        cur_td = _ux_hcd_xhci_list_first_entry(&ep->cancelled_td_list, UX_XHCI_TD, cancelled_td_list);
        _ux_hcd_xhci_list_del_init(&cur_td->cancelled_td_list);

        /* Clean up the cancelled URB */
        /* Doesn't matter what we pass for status, since the core will
         * just overwrite it (because the URB has been unlinked).
         */
        _ux_hcd_xhci_unmap_td_bounce_buffer(xhci, ep_ring, cur_td);
        inc_td_cnt(cur_td);
        if (last_td_in_urb(cur_td))
            _ux_hcd_xhci_giveback_urb_in_irq(xhci, cur_td, 0);
        /* Stop processing the cancelled list if the watchdog timer is
         * running.
         */
        if (xhci->xhc_state & UX_XHCI_STATE_DYING)
            return;
    } while (cur_td != last_unlinked_td);

}

static void _ux_hcd_xhci_kill_ring_urbs(UX_HCD_XHCI *xhci, UX_XHCI_RING *ring)
{
    UX_XHCI_TD *cur_td;
    UX_XHCI_TD *tmp;
    printf("kill ring urb's\n");
    _ux_hcd_xhci_list_for_each_entry_safe(cur_td, tmp, &ring->td_list, td_list, UX_XHCI_TD)
    {
        _ux_hcd_xhci_list_del_init(&cur_td->td_list);

        if (!_ux_hcd_xhci_list_empty(&cur_td->cancelled_td_list))
            _ux_hcd_xhci_list_del_init(&cur_td->cancelled_td_list);

        _ux_hcd_xhci_unmap_td_bounce_buffer(xhci, ring, cur_td);
        inc_td_cnt(cur_td);
        if (last_td_in_urb(cur_td))
            _ux_hcd_xhci_giveback_urb_in_irq(xhci, cur_td, -1);
    }
}
static void _ux_hcd_xhci_kill_endpoint_urbs(UX_HCD_XHCI *xhci, int slot_id, int ep_index)
{
    UX_XHCI_TD    *cur_td;
    UX_XHCI_TD    *tmp;
    UX_XHCI_VIRT_EP   *ep;
    UX_XHCI_RING    *ring;
    printf("Kill URb's\n");
    ep = &xhci->devs[slot_id]->eps[ep_index];
    if ((ep->ep_state & EP_HAS_STREAMS) || (ep->ep_state & EP_GETTING_NO_STREAMS))
    {
        int32_t stream_id;
        for (stream_id = 1; stream_id < ep->stream_info->num_streams;stream_id++)
        {
            ring = ep->stream_info->stream_rings[stream_id];
            if (!ring)
                continue;
#ifdef DEBUG
            printf("Killing URBs for slot ID %u, ep index %u, stream %u",
                    slot_id, ep_index, stream_id);
#endif
            _ux_hcd_xhci_kill_ring_urbs(xhci, ring);
        }
    }
    else
    {
        ring = ep->ring;
        if (!ring)
            return;
#ifdef DEBUG
        printf("Killing URBs for slot ID %u, ep index %u", slot_id, ep_index);
#endif
        _ux_hcd_xhci_kill_ring_urbs(xhci, ring);
    }
    _ux_hcd_xhci_list_for_each_entry_safe(cur_td, tmp, &ep->cancelled_td_list, cancelled_td_list, UX_XHCI_TD)
    {
        _ux_hcd_xhci_list_del_init(&cur_td->cancelled_td_list);
        inc_td_cnt(cur_td);
        if (last_td_in_urb(cur_td))
            _ux_hcd_xhci_giveback_urb_in_irq(xhci, cur_td, -1);
    }
}

/*
 * host controller died, register read returns 0xffffffff
 * Complete pending commands, mark them ABORTED.
 * URBs need to be given back as usb core might be waiting with device locks
 * held for the URBs to finish during device disconnect, blocking host remove.
 *
 */
void _ux_hcd_xhci_hc_died(UX_HCD_XHCI *xhci)
{
    int32_t i, j;
    if (xhci->xhc_state & UX_XHCI_STATE_DYING)
        return;
#ifdef DEBUG
    printf("xHCI host controller not responding, assume dead\n");
#endif
    xhci->xhc_state |= UX_XHCI_STATE_DYING;
    _ux_hcd_xhci_cleanup_command_queue(xhci);

    /* return any pending urbs, remove may be waiting for them */
    for (i = 0; i <= HCS_MAX_SLOTS(xhci->hcs_params1); i++)
    {
        if (!xhci->devs[i])
            continue;
        for (j = 0; j < 31; j++)
            _ux_hcd_xhci_kill_endpoint_urbs(xhci, i, j);
    }
}

static void update_ring_for_set_deq_completion(
        UX_HCD_XHCI *xhci,
        UX_XHCI_VIRT_DEVICE *dev,
        UX_XHCI_RING *ep_ring,
        uint32_t ep_index)
{
    UX_XHCI_TRB *dequeue_temp;
    uint32_t num_trbs_free_temp;
    bool revert = false;
    num_trbs_free_temp = ep_ring->num_trbs_free;
    dequeue_temp = ep_ring->dequeue;

    /* If we get two back-to-back stalls, and the first stalled transfer
     * ends just before a link TRB, the dequeue pointer will be left on
     * the link TRB by the code in the while loop.  So we have to update
     * the dequeue pointer one segment further, or we'll jump off
     * the segment into la-la-land.
     */
    if (trb_is_link(ep_ring->dequeue))
    {
        ep_ring->deq_seg = ep_ring->deq_seg->next;
        ep_ring->dequeue = ep_ring->deq_seg->trbs;
    }

    while (ep_ring->dequeue != dev->eps[ep_index].queued_deq_ptr)
    {
        /* We have more usable TRBs */
        ep_ring->num_trbs_free++;
        ep_ring->dequeue++;
        if (trb_is_link(ep_ring->dequeue))
        {
            if (ep_ring->dequeue == dev->eps[ep_index].queued_deq_ptr)
                break;
            ep_ring->deq_seg = ep_ring->deq_seg->next;
            ep_ring->dequeue = ep_ring->deq_seg->trbs;
        }
        if (ep_ring->dequeue == dequeue_temp)
        {
            revert = true;
            break;
        }
    }

    if (revert)
    {
        ep_ring->num_trbs_free = num_trbs_free_temp;
    }
}

/*
 * When we get a completion for a Set Transfer Ring Dequeue Pointer command,
 * we need to clear the set deq pending flag in the endpoint ring state, so that
 * the TD queueing code can ring the doorbell again.  We also need to ring the
 * endpoint doorbell to restart the ring, but only if there aren't more
 * cancellations pending.
 */
static void _ux_hcd_xhci_handle_cmd_set_deq(
        UX_HCD_XHCI *xhci,
        uint32_t slot_id,
        UX_XHCI_TRB *trb,
        uint32_t cmd_comp_code)
{
    uint32_t ep_index;
    uint32_t stream_id;
    UX_XHCI_RING *ep_ring;
    UX_XHCI_VIRT_DEVICE *dev;
    UX_XHCI_VIRT_EP *ep;
    UX_XHCI_EP_CTX *ep_ctx;
    UX_XHCI_SLOT_CTX *slot_ctx;
    ep_index = TRB_TO_EP_INDEX((trb->generic.field[3]));
    stream_id = TRB_TO_STREAM_ID((trb->generic.field[2]));
    dev = xhci->devs[slot_id];
    ep = &dev->eps[ep_index];
    ep_ring = _ux_hcd_xhci_stream_id_to_ring(dev, ep_index, stream_id);
    if (!ep_ring)
    {
#ifdef DEBUG
        printf("Set TR deq ptr command for freed stream ID %u\n", stream_id);
#endif
        goto cleanup;
    }

    ep_ctx = _ux_hcd_xhci_get_ep_ctx(xhci, dev->out_ctx, ep_index);
    if (cmd_comp_code != COMP_SUCCESS)
    {
        uint32_t ep_state;
        uint32_t slot_state;

        switch (cmd_comp_code)
        {
            case COMP_TRB_ERROR:
#ifdef DEBUG
                printf("WARN Set TR Deq Ptr cmd invalid because of stream ID configuration\n");
#endif
                break;
            case COMP_CONTEXT_STATE_ERROR:
#ifdef DEBUG
                printf("WARN Set TR Deq Ptr cmd failed due to incorrect slot or ep state.\n");
#endif
                break;
            case COMP_SLOT_NOT_ENABLED_ERROR:
#ifdef DEBUG
                printf("WARN Set TR Deq Ptr cmd failed because slot %u was not enabled.\n",slot_id);
#endif
                break;
            default:
#ifdef DEBUG
                printf("WARN Set TR Deq Ptr cmd with unknown completion code of %u.\n",cmd_comp_code);
#endif
                break;
        }
    }
    else
    {
        uint64_t deq;
        /* 4.6.10 deq ptr is written to the stream ctx for streams */
        if (ep->ep_state & EP_HAS_STREAMS)
        {
            UX_XHCI_STREAM_CTX *ctx = &ep->stream_info->stream_ctx_array[stream_id];
            deq = (ctx->stream_ring) & SCTX_DEQ_MASK;
        }
        else
        {
            deq = (ep_ctx->deq) & ~EP_CTX_CYCLE_MASK;
        }
#ifdef DEBUG
        printf("Successful Set TR Deq Ptr cmd, deq = @%08llx", deq);
#endif
        if (_ux_hcd_xhci_trb_virt_to_dma(ep->queued_deq_seg,  ep->queued_deq_ptr) == deq)
        {
            /* Update the ring's dequeue segment and dequeue pointer
             * to reflect the new position.
             */
            update_ring_for_set_deq_completion(xhci, dev, ep_ring, ep_index);
        }
        else
        {
#ifdef DEBUG
            printf("Mismatch between completed Set TR Deq Ptr command & xHCI internal state.\n");
#endif
        }
    }

cleanup:
    dev->eps[ep_index].ep_state &= ~SET_DEQ_PENDING;
    dev->eps[ep_index].queued_deq_seg = NULL;
    dev->eps[ep_index].queued_deq_ptr = NULL;
    /* Restart any rings with pending URBs */
    ring_doorbell_for_active_rings(xhci, slot_id, ep_index);
}

static void _ux_hcd_xhci_handle_cmd_reset_ep(
        UX_HCD_XHCI *xhci,
        uint32_t slot_id,
        UX_XHCI_TRB *trb,
        uint32_t cmd_comp_code)
{
    UX_XHCI_VIRT_DEVICE    *vdev;
    UX_XHCI_EP_CTX   *ep_ctx;
    uint32_t ep_index;

    ep_index = TRB_TO_EP_INDEX((trb->generic.field[3]));
    vdev = xhci->devs[slot_id];
    ep_ctx = _ux_hcd_xhci_get_ep_ctx(xhci, vdev->out_ctx, ep_index);

    /* This command will only fail if the endpoint wasn't halted,
     * but we don't care.
     */
#ifdef DEBUG
    printf("Ignoring reset ep completion code of %u\n", cmd_comp_code);
#endif
    /* Clear our internal halted state */
    xhci->devs[slot_id]->eps[ep_index].ep_state &= ~EP_HALTED;
    /* if this was a soft reset, then restart */
    if (((trb->generic.field[3])) & TRB_TSP)
        ring_doorbell_for_active_rings(xhci, slot_id, ep_index);
}

static void _ux_hcd_xhci_handle_cmd_disable_slot(UX_HCD_XHCI *xhci, uint32_t slot_id)
{
    UX_XHCI_VIRT_DEVICE   *virt_dev;
    virt_dev = xhci->devs[slot_id];
    if (!virt_dev)
        return;
    /* Delete default control endpoint resources */
    _ux_hcd_xhci_free_device_endpoint_resources(xhci, virt_dev, true);
    _ux_hcd_xhci_free_virt_device(xhci, slot_id);
}

static void _ux_hcd_xhci_handle_cmd_config_ep(
        UX_HCD_XHCI *xhci,
        uint32_t slot_id,
        UX_XHCI_EVENT_CMD *event,
        uint32_t cmd_comp_code)
{
    UX_XHCI_VIRT_DEVICE     *virt_dev;
    UX_XHCI_INPUT_CONTROL_CTX    *ctrl_ctx;
    UX_XHCI_EP_CTX       *ep_ctx;
    uint32_t         ep_index;
    uint32_t        ep_state;
    uint32_t     add_flags, drop_flags;

    /*
     * Configure endpoint commands can come from the USB core
     * configuration or alt setting changes, or because the HW
     * needed an extra configure endpoint command after a reset
     * endpoint command or streams were being configured.
     * If the command was for a halted endpoint, the xHCI driver
     * is not waiting on the configure endpoint command.
     */
    virt_dev = xhci->devs[slot_id];
    ctrl_ctx = _ux_hcd_xhci_get_input_control_ctx(virt_dev->in_ctx);
    if (!ctrl_ctx)
    {
#ifdef DEBUG
        printf("Could not get input context, bad type.\n");
#endif
        return;
    }
    add_flags = (ctrl_ctx->add_flags);
    drop_flags = (ctrl_ctx->drop_flags);
    /* Input ctx add_flags are the endpoint index plus one */
    ep_index = _ux_hcd_xhci_last_valid_endpoint(add_flags) - 1;
    ep_ctx = _ux_hcd_xhci_get_ep_ctx(xhci, virt_dev->out_ctx, ep_index);

    if ( ep_index != (uint32_t) -1 && add_flags - SLOT_FLAG == drop_flags)
    {
        ep_state = virt_dev->eps[ep_index].ep_state;
        if (!(ep_state & EP_HALTED))
            return;

        /* Clear internal halted state and restart ring(s) */
        virt_dev->eps[ep_index].ep_state &= ~EP_HALTED;
        ring_doorbell_for_active_rings(xhci, slot_id, ep_index);
        return;
    }
    return;
}

static void _ux_hcd_xhci_complete_del_and_free_cmd(UX_XHCI_COMMAND *cmd, uint32_t status)
{
    _ux_hcd_xhci_list_del(&cmd->cmd_list);
    if (cmd->fCompletion)
    {
        cmd->status = status;
        cmd->fCompletion = 0;
    }
    else
    {
        _ux_utility_memory_free(cmd);
    }
}

void _ux_hcd_xhci_cleanup_command_queue(UX_HCD_XHCI *xhci)
{
    UX_XHCI_COMMAND *cur_cmd, *tmp_cmd;
    xhci->current_cmd = NULL;
    _ux_hcd_xhci_list_for_each_entry_safe(cur_cmd, tmp_cmd, &xhci->cmd_list, cmd_list, UX_XHCI_COMMAND)
        _ux_hcd_xhci_complete_del_and_free_cmd(cur_cmd, COMP_COMMAND_ABORTED);
}

void handle_cmd_completion(UX_HCD_XHCI *xhci, UX_XHCI_EVENT_CMD *event)
{

    uint64_t cmd_dma;
    uint64_t cmd_dequeue_dma;
    uint32_t cmd_comp_code;
    UX_XHCI_TRB *cmd_trb;
    UX_XHCI_COMMAND *cmd;
    uint32_t cmd_type;
    uint32_t slot_id = TRB_TO_SLOT_ID((event->flags));
    cmd_dma = event->cmd_trb;
    cmd_trb = xhci->cmd_ring->dequeue;
    cmd_dequeue_dma = _ux_hcd_xhci_trb_virt_to_dma(xhci->cmd_ring->deq_seg, cmd_trb);
    /*
     * Check whether the completion event is for our internal kept
     * command.
     */
    if (!cmd_dequeue_dma || cmd_dma != (uint64_t)cmd_dequeue_dma)
    {
#ifdef DEBUG
        printf("ERROR mismatched command completion event\n ");
#endif
        return;
    }
    cmd = _ux_hcd_xhci_list_first_entry(&xhci->cmd_list, UX_XHCI_COMMAND, cmd_list);
    cmd_comp_code = GET_COMP_CODE((event->status));

    /* If CMD ring stopped we own the trbs between enqueue and dequeue */
    if (cmd_comp_code == COMP_COMMAND_RING_STOPPED)
    {
#ifdef DEBUG
        printf("Ring Stopped\n");
#endif
        return;
    }
    if (cmd->command_trb != xhci->cmd_ring->dequeue)
    {
#ifdef DEBUG
        printf("Command completion event does not match command\n");
#endif
        return;
    }
    /*
     * Host aborted the command ring, check if the current command was
     * supposed to be aborted, otherwise continue normally.
     * The command ring is stopped now, but the xHC will issue a Command
     * Ring Stopped event which will cause us to restart it.
     */
    if (cmd_comp_code == COMP_COMMAND_ABORTED)
    {
        xhci->cmd_ring_state = CMD_RING_STATE_STOPPED;
#ifdef DEBUG
        printf("Command ring stopped \n");
#endif
        if (cmd->status == COMP_COMMAND_ABORTED)
        {
            if (xhci->current_cmd == cmd)
                xhci->current_cmd = NULL;
            goto event_handled;
        }
    }
    RTSS_InvalidateDCache_by_Addr(&cmd_trb->generic, sizeof(cmd_trb->generic));
    cmd_type = TRB_FIELD_TO_TYPE((cmd_trb->generic.field[3]));
    switch (cmd_type)
    {
        case TRB_ENABLE_SLOT:
            if(cmd_comp_code == COMP_SUCCESS)
            {
                cmd->slot_id =  slot_id;
            }
            else
            {
                cmd->slot_id =  0;
            }
            break;
        case TRB_DISABLE_SLOT:
            _ux_hcd_xhci_handle_cmd_disable_slot(xhci, slot_id);
            break;
        case TRB_CONFIG_EP:
            _ux_hcd_xhci_handle_cmd_config_ep(xhci, slot_id, event, cmd_comp_code);
            break;
        case TRB_EVAL_CONTEXT:
            break;
        case TRB_ADDR_DEV:
            break;
        case TRB_STOP_RING:
            _ux_hcd_xhci_handle_cmd_stop_ep(xhci, slot_id, cmd_trb, event);
            break;
        case TRB_SET_DEQ:
            _ux_hcd_xhci_handle_cmd_set_deq(xhci, slot_id, cmd_trb, cmd_comp_code);
            break;
        case TRB_CMD_NOOP:
            /* Is this an aborted command turned to NO-OP? */
            if (cmd->status == COMP_COMMAND_RING_STOPPED)
                cmd_comp_code = COMP_COMMAND_RING_STOPPED;
            break;
        case TRB_RESET_EP:
            _ux_hcd_xhci_handle_cmd_reset_ep(xhci, slot_id, cmd_trb, cmd_comp_code);
            break;
        case TRB_RESET_DEV:
            /* SLOT_ID field in reset device cmd completion event TRB is 0.
             * Use the SLOT_ID from the command TRB instead (xhci 4.6.11) */
            slot_id = TRB_TO_SLOT_ID((cmd_trb->generic.field[3]));
            break;
        case TRB_NEC_GET_FW:
            break;
        default:
            /* Skip over unknown commands on the event ring */
#ifdef DEBUG
            printf("INFO unknown command type %d\n", cmd_type);
#endif
            break;
    }
    /* if this wasn't the last command */
    if (!_ux_hcd_xhci_list_is_singular(&xhci->cmd_list))
    {
        xhci->current_cmd = _ux_hcd_xhci_list_first_entry(&cmd->cmd_list, UX_XHCI_COMMAND, cmd_list);
    }
    else if (xhci->current_cmd == cmd)
    {
        xhci->current_cmd = NULL;
    }

event_handled:
    _ux_hcd_xhci_complete_del_and_free_cmd(cmd, cmd_comp_code);
    inc_deq(xhci, xhci->cmd_ring);
}

void handle_vendor_event(
        UX_HCD_XHCI *xhci,
        UX_XHCI_TRB *event)
{
    unsigned int trb_type;
    trb_type = TRB_FIELD_TO_TYPE((event->generic.field[3]));
    if (trb_type == TRB_NEC_CMD_COMP)
        handle_cmd_completion(xhci, &event->event_cmd);
}

void handle_device_notification(
        UX_HCD_XHCI *xhci,
        UX_XHCI_TRB *event)
{
    uint32_t slot_id;
    RTSS_InvalidateDCache_by_Addr(&event->generic, sizeof(event->generic));
    slot_id = TRB_TO_SLOT_ID((event->generic.field[3]));
    if (!xhci->devs[slot_id])
    {
#ifdef DEBUG
        printf("Device Notification event for ""unused slot %u\n", slot_id);
#endif
        return;
    }
#ifdef DEBUG
    printf("Device Wake Notification event for slot ID %d\n",slot_id);
#endif
}
/* Port status handler  */
void handle_port_status(UX_HCD_XHCI *xhci, UX_XHCI_TRB *event)
{
    uint32_t port_id;
    uint32_t portsc, cmd_reg;
    int32_t max_ports;
    //uint32_t slot_id;
    uint32_t hcd_portnum;
    bool bogus_port_status = false;
    UX_XHCI_PORT *port;
    RTSS_InvalidateDCache_by_Addr(&event->generic, sizeof(event->generic));
    /* Port status change events always have a successful completion code */
    if (GET_COMP_CODE((event->generic.field[2])) != COMP_SUCCESS)
    {
#ifdef DEBUG
        printf("xHC returned failed port status event\n");
#endif
    }
    port_id = GET_PORT_ID((event->generic.field[0]));
    max_ports = HCS_MAX_PORTS(xhci->hcs_params1);

    if ((port_id <= 0) || (port_id > max_ports))
    {
#ifdef DEBUG
        printf("Port change event with invalid port ID %d\n",  port_id);
#endif
        inc_deq(xhci, xhci->event_ring);
        return;
    }
    port = &xhci->hw_ports[port_id - 1];
    if (!port || !port->rhub || port->hcd_portnum == DUPLICATE_ENTRY)
    {
#ifdef DEBUG
        printf("Port change event, no port for port ID %u\n",port_id);
#endif
        bogus_port_status = true;
        goto cleanup;
    }
    hcd_portnum = port->hcd_portnum;
    portsc = xhci->op_regs->PORTSC;
    if ((portsc & PORT_PLC) && (portsc & PORT_PLS_MASK) == XDEV_RESUME)
    {
#ifdef DEBUG
        printf("port resume event for port %d\n", port_id);
#endif
        cmd_reg = xhci->op_regs->USBCMD;
        if (!(cmd_reg & UX_XHCI_CMD_RUN))
        {
#ifdef DEBUG
            printf("xHC is not running.\n");
#endif
            goto cleanup;
        }
    }
    _ux_hcd_xhci_test_and_clear_bit(xhci, port, PORT_PLC);

cleanup:
    /* Update event ring dequeue pointer before dropping the lock */
    inc_deq(xhci, xhci->event_ring);

    if (bogus_port_status)
        return;
    /*
     * xHCI port-status-change events occur when the "or" of all the
     * status-change bits in the portsc register changes from 0 to 1.
     * New status changes won't cause an event if any other change
     * bits are still set.  When an event occurs, switch over to
     * polling to avoid losing status changes.
     */
    tx_timer_activate(&xhci->port_status_timer);
}
/*
 * This TD is defined by the TRBs starting at start_trb in start_seg and ending
 * at end_trb, which may be in another segment.  If the suspect DMA address is a
 * TRB in this TD, this function returns that TRB's segment.  Otherwise it
 * returns 0.
 */

UX_XHCI_SEGMENT *trb_in_td(
        UX_HCD_XHCI      *xhci,
        UX_XHCI_SEGMENT    *start_seg,
        UX_XHCI_TRB        *start_trb,
        UX_XHCI_TRB      *end_trb,
        uint64_t        suspect_dma,
        bool    debug)
{
    uint64_t start_dma;
    uint64_t end_seg_dma;
    uint64_t end_trb_dma;
    UX_XHCI_SEGMENT *cur_seg;

    start_dma = _ux_hcd_xhci_trb_virt_to_dma(start_seg, start_trb);
    cur_seg = start_seg;
    do
    {
        if (start_dma == 0)
            return NULL;
        /* We may get an event for a Link TRB in the middle of a TD */
        end_seg_dma = _ux_hcd_xhci_trb_virt_to_dma(cur_seg,&cur_seg->trbs[TRBS_PER_SEGMENT - 1]);
        /* If the end TRB isn't in this segment, this is set to 0 */
        end_trb_dma = _ux_hcd_xhci_trb_virt_to_dma(cur_seg, end_trb);

        if (debug)
        {
#ifdef DEBUG
            printf("Looking for trb start trb end, start seg, end seg, curr seg\n");
#endif
        }
        if (end_trb_dma > 0)
        {
            /* The end TRB is in this segment, so suspect should be here */
            if (start_dma <= end_trb_dma)
            {
                if (suspect_dma >= start_dma && suspect_dma <= end_trb_dma)
                    return cur_seg;
            }
            else
            {
                /* Case for one segment with a TD wrapped around to the top */
                if ((suspect_dma >= start_dma && suspect_dma <= end_seg_dma) ||
                        (suspect_dma >= cur_seg->dma && suspect_dma <= end_trb_dma))
                    return cur_seg;
            }
            return NULL;
        }
        else
        {
            /* Might still be somewhere in this segment */
            if (suspect_dma >= start_dma && suspect_dma <= end_seg_dma)
                return cur_seg;
        }
        cur_seg = cur_seg->next;
        start_dma = _ux_hcd_xhci_trb_virt_to_dma(cur_seg, &cur_seg->trbs[0]);
    } while (cur_seg != start_seg);
    return NULL;
}

void _ux_hcd_xhci_cleanup_halted_endpoint(
        UX_HCD_XHCI *xhci,
        uint32_t slot_id,
        uint32_t ep_index,
        uint32_t stream_id,
        UX_XHCI_TD *td,
        UX_XHCI_EP_RESET_TYPE reset_type)
{
    UX_XHCI_VIRT_EP *ep = &xhci->devs[slot_id]->eps[ep_index];
    UX_XHCI_COMMAND *command;
    /*
     * Avoid resetting endpoint if link is inactive. Can cause host hang.
     * Device will be reset soon to recover the link so don't do anything
     */
    if (xhci->devs[slot_id]->flags & VDEV_PORT_ERROR)
        return;
    command = _ux_hcd_xhci_alloc_command(xhci);
    if (!command)
        return;
    ep->ep_state |= EP_HALTED;

    _ux_hcd_xhci_queue_reset_ep(xhci, command, slot_id, ep_index, reset_type);

    if (reset_type == EP_HARD_RESET)
    {
        ep->ep_state |= EP_HARD_CLEAR_TOGGLE;
        _ux_hcd_xhci_cleanup_stalled_ring(xhci, ep_index, stream_id, td);
    }
    _ux_hcd_xhci_ring_cmd_db(xhci);
}

/* Check if an error has halted the endpoint ring.  The class driver will
 * cleanup the halt for a non-default control endpoint if we indicate a stall.
 * However, a babble and other errors also halt the endpoint ring, and the class
 * driver won't clear the halt in that case, so we need to issue a Set Transfer
 * Ring Dequeue Pointer command manually.
 */
int32_t _ux_hcd_xhci_requires_manual_halt_cleanup(
        UX_HCD_XHCI *xhci,
        UX_XHCI_EP_CTX *ep_ctx,
        uint32_t trb_comp_code)
{
    /* TRB completion codes that may require a manual halt cleanup */
    if (trb_comp_code == COMP_USB_TRANSACTION_ERROR ||
            trb_comp_code == COMP_BABBLE_DETECTED_ERROR ||
            trb_comp_code == COMP_SPLIT_TRANSACTION_ERROR)
        if (GET_EP_CTX_STATE(ep_ctx) == EP_STATE_HALTED)
            return 1;
    return 0;
}

int32_t _ux_hcd_xhci_is_vendor_info_code(UX_HCD_XHCI *xhci, uint32_t trb_comp_code)
{
    if (trb_comp_code >= 224 && trb_comp_code <= 255)
    {
        /* Vendor defined "informational" completion code,
         * treat as not-an-error.
         */
#ifdef DEBUG
        printf("Vendor defined info completion code %u\n",trb_comp_code);
        printf("Treating code as success.\n");
#endif
        return 1;
    }
    return 0;
}

int32_t _ux_hcd_xhci_td_cleanup(
        UX_HCD_XHCI    *xhci,
        UX_XHCI_TD     *td,
        UX_XHCI_RING    *ep_ring,
        int32_t     *status)
{
    UX_TRANSFER *urb = NULL;
    /* Clean up the endpoint's TD list */
    urb = td->urb;
    /* if a bounce buffer was used to align this td then unmap it */
    _ux_hcd_xhci_unmap_td_bounce_buffer(xhci, ep_ring, td);
    if (urb->ux_transfer_request_actual_length > urb->ux_transfer_request_requested_length)
    {
#ifdef DEBUG
        printf("URB req %lu and actual %lu transfer length mismatch\n",
                urb->ux_transfer_request_requested_length, urb->ux_transfer_request_actual_length);
#endif
        urb->ux_transfer_request_actual_length = 0;
        *status = 0;
    }
    _ux_hcd_xhci_list_del_init(&td->td_list);
    /* Was this TD slated to be cancelled but completed anyway? */
    if (!_ux_hcd_xhci_list_empty(&td->cancelled_td_list))
        _ux_hcd_xhci_list_del_init(&td->cancelled_td_list);
    inc_td_cnt(td);
    /* Giveback the urb when all the tds are completed */
    if (last_td_in_urb(td))
    {
        if ((urb->ux_transfer_request_actual_length != urb->ux_transfer_request_requested_length) &&
                (*status != 0 && !ux_endpoint_xfer_isoc(&urb->ux_transfer_request_endpoint->ux_endpoint_descriptor) ))
        {
#ifdef DEBUG
            printf("Giveback URB %p, len = %lu, expected = %lu, status = %d\n",
                    urb, urb->ux_transfer_request_actual_length,
                    urb->ux_transfer_request_requested_length, *status);
#endif
        }
        _ux_hcd_xhci_giveback_urb_in_irq(xhci, td, *status);
    }
    else
    {
        _ux_hcd_xhci_giveback_urb_in_irq(xhci, td, *status);
    }
    return 0;
}

int32_t finish_td(
        UX_HCD_XHCI *xhci,
        UX_XHCI_TD *td,
        UX_XHCI_TRANSFER_EVENT *event,
        UX_XHCI_VIRT_EP *ep,
        int32_t *status)
{
    UX_XHCI_VIRT_DEVICE    *xdev;
    UX_XHCI_EP_CTX      *ep_ctx;
    UX_XHCI_RING    *ep_ring;
    uint32_t   slot_id;
    uint32_t    trb_comp_code;
    int32_t ep_index;
    slot_id = TRB_TO_SLOT_ID((event->flags));
    xdev = xhci->devs[slot_id];
    ep_index = TRB_TO_EP_ID((event->flags)) - 1;
    ep_ring = ep->ring;
    ep_ctx = _ux_hcd_xhci_get_ep_ctx(xhci, xdev->out_ctx, ep_index);
    trb_comp_code = GET_COMP_CODE((event->transfer_len));
    if (trb_comp_code == COMP_STOPPED_LENGTH_INVALID || trb_comp_code == COMP_STOPPED ||
            trb_comp_code == COMP_STOPPED_SHORT_PACKET)
    {
        /* The Endpoint Stop Command completion will take care of any
         * stopped TDs.  A stopped TD may be restarted, so don't update
         * the ring dequeue pointer or take this TD off any lists yet.
         */
        return 0;
    }
    if (trb_comp_code == COMP_STALL_ERROR ||
            _ux_hcd_xhci_requires_manual_halt_cleanup(xhci, ep_ctx, trb_comp_code))
    {
        /* Issue a reset endpoint command to clear the host side
         * halt, followed by a set dequeue command to move the
         * dequeue pointer past the TD.
         * The class driver clears the device side halt later.
         */
        _ux_hcd_xhci_cleanup_halted_endpoint(xhci, slot_id, ep_index, ep_ring->stream_id, td, EP_HARD_RESET);
    }
    else
    {
        /* Update ring dequeue pointer */
        while (ep_ring->dequeue != td->last_trb)
            inc_deq(xhci, ep_ring);
        inc_deq(xhci, ep_ring);
    }
    return _ux_hcd_xhci_td_cleanup(xhci, td, ep_ring, status);
}

/* sum trb lengths from ring dequeue up to stop_trb, _excluding_ stop_trb */
int32_t sum_trb_lengths(
        UX_HCD_XHCI *xhci,
        UX_XHCI_RING *ring,
        UX_XHCI_TRB *stop_trb)
{
    uint32_t sum;
    UX_XHCI_TRB *trb = ring->dequeue;
    UX_XHCI_SEGMENT *seg = ring->deq_seg;
    for (sum = 0; trb != stop_trb; next_trb(xhci, ring, &seg, &trb))
    {
        if (!trb_is_noop(trb) && !trb_is_link(trb))
            sum += TRB_LEN((trb->generic.field[2]));
    }
    return sum;
}
/*
 * Update Event Ring Dequeue Pointer:
 * - When all events have finished
 * - To avoid "Event Ring Full Error" condition
 */
void _ux_hcd_xhci_update_erst_dequeue(
        UX_HCD_XHCI *xhci,
        UX_XHCI_TRB *event_ring_deq)
{
    uint64_t temp_64;
    uint64_t deq;
    temp_64 = xhci->run_regs->ir_set[0].ERDP;
    /* If necessary, update the HW's version of the event ring deq ptr. */
    if (event_ring_deq != xhci->event_ring->dequeue)
    {
        deq = _ux_hcd_xhci_trb_virt_to_dma(xhci->event_ring->deq_seg, xhci->event_ring->dequeue);
        if (deq == 0)
        {
#ifdef DEBUG
            printf("WARN something wrong with SW event ring dequeue ptr\n");
#endif
        }
        /*
         * Per 4.9.4, Software writes to the ERDP register shall
         * always advance the Event Ring Dequeue Pointer value.
         */
        if ((temp_64 & (uint64_t) ~ERST_PTR_MASK) == ( deq & (uint64_t) ~ERST_PTR_MASK))
            return;

        /* Update HC event ring dequeue pointer */
        temp_64 &= ERST_PTR_MASK;
        temp_64 |= ( deq & (uint64_t) ~ERST_PTR_MASK);
    }
    /* Clear the event handler busy flag (RW1C) */
    temp_64 |= ERST_EHB;
    xhci->run_regs->ir_set[0].ERDP = temp_64;
}


/****  Endpoint Ring Operations   ****/

/*
 * Generic function for queueing a TRB on a ring.
 * The caller must have checked to make sure there's room on the ring.
 *
 * @more_trbs_coming: Will you enqueue more TRBs before calling
 *   prepare_transfer()?
 */
void queue_trb(
        UX_HCD_XHCI *xhci,
        UX_XHCI_RING *ring,
        bool more_trbs_coming,
        UX_XHCI_TRB_INFO *trb_info)
{
    UX_XHCI_GENERIC_TRB *trb;
    trb = &ring->enqueue->generic;
    trb->field[0] = trb_info->low_address;
    trb->field[1] = trb_info->high_address;
    trb->field[2] = trb_info->size;
    trb->field[3] = trb_info->cntrl_field;
    RTSS_CleanDCache_by_Addr(trb, sizeof(*trb));
    inc_enq(xhci, ring, more_trbs_coming);
}
/*
 * Does various checks on the endpoint ring, and makes it ready to queue num_trbs.
 * FIXME allocate segments if the ring is full.
 */
int32_t prepare_ring(UX_HCD_XHCI *xhci, UX_XHCI_RING *ep_ring, uint32_t ep_state, uint32_t num_trbs)
{
    uint32_t num_trbs_needed = 0;
    /* Make sure the endpoint has been added to xHC schedule */
    switch (ep_state)
    {
        case EP_STATE_DISABLED:
            /*
             * USB core changed config/interfaces without notifying us,
             * or hardware is reporting the wrong state.
             */
#ifdef DEBUG
            printf("WARN urb submitted to disabled ep\n");
#endif
            return -1;
        case EP_STATE_ERROR:
#ifdef DEBUG
            printf("WARN waiting for error on ep to be cleared\n");
#endif
            /* FIXME event handling code for error needs to clear it */
            return -2;
        case EP_STATE_HALTED:
#ifdef DEBUG
            printf("WARN halted endpoint, queueing URB anyway.\n");
#endif
        case EP_STATE_STOPPED:
        case EP_STATE_RUNNING:
            break;
        default:
#ifdef DEBUG
            printf("ERROR unknown endpoint state for ep\n");
#endif
            /*
             * FIXME issue Configure Endpoint command to try to get the HC
             * back into a known state.
             */
            return -3;
    }
    while (1)
    {
        if (room_on_ring(xhci, ep_ring, num_trbs))
            break;
        if (ep_ring == xhci->cmd_ring)
        {
#ifdef DEBUG
            printf("Do not support expand command ring\n");
#endif
            return -1;
        }
#ifdef DEBUG
        printf("ERROR no room on ep ring, try ring expansion\n");
#endif
        num_trbs_needed = num_trbs - ep_ring->num_trbs_free;
        if (_ux_hcd_xhci_ring_expansion(xhci, ep_ring, num_trbs_needed))
        {
#ifdef DEBUG
            printf("Ring expansion failed\n");
#endif
            return -1;
        }
    }
    while (trb_is_link(ep_ring->enqueue))
    {
        if (ep_ring->type != TYPE_ISOC)
            ep_ring->enqueue->link.control &= (~TRB_CHAIN);
        else
            ep_ring->enqueue->link.control |= (TRB_CHAIN);

        ep_ring->enqueue->link.control ^= (TRB_CYCLE);

        /* Toggle the cycle bit after the last ring segment. */
        if (link_trb_toggles_cycle(ep_ring->enqueue))
            ep_ring->cycle_state ^= 1;

        ep_ring->enq_seg = ep_ring->enq_seg->next;
        ep_ring->enqueue = ep_ring->enq_seg->trbs;
    }
    return 0;
}

int32_t prepare_transfer(UX_HCD_XHCI *xhci, UX_XHCI_VIRT_DEVICE *xdev,
        uint32_t ep_index, uint32_t stream_id, uint32_t num_trbs, UX_TRANSFER *urb, uint32_t td_index)
{
    int32_t ret;
    UX_URB_PRIV *urb_priv;
    UX_XHCI_TD   *td;
    UX_XHCI_RING *ep_ring;
    UX_XHCI_EP_CTX *ep_ctx;

    if (!xhci || ! xdev || !urb)
    {
#ifdef DEBUG
        printf("argument invalid\n");
#endif
        return -1;
    }
    ep_ctx = _ux_hcd_xhci_get_ep_ctx(xhci, xdev->out_ctx, ep_index);
    ep_ring = _ux_hcd_xhci_stream_id_to_ring(xdev, ep_index, stream_id);
    if (!ep_ring)
    {
#ifdef DEBUG
        printf("Can't prepare ring for bad stream ID %u\n",stream_id);
#endif
        return -1;
    }
    ret = prepare_ring(xhci, ep_ring, GET_EP_CTX_STATE(ep_ctx), num_trbs);
    if (ret)
        return ret;
    urb_priv = urb->hcpriv;
    td = &urb_priv->td[td_index];

    INIT_LIST_HEAD(&td->td_list);
    INIT_LIST_HEAD(&td->cancelled_td_list);

    td->urb = urb;
    /* Add this TD to the tail of the endpoint ring's TD list */
    _ux_hcd_xhci_list_add_tail(&td->td_list, &ep_ring->td_list);
    td->start_seg = ep_ring->enq_seg;
    td->first_trb = ep_ring->enqueue;
    return 0;
}

uint32_t count_trbs(uint64_t addr, uint32_t len)
{
    uint32_t num_trbs;
    num_trbs = DIV_ROUND_UP(len + (addr & (TRB_MAX_BUFF_SIZE - 1)),TRB_MAX_BUFF_SIZE);
    if (num_trbs == 0)
        num_trbs++;
    return num_trbs;
}

void giveback_first_trb(
        UX_HCD_XHCI *xhci,
        int32_t slot_id,
        uint32_t ep_index,
        uint32_t stream_id,
        uint32_t start_cycle,
        UX_XHCI_GENERIC_TRB *start_trb)
{
    /*
     * Pass all the TRBs to the hardware at once and make sure this write
     * isn't reordered.
     */
    if (start_cycle)
        start_trb->field[3] |= (start_cycle);
    else
        start_trb->field[3] &= (~TRB_CYCLE);
    _ux_hcd_xhci_ring_ep_doorbell(xhci, slot_id, ep_index, stream_id);
}

void check_interval(UX_HCD_XHCI *xhci,UX_TRANSFER *urb,UX_XHCI_EP_CTX *ep_ctx)
{
    uint32_t xhci_interval;
    uint32_t ep_interval;
    xhci_interval = EP_INTERVAL_TO_UFRAMES((ep_ctx->ep_info));
    ep_interval = xhci->interval;
    /* Convert to microframes */
    if (xhci->device->ux_device_speed == UX_LOW_SPEED_DEVICE ||
            xhci->device->ux_device_speed == UX_FULL_SPEED_DEVICE)
        ep_interval *= 8;

    if (xhci_interval != ep_interval)
    {
        xhci->interval = xhci_interval;

        /* Convert back to frames for LS/FS devices */
        if (xhci->device->ux_device_speed == UX_LOW_SPEED_DEVICE ||
                xhci->device->ux_device_speed == UX_FULL_SPEED_DEVICE)
            xhci->interval /= 8;
    }
}

/*
 * For xHCI 1.0 host controllers, TD size is the number of max packet sized
 * packets remaining in the TD (*not* including this TRB).
 *
 * Total TD packet count = total_packet_count =
 *     DIV_ROUND_UP(TD size in bytes / wMaxPacketSize)
 *
 * Packets transferred up to and including this TRB = packets_transferred =
 *     ROUNDDOWN(total bytes transferred including this TRB / wMaxPacketSize)
 *
 * TD size = total_packet_count - packets_transferred
 * For all hosts it must fit in bits 21:17, so it can't be bigger than 31.
 * This is taken care of in the TRB_TD_SIZE() macro
 *
 * The last TRB in a TD must have the TD size set to zero.
 */
uint32_t _ux_hcd_xhci_td_remainder(UX_HCD_XHCI *xhci, int32_t transferred,
        int32_t trb_buff_len, uint32_t td_total_len, UX_TRANSFER *urb,bool more_trbs_coming)
{
    uint32_t maxp, total_packet_count;

    /* MTK xHCI 0.96 contains some features from 1.0 */
    if (xhci->hci_version < 0x100)
        return ((td_total_len - transferred) >> 10);

    /* One TRB with a zero-length data packet. */
    if (!more_trbs_coming || (transferred == 0 && trb_buff_len == 0) ||
            trb_buff_len == td_total_len)
        return 0;

    /* for MTK xHCI 0.96, TD size include this TRB, but not in 1.x */
    if (xhci->hci_version < 0x100)
        trb_buff_len = 0;
    maxp = ux_endpoint_maxp(&urb->ux_transfer_request_endpoint->ux_endpoint_descriptor);
    total_packet_count = DIV_ROUND_UP(td_total_len, maxp);

    /* Queueing functions don't count the current TRB into transferred */
    return (total_packet_count - ((transferred + trb_buff_len) / maxp));
}

/*
 * The transfer burst count field of the isochronous TRB defines the number of
 * bursts that are required to move all packets in this TD.  Only SuperSpeed
 * devices can burst up to bMaxBurst number of packets per service interval.
 * This field is zero based, meaning a value of zero in the field means one
 * burst.  Basically, for everything but SuperSpeed devices, this field will be
 * zero.  Only xHCI 1.0 host controllers support this field.
 */
uint32_t _ux_hcd_xhci_get_burst_count(UX_HCD_XHCI *xhci, uint32_t total_packet_count)
{

    if (xhci->hci_version < 0x100 || (xhci->device->ux_device_speed == UX_HIGH_SPEED_DEVICE))
    {
        return 0;
    }
    return 0;//FIXME
}

/*
 * Returns the number of packets in the last "burst" of packets.  This field is
 * valid for all speeds of devices.  USB 2.0 devices can only do one "burst", so
 * the last burst packet count is equal to the total number of packets in the
 * TD.  SuperSpeed endpoints can have up to 3 bursts.  All but the last burst
 * must contain (bMaxBurst + 1) number of packets, but the last burst can
 * contain 1 to (bMaxBurst + 1) packets.
 */
uint32_t _ux_hcd_xhci_get_last_burst_packet_count(UX_HCD_XHCI *xhci, uint32_t total_packet_count)
{
    if (xhci->hci_version < 0x100)
        return 0;
    if (total_packet_count == 0)
        return 0;
    return total_packet_count - 1;
}


/****    Command Ring Operations   ****/

/* Generic function for queueing a command TRB on the command ring.
 * Check to make sure there's room on the command ring for one command TRB.
 * Also check that there's room reserved for commands that must not fail.
 * If this is a command that must not fail, meaning command_must_succeed = TRUE,
 * then only check for the number of reserved spots.
 * Don't decrement xhci->cmd_ring_reserved_trbs after we've queued the TRB
 * because the command event handler may want to resubmit a failed command.
 */

int32_t queue_command(UX_HCD_XHCI *xhci, UX_XHCI_COMMAND *cmd, UX_XHCI_TRB_INFO *trb_info,
        bool command_must_succeed)
{
    int32_t reserved_trbs = xhci->cmd_ring_reserved_trbs;
    int32_t ret;
    if ((xhci->xhc_state & UX_XHCI_STATE_DYING) || (xhci->xhc_state & UX_XHCI_STATE_HALTED))
    {
#ifdef DEBUG
        printf("xHCI dying or halted, can't queue_command\n");
#endif
        return -1;
    }
    if (!command_must_succeed)
        reserved_trbs++;
    ret = prepare_ring(xhci, xhci->cmd_ring, EP_STATE_RUNNING, reserved_trbs);
    if (ret < 0)
    {
#ifdef DEBUG
        printf("ERR: No room for command on command ring\n");
#endif
        if (command_must_succeed)
        {
#ifdef DEBUG
            printf("ERR: Reserved TRB counting for ""unfailable commands failed.\n");
#endif
        }
        return ret;
    }
    cmd->command_trb = xhci->cmd_ring->enqueue;

    /* if there are no other commands queued we start the timeout timer */
    if (_ux_hcd_xhci_list_empty(&xhci->cmd_list))
    {
        xhci->current_cmd = cmd;
    }

    _ux_hcd_xhci_list_add_tail(&cmd->cmd_list, &xhci->cmd_list);
    trb_info->cntrl_field |=  xhci->cmd_ring->cycle_state;
    /* Queue the TRB */
    queue_trb(xhci,xhci->cmd_ring, false, trb_info);
    return 0;
}

/* Queue a slot enable or disable request on the command ring */
int32_t _ux_hcd_xhci_queue_slot_control(UX_HCD_XHCI *xhci, UX_XHCI_COMMAND *cmd,
        uint32_t trb_type, uint32_t slot_id)
{
    UX_XHCI_TRB_INFO trb_info;
    trb_info.low_address = 0;
    trb_info.high_address = 0;
    trb_info.size = 0;
    trb_info.cntrl_field =  TRB_TYPE(trb_type) | SLOT_ID_FOR_TRB(slot_id);
    return queue_command(xhci, cmd, &trb_info,false);
}

/* Queue an address device command TRB */
int32_t _ux_hcd_xhci_queue_address_device(UX_HCD_XHCI *xhci, UX_XHCI_COMMAND *cmd,
        uint64_t in_ctx_ptr,
        uint32_t slot_id,
        UX_XHCI_SETUP_DEV setup)
{
    UX_XHCI_TRB_INFO  trb_info;
    trb_info.low_address = lower_32_bits(in_ctx_ptr);
    trb_info.high_address = upper_32_bits(in_ctx_ptr);
    trb_info.size = 0;
    trb_info.cntrl_field = TRB_TYPE(TRB_ADDR_DEV) | SLOT_ID_FOR_TRB(slot_id)
        | (setup == SETUP_CONTEXT_ONLY ? TRB_BSR : 0);
    return queue_command(xhci, cmd, &trb_info,false);
}

/* Queue a reset device command TRB */
int32_t _ux_hcd_xhci_queue_reset_device(UX_HCD_XHCI *xhci, UX_XHCI_COMMAND *cmd, uint32_t slot_id)
{
    UX_XHCI_TRB_INFO  trb_info;
    trb_info.low_address = 0;
    trb_info.high_address = 0;
    trb_info.size = 0;
    trb_info.cntrl_field = TRB_TYPE(TRB_RESET_DEV) | SLOT_ID_FOR_TRB(slot_id);
    /* Queue the command */
    return queue_command(xhci, cmd, &trb_info, false);
}

/* Queue a configure endpoint command TRB */
int32_t _ux_hcd_xhci_queue_configure_endpoint(
        UX_HCD_XHCI *xhci,
        UX_XHCI_COMMAND *cmd,
        uint64_t in_ctx_ptr,
        uint32_t slot_id,
        bool command_must_succeed)
{
    UX_XHCI_TRB_INFO  trb_info;
    trb_info.low_address = lower_32_bits(in_ctx_ptr);
    trb_info.high_address = upper_32_bits(in_ctx_ptr);
    trb_info.size = 0;
    trb_info.cntrl_field = TRB_TYPE(TRB_CONFIG_EP) | SLOT_ID_FOR_TRB(slot_id);
    /* Queue the command */
    return queue_command(xhci,cmd, &trb_info, command_must_succeed);
}

/* Queue an evaluate context command TRB */
int32_t _ux_hcd_xhci_queue_evaluate_context(
        UX_HCD_XHCI *xhci,
        UX_XHCI_COMMAND *cmd,
        uint64_t in_ctx_ptr,
        uint32_t slot_id,
        bool command_must_succeed)
{
    UX_XHCI_TRB_INFO  trb_info;
    trb_info.low_address = lower_32_bits(in_ctx_ptr);
    trb_info.high_address = upper_32_bits(in_ctx_ptr);
    trb_info.size = 0;
    trb_info.cntrl_field = TRB_TYPE(TRB_EVAL_CONTEXT) | SLOT_ID_FOR_TRB(slot_id);
    /* Queue the command */
    return queue_command(xhci,cmd, &trb_info, command_must_succeed);
}

/* Set Transfer Ring Dequeue Pointer command */
void _ux_hcd_xhci_queue_new_dequeue_state(
        UX_HCD_XHCI *xhci,
        uint32_t slot_id,
        uint32_t ep_index,
        UX_XHCI_DEQUEUE_STATE *deq_state)
{
    uint64_t addr;
    uint32_t trb_slot_id = SLOT_ID_FOR_TRB(slot_id);
    uint32_t trb_ep_index = EP_ID_FOR_TRB(ep_index);
    uint32_t trb_stream_id = STREAM_ID_FOR_TRB(deq_state->stream_id);
    uint32_t trb_sct = 0;
    uint32_t type = TRB_TYPE(TRB_SET_DEQ);
    UX_XHCI_VIRT_EP *ep;
    UX_XHCI_COMMAND *cmd;
    UX_XHCI_TRB_INFO  trb_info;
    int32_t ret;
    addr = _ux_hcd_xhci_trb_virt_to_dma(deq_state->new_deq_seg, deq_state->new_deq_ptr);
    if (addr == 0)
    {
#ifdef DEBUG
        printf("WARN Cannot submit Set TR Deq Ptr\n");
#endif
        return;
    }
    ep = &xhci->devs[slot_id]->eps[ep_index];
    if ((ep->ep_state & SET_DEQ_PENDING))
    {
#ifdef DEBUG
        printf("WARN Cannot submit Set TR Deq Ptr\n");
        printf("A Set TR Deq Ptr command is pending.\n");
#endif
        return;
    }
    /* Allocate the command */
    cmd = _ux_hcd_xhci_alloc_command(xhci);
    if (!cmd)
        return;
    ep->queued_deq_seg = deq_state->new_deq_seg;
    ep->queued_deq_ptr = deq_state->new_deq_ptr;
    if (deq_state->stream_id)
        trb_sct = SCT_FOR_TRB(SCT_PRI_TR);
    trb_info.low_address = lower_32_bits(addr) | trb_sct | deq_state->new_cycle_state;
    trb_info.high_address = upper_32_bits(addr);
    trb_info.size =  trb_stream_id;
    trb_info.cntrl_field = trb_slot_id | trb_ep_index | type;
    /* Queue the command */
    ret  = queue_command(xhci, cmd, &trb_info, false);
    if (ret < 0)
    {
        _ux_hcd_xhci_free_command(xhci, cmd);
        return;
    }

    /* Stop the TD queueing code from ringing the doorbell until
     * this command completes.  The HC won't set the dequeue pointer
     * if the ring is running, and ringing the doorbell starts the
     * ring running.
     */
    ep->ep_state |= SET_DEQ_PENDING;
}

int32_t _ux_hcd_xhci_queue_reset_ep(UX_HCD_XHCI *xhci, UX_XHCI_COMMAND *cmd, uint32_t slot_id,
        uint32_t ep_index, UX_XHCI_EP_RESET_TYPE reset_type)
{
    uint32_t trb_slot_id = SLOT_ID_FOR_TRB(slot_id);
    uint32_t trb_ep_index = EP_ID_FOR_TRB(ep_index);
    uint32_t type = TRB_TYPE(TRB_RESET_EP);
    UX_XHCI_TRB_INFO  trb_info;
    trb_info.low_address = 0;
    trb_info.high_address = 0;
    trb_info.size = 0;
    trb_info.cntrl_field = trb_slot_id | trb_ep_index | type;
    if (reset_type == EP_SOFT_RESET)
        type |= TRB_TSP;
    /* Queue the command */
    return queue_command(xhci, cmd, &trb_info, false);
}

void _ux_hcd_xhci_setup_input_ctx_for_config_ep(
        UX_HCD_XHCI *xhci,
        UX_XHCI_CONTAINER_CTX *in_ctx,
        UX_XHCI_CONTAINER_CTX *out_ctx,
        UX_XHCI_INPUT_CONTROL_CTX *ctrl_ctx,
        uint32_t add_flags,
        uint32_t drop_flags)
{
    ctrl_ctx->add_flags = (add_flags);
    ctrl_ctx->drop_flags = (drop_flags);
    _ux_hcd_xhci_slot_copy(xhci, in_ctx, out_ctx);
    ctrl_ctx->add_flags |= (SLOT_FLAG);
}

static void _ux_hcd_xhci_setup_input_ctx_for_quirk(
        UX_HCD_XHCI *xhci,
        uint32_t slot_id,
        uint32_t ep_index,
        UX_XHCI_DEQUEUE_STATE *deq_state)
{
    UX_XHCI_INPUT_CONTROL_CTX *ctrl_ctx;
    UX_XHCI_CONTAINER_CTX *in_ctx;
    UX_XHCI_EP_CTX *ep_ctx;
    uint32_t added_ctxs;
    uint64_t addr;
    in_ctx = xhci->devs[slot_id]->in_ctx;
    ctrl_ctx = _ux_hcd_xhci_get_input_control_ctx(in_ctx);
    if (!ctrl_ctx)
    {
#ifdef DEBUG
        printf("Could not get input context, bad type\n");
#endif
        return;
    }
    _ux_hcd_xhci_endpoint_copy(xhci, xhci->devs[slot_id]->in_ctx, xhci->devs[slot_id]->out_ctx, ep_index);
    ep_ctx = _ux_hcd_xhci_get_ep_ctx(xhci, in_ctx, ep_index);
    addr = _ux_hcd_xhci_trb_virt_to_dma(deq_state->new_deq_seg, deq_state->new_deq_ptr);
    if (addr == 0)
    {
#ifdef DEBUG
        printf("WARN Cannot submit config ep after ""reset ep command\n");
#endif
        return;
    }
    ep_ctx->deq = (addr | deq_state->new_cycle_state);
    added_ctxs = _ux_hcd_xhci_get_endpoint_flag_from_index(ep_index);
    _ux_hcd_xhci_setup_input_ctx_for_config_ep(xhci, xhci->devs[slot_id]->in_ctx,
            xhci->devs[slot_id]->out_ctx, ctrl_ctx, added_ctxs, added_ctxs);
}

void _ux_hcd_xhci_cleanup_stalled_ring(
        UX_HCD_XHCI *xhci,
        uint32_t ep_index,
        uint32_t stream_id,
        UX_XHCI_TD *td)
{
    UX_XHCI_DEQUEUE_STATE deq_state;
    UX_DEVICE *udev = xhci->device;
    /* We need to move the HW's dequeue pointer past this TD,
     * or it will attempt to resend it on the next doorbell ring.
     */
    xhci->slot_id = 1;
    _ux_hcd_xhci_find_new_dequeue_state(xhci, xhci->slot_id, ep_index, stream_id, td, &deq_state);

    if (!deq_state.new_deq_ptr || !deq_state.new_deq_seg)
        return;

#ifdef DEBUG
    printf("Queueing new dequeue state");
#endif
    _ux_hcd_xhci_queue_new_dequeue_state(xhci, xhci->slot_id, ep_index, &deq_state);

}

uint32_t _ux_hcd_xhci_get_endpoint_index(UX_ENDPOINT_DESCRIPTOR *desc)
{
    uint32_t index;
    if ((desc->bmAttributes.xfer) == UX_CONTROL_ENDPOINT)
        index = (unsigned int) ((desc->bEndpointAddress & 0x0f) * 2);
    else
        index = (unsigned int) ((desc->bEndpointAddress & 0x0f ) * 2) +
            (ux_endpoint_dir_in(desc) ? 1 : 0) - 1;
    return index;
}
/* Add an endpoint to a new possible bandwidth configuration for this device.
 * Only one call to this function is allowed per endpoint before
 * check_bandwidth() or reset_bandwidth() must be called.  */
int32_t _ux_hcd_xhci_add_endpoint(UX_HCD_XHCI *xhci, UX_DEVICE *udev, UX_ENDPOINT *ep)
{
    UX_XHCI_CONTAINER_CTX *in_ctx;
    uint32_t ep_index;
    UX_XHCI_INPUT_CONTROL_CTX *ctrl_ctx;
    uint32_t added_ctxs;
    UX_XHCI_VIRT_DEVICE *virt_dev;
    int32_t ret = 0;

    if (!xhci || !udev || !ep)
    {
        /* So we won't queue a reset ep command for a root hub */
#ifdef DEBUG
        printf(" invalid device and endpoint \n");
#endif
        return ret;
    }
    if (xhci->xhc_state & UX_XHCI_STATE_DYING)
        return -1;
    added_ctxs = _ux_hcd_xhci_get_endpoint_flag(&ep->ux_endpoint_descriptor);
    if (added_ctxs == SLOT_FLAG || added_ctxs == EP0_FLAG)
    {
#ifdef DEBUG
        printf("xHCI can't add slot or ep 0 %#x\n", added_ctxs);
#endif
        return 0;
    }
    virt_dev = xhci->devs[xhci->slot_id];
    in_ctx = virt_dev->in_ctx;
    /* Get the input control context address  */
    ctrl_ctx = _ux_hcd_xhci_get_input_control_ctx(in_ctx);
    if (!ctrl_ctx)
    {
#ifdef DEBUG
        printf("Could not get input context, bad type\n");
#endif
        return 0;
    }
    /* Get the endpoint index  */
    ep_index = _ux_hcd_xhci_get_endpoint_index(&ep->ux_endpoint_descriptor);
    /* If this endpoint is already in use, and the upper layers are trying
     * to add it again without dropping it, reject the addition.
     */
    if (virt_dev->eps[ep_index].ring && !((ctrl_ctx->drop_flags) & added_ctxs))
    {
#ifdef DEBUG
        printf("Trying to add endpoint without dropping it\n");
#endif
        return -1;
    }
    /* If the HCD has already noted the endpoint is enabled,
     * ignore this request.
     */
    if ((ctrl_ctx->add_flags) & added_ctxs)
    {
#ifdef DEBUG
        printf("Endpoints already enabled\n");
#endif
        return 0;
    }
    /* Initialize the endpoints  */
    if (_ux_hcd_xhci_endpoint_init(xhci, virt_dev, udev, ep) < 0)
    {
#ifdef DEBUG
        printf("could not initialize ep\n");
#endif
        return -1;
    }
    ctrl_ctx->add_flags |= (added_ctxs);
    return 0;
}
/**
  \fn           _ux_hcd_xhci_disable_slot
  \brief        Disable the device slot id
  \param[in]    xhci pointer
  \param[in]    slot id
  \return       On success 0 remains error
 */

int32_t _ux_hcd_xhci_disable_slot(UX_HCD_XHCI *xhci, uint32_t slot_id)
{
    UX_XHCI_COMMAND *command;
    uint32_t state;
    int32_t ret = 0;
    /* Allocate the command to be execute  */
    command = _ux_hcd_xhci_alloc_command(xhci);
    if (!command)
        return -1;

    /* Read the Host controller status   */
    /* Don't disable the slot if the host controller is dead. */
    state =  xhci->op_regs->USBSTS;
    if (state == 0xffffffff || (xhci->xhc_state & UX_XHCI_STATE_DYING) ||
            (xhci->xhc_state & UX_XHCI_STATE_HALTED))
    {
        _ux_utility_memory_free(command);
        return -1;
    }
    /*Queue the disable slot command  */
    ret = _ux_hcd_xhci_queue_slot_control(xhci, command, TRB_DISABLE_SLOT, slot_id);
    if (ret)
    {
        _ux_utility_memory_free(command);
        return ret;
    }
    /* Ring the HC doorbell for command ring */
    _ux_hcd_xhci_ring_cmd_db(xhci);
    return ret;
}

void _ux_hcd_xhci_free_dev(UX_HCD_XHCI *xhci, UX_DEVICE *udev)
{
    UX_XHCI_VIRT_DEVICE *virt_dev;
    UX_XHCI_SLOT_CTX *slot_ctx;
    int32_t ret;
    virt_dev = xhci->devs[xhci->slot_id];
    slot_ctx = _ux_hcd_xhci_get_slot_ctx(xhci, virt_dev->out_ctx);
    virt_dev->udev = NULL;
    ret = _ux_hcd_xhci_disable_slot(xhci, xhci->slot_id);
    if (ret)
        _ux_hcd_xhci_free_virt_device(xhci, xhci->slot_id);
}
/**
  \fn           _ux_hcd_xhci_alloc_dev
  \brief        enable the device slot commands
  \param[in]    xhci pointer
  \param[in]    Device pointer
  \return       On success 0 remains error
 */

int32_t _ux_hcd_xhci_alloc_dev(UX_HCD_XHCI *xhci,UX_DEVICE *udev)
{
    int32_t ret;
    uint32_t slot_id;
    UX_XHCI_COMMAND *command;
    osal_mutex_lock(&xhci -> ux_hcd_xhci_periodic_mutex, OSAL_TIMEOUT_WAIT_FOREVER);
    /* Allocate memory to command  */
    command = _ux_hcd_xhci_alloc_command(xhci);
    if (!command)
        return 1;
    command->fCompletion = 1;
    /* Queue the slot enable or disable on command ring  */
    ret = _ux_hcd_xhci_queue_slot_control(xhci, command, TRB_ENABLE_SLOT, 0);
    if (ret)
    {
        osal_mutex_unlock(&xhci -> ux_hcd_xhci_periodic_mutex);
#ifdef DEBUG
        printf("FIXME: allocate a command ring segment\n");
#endif
        _ux_hcd_xhci_free_command(xhci, command);
        return 1;
    }
    /* Ring the HC doorbell for command ring  */
    _ux_hcd_xhci_ring_cmd_db(xhci);
    osal_mutex_unlock(&xhci -> ux_hcd_xhci_periodic_mutex);

    /* wait for the command completion  */
    while(command->fCompletion);
    slot_id = command->slot_id;
    if (!slot_id || command->status != COMP_SUCCESS)
    {
#ifdef DEBUG
        printf("Error while assigning device slot ID : %d\n", command->status);
#endif
        _ux_hcd_xhci_free_command(xhci, command);
        return 1;
    }
    _ux_hcd_xhci_free_command(xhci, command);
    if (!_ux_hcd_xhci_alloc_virt_device(xhci, slot_id, udev))
    {
#ifdef DEBUG
        printf("Could not allocate xHCI USB device data structures\n");
#endif
        goto disable_slot;
    }
    xhci->slot_id = slot_id;
    return 0;

disable_slot:
    ret = _ux_hcd_xhci_disable_slot(xhci, xhci->slot_id);
    if (ret)
        _ux_hcd_xhci_free_virt_device(xhci, xhci->slot_id);
    return 1;
}

static int32_t _ux_hcd_xhci_evaluate_context_result(UX_HCD_XHCI *xhci, UX_DEVICE *udev, volatile uint32_t *cmd_status)
{
    int32_t ret;
    switch (*cmd_status)
    {
        case COMP_COMMAND_ABORTED:
        case COMP_COMMAND_RING_STOPPED:
#ifdef DEBUG
            printf( "Timeout while waiting for evaluate context command\n");
#endif
            ret = -1;
            break;
        case COMP_PARAMETER_ERROR:
#ifdef DEBUG
            printf("WARN: xHCI driver setup invalid evaluate context command.\n");
#endif
            ret = -1;
            break;
        case COMP_SLOT_NOT_ENABLED_ERROR:
#ifdef DEBUG
            printf("WARN: slot not enabled for evaluate context command.\n");
#endif
            ret = -1;
            break;
        case COMP_CONTEXT_STATE_ERROR:
#ifdef DEBUG
            printf("WARN: invalid context state for evaluate context command.\n");
#endif
            ret = -1;
            break;
        case COMP_INCOMPATIBLE_DEVICE_ERROR:
#ifdef DEBUG
            printf("ERROR: Incompatible device for evaluate context command.\n");
#endif
            ret = -1;
            break;
        case COMP_MAX_EXIT_LATENCY_TOO_LARGE_ERROR:
            /* Max Exit Latency too large error */
#ifdef DEBUG
            printf( "WARN: Max Exit Latency too large\n");
#endif
            ret = -1;
            break;
        case COMP_SUCCESS:
#ifdef DEBUG
            printf("Successful evaluate context command");
#endif
            ret = 0;
            break;
        default:
#ifdef DEBUG
            printf( "ERROR: unexpected command completion code 0x%x.\n",*cmd_status);
#endif
            ret = -1;
            break;
    }
    return ret;
}

static int32_t _ux_hcd_xhci_configure_endpoint_result(
        UX_HCD_XHCI *xhci,
        UX_DEVICE *udev,
        volatile uint32_t *cmd_status)
{
    int32_t ret;
    switch (*cmd_status)
    {
        case COMP_COMMAND_ABORTED:
        case COMP_COMMAND_RING_STOPPED:
#ifdef DEBUG
            printf("Timeout while waiting for configure endpoint command\n");
#endif
            ret = -1;
            break;
        case COMP_RESOURCE_ERROR:
#ifdef DEBUG
            printf("Not enough host controller resources for new device state.\n");
#endif
            ret = -1;
            break;
        case COMP_BANDWIDTH_ERROR:
        case COMP_SECONDARY_BANDWIDTH_ERROR:
#ifdef DEBUG
            printf("Not enough bandwidth for new device state.\n");
#endif
            ret = -1;
            break;
        case COMP_TRB_ERROR:
            /* the HCD set up something wrong */
#ifdef DEBUG
            printf("ERROR: Endpoint drop flag = 0, ""add flag = 1, "
                    "and endpoint is not disabled.\n");
#endif
            ret = -1;
            break;
        case COMP_INCOMPATIBLE_DEVICE_ERROR:
#ifdef DEBUG
            printf("ERROR: Incompatible device for endpoint configure command.\n");
#endif
            ret = -1;
            break;
        case COMP_SUCCESS:
#ifdef DEBUG
            printf("Successful Endpoint Configure command\n");
#endif
            ret = 0;
            break;
        default:
#ifdef DEBUG
            printf("ERROR: unexpected command completion code 0x%x.\n",*cmd_status);
#endif
            ret = -1;
            break;
    }
    return ret;
}

/**
  \fn           _ux_hcd_xhci_configure_endpoint
  \brief        Issue a configure endpoint command or evaluate context command
  and wait for it to finish.
  \param[in]    xhci global pointer
  \param[in]    urb pointer
  \param[in]    command pointer
  \param[in]    context change as false
  \param[in]    command must success as false
  \return       On success 0 remains error
 **/

int32_t _ux_hcd_xhci_configure_endpoint(UX_HCD_XHCI *xhci, UX_DEVICE *udev, UX_XHCI_COMMAND *command,
        bool ctx_change, bool must_succeed)
{
    int32_t ret;
    unsigned long events;
    UX_XHCI_INPUT_CONTROL_CTX *ctrl_ctx;
    if (!command)
        return -1;
    if (xhci->xhc_state & UX_XHCI_STATE_DYING)
    {
#ifdef DEBUG
        printf("xhci state dying\n");
#endif
        return -1;
    }
    ctrl_ctx = _ux_hcd_xhci_get_input_control_ctx(command->in_ctx);
    if (!ctrl_ctx)
    {
        printf(" Could not get input context, bad type.\n");
        return -1;
    }
    command->fCompletion = 1;
    if (!ctx_change)
        /* Queue the endpoint configure command   */
        ret = _ux_hcd_xhci_queue_configure_endpoint(xhci, command, command->in_ctx->dma,
                xhci->slot_id, must_succeed);
    else
        /* Queue the evaluate context command  */
        ret = _ux_hcd_xhci_queue_evaluate_context(xhci, command, command->in_ctx->dma,
                xhci->slot_id, must_succeed);
    if (ret < 0)
    {
#ifdef DEBUG
        printf("FIXME allocate a new ring segment");
#endif
        return -1;
    }
    /* Ring the HC doorbell for command ring  */
    _ux_hcd_xhci_ring_cmd_db(xhci);
    /* Wait for the command complete */
    while(command->fCompletion);
    if (!ctx_change)
        ret = _ux_hcd_xhci_configure_endpoint_result(xhci, udev, &command->status);
    else
        ret = _ux_hcd_xhci_evaluate_context_result(xhci, udev, &command->status);

    return ret;
}

/*
 * Full speed devices may have a max packet size greater than 8 bytes, but the
 * USB core doesn't know that until it reads the first 8 bytes of the
 * descriptor.  If the usb_device's max packet size changes after that point,
 * we need to issue an evaluate context command and wait on it.
 */
int32_t _ux_hcd_xhci_check_maxpacket(UX_HCD_XHCI *xhci,uint32_t slot_id,uint32_t ep_index,
        UX_TRANSFER *urb)
{
    UX_XHCI_CONTAINER_CTX *out_ctx;
    UX_XHCI_INPUT_CONTROL_CTX *ctrl_ctx;
    UX_XHCI_EP_CTX *ep_ctx;
    UX_XHCI_COMMAND *command;
    int32_t max_packet_size;
    int32_t hw_max_packet_size;
    int32_t ret = 0;
    out_ctx = xhci->devs[slot_id]->out_ctx;
    ep_ctx = _ux_hcd_xhci_get_ep_ctx(xhci, out_ctx, ep_index);
    hw_max_packet_size = MAX_PACKET_DECODED((ep_ctx->ep_info2));
    max_packet_size = ux_endpoint_maxp(&urb->ux_transfer_request_endpoint->ux_endpoint_descriptor);
    if (hw_max_packet_size != max_packet_size)
    {
#ifdef DEBUG
        printf("Max Packet Size for ep 0 changed.");
        printf("Max packet size in usb_device = %d\n",max_packet_size);
        printf("Max packet size in xHCI HW = %d\n",hw_max_packet_size);
        printf("Issuing evaluate context command.\n");
#endif

        /* Set up the input context flags for the command */
        /* FIXME: This won't work if a non-default control endpoint
         * changes max packet sizes. */
        command = _ux_hcd_xhci_alloc_command(xhci);
        if (!command)
            return -1;
        command->in_ctx = xhci->devs[slot_id]->in_ctx;
        ctrl_ctx = _ux_hcd_xhci_get_input_control_ctx(command->in_ctx);
        if (!ctrl_ctx)
        {
#ifdef DEBUG
            printf("Could not get input context, bad type.\n");
#endif
            ret = -1;
            goto command_cleanup;
        }
        /* Set up the modified control endpoint 0 */
        _ux_hcd_xhci_endpoint_copy(xhci, xhci->devs[slot_id]->in_ctx,
                xhci->devs[slot_id]->out_ctx, ep_index);

        ep_ctx = _ux_hcd_xhci_get_ep_ctx(xhci, command->in_ctx, ep_index);
        ep_ctx->ep_info2 &= (~MAX_PACKET_MASK);
        ep_ctx->ep_info2 |= (MAX_PACKET(max_packet_size));
        ctrl_ctx->add_flags = (EP0_FLAG);
        ctrl_ctx->drop_flags = 0;
        ret = _ux_hcd_xhci_configure_endpoint(xhci, xhci->device, command,
                true, false);
        /* Clean up the input context for later use by bandwidth
         *   functions.*/
        ctrl_ctx->add_flags = (SLOT_FLAG);
command_cleanup:
        _ux_utility_memory_free(command);
    }
    return ret;
}

/* Setup an xHCI virtual device for a Set Address command */
int32_t _ux_hcd_xhci_setup_addressable_virt_dev(UX_HCD_XHCI *xhci, UX_DEVICE *udev)
{
    UX_XHCI_VIRT_DEVICE *vir_dev;
    UX_XHCI_EP_CTX  *ep0_ctx;
    UX_XHCI_SLOT_CTX    *slot_ctx;
    uint32_t port_num;
    uint32_t max_packets;
    vir_dev = xhci->devs[xhci->slot_id];
    /* Slot ID 0 is reserved */
    if (xhci->slot_id == 0 || !vir_dev)
    {
#ifdef DEBUG
        printf( "Slot ID %d is not assigned to this device\n",xhci->slot_id);
#endif
        return -1;
    }
    ep0_ctx = _ux_hcd_xhci_get_ep_ctx(xhci, vir_dev->in_ctx, 0);
    slot_ctx = _ux_hcd_xhci_get_slot_ctx(xhci, vir_dev->in_ctx);

    /* Only the control endpoint is valid - one endpoint context */
    slot_ctx->dev_info |= (LAST_CTX(1));
    udev->ux_device_speed = UX_HIGH_SPEED_DEVICE;
    switch (udev->ux_device_speed)
    {
        case UX_HIGH_SPEED_DEVICE:
            slot_ctx->dev_info |= (SLOT_SPEED_HS);
            max_packets = MAX_PACKET(64);
            break;
            /* USB core guesses at a 64-byte max packet first for FS devices */
        case UX_FULL_SPEED_DEVICE:
            slot_ctx->dev_info |= (SLOT_SPEED_FS);
            max_packets = MAX_PACKET(64);
            break;
        case UX_LOW_SPEED_DEVICE:
            slot_ctx->dev_info |= (SLOT_SPEED_LS);
            max_packets = MAX_PACKET(8);
            break;
        default:
            /* Speed was set earlier, this shouldn't happen. */
            return -1;
    }
    /* Find the root hub port this device is under */
    port_num = HCS_MAX_PORTS(xhci->hcs_params1);
    if (!port_num)
        return -1;
    slot_ctx->dev_info2 |= (ROOT_HUB_PORT(port_num));
    /* Set the port number in the virtual_device to the faked port number */
    vir_dev->fake_port = udev->ux_device_port_location = port_num;
    vir_dev->real_port = port_num;
    vir_dev->bw_table = &xhci->rh_bw[port_num - 1].bw_table;
    /* ring already allocated */
    ep0_ctx->ep_info2 = (EP_TYPE(CTRL_EP));

    /* EP 0 can handle "burst" sizes of 1, so Max Burst Size field is 0 */
    ep0_ctx->ep_info2 |= (MAX_BURST(0) | ERROR_COUNT(3) |  max_packets);

    ep0_ctx->deq = (vir_dev->eps[0].ring->first_seg->dma |
            vir_dev->eps[0].ring->cycle_state);
    return 0;
}

void _ux_hcd_xhci_copy_ep0_dequeue_into_input_ctx(UX_HCD_XHCI *xhci, UX_DEVICE *udev)
{
    UX_XHCI_VIRT_DEVICE *virt_dev;
    UX_XHCI_EP_CTX  *ep0_ctx;
    UX_XHCI_RING  *ep_ring;
    virt_dev = xhci->devs[xhci->slot_id];
#ifdef DEBUG
    printf("_ux_hcd_xhci_copy_ep0_dequeue_into_input_ctx\n");
#endif
    ep0_ctx = _ux_hcd_xhci_get_ep_ctx(xhci, virt_dev->in_ctx, 0);
    ep_ring = virt_dev->eps[0].ring;
    /*
     * we don't keep track of the dequeue pointer very well after a
     * Set TR dequeue pointer, so we're setting the dequeue pointer of the
     * host to our enqueue pointer.  This should only be called after a
     * configured device has reset, so all control transfers should have
     * been completed or cancelled before the reset.
     */
    ep0_ctx->deq = (_ux_hcd_xhci_trb_virt_to_dma(ep_ring->enq_seg,
                ep_ring->enqueue) | ep_ring->cycle_state);
}

/**
  \fn           _ux_hcd_xhci_setup_device
  \brief        Issue an Address Device command and optionally send a corresponding
  SetAddress request to the device
  \param[in]    xhci global pointer
  \param[in]    Device pointer
  \param[in]    Device context or address
  \return       On success 0 remains error
 */

static int32_t _ux_hcd_xhci_setup_device(UX_HCD_XHCI *xhci, UX_DEVICE *udev,
        UX_XHCI_SETUP_DEV  setup)
{
    const char *act = setup == SETUP_CONTEXT_ONLY ? "context" : "address";
    UX_XHCI_VIRT_DEVICE *virt_dev;
    int32_t ret = 0;
    UX_XHCI_SLOT_CTX *slot_ctx;
    UX_XHCI_INPUT_CONTROL_CTX *ctrl_ctx;
    UX_XHCI_COMMAND *command = NULL;
    if (xhci->xhc_state)
    {    /* dying, removing or halted */
        ret = -1;
        goto out;
    }
    /* check the slot id */
    if (!xhci->slot_id)
    {
#ifdef DEBUG
        printf("Bad Slot ID %d", xhci->slot_id);
#endif
        ret = -1;
        goto out;
    }

    virt_dev = xhci->devs[xhci->slot_id];
    /* if virtual device is invalid then return error  */
    if ((!virt_dev))
    {
#ifdef DEBUG
        printf("Virt dev invalid for slot_id 0x%d!\n",xhci->slot_id);
#endif
        ret = -1;
        goto out;
    }
    /* Get the slot context  */
    slot_ctx = _ux_hcd_xhci_get_slot_ctx(xhci, virt_dev->out_ctx);

    if (setup == SETUP_CONTEXT_ONLY)
    {
        if (GET_SLOT_STATE((slot_ctx->dev_state)) == SLOT_STATE_DEFAULT)
        {
#ifdef DEBUG
            printf("Slot already in default state\n");
#endif
            goto out;
        }
    }
    command = _ux_hcd_xhci_alloc_command(xhci);
    if (!command)
    {
        ret = -1;
        goto out;
    }
    osal_mutex_lock(&xhci -> ux_hcd_xhci_periodic_mutex, OSAL_TIMEOUT_WAIT_FOREVER);
    command->in_ctx = virt_dev->in_ctx;
    command->fCompletion = 1;
    slot_ctx = _ux_hcd_xhci_get_slot_ctx(xhci, virt_dev->in_ctx);
    ctrl_ctx = _ux_hcd_xhci_get_input_control_ctx(virt_dev->in_ctx);
    if (!ctrl_ctx)
    {
#ifdef DEBUG
        printf(" Could not get input context, bad type.\n");
#endif
        ret = -1;
        goto out;
    }
    /*
     * If this is the first Set Address since device plug-in or
     * virt_device realloaction after a resume with an xHCI power loss,
     * then set up the slot context.
     */
    if (!slot_ctx->dev_info)
        _ux_hcd_xhci_setup_addressable_virt_dev(xhci, udev);
    /* Otherwise, update the control endpoint ring enqueue pointer. */
    else
        _ux_hcd_xhci_copy_ep0_dequeue_into_input_ctx(xhci, udev);

    ctrl_ctx->add_flags = (SLOT_FLAG | EP0_FLAG);
    ctrl_ctx->drop_flags = 0;
    /* queue the address device command TRB  */
    ret = _ux_hcd_xhci_queue_address_device(xhci, command, (uint64_t)(virt_dev->in_ctx->dma),
            xhci->slot_id, setup);
    if (ret)
    {
#ifdef DEBUG
        printf("FIXME: allocate a command ring segment");
#endif
        goto out;
    }
    /* Ring the command ring doorbell  */
    _ux_hcd_xhci_ring_cmd_db(xhci);
    osal_mutex_unlock(&xhci -> ux_hcd_xhci_periodic_mutex);
    /* Wait for the command completion  */
    while(command->fCompletion);

    switch (command->status){
        case COMP_COMMAND_ABORTED:
        case COMP_COMMAND_RING_STOPPED:
#ifdef DEBUG
            printf( "Timeout while waiting for setup device command\n");
#endif
            ret = -1;
            break;
        case COMP_CONTEXT_STATE_ERROR:
        case COMP_SLOT_NOT_ENABLED_ERROR:
#ifdef DEBUG
            printf("Setup ERROR: setup %s command for slot %d.\n", act, xhci->slot_id);
#endif
            ret = -1;
            break;
        case COMP_USB_TRANSACTION_ERROR:
#ifdef DEBUG
            printf("Device not responding to setup %s.\n", act);
#endif
            _ux_utility_memory_free(command);
            return -1;
        case COMP_INCOMPATIBLE_DEVICE_ERROR:
#ifdef DEBUG
            printf("ERROR: Incompatible device for setup %s command\n", act);
#endif
            ret = -1;
            break;
        case COMP_SUCCESS:
#ifdef DEBUG
            printf("Successful setup %s command\n", act);
#endif
            ret = 0;
            break;
        default:
#ifdef DEBUG
            printf("ERROR: unexpected setup %s command completion code 0x%x.\n",
                    act, command->status);
#endif
            ret = -1;
            break;
    }
    if (ret)
        goto out;
    /*
     * USB core uses address 1 for the roothubs, so we add one to the
     * address given back to us by the HC.
     */
    ctrl_ctx->add_flags = 0;
    ctrl_ctx->drop_flags = 0;
    slot_ctx = _ux_hcd_xhci_get_slot_ctx(xhci, virt_dev->out_ctx);
    udev->ux_device_address = (uint8_t)((slot_ctx->dev_state) & DEV_ADDR_MASK);
out:
    if (command)
    {
        _ux_utility_memory_free(command);
    }
    return ret;
}

int32_t skip_isoc_td(UX_HCD_XHCI *xhci, UX_XHCI_TD *td, UX_XHCI_TRANSFER_EVENT *event, UX_XHCI_VIRT_EP *ep, int32_t *status)
{

    UX_XHCI_RING *ep_ring;
    UX_URB_PRIV *urb_priv;
    ep_ring = ep->ring;
    /* The transfer is partly done. */
    td->urb->ux_transfer_request_status = -EXDEV;
    td->urb->ux_transfer_request_actual_length = 0;

    /* Update ring dequeue pointer */
    while (ep_ring->dequeue != td->last_trb)
        inc_deq(xhci, ep_ring);
    inc_deq(xhci, ep_ring);

    return _ux_hcd_xhci_td_cleanup(xhci, td, ep_ring, status);
}

int32_t _ux_hcd_xhci_address_device(UX_HCD_XHCI  *xhci, UX_DEVICE  *udev)
{
    return _ux_hcd_xhci_setup_device(xhci, udev, SETUP_CONTEXT_ADDRESS);
}

int32_t _ux_hcd_xhci_enable_device(UX_HCD_XHCI  *xhci, UX_DEVICE  *udev)
{
    return _ux_hcd_xhci_setup_device(xhci, udev, SETUP_CONTEXT_ONLY);
}
#endif //CFG_TUH_ENABLED
