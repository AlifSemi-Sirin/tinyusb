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
 * @file     ux_hcd_xhci_interrupt_handler.c
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
extern TX_EVENT_FLAGS_GROUP CONTROL_EP_FLAG;
/**
  \fn           _ux_xhci_event_irq_handler
  \brief        xhci driver interrupt handler
  \param[in]    xhci global pointer
  \param[in]    urb pointer
  \param[in]    slot id
  \param[in]    endpoint index
  \return       On success 0 remains error
 */
/*
 * xHCI spec says we can get an interrupt, and if the HC has an error condition,
 * we might get bad data out of the event ring.  Section 4.10.2.7 has a list of
 * indicators of an event TRB error, but we check the status *first* to be safe.
 */

void _ux_xhci_event_irq_handler(UX_HCD_XHCI *xhci)
{
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
    event_ring_deq = xhci->event_ring->dequeue;
    if(_ux_hcd_xhci_handle_events(xhci))
        _ux_hcd_xhci_update_erst_dequeue(xhci, event_ring_deq);
}

int32_t _ux_hcd_xhci_handle_events(UX_HCD_XHCI *xhci)
{
    UX_XHCI_TRB *event;
    int32_t update_ptrs = 1;
    int32_t ret;

    /* Event ring hasn't been allocated yet. */
    if (!xhci->event_ring || !xhci->event_ring->dequeue)
    {
#ifdef DEBUG
        printf("ERROR event ring not ready\n");
#endif
        return -4;
    }
    event = xhci->event_ring->dequeue;
    RTSS_InvalidateDCache_by_Addr(&event->event_cmd, sizeof(event->event_cmd));

    if (((event->event_cmd.flags) & TRB_CYCLE) != xhci->event_ring->cycle_state)
        return 0;
    /* FIXME: Handle more event types. */
    switch ((event->event_cmd.flags) & TRB_TYPE_BITMASK)
    {
        case TRB_TYPE(TRB_COMPLETION):
            handle_cmd_completion(xhci, &event->event_cmd);
            break;
        case TRB_TYPE(TRB_PORT_STATUS):
            handle_port_status(xhci, event);
            update_ptrs = 0;
            break;
        case TRB_TYPE(TRB_TRANSFER):
            ret = handle_transfer_event(xhci, &event->trans_event);
            if (ret >= 0)
                update_ptrs = 0;
            break;
        case TRB_TYPE(TRB_DEV_NOTE):
            handle_device_notification(xhci, event);
            break;
        default:
            if (((event->event_cmd.flags) & TRB_TYPE_BITMASK) >= TRB_TYPE(48))
            {
                handle_vendor_event(xhci, event);
            }
            else
            {
#ifdef DEBUG
                printf("ERROR unknown event type\n");
#endif
            }
    }
    if (xhci->xhc_state & UX_XHCI_STATE_DYING)
    {
#ifdef DEBUG
        printf("xHCI host dying, returning from ""event handler.\n");
#endif
        return 0;
    }
    if (update_ptrs)
        /* Update SW event ring dequeue pointer */
        inc_deq(xhci, xhci->event_ring);
    return 1;
}

/*
 * If this function returns an error condition, it means it got a Transfer
 * event with a corrupted Slot ID, Endpoint ID, or TRB DMA address.
 * At this point, the host controller is probably hosed and should be reset.
 */
int32_t handle_transfer_event(UX_HCD_XHCI *xhci, UX_XHCI_TRANSFER_EVENT *event)
{
    UX_XHCI_VIRT_DEVICE  *xdev;
    UX_XHCI_VIRT_EP   *ep;
    UX_XHCI_RING   *ep_ring;
    uint32_t  slot_id;
    int32_t  ep_index;
    UX_XHCI_TD   *td = NULL;
    uint64_t     ep_trb_dma;
    UX_XHCI_SEGMENT   *ep_seg;
    UX_XHCI_TRB     *ep_trb;
    int32_t status = -1;
    UX_XHCI_EP_CTX    *ep_ctx;
    UX_HCD_XHCI_LIST  *tmp;
    uint32_t trb_comp_code;
    int32_t td_num = 0;
    bool handling_skipped_tds = false;
    RTSS_InvalidateDCache_by_Addr(event, sizeof(*event));
    slot_id = TRB_TO_SLOT_ID((event->flags));
    ep_index = TRB_TO_EP_ID((event->flags)) - 1;
    trb_comp_code = GET_COMP_CODE((event->transfer_len));
    //RTSS_InvalidateDCache_by_Addr(&event->buffer, sizeof(event->buffer));
    ep_trb_dma = (event->buffer);

    xdev = xhci->devs[slot_id];
    if (!xdev)
    {
#ifdef DEBUG
        printf("ERROR Transfer event pointed to bad slot %u\n",slot_id);
#endif
        goto err_out;
    }
    ep = &xdev->eps[ep_index];
    ep_ring = ep->ring;
    ep_ctx = _ux_hcd_xhci_get_ep_ctx(xhci, xdev->out_ctx, ep_index);
    if (GET_EP_CTX_STATE(ep_ctx) == EP_STATE_DISABLED)
    {
#ifdef DEBUG
        printf("ERROR Transfer event for disabled endpoint slot %u ep %u\n", slot_id, ep_index);
#endif
        goto err_out;
    }
    /* Some transfer events don't always point to a trb, see xhci 4.17.4 */
    if (!ep_ring)
    {
        switch (trb_comp_code)
        {
            case COMP_STALL_ERROR:
            case COMP_USB_TRANSACTION_ERROR:
            case COMP_INVALID_STREAM_TYPE_ERROR:
            case COMP_INVALID_STREAM_ID_ERROR:
                _ux_hcd_xhci_cleanup_halted_endpoint(xhci, slot_id, ep_index, 0, NULL, EP_SOFT_RESET);
                goto cleanup;
            case COMP_RING_UNDERRUN:
            case COMP_RING_OVERRUN:
            case COMP_STOPPED_LENGTH_INVALID:
                goto cleanup;
            default:
#ifdef DEBUG
                printf("ERROR Transfer event for unknown stream ring slot %u ep %u\n", slot_id, ep_index);
#endif
                goto err_out;
        }
    }
    /* Count current td numbers if ep->skip is set */
    if (ep->skip)
    {
        _ux_hcd_xhci_list_for_each(tmp, &ep_ring->td_list)
            td_num++;
    }
    /* Look for common error cases */
    switch (trb_comp_code)
    {
        /* Skip codes that require special handling depending on
         * transfer type
         */
        case COMP_SUCCESS:
            if (EVENT_TRB_LEN((event->transfer_len)) == 0)
                break;
            if ( ep_ring->last_td_was_short)
            {
#ifdef DEBUG
                printf(" reeived short packed in last td\n");
#endif
                trb_comp_code = COMP_SHORT_PACKET;
            }
            else
            {
#ifdef DEBUG
                printf("Successful completion on short packet\n");
#endif
            }
            break;
        case COMP_SHORT_PACKET:
            break;
            /* Completion codes for endpoint stopped state */
        case COMP_STOPPED:
#ifdef DEBUG
            printf("Stopped on Transfer TRB for slot %u ep %u\n", slot_id, ep_index);
#endif
            break;
        case COMP_STOPPED_LENGTH_INVALID:
#ifdef DEBUG
            printf("Stopped on No-op or Link TRB for slot %u ep %u\n", slot_id, ep_index);
#endif
            break;
        case COMP_STOPPED_SHORT_PACKET:
#ifdef DEBUG
            printf("Stopped with short packet transfer detected for slot %u ep %u\n", slot_id, ep_index);
#endif
            break;
            /* Completion codes for endpoint halted state */
        case COMP_STALL_ERROR:
#ifdef DEBUG
            printf("Stalled endpoint for slot %u ep %u\n", slot_id, ep_index);
#endif
            ep->ep_state |= EP_HALTED;
            status = -1;
            break;
        case COMP_SPLIT_TRANSACTION_ERROR:
        case COMP_USB_TRANSACTION_ERROR:
#ifdef DEBUG
            printf("Transfer error for slot %u ep %u on endpoint - %d\n", slot_id, ep_index, trb_comp_code);
#endif
            status = -1;
            break;
        case COMP_BABBLE_DETECTED_ERROR:
#ifdef DEBUG
            printf("Babble error for slot %u ep %u on endpoint\n", slot_id, ep_index);
#endif
            status = -1;
            break;
            /* Completion codes for endpoint error state */
        case COMP_TRB_ERROR:
#ifdef DEBUG
            printf("WARN: TRB error for slot %u ep %u on endpoint\n", slot_id, ep_index);
#endif
            status = -1;
            break;
            /* completion codes not indicating endpoint state change */
        case COMP_DATA_BUFFER_ERROR:
#ifdef DEBUG
            printf("WARN: HC couldn't access mem fast enough for slot %u ep %u\n", slot_id, ep_index);
#endif
            status = -1;
            break;
        case COMP_BANDWIDTH_OVERRUN_ERROR:
#ifdef DEBUG
            printf("WARN: bandwidth overrun event for slot %u ep %u on endpoint\n", slot_id, ep_index);
#endif
            break;
        case COMP_ISOCH_BUFFER_OVERRUN:
#ifdef DEBUG
            printf("WARN: buffer overrun event for slot %u ep %u on endpoint", slot_id, ep_index);
#endif
            break;
        case COMP_RING_UNDERRUN:
            /*
             * When the Isoch ring is empty, the xHC will generate
             * a Ring Overrun Event for IN Isoch endpoint or Ring
             * Underrun Event for OUT Isoch endpoint.
             */
            if (!_ux_hcd_xhci_list_empty(&ep_ring->td_list))
            {
#ifdef DEBUG
                printf("Underrun Event for slot %d ep %d ""still with TDs queued?\n",
                        TRB_TO_SLOT_ID((event->flags)), ep_index);
#endif
            }
            goto cleanup;
        case COMP_RING_OVERRUN:
            if (!_ux_hcd_xhci_list_empty(&ep_ring->td_list))
            {
#ifdef DEBUG
                printf("Overrun Event for slot %d ep %d ""still with TDs queued?\n",
                        TRB_TO_SLOT_ID((event->flags)),ep_index);
#endif
            }
            goto cleanup;
        case COMP_MISSED_SERVICE_ERROR:
            /*
             *  * When encounter missed service error, one or more isoc tds
             * may be missed by xHC.
             * Set skip flag of the ep_ring; Complete the missed tds as
             * short transfer when process the ep_ring next time.
             */
            ep->skip = true;
#ifdef DEBUG
            printf("Miss service interval error for slot %u ep %u, set skip flag\n",slot_id, ep_index);
#endif
            goto cleanup;
        case COMP_NO_PING_RESPONSE_ERROR:
            ep->skip = true;
#ifdef DEBUG
            printf("No Ping response error for slot %u ep %u, Skip one Isoc TD\n",slot_id, ep_index);
#endif
            goto cleanup;

        case COMP_INCOMPATIBLE_DEVICE_ERROR:
            /* needs disable slot command to recover */
#ifdef DEBUG
            printf("WARN: detect an incompatible device for slot %u ep %u", slot_id, ep_index);
#endif
            status = -2;
            break;
        default:
            if (_ux_hcd_xhci_is_vendor_info_code(xhci, trb_comp_code))
            {
                status = 0;
                break;
            }
#ifdef DEBUG
            printf("ERROR Unknown event condition %u for slot %u ep %u , HC probably busted\n",
                    trb_comp_code, slot_id, ep_index);
#endif
            goto cleanup;
    }

    do
    {
        /* This TRB should be in the TD at the head of this ring's
         *  * TD list.
         */
        if (_ux_hcd_xhci_list_empty(&ep_ring->td_list))
        {
            /*
             * Don't print wanings if it's due to a stopped endpoint
             * generating an extra completion event if the device
             * was suspended. Or, a event for the last TRB of a
             * short TD we already got a short event for.
             * The short TD is already removed from the TD list.
             */

            if (!(trb_comp_code == COMP_STOPPED || trb_comp_code == COMP_STOPPED_LENGTH_INVALID ||
                        ep_ring->last_td_was_short))
            {
#ifdef DEBUG
                printf("WARN Event TRB for slot %d ep %d with no TDs queued?\n",TRB_TO_SLOT_ID((event->flags)),
                        ep_index);
#endif
            }
            if (ep->skip)
            {
                ep->skip = false;
#ifdef DEBUG
                printf("td_list is empty while skip flag set. Clear skip flag for slot %u ep %u.\n",
                        slot_id, ep_index);
#endif
            }
            goto cleanup;
        }

        /* We've skipped all the TDs on the ep ring when ep->skip set */
        if (ep->skip && td_num == 0)
        {
            ep->skip = false;
#ifdef DEBUG
            printf("All tds on the ep_ring skipped. Clear skip flag for slot %u ep %u.\n", slot_id, ep_index);
#endif
            goto cleanup;
        }

        td = _ux_hcd_xhci_list_first_entry(&ep_ring->td_list, UX_XHCI_TD, td_list);
        if (ep->skip)
            td_num--;

        /* Is this a TRB in the currently executing TD? */
        ep_seg = trb_in_td(xhci, ep_ring->deq_seg, ep_ring->dequeue, td->last_trb, ep_trb_dma, false);
        if(ep_seg == NULL)
        {
#ifdef DEBUG
            printf("Invalid EP seg\n");
#endif
        }
        /*
         * Skip the Force Stopped Event. The event_trb(event_dma) of FSE
         * is not in the current TD pointed by ep_ring->dequeue because
         * that the hardware dequeue pointer still at the previous TRB
         * of the current TD. The previous TRB maybe a Link TD or the
         * last TRB of the previous TD. The command completion handle
         * will take care the rest.
         */
        if (!ep_seg && (trb_comp_code == COMP_STOPPED || trb_comp_code == COMP_STOPPED_LENGTH_INVALID))
        {
            goto cleanup;
        }

        if (!ep_seg)
        {
            if (!ep->skip  || !ux_endpoint_xfer_isoc(&td->urb->ux_transfer_request_endpoint->ux_endpoint_descriptor))
            {
                /* Some host controllers give a spurious
                 * successful event after a short transfer.
                 * Ignore it.
                 */
                if (/*(xhci->quirks & UX_XHCI_SPURIOUS_SUCCESS) && */ (ep_ring->last_td_was_short))
                {
                    ep_ring->last_td_was_short = false;
                    goto cleanup;
                }
                /* HC is busted, give up! */
#ifdef DEBUG
                printf("ERROR Transfer event TRB DMA ptr not "
                        "part of current TD ep_index %d ""comp_code %u\n", ep_index,trb_comp_code);
#endif
                trb_in_td(xhci, ep_ring->deq_seg, ep_ring->dequeue, td->last_trb, ep_trb_dma, true);
                return -3;
            }
            skip_isoc_td(xhci, td, event, ep, &status);
            goto cleanup;
        }
        if (trb_comp_code == COMP_SHORT_PACKET)
            ep_ring->last_td_was_short = true;
        else
            ep_ring->last_td_was_short = false;

        if (ep->skip)
        {
#ifdef DEBUG
            printf("Found td. Clear skip flag for slot %u ep %u.\n",slot_id, ep_index);
#endif
            ep->skip = false;
        }
        ep_trb = &ep_seg->trbs[(ep_trb_dma - (uint64_t)ep_seg->dma) /sizeof(*ep_trb)];

        if (trb_is_noop(ep_trb))
        {
#ifdef DEBUG
            printf("TRB is noop\n");
#endif
            if (trb_comp_code == COMP_STALL_ERROR ||
                    _ux_hcd_xhci_requires_manual_halt_cleanup(xhci, ep_ctx, trb_comp_code))
                _ux_hcd_xhci_cleanup_halted_endpoint(xhci, slot_id,ep_index,ep_ring->stream_id,td, EP_HARD_RESET);
            goto cleanup;
        }
        /* update the urb's actual_length and give back to the core */
        if ((td->urb->ux_transfer_request_endpoint->ux_endpoint_descriptor.bmAttributes.xfer) == UX_CONTROL_ENDPOINT)
        {
            process_ctrl_td(xhci, td, ep_trb, event, ep, &status);
            _ux_utility_event_flags_set(&CONTROL_EP_FLAG, UX_XHCI_CONTROL_EP_EVENT, TX_OR);
        }
        else if (ux_endpoint_xfer_isoc(&td->urb->ux_transfer_request_endpoint->ux_endpoint_descriptor))
        {
            process_isoc_td(xhci, td,ep_trb, event, ep, &status);
            if (td->urb-> ux_transfer_request_completion_function)
                td->urb -> ux_transfer_request_completion_function(td->urb);
        }
        else
        {
            process_bulk_intr_td(xhci, td, ep_trb, event, ep, &status);
            if(ux_endpoint_xfer_int(&td->urb->ux_transfer_request_endpoint->ux_endpoint_descriptor)
                    || ux_endpoint_is_bulk_out(&td->urb->ux_transfer_request_endpoint->ux_endpoint_descriptor))
            {
                if(td->urb-> ux_transfer_request_completion_function)
                {
                    td->urb -> ux_transfer_request_completion_function(td->urb);
                }
            }
        }
cleanup:
        handling_skipped_tds = ep->skip && trb_comp_code != COMP_MISSED_SERVICE_ERROR &&
            trb_comp_code != COMP_NO_PING_RESPONSE_ERROR;

        /*
         * Do not update event ring dequeue pointer if we're in a loop
         * processing missed tds.
         */
        if (!handling_skipped_tds)
            inc_deq(xhci, xhci->event_ring);
        /*
         * If ep->skip is set, it means there are missed tds on the
         * endpoint ring need to take care of.
         * Process them as short transfer until reach the td pointed by
         * the event.
         */
    } while (handling_skipped_tds);

    if(td->urb->ux_transfer_request_completion_code)
    {
        td->urb->ux_transfer_request_completion_code = trb_comp_code == COMP_SUCCESS || trb_comp_code == COMP_SHORT_PACKET ? UX_SUCCESS : UX_TRANSFER_ERROR;
        osal_semaphore_post(&td->urb->ux_transfer_request_semaphore, true);
    }
    return 0;

err_out:
    return -1;
}
#endif //CFG_TUH_ENABLED
