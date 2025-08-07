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
 * @file     ux_hcd_xhci_td_process.c
 * @author   Mahesh Avula
 * @email    mahesh.avula@alifsemi.com
 * @version  V1.0.0
 * @date     12-Dec-2023
 * @brief    Process the all types(control, bulk, interrupt, isochronous) of xhci driver TD's(Transfer descriptor).
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
  \fn           process_ctrl_td
  \brief        Process control tds, update transfer status and actual_length
  \param[in]    xhci global pointer
  \param[in]    td pointer
  \param[in]    endpoint trb pointer
  \param[in]    transfer event pointer
  \param[in]    endpoint pointer
  \param[in]    status of the transfer
  \return       On success 0 remains error
 **/
int32_t process_ctrl_td(
        UX_HCD_XHCI *xhci,
        UX_XHCI_TD *td,
        UX_XHCI_TRB *ep_trb,
        UX_XHCI_TRANSFER_EVENT *event,
        UX_XHCI_VIRT_EP *ep,
        int32_t *status)
{
    UX_XHCI_VIRT_DEVICE *xdev;
    uint32_t slot_id;
    int32_t ep_index;
    UX_XHCI_EP_CTX *ep_ctx;
    uint32_t trb_comp_code;
    uint32_t remaining, requested;
    uint32_t trb_type;
    RTSS_InvalidateDCache_by_Addr(&ep_trb->generic, sizeof(ep_trb->generic));
    trb_type = TRB_FIELD_TO_TYPE((ep_trb->generic.field[3]));
    slot_id = TRB_TO_SLOT_ID((event->flags));
    xdev = xhci->devs[slot_id];
    ep_index = TRB_TO_EP_ID((event->flags)) - 1;
    ep_ctx = _ux_hcd_xhci_get_ep_ctx(xhci, xdev->out_ctx, ep_index);
    trb_comp_code = GET_COMP_CODE((event->transfer_len));
    requested = td->urb->ux_transfer_request_requested_length;
    remaining = EVENT_TRB_LEN((event->transfer_len));
    switch (trb_comp_code)
    {
        case COMP_SUCCESS:
            if (trb_type != TRB_STATUS_)
            {
#ifdef DEBUG
                printf("WARN: Success on ctrl %s TRB without IOC set?\n",(trb_type == TRB_DATA) ? "data" : "setup");
#endif
                *status = -1;
                break;
            }
            *status = 0;
            break;
        case COMP_SHORT_PACKET:
            *status = 0;
            break;
        case COMP_STOPPED_SHORT_PACKET:
            if (trb_type == TRB_DATA || trb_type == TRB_NORMAL)
                td->urb->ux_transfer_request_actual_length = remaining;
            else
            {
#ifdef DEBUG
                printf("WARN: Stopped Short Packet on ctrl setup or status TRB\n");
#endif
            }
            goto finish_td;
        case COMP_STOPPED:
            switch (trb_type)
            {
                case TRB_SETUP:
                    td->urb->ux_transfer_request_actual_length = 0;
                    goto finish_td;
                case TRB_DATA:
                case TRB_NORMAL:
                    td->urb->ux_transfer_request_actual_length = requested - remaining;
                    goto finish_td;
                case TRB_STATUS_:
                    td->urb->ux_transfer_request_actual_length = requested;
                    goto finish_td;
                default:
                    printf("WARN: unexpected TRB Type %d\n", trb_type);
                    goto finish_td;
            }
        case COMP_STOPPED_LENGTH_INVALID:
            goto finish_td;
        default:
            if (!_ux_hcd_xhci_requires_manual_halt_cleanup(xhci, ep_ctx, trb_comp_code))
                break;
#ifdef DEBUG
            printf("TRB error %u, halted endpoint index = %u\n", trb_comp_code, ep_index);
#endif
            /* else fall through */
        case COMP_STALL_ERROR:
            /* Did we transfer part of the data (middle) phase? */
            if (trb_type == TRB_DATA || trb_type == TRB_NORMAL)
                td->urb->ux_transfer_request_actual_length = requested - remaining;
            else if (!td->urb_length_set)
                td->urb->ux_transfer_request_actual_length = 0;
            goto finish_td;
    }
    /* stopped at setup stage, no data transferred */
    if (trb_type == TRB_SETUP)
        goto finish_td;
    /*
     * if on data stage then update the actual_length of the transfer and flag it
     * as set, so it won't be overwritten in the event for the last TRB.
     */
    RTSS_InvalidateDCache_by_Addr(td->urb->ux_transfer_request_data_pointer, td->urb->ux_transfer_request_requested_length);
    if (trb_type == TRB_DATA || trb_type == TRB_NORMAL)
    {
        td->urb_length_set = true;
        td->urb->ux_transfer_request_actual_length = requested - remaining;

        return 0;
    }
    /* at status stage */
    if (!td->urb_length_set)
        td->urb->ux_transfer_request_actual_length = requested;
finish_td:
    return finish_td(xhci, td, event, ep, status);
}

/**
  \fn           process_bulk_intr_td
  \brief        Process bulk and interrupt tds, update transfer status and actual_length
  \param[in]    xhci global pointer
  \param[in]    td pointer
  \param[in]    endpoint trb pointer
  \param[in]    transfer event pointer
  \param[in]    endpoint pointer
  \param[in]    status of the transfer
  \return       On success 0 remains error
 **/

int32_t process_bulk_intr_td(
        UX_HCD_XHCI *xhci,
        UX_XHCI_TD *td,
        UX_XHCI_TRB *ep_trb,
        UX_XHCI_TRANSFER_EVENT *event,
        UX_XHCI_VIRT_EP *ep,
        int32_t *status)
{
    UX_XHCI_SLOT_CTX *slot_ctx;
    UX_XHCI_RING *ep_ring;
    uint32_t trb_comp_code;
    uint32_t remaining, requested, ep_trb_len;
    uint32_t slot_id;
    int32_t ep_index;
    RTSS_InvalidateDCache_by_Addr(&ep_trb->generic, sizeof(ep_trb->generic));
    slot_id = TRB_TO_SLOT_ID((event->flags));
    slot_ctx = _ux_hcd_xhci_get_slot_ctx(xhci, xhci->devs[slot_id]->out_ctx);
    ep_index = TRB_TO_EP_ID((event->flags)) - 1;
    ep_ring = ep->ring;
    trb_comp_code = GET_COMP_CODE((event->transfer_len));
    remaining = EVENT_TRB_LEN((event->transfer_len));
    ep_trb_len = TRB_LEN((ep_trb->generic.field[2]));
    requested = td->urb->ux_transfer_request_requested_length;
    switch (trb_comp_code)
    {
        case COMP_SUCCESS:
            ep_ring->err_count = 0;
            /* handle success with untransferred data as short packet */
            if (ep_trb != td->last_trb || remaining)
            {
#ifdef DEBUG
                printf("WARN Successful completion on short transfer\n");
                printf("asked for %d bytes, %d bytes untransferred\n", requested, remaining);
#endif
            }
            *status = 0;
            break;
        case COMP_SHORT_PACKET:
            *status = 0;
            break;
        case COMP_STOPPED_SHORT_PACKET:
            td->urb->ux_transfer_request_actual_length = remaining;
            goto finish_td;
        case COMP_STOPPED_LENGTH_INVALID:
            /* stopped on ep trb with invalid length, exclude it */
            ep_trb_len   = 0;
            remaining    = 0;
            break;
        case COMP_USB_TRANSACTION_ERROR:
            if ((ep_ring->err_count++ > MAX_SOFT_RETRY) || ((slot_ctx->tt_info) & TT_SLOT))
                break;
            *status = 0;
            _ux_hcd_xhci_cleanup_halted_endpoint(xhci, slot_id, ep_index, ep_ring->stream_id, td, EP_SOFT_RESET);
            if(td->urb->ux_transfer_request_completion_code)
            {
                td->urb->ux_transfer_request_completion_code = UX_TRANSFER_ERROR;
                osal_semaphore_post(&td->urb->ux_transfer_request_semaphore, true);
            }
            return 0;
        default:
            /* do nothing */
            break;
    }
    RTSS_InvalidateDCache_by_Addr(td->urb->ux_transfer_request_data_pointer, td->urb->ux_transfer_request_requested_length);
    if (ep_trb == td->last_trb)
        td->urb->ux_transfer_request_actual_length = requested - remaining;
    else
        td->urb->ux_transfer_request_actual_length = sum_trb_lengths(xhci, ep_ring, ep_trb) +
            ep_trb_len - remaining;
finish_td:
    if (remaining > requested)
    {
#ifdef DEBUG
        printf("bad transfer trb length %d in event trb\n",remaining);
#endif
        td->urb->ux_transfer_request_actual_length = 0;
    }
    if(td->urb->ux_transfer_request_completion_code)
    {
        td->urb->ux_transfer_request_completion_code = trb_comp_code == COMP_SUCCESS || trb_comp_code == COMP_SHORT_PACKET? UX_SUCCESS : UX_TRANSFER_ERROR;
        osal_semaphore_post(&td->urb->ux_transfer_request_semaphore, true);
    }
    return finish_td(xhci, td, event, ep, status);
}

/**
  \fn           process_isoc_td
  \brief        Process isochronous tds, update urb packet status and actual_length
  \param[in]    xhci global pointer
  \param[in]    td pointer
  \param[in]    endpoint trb pointer
  \param[in]    transfer event pointer
  \param[in]    endpoint pointer
  \param[in]    status of the transfer
  \return       On success 0 remains error
 **/

int32_t process_isoc_td(
        UX_HCD_XHCI *xhci,
        UX_XHCI_TD *td,
        UX_XHCI_TRB *ep_trb,
        UX_XHCI_TRANSFER_EVENT *event,
        UX_XHCI_VIRT_EP *ep,
        int32_t *status)
{
    UX_XHCI_RING *ep_ring;
    uint32_t trb_comp_code;
    bool sum_trbs_for_length = false;
    uint32_t remaining, requested, ep_trb_len;
    ep_ring = ep->ring;
    RTSS_InvalidateDCache_by_Addr(&ep_trb->generic, sizeof(ep_trb->generic));
    trb_comp_code = GET_COMP_CODE((event->transfer_len));
    requested = td->urb->ux_transfer_request_requested_length;
    remaining = EVENT_TRB_LEN((event->transfer_len));/*  number of bytes has not been transferred  */
    ep_trb_len = TRB_LEN((ep_trb->generic.field[2]));

    /* handle completion code */
    switch (trb_comp_code)
    {
        case COMP_SUCCESS:
            if (remaining)
            {
                td->urb->ux_transfer_request_status = 0;
                break;
            }
            td->urb->ux_transfer_request_status = 0;
            break;
        case COMP_SHORT_PACKET:
            td->urb->ux_transfer_request_status = 0;
            sum_trbs_for_length = true;
            break;
        case COMP_DATA_BUFFER_ERROR:
            td->urb->ux_transfer_request_status = -1;
            break;
        case COMP_BANDWIDTH_OVERRUN_ERROR:
            td->urb->ux_transfer_request_status = -ECOMM;
            break;
        case COMP_ISOCH_BUFFER_OVERRUN:
        case COMP_BABBLE_DETECTED_ERROR:
            td->urb->ux_transfer_request_status = -EOVERFLOW;
            break;
        case COMP_INCOMPATIBLE_DEVICE_ERROR:
        case COMP_STALL_ERROR:
            td->urb->ux_transfer_request_status = -EPROTO;
            break;
        case COMP_USB_TRANSACTION_ERROR:
            td->urb->ux_transfer_request_status = -EPROTO;
            if (ep_trb != td->last_trb)
                return 0;
            break;
        case COMP_STOPPED:
            sum_trbs_for_length = true;
            break;
        case COMP_STOPPED_SHORT_PACKET:
            td->urb->ux_transfer_request_status = 0;
            requested = remaining;
            break;
        case COMP_STOPPED_LENGTH_INVALID:
            requested = 0;
            remaining = 0;
            break;

        default:
            sum_trbs_for_length = true;
            td->urb->ux_transfer_request_status = -1;
            break;
    }
    RTSS_InvalidateDCache_by_Addr(td->urb->ux_transfer_request_data_pointer, td->urb->ux_transfer_request_requested_length);
    if (sum_trbs_for_length)
    {
        td->urb->ux_transfer_request_actual_length = ep_trb_len - remaining;
    }
    else
    {
        td->urb->ux_transfer_request_actual_length = requested;
    }
    return finish_td(xhci, td, event, ep, status);
}
#endif //CFG_TUH_ENABLED
