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
 * @file     ux_hcd_xhci_control_transfer.c
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

/**
  \fn           _ux_hcd_xhci_control_transfer_request
  \brief        xhci driver control endpoint transfers
  \param[in]    xhci global pointer
  \param[in]    urb pointer
  \param[in]    slot id
  \param[in]    endpoint index
  \return       On success 0 remains error
 */

int32_t _ux_hcd_xhci_control_transfer_request(UX_HCD_XHCI *xhci, UX_TRANSFER *urb,int32_t slot_id,
        uint32_t ep_index)
{
    UX_XHCI_RING *ep_ring;
    int32_t num_trbs;
    int32_t ret;
    UX_XHCI_GENERIC_TRB *start_trb;
    UX_XHCI_TRB_INFO  trb_info;
    int32_t start_cycle;
    uint32_t field;
    UX_URB_PRIV *urb_priv;
    UX_XHCI_TD *td;
    UX_XHCI_VIRT_EP *ep;
    ep = &xhci->devs[slot_id]->eps[ep_index];
    ep_ring = ep->ring;
    if (!ep_ring)
    {
        return -1;
    }
    /* USB Control transfers minimally require two transaction stages on the bus:
     *  Setup and Status   */
    /* 1 TRB for setup, 1 for status */
    num_trbs = 2;
    /*
     * Don't need to check if we need additional event data and normal TRBs,
     * since data in control transfers will never get bigger than 16MB
     */
    if (urb->ux_transfer_request_requested_length > 0)
        num_trbs++;

    ret = prepare_transfer(xhci, xhci->devs[slot_id], ep_index, xhci->stream_id, num_trbs, urb, 0);
    if (ret < 0)
    {
        return ret;
    }
    urb_priv = urb->hcpriv;
    td = &urb_priv->td[0];
    /*
     * Don't give the first TRB to the hardware (by toggling the cycle bit)
     * until we've finished creating all the other TRBs.  The ring's cycle
     * state may change as we enqueue the other TRBs, so save it too.
     */
    start_trb = &ep_ring->enqueue->generic;
    start_cycle = ep_ring->cycle_state;
    field = 0;
    /* Immediate Data (IDT).bit and SETUP TRB  */
    field |= TRB_IDT | TRB_TYPE(TRB_SETUP);
    if (start_cycle == 0)
        field |= 0x1;

    /* xHCI 1.0/1.1 6.4.1.2.1: Transfer Type field */
    if (xhci->hci_version >= 0x100)
    {
        if (urb->ux_transfer_request_requested_length > 0)
        {
            if ((urb -> ux_transfer_request_type & UX_REQUEST_DIRECTION) == UX_REQUEST_IN)
                field |= TRB_TX_TYPE(TRB_DATA_IN);
            else
                field |= TRB_TX_TYPE(TRB_DATA_OUT);
        }
    }

    trb_info.low_address = urb -> ux_transfer_request_type | urb -> ux_transfer_request_function << 8
        | urb -> ux_transfer_request_value << 16;
    trb_info.high_address = urb -> ux_transfer_request_index | urb -> ux_transfer_request_requested_length << 16;
    trb_info.size =  TRB_LEN(8) | TRB_INTR_TARGET(0);
    trb_info.cntrl_field = field;
    /* Queue the SETUP Stage TRB   */
    queue_trb(xhci, ep_ring,true, &trb_info);

    /* If there's data, queue data TRBs */
    /* Only set interrupt on short packet for IN endpoints   */
    if ((urb -> ux_transfer_request_type & UX_REQUEST_DIRECTION ) == UX_REQUEST_IN)
        field = TRB_ISP | TRB_TYPE(TRB_DATA);
    else
        field = TRB_TYPE(TRB_DATA);

    if (urb->ux_transfer_request_requested_length > 0)
    {
        uint32_t length_field, remainder;
        uint64_t addr;

        if (_ux_hcd_xhci_urb_suitable_for_idt(urb))
        {
            memcpy(&addr, urb->ux_transfer_request_data_pointer, urb->ux_transfer_request_requested_length);
            field |= TRB_IDT;
        }
        else
        {
            addr = (uint64_t) urb->ux_transfer_request_data_pointer;
        }
        remainder = _ux_hcd_xhci_td_remainder(xhci, 0, urb->ux_transfer_request_requested_length,
                urb->ux_transfer_request_requested_length, urb, 1);
        length_field = TRB_LEN(urb->ux_transfer_request_requested_length) | TRB_TD_SIZE(remainder) |
            TRB_INTR_TARGET(0);
        if ((urb -> ux_transfer_request_type & UX_REQUEST_DIRECTION ) == UX_REQUEST_IN)
            field |= TRB_DIR_IN;
        RTSS_CleanDCache_by_Addr((void *)addr, urb->ux_transfer_request_requested_length);
        trb_info.low_address = LocalToGlobal((void *)lower_32_bits(addr));
        trb_info.high_address = LocalToGlobal((void *)upper_32_bits(addr));
        trb_info.size =  length_field;
        trb_info.cntrl_field = field | ep_ring->cycle_state;
        /* Queue the DATA stage TRB  */
        queue_trb(xhci, ep_ring,true, &trb_info);
    }

    /* Save the DMA address of the last TRB in the TD */
    td->last_trb = ep_ring->enqueue;
    /* Queue status TRB - sections 4.11.2.2 and 6.4.1.2.3 */
    /* If the device sent data, the status stage is an OUT transfer */
    if (urb->ux_transfer_request_requested_length > 0
            && ((urb -> ux_transfer_request_type & UX_REQUEST_DIRECTION ) == UX_REQUEST_IN))
        field = 0;
    else
        field = TRB_DIR_IN;
    trb_info.low_address = 0;
    trb_info.high_address = 0;
    trb_info.size =  TRB_INTR_TARGET(0); /* Event on completion */
    trb_info.cntrl_field = field | TRB_IOC | TRB_TYPE(TRB_STATUS_) | ep_ring->cycle_state;
    /* Queue status TRB   */
    queue_trb(xhci, ep_ring, false, &trb_info);
    /* Give the trb to endpoint doorbell */
    giveback_first_trb(xhci, slot_id, ep_index, 0, start_cycle, start_trb);
    return 0;
}
#endif //CFG_TUH_ENABLED
