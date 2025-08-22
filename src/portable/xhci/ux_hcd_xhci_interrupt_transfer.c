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
 * @file     _ux_hcd_xhci_interrupt_transfer.c
 * @author   Mahesh Avula
 * @email    mahesh.avula@alifsemi.com
 * @version  V1.0.0
 * @date     12-Dec-2023
 * @brief    XHCI host interrupt endpoint transfers.
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
  \fn           _ux_hcd_xhci_interrupt_transfer_request
  \brief        xhci driver interrupt transfers
  \param[in]    xhci global pointer
  \param[in]    urb pointer
  \param[in]    slot id
  \param[in]    endpoint index
  \return       On success 0 remains error
 */


/*
 * xHCI uses normal TRBs for both bulk and interrupt.  When the interrupt
 * endpoint is to be serviced, the xHC will consume (at most) one TD.  A TD
 * can take several service intervals to transmit.
 */

int32_t _ux_hcd_xhci_interrupt_transfer_request(UX_HCD_XHCI *xhci, UX_TRANSFER *urb, uint32_t slot_id,
        uint32_t ep_index)
{
    UX_XHCI_EP_CTX *ep_ctx;
    ep_ctx = _ux_hcd_xhci_get_ep_ctx(xhci, xhci->devs[slot_id]->out_ctx, ep_index);
    check_interval(xhci, urb, ep_ctx);
    return _ux_hcd_xhci_bulk_transfer_request(xhci, urb, slot_id, ep_index);
}
#endif //CFG_TUH_ENABLED
