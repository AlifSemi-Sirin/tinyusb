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
 * @file     ux_hcd_xhci_ep_bandwidth.c
 * @author   Mahesh Avula
 * @email    mahesh.avula@alifsemi.com
 * @version  V1.0.0
 * @date     12-Dec-2023
 * @brief    This is file for checking endpoint bandwidth of XHCI driver
 * @bug      None
 * @Note     None
 ******************************************************************************/


/* Include necessary system files.  */

#include "tusb_option.h"

#if CFG_TUH_ENABLED && CFG_TUSB_MCU == OPT_MCU_NONE

#include "host/hcd.h"
#include "ux_hcd_xhci_api.h"
//#include "ux_host_stack.h"
#include "ux_hcd_xhci_list.h"

static void _ux_hcd_xhci_zero_in_ctx(UX_HCD_XHCI *xhci, UX_XHCI_VIRT_DEVICE *virt_dev)
{
    UX_XHCI_INPUT_CONTROL_CTX *ctrl_ctx;
    UX_XHCI_EP_CTX *ep_ctx;
    UX_XHCI_SLOT_CTX *slot_ctx;
    int32_t i;
    ctrl_ctx = _ux_hcd_xhci_get_input_control_ctx(virt_dev->in_ctx);
    if (!ctrl_ctx)
    {
#ifdef DEBUG
        printf("Could not get input context, bad type\n");
#endif
        return;
    }

    /* When a device's add flag and drop flag are zero, any subsequent
     * configure endpoint command will leave that endpoint's state
     * untouched.  Make sure we don't leave any old state in the input
     * endpoint contexts.
     */
    ctrl_ctx->drop_flags = 0;
    ctrl_ctx->add_flags = 0;
    slot_ctx = _ux_hcd_xhci_get_slot_ctx(xhci, virt_dev->in_ctx);
    slot_ctx->dev_info &= (~LAST_CTX_MASK);
    /* Endpoint 0 is always valid */
    slot_ctx->dev_info |= (LAST_CTX(1));
    for (i = 1; i < 31; i++)
    {
        ep_ctx = _ux_hcd_xhci_get_ep_ctx(xhci, virt_dev->in_ctx, i);
        ep_ctx->ep_info = 0;
        ep_ctx->ep_info2 = 0;
        ep_ctx->deq = 0;
        ep_ctx->tx_info = 0;
    }
}


int32_t _ux_hcd_xhci_check_bandwidth( UX_HCD_XHCI *xhci, UX_DEVICE *udev)
{
    int32_t i;
    int32_t ret = 0;;
    UX_XHCI_VIRT_DEVICE  *virt_dev;
    UX_XHCI_INPUT_CONTROL_CTX *ctrl_ctx;
    UX_XHCI_SLOT_CTX *slot_ctx;
    UX_XHCI_COMMAND *command;
    if ((xhci->xhc_state & UX_XHCI_STATE_DYING) ||
            (xhci->xhc_state & UX_XHCI_STATE_REMOVING))
        return -1;

    virt_dev = xhci->devs[xhci->slot_id];
    /* allocate the command  */
    command = _ux_hcd_xhci_alloc_command(xhci);
    if (!command)
    {
#ifdef DEBUG
        printf("alloc command failed\n");
#endif
        return -1;
    }
    command->in_ctx = virt_dev->in_ctx;

    ctrl_ctx = _ux_hcd_xhci_get_input_control_ctx(command->in_ctx);
    if (!ctrl_ctx)
    {
#ifdef DEBUG
        printf("Could not get input context, bad type\n");
#endif
        ret = -1;
        goto command_cleanup;
    }
    /*  Add context flag A1 and Drop contex flags D0 and D1 of the input control context shall be cleared to 0  */
    /* See section 4.6.6 - A0 = 1; A1 = D0 = D1 = 0 */
    ctrl_ctx->add_flags |= (SLOT_FLAG);
    ctrl_ctx->add_flags &= (~EP0_FLAG);
    ctrl_ctx->drop_flags &= (~(SLOT_FLAG | EP0_FLAG));

    /* Don't issue the command if there's no endpoints to update. */
    if (ctrl_ctx->add_flags == (SLOT_FLAG) && ctrl_ctx->drop_flags == 0)
    {
        ret = 0;
        goto command_cleanup;
    }
    slot_ctx = _ux_hcd_xhci_get_slot_ctx(xhci, virt_dev->in_ctx);
    for (i = 31; i >= 1; i--)
    {
        uint32_t temp = (0x1 << i);
        if ((virt_dev->eps[i-1].ring && !(ctrl_ctx->drop_flags & temp))
                || (ctrl_ctx->add_flags & temp) || i == 1)
        {
            slot_ctx->dev_info &= (~LAST_CTX_MASK);
            slot_ctx->dev_info |= (LAST_CTX(i));
            break;
        }
    }
    /* Configure the Endpoint  */
    ret = _ux_hcd_xhci_configure_endpoint(xhci, udev, command, false, false);
    if (ret)
        goto command_cleanup;

    /* Free any rings that were dropped, but not changed. */
    for (i = 1; i < 31; i++)
    {
        if (((ctrl_ctx->drop_flags) & (1 << (i + 1))) && !((ctrl_ctx->add_flags) & (1 << (i + 1))))
        {
            _ux_hcd_xhci_free_endpoint_ring(xhci, virt_dev, i);
        }
    }

    _ux_hcd_xhci_zero_in_ctx(xhci, virt_dev);
    /*
     * Install any rings for completely new endpoints or changed endpoints,
     * and free any old rings from changed endpoints.
     */
    for (i = 1; i < 31; i++)
    {
        if (!virt_dev->eps[i].new_ring)
            continue;
        /* Only free the old ring if it exists.
         * It may not if this is the first add of an endpoint.
         */
        if (virt_dev->eps[i].ring)
        {
            _ux_hcd_xhci_free_endpoint_ring(xhci, virt_dev, i);
        }
        virt_dev->eps[i].ring = virt_dev->eps[i].new_ring;
        virt_dev->eps[i].new_ring = NULL;
    }
command_cleanup:

    _ux_utility_memory_free(command);

    return ret;
}
#endif //CFG_TUH_ENABLED
