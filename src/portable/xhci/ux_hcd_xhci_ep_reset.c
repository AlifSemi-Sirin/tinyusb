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
 * @file     ux_hcd_xhci_ep_reset.c
 * @author   Mahesh Avula
 * @email    mahesh.avula@alifsemi.com
 * @version  V1.0.0
 * @date     12-Dec-2023
 * @brief    To reset the XHCI driver endpoints
 * @bug      None
 * @Note     None
 ******************************************************************************/

/* Include necessary system files.  */

#include "tusb_option.h"

#if CFG_TUH_ENABLED && CFG_TUSB_MCU == OPT_MCU_NONE

#include "host/hcd.h"
#include "ux_hcd_xhci_api.h"
/**
  \fn           _ux_hcd_xhci_queue_stop_endpoint
  \brief        Queue the stop endpoint command
  \param[in]    xhci global pointer
  \param[in]    command pointer
  \param[in]    slot id
  \param[in]    endpoint index
  \param[in]   suspend value
  \return       On success 0 remains error
 */

/*
 * Suspend is set to indicate "Stop Endpoint Command" is being issued to stop
 * activity on an endpoint that is about to be suspended.
 */
int32_t _ux_hcd_xhci_queue_stop_endpoint(UX_HCD_XHCI *xhci, UX_XHCI_COMMAND *cmd,
        uint32_t slot_id, uint32_t ep_index, int32_t suspend)
{
    uint32_t trb_slot_id = SLOT_ID_FOR_TRB(slot_id);
    uint32_t trb_ep_index = EP_ID_FOR_TRB(ep_index);
    uint32_t type = TRB_TYPE(TRB_STOP_RING);
    uint32_t trb_suspend = SUSPEND_PORT_FOR_TRB(suspend);
    UX_XHCI_TRB_INFO  trb_info;
    trb_info.low_address = 0;
    trb_info.high_address = 0;
    trb_info.size = 0;
    trb_info.cntrl_field = trb_slot_id | trb_ep_index | type | trb_suspend;
    return queue_command(xhci, cmd, &trb_info, false);
}

/**
  \fn           _ux_hcd_xhci_endpoint_reset
  \brief        Reset the current executing endpoint
  \param[in]    xhci global pointer
  \param[in]    endpoint pointer
  \return       none
 */

/*
 * Called after usb core issues a clear halt control message.
 * The host side of the halt should already be cleared by a reset endpoint
 * command issued when the STALL event was received.
 *
 * The reset endpoint command may only be issued to endpoints in the halted
 * state. For software that wishes to reset the data toggle or sequence number
 * of an endpoint that isn't in the halted state this function will issue a
 * configure endpoint command with the Drop and Add bits set for the target
 * endpoint. Refer to the additional note in xhci spcification section 4.6.8.
 */

void _ux_hcd_xhci_endpoint_reset(UX_HCD_XHCI *xhci,UX_ENDPOINT *host_ep)
{
    UX_XHCI_VIRT_DEVICE *vdev;
    UX_XHCI_VIRT_EP *ep;
    UX_XHCI_INPUT_CONTROL_CTX *ctrl_ctx;
    UX_XHCI_COMMAND *stop_cmd, *cfg_cmd;
    uint32_t ep_index;
    uint32_t ep_flag;
    int32_t err, ret;
    unsigned long events;
    vdev = xhci->devs[xhci->slot_id];

    if (!xhci->slot_id || !vdev)
        return;
    ep_index = _ux_hcd_xhci_get_endpoint_index(&host_ep->ux_endpoint_descriptor);
    ep = &vdev->eps[ep_index];
    if (!ep)
        return;

    /* Bail out if toggle is already being cleared by a endpoint reset */
    if (ep->ep_state & EP_HARD_CLEAR_TOGGLE)
    {
        ep->ep_state &= ~EP_HARD_CLEAR_TOGGLE;
        return;
    }

    /* Only interrupt and bulk ep's use data toggle, USB2 spec 5.5.4-> */
    if(ux_endpoint_xfer_control(&host_ep->ux_endpoint_descriptor) ||
            ux_endpoint_xfer_isoc(&host_ep->ux_endpoint_descriptor))
        return;
    ep_flag = _ux_hcd_xhci_get_endpoint_flag(&host_ep->ux_endpoint_descriptor);

    if(ep_flag == SLOT_FLAG || ep_flag == EP0_FLAG)
        return;
    /* Allocate the command */
    stop_cmd = _ux_hcd_xhci_alloc_command(xhci);
    if (!stop_cmd)
        return;

    cfg_cmd = _ux_hcd_xhci_alloc_command_with_ctx_sz(xhci);
    if (!cfg_cmd)
        goto cleanup;

    /* block queuing new trbs and ringing ep doorbell */
    ep->ep_state |= EP_SOFT_CLEAR_TOGGLE;

    /*
     * Make sure endpoint ring is empty before resetting the toggle/seq.
     * Driver is required to synchronously cancel all transfer request.
     * Stop the endpoint to force xHC to update the output context
     */

    if (!_ux_hcd_xhci_list_empty(&ep->ring->td_list))
    {
#ifdef DEBUG
        printf("Ring is not empty, refuse reset\n");
#endif
        _ux_hcd_xhci_free_command(xhci, cfg_cmd);
        goto cleanup;
    }
    /*Queue the Stop endpoint command  */
    err = _ux_hcd_xhci_queue_stop_endpoint(xhci, stop_cmd, xhci->slot_id, ep_index, 0);
    if (err < 0)
    {
        _ux_hcd_xhci_free_command(xhci, cfg_cmd);
#ifdef DEBUG
        printf("Failed to queue stop ep command, %d ", err);
#endif
        goto cleanup;
    }
    stop_cmd->fCompletion = 1;
    /* Ring the HC doorbell for command ring  */
    _ux_hcd_xhci_ring_cmd_db(xhci);
    /* wait for the command completion */
    while(stop_cmd->fCompletion);
    /* config ep command clears toggle if add and drop ep flags are set */
    ctrl_ctx = _ux_hcd_xhci_get_input_control_ctx(cfg_cmd->in_ctx);
    /* Setup input context for configuring endpoint  */
    _ux_hcd_xhci_setup_input_ctx_for_config_ep(xhci, cfg_cmd->in_ctx, vdev->out_ctx,
            ctrl_ctx, ep_flag, ep_flag);
    _ux_hcd_xhci_endpoint_copy(xhci, cfg_cmd->in_ctx, vdev->out_ctx, ep_index);
    /* Queue the configure endpoint command */
    err = _ux_hcd_xhci_queue_configure_endpoint(xhci, cfg_cmd, cfg_cmd->in_ctx->dma, xhci->slot_id, false);

    if (err < 0)
    {
        _ux_hcd_xhci_free_command(xhci, cfg_cmd);
#ifdef DEBUG
        printf("Failed to queue config ep command, %d ",err);
#endif
        goto cleanup;
    }
    cfg_cmd->fCompletion = 1;
    /* Ring the HC doorbell for command ring  */
    _ux_hcd_xhci_ring_cmd_db(xhci);
    /* Wait for the command completion */
    while(cfg_cmd->fCompletion);
    ep->ep_state &= ~EP_SOFT_CLEAR_TOGGLE;
    _ux_hcd_xhci_free_command(xhci, cfg_cmd);
cleanup:
    _ux_hcd_xhci_free_command(xhci, stop_cmd);
}
#endif //CFG_TUH_ENABLED
