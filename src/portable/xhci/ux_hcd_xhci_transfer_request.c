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
 * @file     _ux_hcd_xhci_transfer_request.c
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
#if 0
typedef struct URB_PRV
{
    UX_URB_PRIV *transfer_urb_priv;
    uint32_t num_tds;
}URB_PRV;
uint32_t cnt_t = 0;
URB_PRV log_prev[100];
#endif
extern TX_EVENT_FLAGS_GROUP CONTROL_EP_FLAG;
/**
  \fn           _ux_hcd_xhci_transfer_request
  \brief        xhci driver transfer requests
  \param[in]    xhci global pointer
  \param[in]    urb pointer
  \return       On success 0 remains error
 */

int32_t _ux_hcd_xhci_transfer_request(UX_HCD_XHCI *xhci, UX_TRANSFER *urb)
{
    int32_t ret = 0;
    uint32_t slot_id, ep_index;
    uint32_t *ep_state;
    uint32_t num_tds;
    unsigned long events;
    uint32_t max_ep_pkt_size;
    uint32_t n_tran;
    UX_URB_PRIV *urb_priv = NULL;
    UX_DEVICE  *device = urb->ux_transfer_request_endpoint->ux_endpoint_device;
    slot_id = xhci->slot_id;
    ep_index = _ux_hcd_xhci_get_endpoint_index(&urb->ux_transfer_request_endpoint->ux_endpoint_descriptor);
    ep_state = &xhci->devs[slot_id]->eps[ep_index].ep_state;
    xhci->device = device;

    if (xhci->devs[slot_id]->flags & VDEV_PORT_ERROR)
    {
#ifdef DEBUG
        printf("Can't queue urb, port error, link inactive\n");
#endif
        return -1;
    }
    if (ux_endpoint_xfer_isoc(&urb->ux_transfer_request_endpoint->ux_endpoint_descriptor))
    {
        max_ep_pkt_size = ux_endpoint_maxp(&urb->ux_transfer_request_endpoint->ux_endpoint_descriptor);
        n_tran = ux_endpoint_maxp_mult(&urb->ux_transfer_request_endpoint->ux_endpoint_descriptor);
        max_ep_pkt_size *= n_tran;
        num_tds = urb->ux_transfer_request_requested_length /max_ep_pkt_size;

    }
    else if (ux_endpoint_is_bulk_out(&urb->ux_transfer_request_endpoint->ux_endpoint_descriptor) &&
            urb->ux_transfer_request_requested_length > 0 &&
            !(urb->ux_transfer_request_requested_length % ux_endpoint_maxp(&urb->ux_transfer_request_endpoint->ux_endpoint_descriptor)))
    {
        num_tds = 2;
    }
    else
    {
        num_tds = 1;
    }
    xhci->interval  = urb-> ux_transfer_request_endpoint-> ux_endpoint_descriptor.bInterval;
    urb_priv = _ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY, sizeof(*urb_priv) + (sizeof(UX_XHCI_TD) * num_tds));
    if (!urb_priv)
    {
#ifdef DEBUG
        printf("urb_priv alloc failed %d \n",sizeof(UX_XHCI_TD) * num_tds);
#endif
        return -1;
    }
    urb_priv->num_tds = num_tds;
    urb_priv->num_tds_done = 0;
    urb->hcpriv = urb_priv;
    if (ux_endpoint_xfer_control(&urb->ux_transfer_request_endpoint->ux_endpoint_descriptor))
    {
        /* Check to see if the max packet size for the default control
         * endpoint changed during FS device enumeration */
        if (xhci->device->ux_device_speed == UX_FULL_SPEED_DEVICE)
        {
            ret = _ux_hcd_xhci_check_maxpacket(xhci, slot_id,ep_index, urb);
            if (ret < 0)
            {
#ifdef DEBUG
                printf("free the urb prev ptr\n");
#endif
                _ux_hcd_xhci_urb_free_priv(urb_priv);
                urb->hcpriv = NULL;
                return ret;
            }
        }
    }

    if (xhci->xhc_state & UX_XHCI_STATE_DYING)
    {
#ifdef DEBUG
        printf("Ep 0x%lx: URB %p submitted for non-responsive xHCI host.\n",
                urb->ux_transfer_request_endpoint->ux_endpoint_descriptor.bEndpointAddress, urb);
#endif
        ret = -1;
        goto free_priv;
    }

    if (*ep_state & (EP_GETTING_STREAMS | EP_GETTING_NO_STREAMS))
    {
#ifdef DEBUG
        printf("WARN: Can't enqueue URB, ep in streams transition state %x\n",*ep_state);
#endif
        ret = -1;
        goto free_priv;
    }
    if (*ep_state & EP_SOFT_CLEAR_TOGGLE)
    {
#ifdef DEBUG
        printf("Can't enqueue URB while manually clearing toggle\n");
#endif
        ret = -1;
        goto free_priv;
    }
    switch (ux_endpoint_type(&urb->ux_transfer_request_endpoint->ux_endpoint_descriptor))
    {
        case UX_CONTROL_ENDPOINT:
            ret = _ux_hcd_xhci_control_transfer_request(xhci,  urb,slot_id, ep_index);
            ret = _ux_utility_event_flags_get(&CONTROL_EP_FLAG, UX_XHCI_CONTROL_EP_EVENT, TX_AND_CLEAR, &events,
                    UX_WAIT_FOREVER );

            if(ret != TX_SUCCESS)
            {
                return ret;
            }
            break;
        case UX_BULK_ENDPOINT:
            ret = _ux_hcd_xhci_bulk_transfer_request(xhci, urb,slot_id, ep_index);
              //sys_busy_loop_us(1000);
            break;
        case UX_INTERRUPT_ENDPOINT:
            ret = _ux_hcd_xhci_interrupt_transfer_request(xhci, urb, slot_id, ep_index);
            break;
        case UX_ISOCHRONOUS_ENDPOINT:
            ret = _ux_hcd_xhci_isoc_transfer_request(xhci, urb, slot_id,ep_index);
    }
free_priv:
    if (ret)
    {
        _ux_hcd_xhci_urb_free_priv(urb_priv);
        urb->hcpriv = NULL;
    }
    return ret;
}

#endif //CFG_TUH_ENABLED
