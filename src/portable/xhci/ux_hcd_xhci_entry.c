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
 * @file     ux_hcd_xhci_entry.c
 * @author   Mahesh Avula
 * @email    mahesh.avula@alifsemi.com
 * @version  V1.0.0
 * @date     12-Dec-2023
 * @brief    This function dispatch the HCD function internally to the XHCI  controller.
 * @bug      None
 * @Note     None
 ******************************************************************************/


#include "tusb_option.h"

#if CFG_TUH_ENABLED && CFG_TUSB_MCU == OPT_MCU_NONE

#include "host/hcd.h"
#include "ux_hcd_xhci_api.h"
//#include "ux_host_stack.h"

uint32_t  _ux_hcd_xhci_entry(UX_HCD *hcd, uint32_t function, void *parameter)
{

    uint32_t status;
    UX_HCD_XHCI *xhci;
    /* Check the status of the controller.  */
    if (hcd -> ux_hcd_status == UX_UNUSED)
    {
        /* Error trap. */
        _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_HCD, UX_CONTROLLER_UNKNOWN);

        /* If trace is enabled, insert this event into the trace buffer.  */
        UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_CONTROLLER_UNKNOWN, 0, 0, 0, UX_TRACE_ERRORS, 0, 0)
            return(UX_CONTROLLER_UNKNOWN);
    }

    /* Get the pointer to the XHCI HCD.  */
    xhci =  (UX_HCD_XHCI *) hcd -> ux_hcd_controller_hardware;

    /* Look at the function and route it.  */
    switch(function)
    {
        case UX_HCD_DISABLE_CONTROLLER:
            status =  UX_SUCCESS;
            break;
        case UX_HCD_GET_PORT_STATUS:
            status =  _ux_hcd_xhci_port_status_get(xhci, (uint32_t) parameter);
            break;
        case UX_HCD_ENABLE_PORT:
            status = UX_SUCCESS;
            break;
        case UX_HCD_DISABLE_PORT:
            status =  _ux_hcd_xhci_disable_port(xhci, (uint32_t) parameter);
            break;
        case UX_HCD_POWER_ON_PORT:
            status =  UX_SUCCESS;
            break;
        case UX_HCD_POWER_DOWN_PORT:
            status =  UX_SUCCESS;
            break;
        case UX_HCD_SUSPEND_PORT:
            status =   UX_SUCCESS;
            break;
        case UX_HCD_RESUME_PORT:
            status =  UX_SUCCESS;
            break;
        case UX_HCD_RESET_PORT:
            status =  _ux_hcd_xhci_reset_port(xhci, (uint32_t) parameter);
            /* delay is required to HC port will back to normal state after port reset  */
            sys_busy_loop_us(1000*50);
            break;
        case UX_HCD_GET_FRAME_NUMBER:
            break;
        case UX_HCD_SET_FRAME_NUMBER:
            status =  UX_SUCCESS;
            break;
        case UX_HCD_TRANSFER_REQUEST:
            if(((UX_TRANSFER *) parameter)->ux_transfer_request_function == UX_SET_ADDRESS)
            {
                /* Assign the address to the Device..*/
                status = _ux_hcd_xhci_address_device(xhci, (((UX_TRANSFER*) parameter)->ux_transfer_request_endpoint->ux_endpoint_device));
                break;
            }
            else
            {
                /* transfer urb's  */
                status = _ux_hcd_xhci_transfer_request(xhci, (UX_TRANSFER *) parameter);
                break;
            }
        case UX_HCD_TRANSFER_ABORT:
            status =  UX_SUCCESS;
            break;
        case UX_HCD_CREATE_ENDPOINT:
            switch ((((UX_ENDPOINT*) parameter) -> ux_endpoint_descriptor.bmAttributes.xfer))
            {
                case UX_CONTROL_ENDPOINT:
                    /* Enable the device slot id */
                    status = _ux_hcd_xhci_alloc_dev(xhci, (((UX_ENDPOINT*) parameter)->ux_endpoint_device));
                    break;

                case UX_BULK_ENDPOINT:
                case UX_INTERRUPT_ENDPOINT:
                case UX_ISOCHRONOUS_ENDPOINT:
                    status = _ux_hcd_xhci_add_endpoint(xhci, (((UX_ENDPOINT*) parameter)->ux_endpoint_device), (UX_ENDPOINT*) parameter);
                    status = _ux_hcd_xhci_check_bandwidth(xhci, (((UX_ENDPOINT*) parameter)->ux_endpoint_device));
                    break;
                default:
                    break;
            }
            break;
        case UX_HCD_DESTROY_ENDPOINT:
            _ux_hcd_xhci_free_dev(xhci, (((UX_ENDPOINT*) parameter)->ux_endpoint_device));
            status = UX_SUCCESS;
            break;
        case UX_HCD_RESET_ENDPOINT:
            _ux_hcd_xhci_endpoint_reset(xhci, (UX_ENDPOINT*) parameter);
            status = UX_SUCCESS;
            break;
        case UX_HCD_PROCESS_DONE_QUEUE:
            status =  UX_SUCCESS;
            break;
        default:
            /* Error trap. */
            _ux_system_error_handler(UX_SYSTEM_LEVEL_THREAD, UX_SYSTEM_CONTEXT_HCD, UX_FUNCTION_NOT_SUPPORTED);

            /* If trace is enabled, insert this event into the trace buffer.  */
            UX_TRACE_IN_LINE_INSERT(UX_TRACE_ERROR, UX_FUNCTION_NOT_SUPPORTED, 0, 0, 0, UX_TRACE_ERRORS, 0, 0)

                status =  UX_FUNCTION_NOT_SUPPORTED;
            break;
    }
    /* Return status to caller.  */
    return(status);
}
#endif //CFG_TUH_ENABLED
