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
 * @file     ux_hcd_xhci_port_status_get.c
 * @author   Mahesh Avula
 * @email    mahesh.avula@alifsemi.com
 * @version  V1.0.0
 * @date     12-Dec-2023
 * @brief    This function will return the status for each port attached to the root HUB
 * @bug      None
 * @Note     None
 ******************************************************************************/


/* Include necessary system files.  */

#include "tusb_option.h"

#if CFG_TUH_ENABLED && CFG_TUSB_MCU == OPT_MCU_NONE

#include "host/hcd.h"
#include "ux_hcd_xhci_api.h"
//#include "ux_host_stack.h"

uint32_t  _ux_hcd_xhci_port_status_get(UX_HCD_XHCI *xhci, uint32_t port_index)
{
    uint32_t status;
    uint32_t port_status = 0;
    status = xhci->op_regs->PORTSC;
   // printf("port status: %x\n", status);
    /* Port Reset Status.  */
    if (status & PORT_RESET)
    {
        port_status |=  UX_PS_PRS;
    }
    if (status & PORT_RC)
    {
        _ux_hcd_xhci_clear_port_change_bit(xhci, UX_PS_PRS, port_index, status);
        status = xhci->op_regs->PORTSC;
    }

    /* Device Connection Status.bit[0]  */
    if (status & PORT_CONNECT)
    {
        port_status |=  UX_PS_CCS;
    }
    if (status & PORT_CSC)
    {
        _ux_hcd_xhci_clear_port_change_bit(xhci, UX_PS_CCS, port_index, status);
        status = xhci->op_regs->PORTSC;
    }
    /* Port Enable Status.  */
    if (status & PORT_PE)
        port_status |=  UX_PS_PES;

    /* Port Overcurrent Status.  */
    if (status & PORT_OC)
        port_status |=  UX_PS_POCI;

    /* Port Power Status.
     * Default = 1,This bit may or may not represent whether (VBus) power is actually applied to the port.
     * When PP equals a '0' the port is nonfunctional and shall not report attaches, detaches.
     * */
    if (status & PORT_POWER)
        port_status |=  UX_PS_PPS;
    return(port_status);
}

bool  _ux_hcd_xhci_port_current_status_get(UX_HCD_XHCI *xhci, uint32_t port_index)
{
    uint32_t port_status;
    uint32_t cur_status;
    static uint32_t prev_status = 0;
    port_status = xhci->op_regs->PORTSC;
    cur_status = port_status & 0x1;
    if ((port_status & PORT_CSC)&& (cur_status != prev_status))
    {
        prev_status = cur_status;
        return true;
    }
    return false;
}
#endif //CFG_TUH_ENABLED
