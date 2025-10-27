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
 * @file     _ux_hcd_xhci_port_enable.c
 * @author   Mahesh Avula
 * @email    mahesh.avula@alifsemi.com
 * @version  V1.0.0
 * @date     12-Dec-2023
 * @brief    This function enables the port
 * @bug      None
 * @Note     None
 ******************************************************************************/


/* Include necessary system files.  */

#include "tusb_option.h"

#if CFG_TUH_ENABLED && CFG_TUSB_MCU == OPT_MCU_NONE

#include "host/hcd.h"
#include "ux_hcd_xhci_api.h"
//#include "ux_host_stack.h"

uint32_t  _ux_hcd_xhci_port_enable(UX_HCD_XHCI *xhci, uint32_t port_index)
{
    uint32_t reg;
    uint32_t port_status = 0;
    reg = xhci->op_regs->PORTSC;
    reg |= PORT_PE;
    xhci->op_regs->PORTSC = reg;
    return 0;
}
#endif //CFG_TUH_ENABLED
