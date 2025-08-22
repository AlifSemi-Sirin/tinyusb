
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
 * @file     ux_hcd_xhci_hub.c
 * @author   Mahesh Avula
 * @email    mahesh.avula@alifsemi.com
 * @version  V1.0.0
 * @date     12-Dec-2023
 * @brief     This function powers root HUBs attached to the XHCI controller.
 * @bug      None
 * @Note     None
 ******************************************************************************/


/* Include necessary system files.  */

#include "tusb_option.h"

#if CFG_TUH_ENABLED && CFG_TUSB_MCU == OPT_MCU_NONE

#include "host/hcd.h"
#include "ux_hcd_xhci_api.h"
//#include "ux_host_stack.h"

/*
 * Given a port state, this function returns a value that would result in the
 * port being in the same state, if the value was written to the port status
 * control register.
 * Save Read Only (RO) bits and save read/write bits where
 * writing a 0 clears the bit and writing a 1 sets the bit (RWS).
 * For all other types (RW1S, RW1CS, RW, and RZ), writing a '0' has no effect.
 */
uint32_t _ux_hcd_xhci_port_state_to_neutral(uint32_t state)
{
    /* Save read-only status and port state */
    return (state & UX_XHCI_PORT_RO) | (state & UX_XHCI_PORT_RWS);
}

/* Test and clear port RWC bit */
void _ux_hcd_xhci_test_and_clear_bit(UX_HCD_XHCI *xhci, UX_XHCI_PORT *port, uint32_t port_bit)
{
    uint32_t portsc;
    portsc = xhci->op_regs->PORTSC;
    if (portsc & port_bit)
    {
        portsc = _ux_hcd_xhci_port_state_to_neutral(portsc);
        portsc |= port_bit;
        xhci->op_regs->PORTSC = portsc;
    }
}

/*
 * Ring device, it rings the all doorbells unconditionally.
 */
void _ux_hcd_xhci_ring_device(UX_HCD_XHCI *xhci, uint32_t slot_id)
{
    int32_t i, s;
    UX_XHCI_VIRT_EP *ep;

    for (i = 0; i < LAST_EP_INDEX + 1; i++)
    {
        ep = &xhci->devs[slot_id]->eps[i];

        if (ep->ep_state & EP_HAS_STREAMS)
        {
            for (s = 1; s < ep->stream_info->num_streams; s++)
                _ux_hcd_xhci_ring_ep_doorbell(xhci, slot_id, i, s);
        }
        else if (ep->ring && ep->ring->dequeue)
        {
            _ux_hcd_xhci_ring_ep_doorbell(xhci, slot_id, i, 0);
        }
    }
}
/**
  \fn           _ux_hcd_xhci_set_link_state
  \brief        Set the port link state
  \param[in]    xhci pointer
  \param[in]    port pointer
  \param[in]    link state
  \return       None
 */
void _ux_hcd_xhci_set_link_state(UX_HCD_XHCI *xhci, UX_XHCI_PORT *port, uint32_t link_state)
{
    uint32_t temp;
    uint32_t portsc;
    portsc = xhci->op_regs->PORTSC;
    temp = _ux_hcd_xhci_port_state_to_neutral(portsc);
    temp &= ~PORT_PLS_MASK;
    temp |= PORT_LINK_STROBE | link_state;
    xhci->op_regs->PORTSC = temp;
}
/**
  \fn           _ux_hcd_xhci_set_port_power
  \brief        Set the port power
  \param[in]    xhci pointer
  \param[in]    ON or OFF
  \return       None
 */
void _ux_hcd_xhci_set_port_power(UX_HCD_XHCI *xhci, bool ON)
{
    uint32_t reg;
    reg = xhci->op_regs->PORTSC;
    reg = _ux_hcd_xhci_port_state_to_neutral(reg);
    if (ON)
    {
        /* Power on */
        xhci->op_regs->PORTSC = reg | PORT_POWER;
    }
    else
    {
        /* Power off */
        xhci->op_regs->PORTSC = reg & ~PORT_POWER;
    }
}
/**
  \fn           _ux_hcd_xhci_disable_port
  \brief        Disable the USB port
  \param[in]    xhci pointer
  \param[in]    port index
  \return       On success 0
 */
uint32_t _ux_hcd_xhci_disable_port(UX_HCD_XHCI *xhci, uint32_t port_index)
{
    uint32_t reg;
    reg = xhci->op_regs->PORTSC;
    reg &= ~PORT_PE;
    xhci->op_regs->PORTSC = reg;
    return 0;
}
/**
  \fn           _ux_hcd_xhci_reset_port
  \brief        reset the USB port
  \param[in]    xhci pointer
  \param[in]    port index
  \return       On success 0 remains error
 */
uint32_t _ux_hcd_xhci_reset_port(UX_HCD_XHCI *xhci, uint32_t port_index)
{
    uint32_t reg = 0;
    /* Read the Port Status and Control Register (PORTSC)  */
    reg = xhci->op_regs->PORTSC;
    if (reg == ~(uint32_t)0)
    {
        _ux_hcd_xhci_hc_died(xhci);
        return 1;
    }
    reg = _ux_hcd_xhci_port_state_to_neutral(reg);
    /* Reset The port  */
#ifdef DEBUG
    printf("hcd port reset\n");
#endif
    reg |= PORT_RESET;
    /* Write into the Port Status and Control Register (PORTSC)  */
    xhci->op_regs->PORTSC = reg;
    return 0;
}

void _ux_hcd_xhci_clear_port_change_bit(UX_HCD_XHCI *xhci, uint16_t wValue, uint16_t wIndex, uint32_t port_status)
{
    uint32_t status;
    switch (wValue)
    {
        case UX_PS_PRS:
            status = PORT_RC;
            break;
        case UX_PS_CCS:
            status = PORT_CSC;
            break;
        case UX_PS_POCI:
            status = PORT_OCC;
            break;
        case UX_PS_PES:
            status = PORT_PEC;
            break;
        default:
            /* Should never happen */
            return;
    }
    /* Change bits are all write 1 to clear */
    xhci->op_regs->PORTSC = port_status | status;
}
#endif //CFG_TUH_ENABLED
