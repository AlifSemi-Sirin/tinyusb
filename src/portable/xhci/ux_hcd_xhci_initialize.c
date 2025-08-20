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
 * @file     ux_hcd_xhci_initialize.c
 * @author   Mahesh Avula
 * @email    mahesh.avula@alifsemi.com
 * @version  V1.0.0
 * @date     12-Dec-2023
 * @brief    Initializes the XHCI host controller driver,programs all the XHCI registers.
 * @bug      None
 * @Note     None
 ******************************************************************************/


/* Include necessary system files.  */

#include "tusb_option.h"

#if CFG_TUH_ENABLED && CFG_TUSB_MCU == OPT_MCU_NONE

#include "host/hcd.h"
#include "ux_hcd_xhci_api.h"
//#include "ux_host_stack.h"
#include "RTE_Components.h"
#include CMSIS_device_header


UX_HCD_XHCI *hcd_xhci;
unsigned char _ux_system_host_hcd_xhci_name[] =                                     "ux_hcd_xhci";
TX_EVENT_FLAGS_GROUP CONTROL_EP_FLAG;
/*
 * Disable interrupts and begin the xHCI halting process.
 */
void _ux_hcd_xhci_disable_irq(UX_HCD_XHCI *xhci)
{
    uint32_t reg;
    uint32_t mask;
    mask = ~(UX_XHCI_IRQS);
    reg =  xhci->op_regs->USBSTS;
    reg &= STS_HALT;
    if (!reg)
        mask &= ~UX_XHCI_CMD_RUN;
    reg = xhci->op_regs->USBCMD;
    reg &= mask;
    xhci->op_regs->USBCMD = reg;
}
/*
 * Force HC into halt state.
 *
 * Disable any IRQs and clear the run/stop bit.
 * HC will complete any current and actively pipelined transactions, and
 * should halt within 16 ms of the run/stop bit being cleared.
 * Read HC Halted bit in the status register to see when the HC is finished.
 */
uint32_t _ux_hcd_xhci_halt(UX_HCD_XHCI *xhci)
{
    uint32_t reg;
    _ux_hcd_xhci_disable_irq(xhci);
    for (int32_t i = 0; i < 16*1000; i++)
    {
        reg = xhci->op_regs->USBSTS;
        if ((reg & STS_HALT) == 1)
            break;
        sys_busy_loop_us(1);
    }
    if ((reg & STS_HALT) == 0)
    {
#ifdef DEBUG
        printf("Host halt failed, %d\n", reg);
#endif
        return 1;
    }
    xhci->xhc_state |= UX_XHCI_STATE_HALTED;
    xhci->cmd_ring_state = CMD_RING_STATE_STOPPED;
    return 0;
}

static void _ux_hcd_xhci_zero_64b_regs(UX_HCD_XHCI *xhci)
{
    int32_t i;
    uint32_t reg;
    uint64_t reg_64 = 0;
    /* Clear HSEIE so that faults do not get signaled */
    reg = xhci->op_regs->USBCMD;
    reg &= ~UX_XHCI_CMD_HSEIE;
    xhci->op_regs->USBCMD = reg;

    /* Clear HSE (aka FATAL) */
    reg =xhci->op_regs->USBSTS;
    reg |= USB_STS_FATAL;
    xhci->op_regs->USBSTS = reg;

    /* Now zero the registers */
    xhci->op_regs->DCBAAP = reg_64;

    xhci->op_regs->CRCR = reg_64;

    for (i = 0; i < HCS_MAX_INTRS(xhci->hcs_params1); i++)
    {
        xhci->run_regs->ir_set[i].ERSTBA = 0;
        xhci->run_regs->ir_set[i].ERDP = 0;
    }
    /* Wait 16ms for the fault to appear. It will be cleared on reset */
    for (i = 0; i < 16*1000; i++)
    {
        reg = xhci->op_regs->USBSTS;
        if ((reg & USB_STS_FATAL) == 0)
            break;
        sys_busy_loop_us(1);
    }

    if (reg & USB_STS_FATAL)
    {
#ifdef DEBUG
        printf("Host fault failed, %x\n", reg);
#endif
    }
}
/*
 * Reset a halted HC.
 *
 * This resets pipelines, timers, counters, state machines, etc.
 * Transactions will be terminated immediately, and operational registers
 * will be set to their defaults.
 */
uint32_t _ux_hcd_xhci_reset(UX_HCD_XHCI *xhci)
{
    uint32_t reg;
    reg = xhci->op_regs->USBSTS;
    if (reg == ~(uint32_t)0)
    {
#ifdef DEBUG
        printf("WARN:Host not accessible, reset failed.\n");
#endif
        return 1;
    }
    if ((reg & STS_HALT) == 0)
    {
#ifdef DEBUG
        printf("Host controller not halted, aborting reset.\n");
#endif
        return 1;
    }
#ifdef DEBUG
    printf("Reset the Host controller\n");
#endif
    reg = xhci->op_regs->USBCMD;
    reg |= CMD_RESET;
    xhci->op_regs->USBCMD = reg;

    /* Existing Intel xHCI controllers require a delay of 1 mS,
     * after setting the CMD_RESET bit, and before accessing any
     * HC registers. This allows the HC to complete the
     * reset operation and be ready for HC register access.
     * Without this delay, the subsequent HC register access,
     * may result in a system hang very rarely.
     */
    for (int32_t i = 0; i < 1000*1000; i++)
    {
        reg = xhci->op_regs->USBCMD;
        if ((reg & CMD_RESET) == 0)
            break;
        sys_busy_loop_us(1);
    }
    if (reg & CMD_RESET)
    {
#ifdef DEBUG
        printf("Host Reset failed, %d\n", reg);
#endif
        return 1;
    }
    /*
     * xHCI cannot write to any doorbells or operational registers other
     * than status until the "Controller Not Ready" flag is cleared.
     */
    for (int32_t i = 0; i < 1000*1000; i++)
    {
        reg = xhci->op_regs->USBSTS;
        if ((reg & STS_CNR) == 0)
            break;
        sys_busy_loop_us(1);
    }
    /* check HC is ready or not  */
    if(reg & STS_CNR)
    {
#ifdef DEBUG
        printf("host controller is not ready\n");
#endif
        return 1;
    }
    return 0;
}

void _ux_hcd_xhci_start(UX_HCD_XHCI *xhci)
{
    uint32_t reg;
    reg = xhci->op_regs->USBCMD;
    reg |= UX_XHCI_CMD_RUN;
#ifdef DEBUG
    printf("Turn on Host controller\n");
#endif
    xhci->op_regs->USBCMD = reg;
    /*
     * Wait for the HCHalted Status bit to be 0 to indicate the host is
     * running. */
    for (int32_t i = 0; i < 16*1000; i++)
    {
        reg = xhci->op_regs->USBSTS;
        if ((reg & STS_HALT) == 0)
            break;
        sys_busy_loop_us(1);
    }

    if ((reg & STS_HALT))
    {
#ifdef DEBUG
        printf("Host took too long to start, "
                "waited %u microseconds.\n", UX_XHCI_MAX_HALT_USEC);
#endif
        return;
    }
    xhci->xhc_state = 0;
    xhci->cmd_ring_state = CMD_RING_STATE_RUNNING;
    /* After setting the Run/Stop (R/S) flag to ‘1’ and ringing the Host Controller Command
     *Doorbell for the first time    */
    _ux_hcd_xhci_ring_cmd_db(xhci);
}

/*
 * Start the HC after it was halted.
 *
 * This function is called by the USB core when the HC driver is added.
 * Its opposite is xhci_stop().
 *
 * ux_hcd_xhci_initialization() must be called once before this function can be called.
 * Reset the HC, enable device slot contexts, program DCBAAP, and
 * set command ring pointer and event ring pointer.
 *
 * Setup MSI-X vectors and enable interrupts.
 */

uint32_t _ux_hcd_xhci_run(UX_HCD_XHCI *xhci)
{
    uint32_t reg = 0;
    xhci->imod_interval = 4000;
    reg = xhci->run_regs->ir_set[0].IMOD;
    reg &= ~ER_IRQ_INTERVAL_MASK;
    reg |= (xhci->imod_interval / 250) & ER_IRQ_INTERVAL_MASK;
    xhci->run_regs->ir_set[0].IMOD = reg;

    /* Enable the irqs */
    reg = xhci->op_regs->USBCMD;
    reg |= UX_XHCI_CMD_EIE;
    xhci->op_regs->USBCMD = reg;

    reg = xhci->run_regs->ir_set[0].IMAN;
    xhci->run_regs->ir_set[0].IMAN = ER_IRQ_ENABLE(reg);
    return 0;
}

void _ux_hcd_xhci_host_timer_function(void *arg)
{
    UX_HCD_XHCI   *hcd_xhci = (UX_HCD_XHCI  *)arg;

    if (_ux_hcd_xhci_port_current_status_get(hcd_xhci, 0))
    {
        hcd_xhci -> ux_hcd_xhci_hcd_owner -> ux_hcd_root_hub_signal[0]++;
#ifdef TODO
        osal_semaphore_post(&_ux_system_host -> ux_system_host_enum_semaphore, true);
#endif
    }
}

uint32_t  _ux_hcd_xhci_initialize(UX_HCD *hcd)
{
    UX_HCD_XHCI  *xhci;
    uint32_t     xhci_reg;
    uint32_t      ret;
#ifdef DEBUG
    printf("xhci_initialize\n");
#endif
    /* The controller initialized here is of XHCI type.  */
    hcd -> ux_hcd_controller_type =  UX_XHCI_CONTROLLER;


    /* Allocate memory for this XHCI HCD instance.  */
#ifdef USE_STATIC_RAM
    static UX_HCD_XHCI _xhci __attribute__((section("usb_dma_buf")));
    memset(&_xhci, 0, sizeof(_xhci));
    xhci = &_xhci;
#else
    xhci =  _ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY, sizeof(UX_HCD_XHCI));
#endif
    if (xhci == NULL)
        return(UX_MEMORY_INSUFFICIENT);

    hcd_xhci = xhci;

    /* Set the pointer to the XHCI HCD.  */
    hcd -> ux_hcd_controller_hardware =  (void *) xhci;

    /* Get the USB base address.  */
    xhci->regs = (USB_Type  *)hcd -> ux_hcd_io;

    /* Obtain the address of the HCOR registers. This is a byte offset from the
       HCOR Cap registers.  */
    xhci_reg =  xhci->regs->CAPLENGTH;
    xhci -> ux_hcd_xhci_hcor =  (xhci_reg & 0xff);
    xhci -> hcs_params1 =  xhci->regs->HCSPARAMS1;
    xhci -> hcs_params2 =  xhci->regs->HCSPARAMS2;
    xhci -> hcs_params3 =  xhci->regs->HCSPARAMS3;
    xhci -> hcc_params1 = xhci->regs->HCCPARAMS1;
    xhci -> hcc_params2 = xhci->regs->HCCPARAMS2;
    xhci->hci_version   = HC_VERSION(xhci_reg);
    xhci -> dba_off = xhci->regs->DBOFF;

    xhci -> ux_hcd_xhci_hcrr =  xhci->regs->RTSOFF;
    /* Get the base address of the RUNTIME Registers  */
    xhci -> run_regs = (UX_XHCI_RUN_REGS *)((uint8_t *)(xhci -> regs) + (xhci -> ux_hcd_xhci_hcrr & RTSOFF_MASK));
    /* Get the base address of the OPERATIONAL Registers  */
    xhci -> op_regs = (UX_XHCI_OP_REGS *)(( uint8_t*)(xhci -> regs) + xhci ->ux_hcd_xhci_hcor);
    /* Get the base address of the DOORBELL  Registers  */
    xhci -> dba_regs = (UX_XHCI_DB_REGS *)((uint8_t *)(xhci -> regs) + xhci ->dba_off);

    hcd -> ux_hcd_nb_root_hubs = HCS_MAX_PORTS(xhci -> hcs_params1);
    if(hcd -> ux_hcd_nb_root_hubs > UX_MAX_ROOTHUB_PORT)
        hcd -> ux_hcd_nb_root_hubs = UX_MAX_ROOTHUB_PORT;

    /* Set the generic HCD owner for the XHCI HCD.  */
    xhci -> ux_hcd_xhci_hcd_owner =  hcd;

    /* Initialize the function entry for this HCD.  */
    hcd -> ux_hcd_entry_function =  _ux_hcd_xhci_entry;

    /* Set the state of the controller to HALTED first.  */
    hcd -> ux_hcd_status =  UX_HCD_STATUS_HALTED;
    /* Put HC in halt state  */
    ret = _ux_hcd_xhci_halt(xhci);
    if (ret != UX_SUCCESS)
        return ret;
    /* Zeroing 64bit base registers, expecting fault   */
    _ux_hcd_xhci_zero_64b_regs(xhci);
    /* RESET the HC   */
    ret = _ux_hcd_xhci_reset(xhci);
    if (ret != UX_SUCCESS)
        return ret;

    ret = _ux_hcd_xhci_mem_init(xhci);
    if (ret != UX_SUCCESS)
    {
#ifdef DEBUG
        printf("XHCI Init Failed \n");
#endif
        return ret;
    }
    /* Enable the HC interrupts from interrupter Register   */
    _ux_hcd_xhci_run(xhci);

    /*   Start the HC      */
    _ux_hcd_xhci_start(xhci);

    /* Set the state of the controller to OPERATIONAL.  */
    hcd -> ux_hcd_status =  UX_HCD_STATUS_OPERATIONAL;

    ret = _ux_utility_event_flags_create(&CONTROL_EP_FLAG,"USB_CONTROL_EP_EVENT_FLAG");
    if (ret != UX_SUCCESS)
    {
        return ret;
    }
#if OSAL_MUTEX_REQUIRED
    /* Create mutex for periodic list modification.  */
    osal_mutex_t _hcd_mutex = osal_mutex_create(&xhci -> ux_hcd_xhci_periodic_mutex);
    //ret = osal_mutex_create(&xhci -> ux_hcd_xhci_periodic_mutex, "xhci_periodic_mutex");
    if (!_hcd_mutex)
        return UX_ERROR;
#endif
    ret = _ux_utility_timer_create(&xhci ->port_status_timer, "XHCI Port status timer",
            _ux_hcd_xhci_host_timer_function, xhci, 20, 20, UX_NO_ACTIVATE);
    if (ret != UX_SUCCESS)
        return ret;
    /* Enable the interrupts  */
    NVIC_ClearPendingIRQ(USB_IRQ_IRQn);
    NVIC_SetPriority(USB_IRQ_IRQn, 2);
    NVIC_EnableIRQ(USB_IRQ_IRQn);
    /* Return error status code.  */
    return ret;
}

#endif //CFG_TUH_ENABLED
