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
 * @file     ux_hcd_xhci.h
 * @author   Mahesh Avula
 * @email    mahesh.avula@alifsemi.com
 * @version  V1.0.0
 * @date     12-Dec-2023
 * @brief    This file contains all the header and extern functions used by the USBX host XHCI Controller
 * @bug      None.
 * @Note     None
 ******************************************************************************/

#ifndef _UX_HCD_XHCI_H_
#define _UX_HCD_XHCI_H_

#define USE_STATIC_RAM

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
//#include "ux_api.h"
#include "ux_hcd_xhci_list.h"
//#include "../../../../usb_device_driver/ux_dcd_dwc3_private.h"

#if defined(CORE_M55_HE)
#include "M55_HE.h"
#elif defined(CORE_M55_HP)
#include "M55_HP.h"
#else
#error "Unsupported core!"
#endif

#ifdef   __cplusplus

/* Yes, C++ compiler is present.  Use standard C.  */
extern   "C" {

#endif

#if 1 //ported from USBX

#define upper_32_bits(n)    ((uint32_t)(((n) >> 16) >> 16))
#define lower_32_bits(n)    ((uint32_t)(n))

#define UX_MAX_PACKET_SIZE_MASK                                         0x7ffu
#define UX_MAX_NUMBER_OF_TRANSACTIONS_MASK                              0x1800u
#define UX_MAX_NUMBER_OF_TRANSACTIONS_SHIFT                             11

#define UX_ENDPOINT_DIRECTION                                           0x80u
#define UX_ENDPOINT_IN                                                  0x80u
#define UX_ENDPOINT_OUT                                                 0x00u

#define UX_REQUEST_DIRECTION                                            0x80u
#define UX_REQUEST_IN                                                   0x80u
#define UX_REQUEST_OUT                                                  0x00u

#define UX_LOW_SPEED_DEVICE                                             0
#define UX_FULL_SPEED_DEVICE                                            1
#define UX_HIGH_SPEED_DEVICE                                            2

// stop using it #define UX_MASK_ENDPOINT_TYPE                                           3u
#define UX_CONTROL_ENDPOINT                                             0u
#define UX_ISOCHRONOUS_ENDPOINT                                         1u
#define UX_BULK_ENDPOINT                                                2u
#define UX_INTERRUPT_ENDPOINT                                           3u

#define UX_ISOCHRONOUS_ENDPOINT_IN                                      0x81u
#define UX_ISOCHRONOUS_ENDPOINT_OUT                                     0x01u
#define UX_BULK_ENDPOINT_IN                                             0x82u
#define UX_BULK_ENDPOINT_OUT                                            0x02u
#define UX_INTERRUPT_ENDPOINT_IN                                        0x83u
#define UX_INTERRUPT_ENDPOINT_OUT                                       0x03u

/* Define USBX max root hub port (1 ~ n).  */
#ifndef UX_MAX_ROOTHUB_PORT
#define UX_MAX_ROOTHUB_PORT                                 4
#endif

#define TX_SUCCESS                                                      0
#define UX_SUCCESS                                                      0
#define UX_ERROR                                                        0xff
#define UX_MEMORY_INSUFFICIENT                                          0x12
#define UX_TRANSFER_ERROR                                               0x23
#define UX_FUNCTION_NOT_SUPPORTED                                       0x54
#define UX_CONTROLLER_UNKNOWN                                           0x55

#define UX_REGULAR_MEMORY                                               0
#define UX_CACHE_SAFE_MEMORY                                            1

#define UX_UNUSED                                                       0
#define UX_USED                                                         1

#define UX_NO_ALIGN                                                     0u
#define UX_ALIGN_8                                                      0x07u
#define UX_ALIGN_16                                                     0x0fu
#define UX_ALIGN_32                                                     0x1fu
#define UX_ALIGN_64                                                     0x3fu
#define UX_ALIGN_128                                                    0x7fu
#define UX_ALIGN_256                                                    0xffu
#define UX_ALIGN_512                                                    0x1ffu
#define UX_ALIGN_1024                                                   0x3ffu
#define UX_ALIGN_2048                                                   0x7ffu
#define UX_ALIGN_4096                                                   0xfffu
#define UX_SAFE_ALIGN                                                   0xffffffffu
#define UX_MAX_SCATTER_GATHER_ALIGNMENT                                 4096
#ifndef UX_ALIGN_MIN
#define UX_ALIGN_MIN                                                    UX_ALIGN_8
#endif

#define UX_HCD_STATUS_UNUSED                                            0
#define UX_HCD_STATUS_HALTED                                            1
#define UX_HCD_STATUS_OPERATIONAL                                       2
#define UX_HCD_STATUS_DEAD                                              3

#define UX_NO_ACTIVATE                                                  (0ul)

/* Define the system level for error trapping. */
#define UX_SYSTEM_LEVEL_INTERRUPT                                       1
#define UX_SYSTEM_LEVEL_THREAD                                          2

/* Define the system context for error trapping. */
#define UX_SYSTEM_CONTEXT_HCD                                           1
#define UX_SYSTEM_CONTEXT_DCD                                           2
#define UX_SYSTEM_CONTEXT_INIT                                          3
#define UX_SYSTEM_CONTEXT_ENUMERATOR                                    4
#define UX_SYSTEM_CONTEXT_ROOT_HUB                                      5
#define UX_SYSTEM_CONTEXT_HUB                                           6
#define UX_SYSTEM_CONTEXT_CLASS                                         7
#define UX_SYSTEM_CONTEXT_UTILITY                                       8
#define UX_SYSTEM_CONTEXT_DEVICE_STACK                                  9
#define UX_SYSTEM_CONTEXT_HOST_STACK                                    10


#define UX_PS_CCS                                                       0x01u
#define UX_PS_CPE                                                       0x01u
#define UX_PS_PES                                                       0x02u
#define UX_PS_PSS                                                       0x04u
#define UX_PS_POCI                                                      0x08u
#define UX_PS_PRS                                                       0x10u
#define UX_PS_PPS                                                       0x20u
#define UX_PS_DS_LS                                                     0x00u
#define UX_PS_DS_FS                                                     0x40u
#define UX_PS_DS_HS                                                     0x80u

#define UX_PS_DS                                                        6u

/* Define event filters that can be used to selectively disable certain events or groups of events.  */

#define UX_TRACE_ALL_EVENTS                                             0x7F000000  /* All USBX events                          */
#define UX_TRACE_ERRORS                                                 0x01000000  /* USBX Errors events                       */
#define UX_TRACE_HOST_STACK_EVENTS                                      0x02000000  /* USBX Host Class Events                   */
#define UX_TRACE_DEVICE_STACK_EVENTS                                    0x04000000  /* USBX Device Class Events                 */
#define UX_TRACE_HOST_CONTROLLER_EVENTS                                 0x08000000  /* USBX Host Controller Events              */
#define UX_TRACE_DEVICE_CONTROLLER_EVENTS                               0x10000000  /* USBX Device Controllers Events           */
#define UX_TRACE_HOST_CLASS_EVENTS                                      0x20000000  /* USBX Host Class Events                   */
#define UX_TRACE_DEVICE_CLASS_EVENTS                                    0x40000000  /* USBX Device Class Events                 */

#define UX_TRACE_ERROR                                                  999

/* Define USBX standard commands.  */

#define UX_GET_STATUS                                                   0u
#define UX_CLEAR_FEATURE                                                1u
#define UX_SET_FEATURE                                                  3u
#define UX_SET_ADDRESS                                                  5u
#define UX_GET_DESCRIPTOR                                               6u
#define UX_SET_DESCRIPTOR                                               7u
#define UX_GET_CONFIGURATION                                            8u
#define UX_SET_CONFIGURATION                                            9u
#define UX_GET_INTERFACE                                                10u
#define UX_SET_INTERFACE                                                11u
#define UX_SYNCH_FRAME                                                  12u

/* Define USBX HCD API function constants.  */

#define UX_HCD_DISABLE_CONTROLLER                                       1
#define UX_HCD_GET_PORT_STATUS                                          2
#define UX_HCD_ENABLE_PORT                                              3
#define UX_HCD_DISABLE_PORT                                             4
#define UX_HCD_POWER_ON_PORT                                            5
#define UX_HCD_POWER_DOWN_PORT                                          6
#define UX_HCD_SUSPEND_PORT                                             7
#define UX_HCD_RESUME_PORT                                              8
#define UX_HCD_RESET_PORT                                               9
#define UX_HCD_GET_FRAME_NUMBER                                         10
#define UX_HCD_SET_FRAME_NUMBER                                         11
#define UX_HCD_TRANSFER_REQUEST                                         12
#define UX_HCD_TRANSFER_RUN                                             12
#define UX_HCD_TRANSFER_ABORT                                           13
#define UX_HCD_CREATE_ENDPOINT                                          14
#define UX_HCD_DESTROY_ENDPOINT                                         15
#define UX_HCD_RESET_ENDPOINT                                           16
#define UX_HCD_PROCESS_DONE_QUEUE                                       17
#define UX_HCD_TASKS_RUN                                                17
#define UX_HCD_UNINITIALIZE                                             18

#define UX_TRACE_OBJECT_REGISTER(t,p,n,a,b)
#define UX_TRACE_OBJECT_UNREGISTER(o)
#define UX_TRACE_IN_LINE_INSERT(i,a,b,c,d,f,g,h)
#define UX_TRACE_EVENT_UPDATE(e,t,i,a,b,c,d)

/* API input parameters and general constants.  */

#define TX_NO_WAIT                      ((unsigned long)  0)
#define TX_WAIT_FOREVER                 ((unsigned long)  0xFFFFFFFFUL)
#define TX_AND                          ((unsigned long)   2)
#define TX_AND_CLEAR                    ((unsigned long)   3)
#define TX_OR                           ((unsigned long)   0)
#define TX_OR_CLEAR                     ((unsigned long)   1)
#define TX_1_ULONG                      ((unsigned long)   1)
#define TX_2_ULONG                      ((unsigned long)   2)
#define TX_4_ULONG                      ((unsigned long)   4)
#define TX_8_ULONG                      ((unsigned long)   8)
#define TX_16_ULONG                     ((unsigned long)   16)
#define TX_NO_TIME_SLICE                ((unsigned long)  0)
#define TX_AUTO_START                   ((unsigned long)   1)
#define TX_DONT_START                   ((unsigned long)   0)
#define TX_AUTO_ACTIVATE                ((unsigned long)   1)
#define TX_NO_ACTIVATE                  ((unsigned long)   0)
#define TX_TRUE                         ((unsigned long)   1)
#define TX_FALSE                        ((unsigned long)   0)
#define TX_NULL                         ((void *) 0)
#define TX_INHERIT                      ((unsigned long)   1)
#define TX_NO_INHERIT                   ((unsigned long)   0)
#define TX_THREAD_ENTRY                 ((unsigned long)   0)
#define TX_THREAD_EXIT                  ((unsigned long)   1)
#define TX_NO_SUSPENSIONS               ((unsigned long)   0)
#define TX_NO_MESSAGES                  ((unsigned long)   0)
#define TX_EMPTY                        ((unsigned long)  0)
#define TX_CLEAR_ID                     ((unsigned long)  0)
#if defined(TX_ENABLE_RANDOM_NUMBER_STACK_FILLING) && defined(TX_ENABLE_STACK_CHECKING)
#define TX_STACK_FILL                   (thread_ptr -> tx_thread_stack_fill_value)
#else
#define TX_STACK_FILL                   ((unsigned long)  0xEFEFEFEFUL)
#endif

#define UX_WAIT_FOREVER TX_WAIT_FOREVER

typedef tusb_desc_endpoint_t UX_ENDPOINT_DESCRIPTOR;

typedef struct  {
    tusb_desc_endpoint_t ux_endpoint_descriptor;
    void *ux_endpoint_device;
} UX_ENDPOINT;

typedef struct UX_TRANSFER_STRUCT {
    unsigned long ux_transfer_request_status;
    unsigned long ux_transfer_request_actual_length;
    unsigned long ux_transfer_request_requested_length;
    unsigned int ux_transfer_request_type;
    unsigned int ux_transfer_request_function;
    unsigned int ux_transfer_request_value;
    unsigned int ux_transfer_request_index;
    void (*ux_transfer_request_completion_function) (struct UX_TRANSFER_STRUCT *);
    UX_ENDPOINT *ux_transfer_request_endpoint;
    unsigned long ux_transfer_request_maximum_length;
    unsigned long ux_transfer_request_timeout_value;
    unsigned int ux_transfer_request_completion_code;
    unsigned long ux_transfer_request_packet_length;
    void *hcpriv; //UX_URB_PRIV
    unsigned char *ux_transfer_request_data_pointer;
    osal_semaphore_t ux_transfer_request_semaphore;
} UX_TRANSFER;

typedef struct  {
    unsigned long ux_device_address;
    unsigned long ux_device_speed;
    unsigned long ux_device_port_location;
} UX_DEVICE;

typedef struct  {
    unsigned int ux_hcd_status;
    unsigned int ux_hcd_controller_type;
    unsigned int ux_hcd_irq;
    unsigned int ux_hcd_nb_root_hubs;
    unsigned int ux_hcd_root_hub_signal[UX_MAX_ROOTHUB_PORT];
    void *ux_hcd_entry_function;
    void *ux_hcd_controller_hardware;
    void *ux_hcd_io;
} UX_HCD;

typedef struct  {
    int dummy;
} UX_MUTEX;

typedef struct  {
    int dummy;
} UX_TIMER;

typedef struct {
    uint32_t CAPLENGTH;
    uint32_t HCSPARAMS1;
    uint32_t HCSPARAMS2;
    uint32_t HCSPARAMS3;
    uint32_t HCCPARAMS1;
    uint32_t DBOFF;
    uint32_t RTSOFF;
    uint32_t HCCPARAMS2;
} USB_Type;

#define UX_EVENT_FLAGS_GROUP                                            TX_EVENT_FLAGS_GROUP

typedef struct  {
    int dummy;
} TX_EVENT_FLAGS_GROUP;


static inline int ux_endpoint_xfer_bulk(const UX_ENDPOINT_DESCRIPTOR *epd)
{
    return 1;
}

static inline int ux_endpoint_xfer_isoc(const UX_ENDPOINT_DESCRIPTOR *epd)
{
    return 0;
}

static inline int ux_endpoint_xfer_int(const UX_ENDPOINT_DESCRIPTOR *epd)
{
    return 0;
}

static inline int ux_endpoint_xfer_control(const UX_ENDPOINT_DESCRIPTOR *epd)
{
    return 0;
}

static inline int ux_endpoint_maxp(const UX_ENDPOINT_DESCRIPTOR *epd)
{
    return 512;
}

static inline int ux_endpoint_type(const UX_ENDPOINT_DESCRIPTOR *epd)
{
    return 0;
}

static inline void _ux_utility_memory_free(void *ptr)
{
    if (ptr != NULL)
    {
        void *pptr = ptr - sizeof(void*);
        unsigned long palloc = *((unsigned long*)pptr);
        free((void*)palloc);
    }
}

typedef struct {
    unsigned long memory_alignment;
    unsigned long memory_cache_flag;
    unsigned long memory_size_requested;
    void *ptr;
} mallocs_t;

extern mallocs_t _mallocs[32];
extern uint8_t _mallocs_cnt;

static inline void *_ux_utility_memory_allocate(
    unsigned long memory_alignment,
    unsigned long memory_cache_flag,
    unsigned long memory_size_requested)
{
    unsigned long malloc_size = memory_size_requested + sizeof(void*) + memory_alignment;

    void *palloc = malloc(malloc_size);
    void *ptr = NULL;
    if (palloc != NULL)
    {
        memset(palloc, 0, malloc_size);
        ptr = (void*)(((unsigned long)palloc + sizeof(void*) + memory_alignment) & ~memory_alignment);
        void *pptr = ptr - sizeof(void*);
        unsigned long *paddr = (unsigned long*)pptr;
        *paddr = (unsigned long)palloc;
    }

    if (ptr && memory_alignment)
    {
        TU_ASSERT((((unsigned long)ptr) & memory_alignment) == 0);
    }

    _mallocs[_mallocs_cnt].memory_alignment = memory_alignment;
    _mallocs[_mallocs_cnt].memory_cache_flag = memory_cache_flag;
    _mallocs[_mallocs_cnt].memory_size_requested = memory_size_requested;
    _mallocs[_mallocs_cnt].ptr = ptr;
    _mallocs_cnt++;

   return ptr;
}

static void _ux_system_error_handler(unsigned int system_level, unsigned int system_context, unsigned int error_code)
{
}

static inline unsigned int _ux_utility_timer_create(UX_TIMER *timer, char *timer_name, void (*expiration_function) (void*),
                                                    void *expiration_input, unsigned long initial_ticks, unsigned long reschedule_ticks,
                                                    unsigned int activation_flag)
{
    return 0;
}

static inline unsigned int tx_timer_activate(UX_TIMER *timer)
{
    return 0;
}

static inline unsigned int _ux_utility_event_flags_create(UX_EVENT_FLAGS_GROUP *group_ptr, char *name)
{
    return 0;
}

static inline unsigned int _ux_utility_event_flags_set(UX_EVENT_FLAGS_GROUP*group_ptr, unsigned long flags_to_set,
                                                       unsigned int set_option)
{
    return 0;
}

static inline unsigned int _ux_utility_event_flags_get(UX_EVENT_FLAGS_GROUP*group_ptr, unsigned long requested_flags,
                                                       unsigned int get_option, unsigned long *actual_flags_ptr, unsigned long wait_option)
{
    return 0;
}

#endif //ported_from USBX

#define BIT_ULL(nr)                                             (1ULL << (nr))

/* Define XHCI generic definitions.  */
#define UX_XHCI_CONTROLLER                                      3
#define NUM_PORT_REGS                                           4
#define UX_XHCI_MAX_HC_SLOTS                                    256
#define TRBS_PER_SEGMENT                                        256
#define LINK_TOGGLE                                             (0x1 << 1)

/* Control transfers event flag  */
#define UX_XHCI_CONTROL_EP_EVENT                                (1 << 0)
#define TRB_SEGMENT_SIZE                                        (TRBS_PER_SEGMENT*16)

#define TRB_MAX_BUFF_SHIFT                                      16
#define TRB_MAX_BUFF_SIZE                                       (1 << TRB_MAX_BUFF_SHIFT)
/* How much data is left before the 64KB boundary? */
#define TRB_BUFF_LEN_UP_TO_BOUNDARY(addr)                       (TRB_MAX_BUFF_SIZE - \
                                                                    (addr & (TRB_MAX_BUFF_SIZE - 1)))

#define MAX_SOFT_RETRY                                          3
#define CMD_RESET                                               (1 << 1)

/* Transfer event TRB length bit mask */
/* bits 0:23 */
#define EVENT_TRB_LEN(p)                                        ((p) & 0xffffff)

/** Transfer Event bit fields **/
#define TRB_TO_EP_ID(p)                                         (((p) >> 16) & 0x1f)

/* Completion Code - only applicable for some types of TRBs */
#define COMP_CODE_MASK                                          (0xff << 24)
#define GET_COMP_CODE(p)                                        (((p) & COMP_CODE_MASK) >> 24)
#define COMP_INVALID                                            0
#define COMP_SUCCESS                                            1
#define COMP_DATA_BUFFER_ERROR                                  2
#define COMP_BABBLE_DETECTED_ERROR                              3
#define COMP_USB_TRANSACTION_ERROR                              4
#define COMP_TRB_ERROR                                          5
#define COMP_STALL_ERROR                                        6
#define COMP_RESOURCE_ERROR                                     7
#define COMP_BANDWIDTH_ERROR                                    8
#define COMP_NO_SLOTS_AVAILABLE_ERROR                           9
#define COMP_INVALID_STREAM_TYPE_ERROR                          10
#define COMP_SLOT_NOT_ENABLED_ERROR                             11
#define COMP_ENDPOINT_NOT_ENABLED_ERROR                         12
#define COMP_SHORT_PACKET                                       13
#define COMP_RING_UNDERRUN                                      14
#define COMP_RING_OVERRUN                                       15
#define COMP_VF_EVENT_RING_FULL_ERROR                           16
#define COMP_PARAMETER_ERROR                                    17
#define COMP_BANDWIDTH_OVERRUN_ERROR                            18
#define COMP_CONTEXT_STATE_ERROR                                19
#define COMP_NO_PING_RESPONSE_ERROR                             20
#define COMP_EVENT_RING_FULL_ERROR                              21
#define COMP_INCOMPATIBLE_DEVICE_ERROR                          22
#define COMP_MISSED_SERVICE_ERROR                               23
#define COMP_COMMAND_RING_STOPPED                               24
#define COMP_COMMAND_ABORTED                                    25
#define COMP_STOPPED                                            26
#define COMP_STOPPED_LENGTH_INVALID                             27
#define COMP_STOPPED_SHORT_PACKET                               28
#define COMP_MAX_EXIT_LATENCY_TOO_LARGE_ERROR                   29
#define COMP_ISOCH_BUFFER_OVERRUN                               31
#define COMP_EVENT_LOST_ERROR                                   32
#define COMP_UNDEFINED_ERROR                                    33
#define COMP_INVALID_STREAM_ID_ERROR                            34
#define COMP_SECONDARY_BANDWIDTH_ERROR                          35
#define COMP_SPLIT_TRANSACTION_ERROR                            36

#define DB_VALUE(ep, stream)                                    ((((ep) + 1) & 0xff) | ((stream) << 16))
#define DB_VALUE_HOST                                           0x00000000

#define SET_DEQ_PENDING                                         (1 << 0)
#define EP_HALTED                                               (1 << 1)   /* For stall handling */
#define EP_STOP_CMD_PENDING                                     (1 << 2)     /* For URB cancellation */

/* Transitioning the endpoint to using streams, don't enqueue URBs */
#define EP_GETTING_STREAMS                                      (1 << 3)
#define EP_HAS_STREAMS                                          (1 << 4)

/* Transitioning the endpoint to not using streams, don't enqueue URBs */
#define EP_GETTING_NO_STREAMS                                   (1 << 5)
#define EP_HARD_CLEAR_TOGGLE                                    (1 << 6)
#define EP_SOFT_CLEAR_TOGGLE                                    (1 << 7)

/* usb_hub_clear_tt_buffer is in progress */
#define EP_CLEARING_TT                                          (1 << 8)

/* command register values to disable interrupts and halt the HC */
/* start/stop HC execution - do not write unless HC is halted*/
#define UX_XHCI_CMD_RUN                                         (1 << 0)

/* Event Interrupt Enable - get irq when EINT bit is set in USBSTS register */
#define UX_XHCI_CMD_EIE                                         (1 << 2)

/* Host System Error Interrupt Enable - get irq when HSEIE bit set in USBSTS */
#define UX_XHCI_CMD_HSEIE                                       (1 << 3)

#define USB_STS_FATAL                                           (1 << 2)

/* Enable Wrap Event - '1' means xHC generates an event when MFINDEX wraps. */
#define UX_XHCI_CMD_EWE                                         (1 << 10)

#define UX_XHCI_IRQS                                            (UX_XHCI_CMD_EIE | UX_XHCI_CMD_HSEIE | UX_XHCI_CMD_EWE)

/* true: Controller Not Ready to accept doorbell or op reg writes after reset */
#define   XHCI_STS_CNR                                          (1 << 11)

#define   XHCI_PAGE_BIT_SET                                     (1 << 0)

/* hc_capbase bitmasks */
/* bits 7:0 - how long is the Capabilities register */
#define HC_LENGTH(p)                                            XHCI_HC_LENGTH(p)
/* bits 31:16  */
#define HC_VERSION(p)                                           (((p) >> 16) & 0xffff)

/* Extended capability register fields */
#define UX_XHCI_EXT_CAPS_ID(p)                                  (((p)>>0) & 0xff)
#define UX_XHCI_EXT_CAPS_NEXT(p)                                (((p)>>8)&0xff)
#define XHCI_EXT_CAPS_VAL(p)                                    ((p)>>16)

/* Extended capability IDs - ID 0 reserved */
#define UX_XHCI_EXT_CAPS_LEGACY                                 1
#define UX_XHCI_EXT_CAPS_PROTOCOL                               2
#define UX_XHCI_EXT_CAPS_PM                                     3
#define UX_XHCI_EXT_CAPS_VIRT                                   4
#define UX_XHCI_EXT_CAPS_ROUTE                                  5
/* IDs 6-9 reserved */
#define UX_XHCI_EXT_CAPS_DEBUG                                  10
/* Vendor caps */
#define UX_XHCI_EXT_CAPS_VENDOR_INTEL                           192

/* USB Legacy Support Capability - section 7.1.1 */
#define UX_XHCI_HC_BIOS_OWNED                                   (1 << 16)
#define UX_XHCI_HC_OS_OWNED                                     (1 << 24)

/* USB Legacy Support Capability - section 7.1.1 */
/* Add this offset, plus the value of xECP in HCCPARAMS to the base address */
#define UX_XHCI_LEGACY_SUPPORT_OFFSET                           (0x00)

/* USB Legacy Support Control and Status Register  - section 7.1.2 */
/* Add this offset, plus the value of xECP in HCCPARAMS to the base address */
#define UX_XHCI_LEGACY_CONTROL_OFFSET                           (0x04)

/* bits 1:3, 5:12, and 17:19 need to be preserved; bits 21:28 should be zero */
#define XHCI_LEGACY_DISABLE_SMI                                 ((0x7 << 1) + (0xff << 5) + (0x7 << 17))
#define UX_XHCI_LEGACY_SMI_EVENTS                               (0x7 << 29)

/* USB 2.0 xHCI 0.96 L1C capability - section 7.2.2.1.3.2 */
#define UX_XHCI_L1C                                             (1 << 16)

/* USB 2.0 xHCI 1.0 hardware LMP capability - section 7.2.2.1.3.2 */
#define UX_XHCI_HLC                                             (1 << 19)
#define UX_XHCI_BLC                                             (1 << 20)

/* HCSPARAMS1 - hcs_params1 - bitmasks */
/* bits 0:7, Max Device Slots */
#define HCS_MAX_SLOTS(p)                                        (((p) >> 0) & 0xff)
#define HCS_SLOTS_MASK                                          0xff
/* bits 8:18, Max Interrupters */
#define HCS_MAX_INTRS(p)                                        (((p) >> 8) & 0x7ff)
/* bits 24:31, Max Ports - max value is 0x7F = 127 ports */
#define HCS_MAX_PORTS(p)                                        (((p) >> 24) & 0x7f)

/* HCSPARAMS2 - hcs_params2 - bitmasks */
/* bits 0:3, frames or uframes that SW needs to queue transactions
 * ahead of the HW to meet periodic deadlines */
#define HCS_IST(p)                                              (((p) >> 0) & 0xf)
/* bits 4:7, max number of Event Ring segments */
#define HCS_ERST_MAX(p)                                         (((p) >> 4) & 0xf)
/* bits 21:25 Hi 5 bits of Scratchpad buffers SW must allocate for the HW */
/* bit 26 Scratchpad restore - for save/restore HW state - not used yet */
/* bits 27:31 Lo 5 bits of Scratchpad buffers SW must allocate for the HW */
#define HCS_MAX_SCRATCHPAD(p)                                   ((((p) >> 16) & 0x3e0) | (((p) >> 27) & 0x1f))

/* HCSPARAMS3 - hcs_params3 - bitmasks */
/* bits 0:7, Max U1 to U0 latency for the roothub ports */
#define HCS_U1_LATENCY(p)                                       (((p) >> 0) & 0xff)
/* bits 16:31, Max U2 to U0 latency for the roothub ports */
#define HCS_U2_LATENCY(p)                                       (((p) >> 16) & 0xffff)

/* HCCPARAMS - hcc_params - bitmasks */
/* true: HC can use 64-bit address pointers */
#define HCC_64BIT_ADDR(p)                                        ((p) & (1 << 0))
/* true: HC can do bandwidth negotiation */
#define HCC_BANDWIDTH_NEG(p)                                     ((p) & (1 << 1))
/* true: HC uses 64-byte Device Context structures
 * FIXME 64-byte context structures aren't supported yet.
 */
#define HCC_64BYTE_CONTEXT(p)                                   ((p) & (1 << 2))
/* true: HC has port power switches */
#define HCC_PPC(p)                                              ((p) & (1 << 3))
/* true: HC has port indicators */
#define HCS_INDICATOR(p)                                        ((p) & (1 << 4))
/* true: HC has Light HC Reset Capability */
#define HCC_LIGHT_RESET(p)                                      ((p) & (1 << 5))
/* true: HC supports latency tolerance messaging */
#define HCC_LTC(p)                                              ((p) & (1 << 6))
/* true: no secondary Stream ID Support */
#define HCC_NSS(p)                                              ((p) & (1 << 7))
/* true: HC supports Stopped - Short Packet */
#define HCC_SPC(p)                                              ((p) & (1 << 9))
/* true: HC has Contiguous Frame ID Capability */
#define HCC_CFC(p)                                              ((p) & (1 << 11))
/* Max size for Primary Stream Arrays - 2^(n+1), where n is bits 12:15 */
#define HCC_MAX_PSA(p)                                          (1 << ((((p) >> 12) & 0xf) + 1))
/* Extended Capabilities pointer from PCI base - section 5.3.6 */
#define HCC_EXT_CAPS(p)                                         XHCI_HCC_EXT_CAPS(p)

#define CTX_SIZE(_hcc)                                          (HCC_64BYTE_CONTEXT(_hcc) ? 64 : 32)

/* db_off bitmask - bits 0:1 reserved */
#define DBOFF_MASK                                              (~0x3)

/* run_regs_off bitmask - bits 0:4 reserved */
#define RTSOFF_MASK                                             (~0x1f)

/* HCCPARAMS2 - hcc_params2 - bitmasks */
/* true: HC supports U3 entry Capability */
#define HCC2_U3C(p)                                             ((p) & (1 << 0))
/* true: HC supports Configure endpoint command Max exit latency too large */
#define HCC2_CMC(p)                                             ((p) & (1 << 1))
/* true: HC supports Force Save context Capability */
#define HCC2_FSC(p)                                             ((p) & (1 << 2))
/* true: HC supports Compliance Transition Capability */
#define HCC2_CTC(p)                                             ((p) & (1 << 3))
/* true: HC support Large ESIT payload Capability > 48k */
#define HCC2_LEC(p)                                             ((p) & (1 << 4))
/* true: HC support Configuration Information Capability */
#define HCC2_CIC(p)                                             ((p) & (1 << 5))
/* true: HC support Extended TBC Capability, Isoc burst count > 65535 */
#define HCC2_ETC(p)                                             ((p) & (1 << 6))

#define UX_XHCI_STATE_DYING                                     (1 << 0)
#define UX_XHCI_STATE_HALTED                                    (1 << 1)
#define UX_XHCI_STATE_REMOVING                                  (1 << 2)

#define UX_XHCI_RESET_EP_QUIRK                                  (1 << 1)
#define UX_XHCI_NEC_HOST                                        (1 << 2)

/* CRCR - Command Ring Control Register - cmd_ring bitmasks */
/* bit 0 is the command ring cycle state */
/* stop ring operation after completion of the currently executing command */
#define CMD_RING_PAUSE                                          (1 << 1)
/* stop ring immediately - abort the currently executing command */
#define CMD_RING_ABORT                                          (1 << 2)
/* true: command ring is running */
#define CMD_RING_RUNNING                                        (1 << 3)
/* bits 4:5 reserved and should be preserved */
/* Command Ring pointer - bit mask for the lower 32 bits. */
#define CMD_RING_RSVD_BITS                                      (0x3f)
#define CMD_RING_SEGS                                           1

/* CONFIG - Configure Register - config_reg bitmasks */
/* bits 0:7 - maximum number of device slots enabled (NumSlotsEn) */
#define MAX_DEVS(p)                                             ((p) & 0xff)
/* bit 8: U3 Entry Enabled, assert PLC when root port enters U3, xhci 1.1 */
#define CONFIG_U3E                                              (1 << 8)
/* bit 9: Configuration Information Enable, xhci 1.1 */
#define CONFIG_CIE                                              (1 << 9)
/* bits 10:31 - reserved and should be preserved */

/* PORTSC - Port Status and Control Register - port_status_base bitmasks */
/* true: device connected */
#define PORT_CONNECT                                            (1 << 0)
/* true: port enabled */
#define PORT_PE                                                 (1 << 1)
/* bit 2 reserved and zeroed */
/* true: port has an over-current condition */
#define PORT_OC                                                 (1 << 3)

/* true: port has power (see HCC_PPC) */
#define PORT_POWER                                              (1 << 9)
/* true: port reset signaling asserted */
#define PORT_RESET                                              (1 << 4)
/* Port Link State - bits 5:8
 * A read gives the current link PM state of the port,
 * a write with Link State Write Strobe set sets the link state.
 *
 */
/*
 * These bits are Read Only (RO) and should be saved and written to the
 * registers: 0, 3, 10:13, 30
 * connect status, over-current status, port speed, and device removable.
 * connect status and port speed are also sticky - meaning they're in
 * the AUX well and they aren't changed by a hot, warm, or cold reset.
 */
#define UX_XHCI_PORT_RO                                         ((1<<0) | (1<<3) | (0xf<<10) | (1<<30))
/*
 * These bits are RW; writing a 0 clears the bit, writing a 1 sets the bit:
 * bits 5:8, 9, 14:15, 25:27
 * link state, port power, port indicator state, "wake on" enable state
 */
#define UX_XHCI_PORT_RWS                                        ((0xf<<5) | (1<<9) | (0x3<<14) | (0x7<<25))
/*
 * These bits are RW; writing a 1 sets the bit, writing a 0 has no effect:
 * bit 4 (port reset)
 */
#define UX_XHCI_PORT_RW1S                                       ((1<<4))
/*
 * These bits are RW; writing a 1 clears the bit, writing a 0 has no effect:
 * bits 1, 17, 18, 19, 20, 21, 22, 23
 * port enable/disable, and
 * change bits: connect, PED, warm port reset changed (reserved zero for USB 2.0 ports),
 * over-current, reset, link state, and L1 change
 */
#define UX_XHCI_PORT_RW1CS                                      ((1<<1) | (0x7f<<17))
#define PORT_PLS_MASK                                           (0xf << 5)
#define XDEV_U0                                                 (0x0 << 5)
#define XDEV_U1                                                 (0x1 << 5)
#define XDEV_U2                                                 (0x2 << 5)
#define XDEV_U3                                                 (0x3 << 5)
#define XDEV_DISABLED                                           (0x4 << 5)
#define XDEV_RXDETECT                                           (0x5 << 5)
#define XDEV_INACTIVE                                           (0x6 << 5)
#define XDEV_POLLING                                            (0x7 << 5)
#define XDEV_RECOVERY                                           (0x8 << 5)
#define XDEV_HOT_RESET                                          (0x9 << 5)
#define XDEV_COMP_MODE                                          (0xa << 5)
#define XDEV_TEST_MODE                                          (0xb << 5)
#define XDEV_RESUME                                             (0xf << 5)


/* bits 10:13 indicate device speed:
 * 0 - undefined speed - port hasn't be initialized by a reset yet
 * 1 - full speed
 * 2 - low speed
 * 3 - high speed
 * 4 - super speed
 * 5-15 reserved
 */
#define DEV_SPEED_MASK                                          (0xf << 10)
#define XDEV_FS                                                 (0x1 << 10)
#define XDEV_LS                                                 (0x2 << 10)
#define XDEV_HS                                                 (0x3 << 10)
#define XDEV_SS                                                 (0x4 << 10)
#define XDEV_SSP                                                (0x5 << 10)
#define DEV_UNDEFSPEED(p)                                       (((p) & DEV_SPEED_MASK) == (0x0<<10))
#define DEV_FULLSPEED(p)                                        (((p) & DEV_SPEED_MASK) == XDEV_FS)
#define DEV_LOWSPEED(p)                                         (((p) & DEV_SPEED_MASK) == XDEV_LS)
#define DEV_HIGHSPEED(p)                                        (((p) & DEV_SPEED_MASK) == XDEV_HS)
#define DEV_SUPERSPEED(p)                                       (((p) & DEV_SPEED_MASK) == XDEV_SS)
#define DEV_SUPERSPEEDPLUS(p)                                   (((p) & DEV_SPEED_MASK) == XDEV_SSP)
#define DEV_SUPERSPEED_ANY(p)                                   (((p) & DEV_SPEED_MASK) >= XDEV_SS)
#define DEV_PORT_SPEED(p)                                       (((p) >> 10) & 0x0f)

/* Bits 20:23 in the Slot Context are the speed for the device */
#define SLOT_SPEED_FS                                           (XDEV_FS << 10)
#define SLOT_SPEED_LS                                           (XDEV_LS << 10)
#define SLOT_SPEED_HS                                           (XDEV_HS << 10)

/* Port Link State Write Strobe - set this when changing link state */
#define PORT_LINK_STROBE                                        (1 << 16)
/* true: connect status change */
#define PORT_CSC                                                (1 << 17)
/* true: port enable change */
#define PORT_PEC                                                (1 << 18)
/* true: warm reset for a USB 3.0 device is done.  A "hot" reset puts the port
 * into an enabled state, and the device into the default state.  A "warm" reset
 * also resets the link, forcing the device through the link training sequence.
 * SW can also look at the Port Reset register to see when warm reset is done.
 */
#define PORT_WRC                                                (1 << 19)
/* true: over-current change */
#define PORT_OCC                                                (1 << 20)
/* true: reset change - 1 to 0 transition of PORT_RESET */
#define PORT_RC                                                 (1 << 21)
/* port link status change - set on some port link state transitions:
 *  Transition                    Reason
 *  ------------------------------------------------------------------------------
 *  - U3 to Resume               Wakeup signaling from a device
 *  - Resume to Recovery to U0     USB 3.0 device resume
 *  - Resume to U0               USB 2.0 device resume
 *  - U3 to Recovery to U0       Software resume of USB 3.0 device complete
 *  - U3 to U0                   Software resume of USB 2.0 device complete
 *  - U2 to U0                   L1 resume of USB 2.1 device complete
 *  - U0 to U0 (???)             L1 entry rejection by USB 2.1 device
 *  - U0 to disabled             L1 entry error with USB 2.1 device
 *  - Any state to inactive       Error on USB 3.0 port
 */
#define PORT_PLC                                                (1 << 22)
/* port configure error change - port failed to configure its link partner */
#define PORT_CEC                                                (1 << 23)
#define PORT_CHANGE_MASK                                        (PORT_CSC | PORT_PEC | PORT_WRC | PORT_OCC | \
                                                                    PORT_RC | PORT_PLC | PORT_CEC)


/* Cold Attach Status - xHC can set this bit to report device attached during
 * Sx state. Warm port reset should be perfomed to clear this bit and move port
 * to connected state.
 */
#define PORT_CAS                                                (1 << 24)
/* wake on connect (enable) */
#define PORT_WKCONN_E                                           (1 << 25)
/* wake on disconnect (enable) */
#define PORT_WKDISC_E                                           (1 << 26)
/* wake on over-current (enable) */
#define PORT_WKOC_E                                             (1 << 27)
/* bits 28:29 reserved */
/* true: device is non-removable - for USB 3.0 roothub emulation */
#define PORT_DEV_REMOVE                                         (1 << 30)
/* Initiate a warm port reset - complete when PORT_WRC is '1' */
#define PORT_WR                                                 (1 << 31)

/* We mark duplicate entries with -1 */
#define DUPLICATE_ENTRY                                         ((uint8_t)(-1))

/* Port Power Management Status and Control - port_power_base bitmasks */
/* Inactivity timer value for transitions into U1, in microseconds.
 * Timeout can be up to 127us.  0xFF means an infinite timeout.
 */
#define PORT_U1_TIMEOUT(p)                                       ((p) & 0xff)
#define PORT_U1_TIMEOUT_MASK                                     0xff
/* Inactivity timer value for transitions into U2 */
#define PORT_U2_TIMEOUT(p)                                      (((p) & 0xff) << 8)
#define PORT_U2_TIMEOUT_MASK                                    (0xff << 8)
/* Bits 24:31 for port testing */

/* USB2 Protocol PORTSPMSC */
#define PORT_L1S_MASK                                           7
#define PORT_L1S_SUCCESS                                        1
#define PORT_RWE                                                (1 << 3)
#define PORT_HIRD(p)                                            (((p) & 0xf) << 4)
#define PORT_HIRD_MASK                                          (0xf << 4)
#define PORT_L1DS_MASK                                          (0xff << 8)
#define PORT_L1DS(p)                                            (((p) & 0xff) << 8)
#define PORT_HLE                                                (1 << 16)
#define PORT_TEST_MODE_SHIFT                                    28

/* USB2 Protocol PORTHLPMC */
#define PORT_HIRDM(p)                                           ((p) & 3)
#define PORT_L1_TIMEOUT(p)                                      (((p) & 0xff) << 2)
#define PORT_BESLD(p)                                           (((p) & 0xf) << 10)

/* use 512 microseconds as USB2 LPM L1 default timeout. */
#define UX_XHCI_L1_TIMEOUT                                      512

/* Set default HIRD/BESL value to 4 (350/400us) for USB2 L1 LPM resume latency.
 * Safe to use with mixed HIRD and BESL systems (host and device) and is used
 * by other operating systems.
 *
 * XHCI 1.0 errata 8/14/12 Table 13 notes:
 * "Software should choose xHC BESL/BESLD field values that do not violate a
 * device's resume latency requirements,
 * e.g. not program values > '4' if BLC = '1' and a HIRD device is attached,
 * or not program values < '4' if BLC = '0' and a BESL device is attached.
 */
#define UX_XHCI_DEFAULT_BESL                                    4

/*
 * USB3 specification define a 360ms tPollingLFPSTiemout for USB3 ports
 * to complete link training. usually link trainig completes much faster
 * so check status 10 times with 36ms sleep in places we need to wait for
 * polling to complete.
 */
#define UX_XHCI_PORT_POLLING_LFPS_TIME                          36

#define TT_HS_OVERHEAD                                          (31 + 94)
#define TT_DMI_OVERHEAD                                         (25 + 12)

/* irq_pending bitmasks */
#define ER_IRQ_PENDING(p)                                       ((p) & 0x1)
/* bits 2:31 need to be preserved */
#define ER_IRQ_CLEAR(p)                                         ((p) & 0xfffffffe)
#define ER_IRQ_ENABLE(p)                                        ((ER_IRQ_CLEAR(p)) | 0x2)
#define ER_IRQ_DISABLE(p)                                       ((ER_IRQ_CLEAR(p)) & ~(0x2))

/* irq_control bitmasks */
/* Minimum interval between interrupts (in 250ns intervals).  The interval
 * between interrupts will be longer if there are no events on the event ring.
 * Default is 4000 (1 ms).
 */
#define ER_IRQ_INTERVAL_MASK                                    (0xffff)
/* Counter used to count down the time to the next interrupt - HW use only */
#define ER_IRQ_COUNTER_MASK                                     (0xffff << 16)

/* erst_size bitmasks */
/* Preserve bits 16:31 of erst_size */
#define ERST_SIZE_MASK                                          (0xffff << 16)

/* erst_dequeue bitmasks */
/* Dequeue ERST Segment Index (DESI) - Segment number (or alias)
 * where the current dequeue pointer lies.  This is an optional HW hint.
 */
#define ERST_DESI_MASK                                          (0x7)
/* Event Handler Busy (EHB) - is the event ring scheduled to be serviced by
 * a work queue (or delayed service routine)?
 */
#define ERST_EHB                                                (1 << 3)
#define ERST_PTR_MASK                                           (0xf)

/* TRB bit mask */
#define TRB_TYPE_BITMASK                                        (0xfc00)
#define TRB_TYPE(p)                                             ((p) << 10)
#define TRB_FIELD_TO_TYPE(p)                                    (((p) & TRB_TYPE_BITMASK) >> 10)
/* TRB type IDs */
/* bulk, interrupt, isoc scatter/gather, and control data stage */
#define TRB_NORMAL                                              1
/* setup stage for control transfers */
#define TRB_SETUP                                               2
/* data stage for control transfers */
#define TRB_DATA                                                3
/* status stage for control transfers */
#define TRB_STATUS_                                             4
/* isoc transfers */
#define TRB_ISOC                                                5
/* TRB for linking ring segments */
#define TRB_LINK                                                6
#define TRB_EVENT_DATA                                          7
/* Transfer Ring No-op (not for the command ring) */
#define TRB_TR_NOOP                                             8
/* Command TRBs */
/* Enable Slot Command */
#define TRB_ENABLE_SLOT                                         9
/* Disable Slot Command */
#define TRB_DISABLE_SLOT                                        10
/* Address Device Command */
#define TRB_ADDR_DEV                                            11
/* Configure Endpoint Command */
#define TRB_CONFIG_EP                                           12
/* Evaluate Context Command */
#define TRB_EVAL_CONTEXT                                        13
/* Reset Endpoint Command */
#define TRB_RESET_EP                                            14
/* Stop Transfer Ring Command */
#define TRB_STOP_RING                                           15
/* Set Transfer Ring Dequeue Pointer Command */
#define TRB_SET_DEQ                                             16
/* Reset Device Command */
#define TRB_RESET_DEV                                           17
/* Force Event Command (opt) */
#define TRB_FORCE_EVENT                                         18
/* Negotiate Bandwidth Command (opt) */
#define TRB_NEG_BANDWIDTH                                       19
/* Set Latency Tolerance Value Command (opt) */
#define TRB_SET_LT                                              20
/* Get port bandwidth Command */
#define TRB_GET_BW                                              21
/* Force Header Command - generate a transaction or link management packet */
#define TRB_FORCE_HEADER                                        22
/* No-op Command - not for transfer rings */
#define TRB_CMD_NOOP                                            23
/* TRB IDs 24-31 reserved */
/* Event TRBS */
/* Transfer Event */
#define TRB_TRANSFER                                            32
/* Command Completion Event */
#define TRB_COMPLETION                                          33
/* Port Status Change Event */
#define TRB_PORT_STATUS                                         34
/* Bandwidth Request Event (opt) */
#define TRB_BANDWIDTH_EVENT                                     35
/* Doorbell Event (opt) */
#define TRB_DOORBELL                                            36
/* Host Controller Event */
#define TRB_HC_EVENT                                            37
/* Device Notification Event - device sent function wake notification */
#define TRB_DEV_NOTE                                            38
/* MFINDEX Wrap Event - microframe counter wrapped */
#define TRB_MFINDEX_WRAP                                        39
/* TRB IDs 40-47 reserved, 48-63 is vendor-defined */

/* Nec vendor-specific command completion event. */
#define TRB_NEC_CMD_COMP                                        48
/* Get NEC firmware revision. */
#define TRB_NEC_GET_FW                                          49

#define TRB_TYPE_LINK(x)                                        (((x) & TRB_TYPE_BITMASK) == TRB_TYPE(TRB_LINK))
/* Above, but for unsigned long types -- can avoid work by swapping constants: */
#define TRB_TYPE_LINK_LE32(x)                                   (((x) & (TRB_TYPE_BITMASK)) == \
                                                                    (TRB_TYPE(TRB_LINK)))
#define TRB_TYPE_NOOP_LE32(x)                                   (((x) & (TRB_TYPE_BITMASK)) == \
                                                                    (TRB_TYPE(TRB_TR_NOOP)))

/* Port Status Change Event TRB fields */
/* Port ID - bits 31:24 */
#define GET_PORT_ID(p)                                          (((p) & (0xff << 24)) >> 24)

#define EVENT_DATA                                              (1 << 2)

#define min(a, b)                                               (a < b ? a : b)

#define EXDEV                                                   18/* Cross-device link */
#define ECOMM                                                   109/* Communication error on send */
#define EOVERFLOW                                               112  /* Value too large for defined data type */
#define EPROTO                                                  85/* Protocol error */

#define XHCI_TRUST_TX_LENGTH                                    BIT_ULL(10)

/* Normal TRB fields */
/* transfer_len bitmasks - bits 0:16 */
#define TRB_LEN(p)                                              ((p) & 0x1ffff)
/* TD Size, packets remaining in this TD, bits 21:17 (5 bits, so max 31) */
#define TRB_TD_SIZE(p)                                          (min((p), (uint32_t)31) << 17)
#define GET_TD_SIZE(p)                                          (((p) & 0x3e0000) >> 17)
/* xhci 1.1 uses the TD_SIZE field for TBC if Extended TBC is enabled (ETE) */
#define TRB_TD_SIZE_TBC(p)                                      (min((p), (uint32_t)31) << 17)
/* Interrupter Target - which MSI-X vector to target the completion event at */
#define TRB_INTR_TARGET(p)                                      (((p) & 0x3ff) << 22)
#define GET_INTR_TARGET(p)                                      (((p) >> 22) & 0x3ff)
/* Total burst count field, Rsvdz on xhci 1.1 with Extended TBC enabled (ETE) */
#define TRB_TBC(p)                                              (((p) & 0x3) << 7)
#define TRB_TLBPC(p)                                            (((p) & 0xf) << 16)

/* Cycle bit - indicates TRB ownership by HC or HCD */
#define TRB_CYCLE                                               (1<<0)

#define XHCI_AVOID_BEI                                          BIT_ULL(15)
/*
 * Force next event data TRB to be evaluated before task switch.
 * Used to pass OS data back after a TD completes.
 */
#define TRB_ENT                                                 (1<<1)
/* Interrupt on short packet */
#define TRB_ISP                                                 (1<<2)
/* Set PCIe no snoop attribute */
#define TRB_NO_SNOOP                                            (1<<3)
/* Chain multiple TRBs into a TD */
#define TRB_CHAIN                                               (1<<4)
/* Interrupt on completion */
#define TRB_IOC                                                 (1<<5)
/* The buffer pointer contains immediate data */
#define TRB_IDT                                                 (1<<6)
/* TDs smaller than this might use IDT */
#define TRB_IDT_MAX_SIZE                                        8

/* Block Event Interrupt */
#define TRB_BEI                                                 (1<<9)

/* flags bitmasks */

/* Address device - disable SetAddress */
#define TRB_BSR                                                 (1<<9)

/* Configure Endpoint - Deconfigure */
#define TRB_DC                                                  (1<<9)

/* Stop Ring - Transfer State Preserve */
#define TRB_TSP                                                 (1<<9)


/* Control transfer TRB specific fields */
#define  TRB_DIR_IN                                             (1<<16)
#define  TRB_TX_TYPE(p)                                         ((p) << 16)
#define  TRB_DATA_OUT                                           2
#define  TRB_DATA_IN                                            3

/* Isochronous TRB specific fields */
#define TRB_SIA                                                 (1<<31)
#define TRB_FRAME_ID(p)                                         (((p) & 0x7ff) << 20)

#define CMD_RING_STATE_RUNNING                                  (1 << 0)
#define CMD_RING_STATE_ABORTED                                  (1 << 1)
#define CMD_RING_STATE_STOPPED                                  (1 << 2)

/*
 * Each segment table entry is 4*32bits long.  1K seems like an ok size:
 * (1K bytes * 8bytes/bit) / (4*32 bits) = 64 segment entries in the table,
 * meaning 64 ring segments.
 * Initial allocated size of the ERST, in number of entries */
#define ERST_NUM_SEGS                                           1
/* Initial allocated size of the ERST, in number of entries */
#define ERST_SIZE                                               64
/* Initial number of event segment rings allocated */
#define ERST_ENTRIES                                            1
/* Poll every 60 seconds */
#define POLL_TIMEOUT                                            60
/* Stop endpoint command timeout (secs) for URB cancellation watchdog timer */
#define UX_XHCI_STOP_EP_CMD_TIMEOUT                             5
#define UX_XHCI_MAX_INTERVAL                                    16

#define UX_XHCI_MAX_HALT_USEC                                   (16*1000)
/* HC not running - set to 1 when run/stop bit is cleared. */
#define UX_XHCI_STS_HALT                                        (1<<0)

/* HCCPARAMS offset from PCI base address */
#define UX_XHCI_HCC_PARAMS_OFFSET                               0x10
/* HCCPARAMS contains the first extended capability pointer */
#define XHCI_HCC_EXT_CAPS(p)                                    (((p)>>16) & 0xffff)

/* IMAN - Interrupt Management Register */
#define IMAN_IE                                                 (1 << 1)
#define IMAN_IP                                                 (1 << 0)

/* USBSTS - USB status - status bitmasks */
/* HC not running - set to 1 when run/stop bit is cleared. */
#define STS_HALT                                                UX_XHCI_STS_HALT
/* serious error, e.g. PCI parity error.  The HC will clear the run/stop bit. */
#define STS_FATAL                                               (1 << 2)
/* event interrupt - clear this prior to clearing any IP flags in IR set*/
#define STS_EINT                                                (1 << 3)
/* port change detect */
#define STS_PORT                                                (1 << 4)
/* bits 5:7 reserved and zeroed */
/* save state status - '1' means xHC is saving state */
#define STS_SAVE                                                (1 << 8)
/* restore state status - '1' means xHC is restoring state */
#define STS_RESTORE                                             (1 << 9)
/* true: save or restore error */
#define STS_SRE                                                 (1 << 10)
/* true: Controller Not Ready to accept doorbell or op reg writes after reset */
#define STS_CNR                                                 XHCI_STS_CNR
/* true: internal Host Controller Error - SW needs to reset and reinitialize */
#define STS_HCE                                                 (1 << 12)
/* bits 13:31 reserved and should be preserved */

/*
 * DNCTRL - Device Notification Control Register - dev_notification bitmasks
 * Generate a device notification event when the HC sees a transaction with a
 * notification type that matches a bit set in this bit field.
 */
#define DEV_NOTE_MASK                                           (0xffff)
#define ENABLE_DEV_NOTE(x)                                      (1 << (x))
/* Most of the device notification types should only be used for debug.
 * SW does need to pay attention to function wake notifications.
 */
#define DEV_NOTE_FWAKE                                          ENABLE_DEV_NOTE(1)

/* bits 16:23 are the virtual function ID */
/* bits 24:31 are the slot ID */
#define TRB_TO_SLOT_ID(p)                                       (((p) & (0xff<<24)) >> 24)
#define SLOT_ID_FOR_TRB(p)                                      (((p) & 0xff) << 24)

/* Stop Endpoint TRB - ep_index to endpoint ID for this TRB */
#define TRB_TO_EP_INDEX(p)                                      ((((p) & (0x1f << 16)) >> 16) - 1)
#define EP_ID_FOR_TRB(p)                                        ((((p) + 1) & 0x1f) << 16)

#define SUSPEND_PORT_FOR_TRB(p)                                 (((p) & 1) << 23)
#define TRB_TO_SUSPEND_PORT(p)                                  (((p) & (1 << 23)) >> 23)
#define LAST_EP_INDEX                                           30

/* Set TR Dequeue Pointer command TRB fields, 6.4.3.9 */
#define TRB_TO_STREAM_ID(p)                                     ((((p) & (0xffff << 16)) >> 16))
#define STREAM_ID_FOR_TRB(p)                                    ((((p)) & 0xffff) << 16)
#define SCT_FOR_TRB(p)                                          (((p) << 1) & 0x7)

/* Link TRB specific fields */
#define TRB_TC                                                  (1<<1)

/* dev_info bitmasks */
/* Device speed - values defined by PORTSC Device Speed field - 20:23 */
#define DEV_SPEED                                               (0xf << 20)
#define GET_DEV_SPEED(n)                                        (((n) & DEV_SPEED) >> 20)
/* bit 24 reserved */
/* Is this LS/FS device connected through a HS hub? - bit 25 */
#define DEV_MTT                                                 (0x1 << 25)
/* Set if the device is a hub - bit 26 */
#define DEV_HUB                                                 (0x1 << 26)
/* Index of the last valid endpoint context in this device context - 27:31 */
#define LAST_CTX_MASK                                           (0x1f << 27)
#define LAST_CTX(p)                                             ((p) << 27)
#define LAST_CTX_TO_EP_NUM(p)                                   (((p) >> 27) - 1)
#define SLOT_FLAG                                               (1 << 0)
#define EP0_FLAG                                                (1 << 1)

/* dev_info2 bitmasks */
/* Max Exit Latency (ms) - worst case time to wake up all links in dev path */
#define MAX_EXIT                                                (0xffff)
/* Root hub port number that is needed to access the USB device */
#define ROOT_HUB_PORT(p)                                        (((p) & 0xff) << 16)
#define DEVINFO_TO_ROOT_HUB_PORT(p)                             (((p) >> 16) & 0xff)
/* Maximum number of ports under a hub device */
#define UX_XHCI_MAX_PORTS(p)                                    (((p) & 0xff) << 24)
#define DEVINFO_TO_MAX_PORTS(p)                                 (((p) & (0xff << 24)) >> 24)

/* tt_info bitmasks */
/*
 * TT Hub Slot ID - for low or full speed devices attached to a high-speed hub
 * The Slot ID of the hub that isolates the high speed signaling from
 * this low or full-speed device.  '0' if attached to root hub port.
 */
#define TT_SLOT                                                 (0xff)

/* dev_state bitmasks */
/* USB device address - assigned by the HC */
#define DEV_ADDR_MASK                                           (0xff)
/* bits 8:26 reserved */
/* Slot state */
#define SLOT_STATE                                              (0x1f << 27)
#define GET_SLOT_STATE(p)                                       (((p) & (0x1f << 27)) >> 27)

#define SLOT_STATE_DISABLED                                     0
#define SLOT_STATE_ENABLED                                      SLOT_STATE_DISABLED
#define SLOT_STATE_DEFAULT                                      1
#define SLOT_STATE_ADDRESSED                                    2
#define SLOT_STATE_CONFIGURED                                   3

/* ep_info bitmasks */
/*
 * Endpoint State - bits 0:2
 * 0 - disabled
 * 1 - running
 * 2 - halted due to halt condition - ok to manipulate endpoint ring
 * 3 - stopped
 * 4 - TRB error
 * 5-7 - reserved
 */
#define EP_STATE_MASK                                           (0xf)
#define EP_STATE_DISABLED                                       0
#define EP_STATE_RUNNING                                        1
#define EP_STATE_HALTED                                         2
#define EP_STATE_STOPPED                                        3
#define EP_STATE_ERROR                                          4
#define GET_EP_CTX_STATE(ctx)                                   (((ctx)->ep_info) & EP_STATE_MASK)

/* Mult - Max number of burtst within an interval, in EP companion desc. */
#define EP_MULT(p)                                              (((p) & 0x3) << 8)
#define CTX_TO_EP_MULT(p)                                       (((p) >> 8) & 0x3)
/* bits 10:14 are Max Primary Streams */
/* bit 15 is Linear Stream Array */
/* Interval - period between requests to an endpoint - 125u increments. */
#define EP_INTERVAL(p)                                          (((p) & 0xff) << 16)
#define EP_INTERVAL_TO_UFRAMES(p)                               (1 << (((p) >> 16) & 0xff))
#define CTX_TO_EP_INTERVAL(p)                                   (((p) >> 16) & 0xff)
#define EP_MAXPSTREAMS_MASK                                     (0x1f << 10)
#define EP_MAXPSTREAMS(p)                                       (((p) << 10) & EP_MAXPSTREAMS_MASK)
#define CTX_TO_EP_MAXPSTREAMS(p)                                (((p) & EP_MAXPSTREAMS_MASK) >> 10)
/* Endpoint is set up with a Linear Stream Array (vs. Secondary Stream Array) */

/* hosts with LEC=1 use bits 31:24 as ESIT high bits. */
#define CTX_TO_MAX_ESIT_PAYLOAD_HI(p)                           ((p) >> 24) & 0xff)

/* ep_info2 bitmasks */
/*
 * Force Event - generate transfer events for all TRBs for this endpoint
 * This will tell the HC to ignore the IOC and ISP flags (for debugging only).
 */
#define FORCE_EVENT                                             (0x1)
#define ERROR_COUNT(p)                                          (((p) & 0x3) << 1)
#define CTX_TO_EP_TYPE(p)                                       (((p) >> 3) & 0x7)
#define EP_TYPE(p)                                              ((p) << 3)
#define ISOC_OUT_EP                                             1
#define BULK_OUT_EP                                             2
#define INT_OUT_EP                                              3
#define CTRL_EP                                                 4
#define ISOC_IN_EP                                              5
#define BULK_IN_EP                                              6
#define INT_IN_EP                                               7
/* bit 6 reserved */
/* bit 7 is Host Initiate Disable - for disabling stream selection */
#define MAX_BURST(p)                                            (((p)& 0xff) << 8)
#define CTX_TO_MAX_BURST(p)                                     (((p) >> 8) & 0xff)
#define MAX_PACKET(p)                                           (((p)& 0xffff) << 16)
#define MAX_PACKET_MASK                                         (0xffff << 16)
#define MAX_PACKET_DECODED(p)                                   (((p) >> 16) & 0xffff)

/* tx_info bitmasks */
#define EP_AVG_TRB_LENGTH(p)                                    ((p) & 0xffff)
#define EP_MAX_ESIT_PAYLOAD_LO(p)                               (((p) & 0xffff) << 16)
#define EP_MAX_ESIT_PAYLOAD_HI(p)                               ((((p) >> 16) & 0xff) << 24)
#define CTX_TO_MAX_ESIT_PAYLOAD(p)                              (((p) >> 16) & 0xffff)

/* deq bitmasks */
#define EP_CTX_CYCLE_MASK                                       (1 << 0)
#define SCTX_DEQ_MASK                                           (~0xfL)

#define DIV_ROUND_UP(n,d)                                       (((n) + (d) - 1) / (d))

/* Primary stream array type, dequeue pointer is to a transfer ring */
#define SCT_PRI_TR                                              1

#define UX_XHCI_EXT_PORT_MAJOR(x)                               (((x) >> 24) & 0xff)
#define UX_XHCI_EXT_PORT_MINOR(x)                               (((x) >> 16) & 0xff)
#define UX_XHCI_EXT_PORT_PSIC(x)                                (((x) >> 28) & 0x0f)
#define UX_XHCI_EXT_PORT_OFF(x)                                 ((x) & 0xff)
#define UX_XHCI_EXT_PORT_COUNT(x)                               (((x) >> 8) & 0xff)

#define UX_XHCI_EXT_PORT_PSIV(x)                                (((x) >> 0) & 0x0f)
#define UX_XHCI_EXT_PORT_PSIE(x)                                (((x) >> 4) & 0x03)
#define UX_XHCI_EXT_PORT_PLT(x)                                 (((x) >> 6) & 0x03)
#define UX_XHCI_EXT_PORT_PFD(x)                                 (((x) >> 8) & 0x01)
#define UX_XHCI_EXT_PORT_LP(x)                                  (((x) >> 14) & 0x03)
#define UX_XHCI_EXT_PORT_PSIM(x)                                (((x) >> 16) & 0xffff)

#define USB_MAXCHILDREN                                         31
#define XHCI_CFC_DELAY                                          10

#define URB_ISO_ASAP                                            0x0002 /* ISO-only */

#define ROUNDUP(x, y)                                           (((((x) + (uint32_t)(y - 1U)) / (uint32_t)y) * (uint32_t)y))
#define rounddown(x,y)                                          (((x) - ((x) %(y))))

/**
 * upper_32_bits - return bits 32-63 of a number
 * @n: the number we're accessing
 *
 * A basic shift-right of a 64- or 32-bit quantity.  Use this to suppress
 * the "right shift count >= width of type" warning when that quantity is
 * 32-bits.
 */
//#define upper_32_bits(n)                                        ((uint32_t)(((n) >> 16) >> 16))

/**
 * lower_32_bits - return bits 0-31 of a number
 * @n: the number we're accessing
 */
//#define lower_32_bits(n)                                        ((uint32_t)((n) & 0xffffffff))

typedef enum UX_XHCI_EP_RESET_TYPE_E
{
    EP_HARD_RESET,
    EP_SOFT_RESET,
}UX_XHCI_EP_RESET_TYPE;

typedef enum UX_XHCI_SETUP_DEV_E
{
   SETUP_CONTEXT_ONLY,
   SETUP_CONTEXT_ADDRESS,
}UX_XHCI_SETUP_DEV;

typedef enum UX_XHCI_OVERHEAD_TYPE_ENUM
{
   LS_OVERHEAD_TYPE = 0,
   FS_OVERHEAD_TYPE,
   HS_OVERHEAD_TYPE,
}UX_XHCI_OVERHEAD_TYPE;

typedef enum UX_XHCI_RING_TYPE_ENUM
{
   TYPE_CTRL = 0,
   TYPE_ISOC,
   TYPE_BULK,
   TYPE_INTR,
   TYPE_STREAM,
   TYPE_COMMAND,
   TYPE_EVENT,
}UX_XHCI_RING_TYPE;

typedef struct UX_XHCI_OP_REGS_STRUCT
{
   union {
       volatile uint32_t            USBCMD;                 /* ! USB COMMAND Register (USBCMD) */
       struct {
           volatile uint32_t        R_S     : 1;
           volatile uint32_t        HCRST   : 1;
           volatile uint32_t        INTE    : 1;
           volatile uint32_t        HSEE    : 1;
           volatile const uint32_t : 3;
           volatile uint32_t        LHCRST  : 1;
           volatile uint32_t        CSS     : 1;
           volatile uint32_t        CRS     : 1;
           volatile uint32_t        EWE     : 1;
           volatile uint32_t        EU3S    : 1;
           volatile const uint32_t : 1;
           volatile uint32_t        CME     : 1;
           volatile uint32_t        ETE     : 1;
           volatile uint32_t        TSC_EN  : 1;
           volatile uint32_t        VTIOE   : 1;
            volatile const uint32_t : 15;
       } USBCMD_b;
   };
   union {
    volatile uint32_t            USBSTS;                  /* ! USB STATUS Register(USBSTS)   */
    struct {
           volatile const uint32_t  HCH : 1;
           volatile const uint32_t : 1;
           volatile uint32_t        HSE : 1;
           volatile uint32_t        EINT: 1;
           volatile uint32_t        PCD : 1;
           volatile const uint32_t : 3;
           volatile const uint32_t  SSS : 1;
           volatile const uint32_t  RSS : 1;
           volatile uint32_t        SRE : 1;
           volatile const uint32_t  CNR : 1;
           volatile const uint32_t  HCE : 1;
           volatile const uint32_t : 19;
      } USBSTS_b;
   };

   volatile const uint32_t      PAGESIZE;               /* ! USB PAGE SIZE register (PAGESIZE)  */
   volatile const uint32_t      RESERVED1;
   volatile const uint32_t      RESERVED2;
   volatile uint32_t            DNCTRL;        /* ! Device Notification Control Register (DNCTRL) */
   volatile uint64_t            CRCR;                /* ! Command Ring Control Register (CRCR)  */
   volatile uint32_t            RESERVED3[4];            /* rsvd: offset 0x20-2F */
   volatile uint64_t            DCBAAP;               /* ! Device Context Base Address Array Pointer Register (DCBAAP)  */
   volatile uint32_t            CONFIG;              /* ! Configure Register (CONFIG)  */
   volatile const uint32_t      RESERVED4[241];          /* rsvd: offset 0x3C-3FF */
   /* port 1 registers, which serve as a base address for other ports */
   volatile uint32_t            PORTSC;        /* !Port Status and Control Register (PORTSC) */
   volatile uint32_t            PORTPMSC;         /* ! Port PM Status and Control Register (PORTPMSC) */
   volatile uint32_t            PORTLI;          /* ! Port Link Info Register (PORTLI)  */
   volatile uint32_t            RESERVED5;
   volatile const uint32_t      RESERVED6[NUM_PORT_REGS*254];     /* registers for ports 2-255 */
}UX_XHCI_OP_REGS;

typedef struct UX_XHCI_INTR_REGS_STRUCT
{
   volatile uint32_t    IMAN;
   volatile uint32_t    IMOD;          /* ! Interrupter Moderation Register (IMOD)   */
   volatile uint32_t    ERSTSZ;     /* ! Event Ring Segment Table Size Register (ERSTSZ)  */
   volatile uint32_t    rsvd;
   volatile uint64_t    ERSTBA;    /* !  Event Ring Segment Table Base Address Register (ERSTBA)  */
   volatile uint64_t    ERDP;          /* ! Event Ring Dequeue Pointer Register (ERDP)   */
}UX_XHCI_INTR_REGS;

typedef struct UX_XHCI_RUN_REGS_STRUCT
{
   volatile uint32_t              MFINDEX;  /* ! Microframe Index Register (MFINDEX) */
   uint32_t                       rsvd[7];
   volatile UX_XHCI_INTR_REGS     ir_set[128];   /* !Interrupter Register Set  */
}UX_XHCI_RUN_REGS;

typedef struct UX_XHCI_DB_REGS_STRUCT
{
   volatile uint32_t   DOORBELL[256];            /* ! 256 Doorbell Registers     */
}UX_XHCI_DB_REGS;

typedef struct UX_XHCI_LINK_TRB_STRUCT
{
   /* 64-bit segment pointer*/
   uint64_t segment_ptr;
   uint32_t intr_target;
   uint32_t control;
}UX_XHCI_LINK_TRB;

typedef struct UX_XHCI_TRANSFER_EVENT_STRUCT
{
   /* 64-bit buffer address, or immediate data */
   uint64_t              buffer;
   uint32_t              transfer_len;
   /* This field is interpreted differently based on the type of TRB */
   uint32_t              flags;
}UX_XHCI_TRANSFER_EVENT;

/* Command completion event TRB */
typedef struct UX_XHCI_EVENT_CMD_STRUCT
{
   /* Pointer to command TRB, or the value passed by the event data trb */
   uint64_t cmd_trb;
   uint32_t status;
   uint32_t flags;
}UX_XHCI_EVENT_CMD;

typedef struct UX_XHCI_GENERIC_TRB_STRUCT
{
   uint32_t field[4];
}UX_XHCI_GENERIC_TRB;

typedef struct UX_XHCI_CONTAINER_CTX_STRUCT
{
   unsigned type;
   #define UX_XHCI_CTX_TYPE_DEVICE  0x1
   #define UX_XHCI_CTX_TYPE_INPUT   0x2

   int32_t size;

   uint8_t *bytes;
   uint64_t  dma;
}UX_XHCI_CONTAINER_CTX;

typedef struct UX_XHCI_EP_CTX_STRUCT
{
   uint32_t         ep_info;
   uint32_t         ep_info2;
   uint64_t         deq;
   uint32_t         tx_info;
   /* offset 0x14 - 0x1f reserved for HC internal use */
   uint32_t         reserved[3];
}UX_XHCI_EP_CTX;

typedef union UX_XHCI_TRB_UNION
{
   UX_XHCI_LINK_TRB          link;
   UX_XHCI_TRANSFER_EVENT    trans_event;
   UX_XHCI_EVENT_CMD         event_cmd;
   UX_XHCI_GENERIC_TRB       generic;
}UX_XHCI_TRB;

typedef struct UX_XHCI_COMMAND_STRUCT
{
   /* Input context for changing device state */
   UX_XHCI_CONTAINER_CTX     *in_ctx;
   volatile uint32_t         status;
   int32_t                   slot_id;
   /* If completion is zero, no one is waiting on this command
    * and the structure can be freed after the command completes.
    */
   volatile uint32_t         fCompletion;
   UX_XHCI_TRB               *command_trb;
   UX_HCD_XHCI_LIST          cmd_list;
}UX_XHCI_COMMAND;

typedef struct UX_XHCI_SEGMENT_STRUCT
{
   UX_XHCI_TRB                     *trbs;
   /* private to HCD */
   struct UX_XHCI_SEGMENT_STRUCT   *next;
   uint64_t                        dma;
   /* Max packet sized bounce buffer for td-fragmant alignment */
   uint64_t                        bounce_dma;
   void                            *bounce_buf;
   uint32_t                        bounce_offs;
   uint32_t                        bounce_len;
}UX_XHCI_SEGMENT;

typedef struct UX_XHCI_RING_STRUCT
{
   UX_XHCI_SEGMENT     *first_seg;
   UX_XHCI_SEGMENT     *last_seg;
   UX_XHCI_TRB         *enqueue;
   UX_XHCI_SEGMENT     *enq_seg;
   UX_XHCI_TRB         *dequeue;
   UX_XHCI_SEGMENT     *deq_seg;
   UX_HCD_XHCI_LIST    td_list;
   /*
    * Write the cycle state into the TRB cycle field to give ownership of
    * the TRB to the host controller (if we are the producer), or to check
    * if we own the TRB (if we are the consumer).  See section 4.9.1.
    */
   uint32_t            cycle_state;
   uint32_t            err_count;
   uint32_t            stream_id;
   uint32_t            num_segs;
   uint32_t            num_trbs_free;
   uint32_t            num_trbs_free_temp;
   uint32_t            bounce_buf_len;
   UX_XHCI_RING_TYPE   type;
   bool                last_td_was_short;
}UX_XHCI_RING;

typedef struct UX_XHCI_ERST_ENTRY_STRUCT
{
   /* 64-bit event ring segment address */
   uint64_t    seg_addr;
   uint32_t    seg_size;
   /* Set to zero */
   uint32_t    rsvd;
}UX_XHCI_ERST_ENTRY;

typedef struct UX_XHCI_ERST_STRUCT
{
   UX_XHCI_ERST_ENTRY   *entries;
   uint32_t             num_entries;
   uint64_t             erst_dma_addr;
   /* Num entries the ERST can contain */
   uint32_t             erst_size;
}UX_XHCI_ERST;

typedef struct UX_XHCI_SCRATCHPAD_S{
   uint64_t     *sp_array;
   uint64_t     *sp_dma;
   void         **sp_buffers;
}UX_XHCI_SCRATCHPAD;

typedef struct UX_XHCI_TD_STRUCT
{
   UX_HCD_XHCI_LIST    td_list;
   UX_HCD_XHCI_LIST    cancelled_td_list;
   UX_TRANSFER         *urb;
   UX_XHCI_SEGMENT     *start_seg;
   UX_XHCI_TRB         *first_trb;
   UX_XHCI_TRB         *last_trb;
   UX_XHCI_SEGMENT     *bounce_seg;
   /* actual_length of the URB has already been set */
   bool                urb_length_set;
}UX_XHCI_TD;

typedef struct UX_URB_PRIV_S {
   int32_t      num_tds;
   int32_t      num_tds_done;
   UX_XHCI_TD   td[];
}UX_URB_PRIV;

typedef struct UX_XHCI_DEVICE_CONTEXT_ARRAY_STRUCT
{
   /* 64-bit device addresses; we only write 32-bit addresses */
   uint64_t   dev_context_ptrs[UX_XHCI_MAX_HC_SLOTS];
   uint64_t   dma;
}UX_XHCI_DEVICE_CONTEXT_ARRAY;

typedef struct UX_XHCI_INTERVAL_BW_S
{
   uint32_t          num_packets;
   /* Sorted by max packet size.
    * Head of the list is the greatest max packet size.
    */
   UX_HCD_XHCI_LIST  endpoints;
   /* How many endpoints of each speed are present. */
   uint32_t          overhead[3];
}UX_XHCI_INTERVAL_BW;

typedef struct UX_XHCI_INTERVAL_BW_TABLE_STRUCT
{
   uint32_t              interval0_esit_payload;
   UX_XHCI_INTERVAL_BW   interval_bw[UX_XHCI_MAX_INTERVAL];
   /* Includes reserved bandwidth for async endpoints */
   uint32_t              bw_used;
}UX_XHCI_INTERVAL_BW_TABLE;

typedef struct UX_XHCI_STREAM_CTX_STRUCT
{
   /* 64-bit stream ring address, cycle state, and stream type */
   uint64_t    stream_ring;
   /* offset 0x14 - 0x1f reserved for HC internal use */
   uint32_t    reserved[2];
}UX_XHCI_STREAM_CTX;

/* Assume no secondary streams for now */
typedef struct UX_XHCI_STREAM_INFO_STRUCT
{
   UX_XHCI_RING          **stream_rings;
   /* Number of streams, including stream 0 (which drivers can't use) */
   uint32_t              num_streams;
   /* The stream context array may be bigger than
   * the number of streams the driver asked for
   */
   UX_XHCI_STREAM_CTX    *stream_ctx_array;
   uint32_t              num_stream_ctxs;
   uint64_t              ctx_array_dma;
   /* For mapping physical TRB addresses to segments in stream rings */
   UX_XHCI_COMMAND       *free_streams_command;
}UX_XHCI_STREAM_INFO;

typedef struct UX_XHCI_BW_INFO_STRUCT
{
   /* ep_interval is zero-based */
   uint32_t       ep_interval;
   /* mult and num_packets are one-based */
   uint32_t       mult;
   uint32_t       num_packets;
   uint32_t       max_packet_size;
   uint32_t       max_esit_payload;
   uint32_t       type;
}UX_XHCI_BW_INFO;

typedef struct UX_XHCI_ROOT_PORT_BW_INFO_STRUCT
{
   UX_HCD_XHCI_LIST          tts;
   uint32_t                  num_active_tts;
   UX_XHCI_INTERVAL_BW_TABLE bw_table;
}UX_XHCI_ROOT_PORT_BW_INFO;

typedef struct UX_XHCI_TT_BW_INFO_STRUCT
{
   UX_HCD_XHCI_LIST        tt_list;
   uint32_t                slot_id;
   int32_t                 ttport;
   UX_XHCI_INTERVAL_BW_TABLE   bw_table;
   int32_t                 active_eps;
}UX_XHCI_TT_BW_INFO;


typedef struct UX_XHCI_VIRT_EP_STRUCT
{
   UX_XHCI_RING           *ring;
   /* Related to endpoints that are configured to use stream IDs only */
   UX_XHCI_STREAM_INFO     *stream_info;
   /* Temporary storage in case the configure endpoint command fails and we
    * have to restore the device state to the previous state
    */
   UX_XHCI_RING           *new_ring;
   uint32_t               ep_state;
   /* ----  Related to URB cancellation ---- */
   UX_HCD_XHCI_LIST       cancelled_td_list;

   /* Dequeue pointer and dequeue segment for a submitted Set TR Dequeue
    * command.  We'll need to update the ring's dequeue segment and dequeue
    * pointer after the command completes.
    */
   UX_XHCI_SEGMENT       *queued_deq_seg;
   UX_XHCI_TRB           *queued_deq_ptr;
   /*
    * Sometimes the xHC can not process isochronous endpoint ring quickly
    * enough, and it will miss some isoc tds on the ring and generate
    * a Missed Service Error Event.
    * Set skip flag when receive a Missed Service Error Event and
    * process the missed tds on the endpoint ring.
    */
   bool                  skip;
   /* Bandwidth checking storage */
   UX_XHCI_BW_INFO       bw_info;
   UX_HCD_XHCI_LIST      bw_endpoint_list;
   /* Isoch Frame ID checking storage */
   int32_t                   next_frame_id;
   /* Use new Isoch TRB layout needed for extended TBC support */
   bool                  use_extended_tbc;
}UX_XHCI_VIRT_EP;

typedef struct UX_XHCI_VIRT_DEVICE_STRUCT
{
   UX_DEVICE      *udev;
   /*
    * Commands to the hardware are passed an "input context" that
    * tells the hardware what to change in its data structures.
    * The hardware will return changes in an "output context" that
    * software must allocate for the hardware.  We need to keep
    * track of input and output contexts separately because
    * these commands might fail and we don't trust the hardware.
    */
   UX_XHCI_CONTAINER_CTX       *out_ctx;
   /* Used for addressing devices and configuration changes */
   UX_XHCI_CONTAINER_CTX       *in_ctx;
   UX_XHCI_VIRT_EP             eps[31];
   uint8_t                     fake_port;
   uint8_t                     real_port;
   UX_XHCI_INTERVAL_BW_TABLE   *bw_table;
   UX_XHCI_TT_BW_INFO          *tt_info;
   /*
    * flags for state tracking based on events and issued commands.
    * Software can not rely on states from output contexts because of
    * latency between events and xHC updating output context values.
    * See xhci 1.1 section 4.8.3 for more details
    */
   unsigned long                  flags;
#define VDEV_PORT_ERROR        (1 << 0) /* Port error, link inactive */

}UX_XHCI_VIRT_DEVICE;

/*
 * For each roothub, keep track of the bandwidth information for each periodic
 * interval.
 *
 * If a high speed hub is attached to the roothub, each TT associated with that
 * hub is a separate bandwidth domain.  The interval information for the
 * endpoints on the devices under that TT will appear in the TT structure.
 */

typedef struct UX_XHCI_DEQUEUE_STATE_STRUCT
{
   UX_XHCI_SEGMENT    *new_deq_seg;
   UX_XHCI_TRB        *new_deq_ptr;
   int32_t            new_cycle_state;
   uint32_t           stream_id;
}UX_XHCI_DEQUEUE_STATE;

typedef struct UX_XHCI_SLOT_CTX_STRUCT {
   uint32_t      dev_info;
   uint32_t      dev_info2;
   uint32_t      tt_info;
   uint32_t      dev_state;
   /* offset 0x10 to 0x1f reserved for HC internal use */
   uint32_t      reserved[4];
}UX_XHCI_SLOT_CTX;

/* Input Control Context data structure   */
typedef struct UX_XHCI_INPUT_CONTROL_CTX_STRUCT
{
   uint32_t    drop_flags;
   uint32_t    add_flags; /* The Add flags indicate which endpoints software
                            * wants to be added to the xHC’s list of valid endpoints  */
   uint32_t    rsvd2[6];
}UX_XHCI_INPUT_CONTROL_CTX;


typedef struct UX_XHCI_PORT_CAP_STRUCT
{
   uint32_t          *psi;/* array of protocol speed ID entries */
   uint8_t            psi_count;
   uint8_t            psi_uid_count;
   uint8_t            maj_rev;
   uint8_t            min_rev;
}UX_XHCI_PORT_CAP;

typedef struct UX_XHCI_HUB_STRUCT
{
   uint32_t      num_ports;
   /* supported prococol extended capabiliy values */
   uint8_t       maj_rev;
   uint8_t       min_rev;
}UX_XHCI_HUB;

typedef struct UX_XHCI_PORT_STRUCT
{
   int32_t           hw_portnum;
   int32_t           hcd_portnum;
   UX_XHCI_HUB       *rhub;
   UX_XHCI_PORT_CAP  *port_cap;
}UX_XHCI_PORT;


/* Define the XHCI structure.  */

typedef struct UX_HCD_XHCI_STRUCT
{
   UX_HCD *ux_hcd_xhci_hcd_owner;
   USB_Type  *regs;   /* USB DWC3 Hardware registers Base */
   UX_XHCI_OP_REGS  *op_regs;  /*XHCI Operational Register Base*/
   UX_XHCI_RUN_REGS  *run_regs; /*XHCI Run time Register Base*/
   UX_XHCI_DB_REGS  *dba_regs;  /* Doorbell register base  */
   uint32_t           ux_hcd_xhci_hcor; /*XHCI Operational Register Base*/
   uint32_t           ux_hcd_xhci_hcrr;
   /* Cached register copies of read-only HC data */
   uint32_t     hcs_params1;
   uint32_t     hcs_params2;
   uint32_t     hcs_params3;
   uint32_t     hcc_params1;
   uint32_t     hcc_params2;
   uint32_t     dba_off;
   uint16_t     hci_version;
   /* imod_interval in ns (I * 250ns) */
   uint32_t     imod_interval;
   /* 4KB min, 128MB max */
   int32_t          page_size;
   /* Valid values are 12 to 20, inclusive */
   int32_t         page_shift;
   UX_MUTEX        ux_hcd_xhci_periodic_mutex;
   UX_TIMER       port_status_timer;
   /* data structures */
   UX_XHCI_DEVICE_CONTEXT_ARRAY    *dcbaa;
   UX_XHCI_RING                    *cmd_ring;
   uint32_t                        xhc_state;
   uint32_t                        cmd_ring_state;
   uint32_t                        cmd_ring_reserved_trbs;
   UX_XHCI_COMMAND                 *current_cmd;
   UX_XHCI_RING                    *event_ring;
   UX_XHCI_ERST                    erst;
   /* Scratchpad */
   UX_XHCI_SCRATCHPAD             *scratchpad;
   uint32_t                       num_active_eps;
   UX_XHCI_PORT                   *hw_ports;
   UX_XHCI_HUB                    usb2_rhub;
   /* support xHCI 1.0 spec USB2 hardware LPM */
   uint32_t                       hw_lpm_support:1;
   /* cached usb2 extened protocol capabilites */
   uint32_t                       *ext_caps;
   uint32_t                       num_ext_caps;

   UX_XHCI_PORT_CAP               *port_caps;
   uint32_t                       num_port_caps;
   /* Internal mirror of the HW's dcbaa */
   UX_XHCI_VIRT_DEVICE            *devs[UX_XHCI_MAX_HC_SLOTS];
   /* For keeping track of bandwidth domains per roothub. */
   UX_XHCI_ROOT_PORT_BW_INFO      *rh_bw;
   UX_HCD_XHCI_LIST               cmd_list;
   UX_DEVICE                      *device;
   uint32_t                       interval;
   int32_t                        start_frame;
   uint32_t                       slot_id;
   uint32_t                       stream_id;          /* (in) stream ID */

} UX_HCD_XHCI;
typedef struct UX_XHCI_TRB_INFO_STRUCT
{
   uint32_t low_address;
   uint32_t high_address;
   uint32_t size;
   uint32_t cntrl_field;
}UX_XHCI_TRB_INFO;

#define ux_hcd_xhci_initialize                      _ux_hcd_xhci_initialize
uint32_t    ux_hcd_xhci_initialize(UX_HCD *hcd);

extern unsigned char _ux_system_host_hcd_xhci_name[];
extern UX_HCD_XHCI *hcd_xhci;

#ifdef  __cplusplus

}
#endif

#endif



