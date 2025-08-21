#ifndef UX_API_H
#define UX_API_H

#include "host/hcd.h"

#if defined(CORE_M55_HE)
#include "M55_HE.h"
#elif defined(CORE_M55_HP)
#include "M55_HP.h"
#else
#error "Unsupported core!"
#endif

#define USE_STATIC_RAM

#ifdef   __cplusplus

/* Yes, C++ compiler is present.  Use standard C.  */
extern   "C" {

#endif

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
#define UX_MUTEX_ERROR                                                  0x17
#define UX_MEMORY_CORRUPTED                                             0x19
#define UX_TRANSFER_ERROR                                               0x23
#define UX_FUNCTION_NOT_SUPPORTED                                       0x54
#define UX_CONTROLLER_UNKNOWN                                           0x55

#define UX_MEMORY_UNUSED                                                0x00000000u
#define UX_MEMORY_USED                                                  0x80000000u
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

#define UX_NULL TX_NULL

#define ALIGN_TYPE unsigned long

#define _ux_utility_memory_set(...) memset(__VA_ARGS__)

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
} UX_TIMER;

typedef struct
{
    volatile uint32_t  DEPCMDPAR2;     /*!< (@ 0x00000000) Device Physical Endpoint-n Command Parameter 2 Register  */
    volatile uint32_t  DEPCMDPAR1;     /*!< (@ 0x00000004) Device Physical Endpoint-n Command Parameter 1 Register  */
    volatile uint32_t  DEPCMDPAR0;     /*!< (@ 0x00000008) Device Physical Endpoint-n Command Parameter 0 Register  */
    volatile uint32_t  DEPCMD;         /*!< (@ 0x0000000C) Device Physical Endpoint-n Command Register              */
} USB_USB_ENDPNT_CMD_Type;

typedef struct {
    uint32_t CAPLENGTH;
    uint32_t HCSPARAMS1;
    uint32_t HCSPARAMS2;
    uint32_t HCSPARAMS3;
    uint32_t HCCPARAMS1;
    uint32_t DBOFF;
    uint32_t RTSOFF;
    uint32_t HCCPARAMS2;
    volatile const  uint32_t  RESERVED[12344];
    volatile uint32_t  GSBUSCFG0;                    /*!< (@ 0x0000C100) Global SoC Bus Configuration Register 0                    */
    volatile uint32_t  GSBUSCFG1;                    /*!< (@ 0x0000C104) Global SoC Bus Configuration Register 1                    */
    volatile const  uint32_t  RESERVED1[2];
    volatile uint32_t  GCTL;                         /*!< (@ 0x0000C110) Global Core Control Register                               */
    volatile const  uint32_t  RESERVED2;
    volatile uint32_t  GSTS;                         /*!< (@ 0x0000C118) Global Status Register                                     */
    volatile uint32_t  GUCTL1;                       /*!< (@ 0x0000C11C) Global User Control Register 1                             */
    volatile const  uint32_t  GSNPSID;                      /*!< (@ 0x0000C120) Global Controller ID Register                              */
    volatile const  uint32_t  RESERVED3;
    volatile uint32_t  GUID;                         /*!< (@ 0x0000C128) Global User ID Register                                    */
    volatile uint32_t  GUCTL;                        /*!< (@ 0x0000C12C) Global User Control Register                               */
    volatile const  uint32_t  GBUSERRADDRLO;                /*!< (@ 0x0000C130) Global SoC Bus Error Address Register-Low                  */
    volatile const  uint32_t  GBUSERRADDRHI;                /*!< (@ 0x0000C134) Global SoC Bus Error Address Register-High                 */
    volatile const  uint32_t  RESERVED4[2];
    volatile const  uint32_t  GHWPARAMS0;                   /*!< (@ 0x0000C140) Global Hardware Parameters Register 0                      */
    volatile const  uint32_t  GHWPARAMS1;                   /*!< (@ 0x0000C144) Global Hardware Parameters Register 1                      */
    volatile const  uint32_t  GHWPARAMS2;                   /*!< (@ 0x0000C148) Global Hardware Parameters Register 2                      */
    volatile const  uint32_t  GHWPARAMS3;                   /*!< (@ 0x0000C14C) Global Hardware Parameters Register 3                      */
    volatile const  uint32_t  GHWPARAMS4;                   /*!< (@ 0x0000C150) Global Hardware Parameters Register 4                      */
    volatile const  uint32_t  GHWPARAMS5;                   /*!< (@ 0x0000C154) Global Hardware Parameters Register 5                      */
    volatile const  uint32_t  GHWPARAMS6;                   /*!< (@ 0x0000C158) Global Hardware Parameters Register 6                      */
    volatile const  uint32_t  GHWPARAMS7;                   /*!< (@ 0x0000C15C) Global Hardware Parameters Register 7                      */
    volatile const  uint32_t  RESERVED5[8];
    volatile uint32_t  GPRTBIMAP_HSLO;               /*!< (@ 0x0000C180) Global High-Speed Port to Bus Instance Mapping
                                                        (Low)                                                      */
    volatile uint32_t  GPRTBIMAP_HSHI;               /*!< (@ 0x0000C184) Global High-Speed Port to Bus Instance Mapping
                                                        (High)                                                     */
    volatile uint32_t  GPRTBIMAP_FSLO;               /*!< (@ 0x0000C188) Global Full-Speed Port to Bus Instance Mapping
                                                        (Low)                                                      */
    volatile uint32_t  GPRTBIMAP_FSHI;               /*!< (@ 0x0000C18C) Global Full-Speed Port to Bus Instance Mapping
                                                        (High)                                                     */
    volatile const  uint32_t  RESERVED6[3];
    volatile uint32_t  GUCTL2;                       /*!< (@ 0x0000C19C) Global User Control Register 2                             */
    volatile const  uint32_t  RESERVED7[24];
    volatile uint32_t  GUSB2PHYCFG0;                 /*!< (@ 0x0000C200) Global USB2 PHY Configuration Register                     */
    volatile const  uint32_t  RESERVED8[63];
    volatile uint32_t  GTXFIFOSIZ[4];                /*!< (@ 0x0000C300) Global Transmit FIFO Size Register n                       */
    volatile const  uint32_t  RESERVED9[28];
    volatile uint32_t  GRXFIFOSIZ[4];                /*!< (@ 0x0000C380) Global Receive FIFO Size Register n                        */
    volatile const  uint32_t  RESERVED10[28];
    volatile uint32_t  GEVNTADRLO0;                  /*!< (@ 0x0000C400) Global Event Buffer Address (Low)                          */
    volatile uint32_t  GEVNTADRHI0;                  /*!< (@ 0x0000C404) Global Event Buffer Address (High)                         */
    volatile uint32_t  GEVNTSIZ0;                    /*!< (@ 0x0000C408) Global Event Buffer Size Register                          */
    volatile uint32_t  GEVNTCOUNT0;                  /*!< (@ 0x0000C40C) Global Event Buffer Count Register                         */
    volatile const  uint32_t  RESERVED11[124];
    volatile const  uint32_t  GHWPARAMS8;                   /*!< (@ 0x0000C600) Global Hardware Parameters Register 8                      */
    volatile const  uint32_t  RESERVED12[3];
    volatile uint32_t  GTXFIFOPRIDEV;                /*!< (@ 0x0000C610) Global Device TX FIFO DMA Priority Register                */
    volatile const  uint32_t  RESERVED13;
    volatile uint32_t  GTXFIFOPRIHST;                /*!< (@ 0x0000C618) Global Host TX FIFO DMA Priority Register                  */
    volatile uint32_t  GRXFIFOPRIHST;                /*!< (@ 0x0000C61C) Global Host RX FIFO DMA Priority Register                  */
    volatile const  uint32_t  RESERVED14[4];
    volatile uint32_t  GFLADJ;                       /*!< (@ 0x0000C630) Global Frame Length Adjustment Register                    */
    volatile const  uint32_t  RESERVED15[3];
    volatile uint32_t  GUSB2RHBCTL0;                 /*!< (@ 0x0000C640) Global USB 2.0 Root Hub Control Register                   */
    volatile const  uint32_t  RESERVED16[47];
    volatile uint32_t  DCFG;                         /*!< (@ 0x0000C700) Device Configuration Register                              */
    volatile uint32_t  DCTL;                         /*!< (@ 0x0000C704) Device Control Register                                    */
    volatile uint32_t  DEVTEN;                       /*!< (@ 0x0000C708) Device Event Enable Register                               */
    volatile uint32_t  DSTS;                         /*!< (@ 0x0000C70C) Device Status Register                                     */
    volatile uint32_t  DGCMDPAR;                     /*!< (@ 0x0000C710) Device Generic Command Parameter Register                  */
    volatile uint32_t  DGCMD;                        /*!< (@ 0x0000C714) Device Generic Command Register                            */
    volatile const  uint32_t  RESERVED17[2];
    volatile uint32_t  DALEPENA;                     /*!< (@ 0x0000C720) Device Active USB Endpoint Enable Register                 */
    volatile const  uint32_t  RESERVED18[55];
    volatile USB_USB_ENDPNT_CMD_Type USB_ENDPNT_CMD[8];/*!< (@ 0x0000C800) [0..7]                                                   */
    volatile const  uint32_t  RESERVED19[96];
    volatile uint32_t  DEV_IMOD0;                    /*!< (@ 0x0000CA00) Device Interrupt Moderation Register                       */
} USB_Type;                                     /*!< Size = 51716 (0xca04)  */


/* Define USBX Memory Management structure.  */

typedef struct UX_MEMORY_BLOCK_STRUCT
{

    unsigned long   ux_memory_block_size;
    unsigned long   ux_memory_block_status;
    struct  UX_MEMORY_BLOCK_STRUCT
                    *ux_memory_block_next;
    struct  UX_MEMORY_BLOCK_STRUCT
                    *ux_memory_block_previous;
} UX_MEMORY_BLOCK;


typedef struct UX_SYSTEM_STRUCT
{

    UX_MEMORY_BLOCK *ux_system_regular_memory_pool_start;
    unsigned long   ux_system_regular_memory_pool_size;
    unsigned long   ux_system_regular_memory_pool_free;
    UX_MEMORY_BLOCK *ux_system_cache_safe_memory_pool_start;
    unsigned long   ux_system_cache_safe_memory_pool_size;
    unsigned long   ux_system_cache_safe_memory_pool_free;
#ifdef UX_ENABLE_MEMORY_STATISTICS
    unsigned char   *ux_system_regular_memory_pool_base;
    ALIGN_TYPE      ux_system_regular_memory_pool_max_start_offset;
    ALIGN_TYPE      ux_system_regular_memory_pool_min_free;
    unsigned char   *ux_system_cache_safe_memory_pool_base;
    ALIGN_TYPE      ux_system_cache_safe_memory_pool_max_start_offset;
    ALIGN_TYPE      ux_system_cache_safe_memory_pool_min_free;
    unsigned long   ux_system_regular_memory_pool_alloc_count;
    unsigned long   ux_system_regular_memory_pool_alloc_total;
    unsigned long   ux_system_regular_memory_pool_alloc_max_count;
    unsigned long   ux_system_regular_memory_pool_alloc_max_total;
    unsigned long   ux_system_cache_safe_memory_pool_alloc_count;
    unsigned long   ux_system_cache_safe_memory_pool_alloc_total;
    unsigned long   ux_system_cache_safe_memory_pool_alloc_max_count;
    unsigned long   ux_system_cache_safe_memory_pool_alloc_max_total;
#endif

    unsigned int            ux_system_thread_lowest_priority;
    OSAL_MUTEX_DEF(ux_system_mutex);

#ifndef UX_DISABLE_ERROR_HANDLER
    unsigned int            ux_system_last_error;
    unsigned int            ux_system_error_count;
    void            (*ux_system_error_callback_function) (unsigned int system_level, unsigned int system_context, unsigned int error_code);
#endif

#ifdef UX_ENABLE_DEBUG_LOG
    unsigned long   ux_system_debug_code;
    unsigned long   ux_system_debug_count;
    unsigned char   *ux_system_debug_log_buffer;
    unsigned char   *ux_system_debug_log_head;
    unsigned char   *ux_system_debug_log_tail;
    unsigned long   ux_system_debug_log_size;
    void            (*ux_system_debug_callback_function) (unsigned char *debug_message, unsigned long debug_value);
#endif
} UX_SYSTEM;

extern UX_SYSTEM *_ux_system;

#define ux_system_initialize                                    _ux_system_initialize
#define ux_system_uninitialize                                  _ux_system_uninitialize

#define UX_EVENT_FLAGS_GROUP                                            TX_EVENT_FLAGS_GROUP

typedef struct  {
    int dummy;
} TX_EVENT_FLAGS_GROUP;

unsigned int  _ux_system_initialize(void *regular_memory_pool_start, unsigned long regular_memory_size,
                                    void *cache_safe_memory_pool_start, unsigned long cache_safe_memory_size);

void            *_ux_utility_memory_allocate(unsigned long memory_alignment, unsigned long memory_cache_flag, unsigned long memory_size_requested);
//unsigned int    _ux_utility_memory_compare(void *memory_source, void *memory_destination, unsigned long length);
//void             _ux_utility_memory_copy(void *memory_destination, void *memory_source, unsigned long length);
void             _ux_utility_memory_free(void *memory);
UX_MEMORY_BLOCK  *_ux_utility_memory_free_block_best_get(unsigned long memory_cache_flag,
                                                         unsigned long memory_size_requested);


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

#ifdef   __cplusplus
}
#endif


#endif
