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
 * @file     ux_hcd_xhci_mem_init.c
 * @author   Mahesh Avula
 * @email    mahesh.avula@alifsemi.com
 * @version  V1.0.0
 * @date     12-Dec-2023
 * @brief    XHCI host controller driver.
 * @bug      None
 * @Note     None
 ******************************************************************************/

/* Include necessary system files.  */

#include "tusb_option.h"

#if CFG_TUH_ENABLED && CFG_TUSB_MCU == OPT_MCU_NONE

#include "host/hcd.h"
#include "ux_hcd_xhci_api.h"
//#include "ux_host_stack.h"

mallocs_t _mallocs[32];
uint8_t _mallocs_cnt = 0;

#define MAX_SEGS 2

static UX_XHCI_SEGMENT * _ux_hcd_xhci_segment_alloc(
        UX_HCD_XHCI *xhci,
        uint32_t cycle_state,
        uint32_t max_packet)
{
    UX_XHCI_SEGMENT *seg;
    int32_t   i;
    static int idx = 0;
    printf("Called %s(%p %u %u) idx=%d\n", __FUNCTION__, xhci, cycle_state, max_packet, idx);
    /* Allocate memery for Segment  */
#if 0 //def USE_STATIC_RAM
    if (idx >= MAX_SEGS)
    {
        printf("*** Out of segs limit (%u)\n", MAX_SEGS);
        return NULL;
    }
    static UX_XHCI_SEGMENT _seg[MAX_SEGS] __attribute__((section("usb_dma_buf")));
    memset(&_seg[idx], 0, sizeof(_seg[idx]));
    xhci = &_seg[idx];
#else
    seg = _ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY, sizeof(*seg));
#endif
    if (!seg)
    {
#ifdef DEBUG
        printf("Segment memory Allocation failed \n");
#endif
        return NULL;
    }
    /* Allocate memeory for TRB's in segment  */
#ifdef USE_STATIC_RAM
    static uint8_t _trbs[MAX_SEGS][TRB_SEGMENT_SIZE] __attribute__((section("usb_dma_buf"))) __attribute__((aligned(64)));
    memset(&_trbs[idx], 0, sizeof(_trbs[idx]));
    UX_XHCI_TRB* t = (UX_XHCI_TRB*)&_trbs[idx];
    seg->trbs = t;
#else
    seg->trbs = _ux_utility_memory_allocate(UX_ALIGN_64, UX_REGULAR_MEMORY,  TRB_SEGMENT_SIZE);
#endif
    if (!seg->trbs)
    {
#ifdef DEBUG
        printf(" memory allocation failed for TRB's in Segment\n");
#endif
        _ux_utility_memory_free(seg);
        return NULL;
    }

    if (max_packet)
    {
        seg->bounce_buf = _ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY, max_packet);
        if (!seg->bounce_buf)
        {
#ifdef DEBUG
            printf("memory allocation failed\n");
#endif
            _ux_utility_memory_free(seg->trbs);
            _ux_utility_memory_free(seg);
            return NULL;
        }
    }
    /* If the cycle state is 0, set the cycle bit to 1 for all the TRBs */
    if (cycle_state == 0)
    {
        for (i = 0; i < TRBS_PER_SEGMENT; i++)
            seg->trbs[i].link.control |= (TRB_CYCLE);
    }
    //RTSS_CleanDCache_by_Addr(seg->trbs, sizeof(*seg->trbs));
    seg->dma = LocalToGlobal(seg->trbs);
    seg->next = NULL;
    idx++;
    return seg;
}

static void _ux_hcd_xhci_segment_free(UX_HCD_XHCI *xhci, UX_XHCI_SEGMENT *seg)
{
    if (seg->trbs)
        _ux_utility_memory_free(seg->trbs);

    if (seg->bounce_buf)
        _ux_utility_memory_free(seg->bounce_buf);

    if (seg)
        _ux_utility_memory_free(seg);
}

static void _ux_hcd_xhci_link_segments(UX_HCD_XHCI *xhci, UX_XHCI_SEGMENT *prev,
        UX_XHCI_SEGMENT *next,
        UX_XHCI_RING_TYPE type)
{
    uint32_t val;

    if (!prev || !next)
        return;
    prev->next = next;
    if (type != TYPE_EVENT)
    {
        prev->trbs[TRBS_PER_SEGMENT-1].link.segment_ptr = next->dma;
        /* Set the last TRB in the segment to have a TRB type ID of Link TRB */
        val = prev->trbs[TRBS_PER_SEGMENT-1].link.control;
        val &= ~TRB_TYPE_BITMASK;
        val |= TRB_TYPE(TRB_LINK);
        /* Set chain bit for isoc rings   */
        if(type == TYPE_ISOC)
            val |= TRB_CHAIN;
        prev->trbs[TRBS_PER_SEGMENT-1].link.control = val;
    }
}

/* Allocate segments and link them for a ring */
static int32_t _ux_hcd_xhci_alloc_segments_for_ring(UX_HCD_XHCI *xhci,
        UX_XHCI_SEGMENT **first,
        UX_XHCI_SEGMENT **last,
        uint32_t num_segs,
        uint32_t cycle_state,
        UX_XHCI_RING_TYPE type,
        uint32_t max_packet)
{
    UX_XHCI_SEGMENT *prev;
    prev = _ux_hcd_xhci_segment_alloc(xhci, cycle_state, max_packet);
    if (!prev)
    {
       printf("Failed at first\n");
        return -1;
    }

    num_segs--;
    *first = prev;
    while (num_segs > 0)
    {
        UX_XHCI_SEGMENT   *next;
        next = _ux_hcd_xhci_segment_alloc(xhci, cycle_state, max_packet);
        if (!next)
        {
            printf("Failed at second\n");
            prev = *first;
            while (prev)
            {
                next = prev->next;
                _ux_hcd_xhci_segment_free(xhci, prev);
                prev = next;
            }
            return -1;
        }
        _ux_hcd_xhci_link_segments(xhci, prev, next, type);
        prev = next;
        num_segs--;
    }
    _ux_hcd_xhci_link_segments(xhci, prev, *first, type);
    *last = prev;
    return 0;
}

static void _ux_hcd_xhci_initialize_ring_info(UX_XHCI_RING *ring, int cycle_state)
{
    /* The ring is empty, so the enqueue pointer == dequeue pointer */
    ring->enqueue = ring->first_seg->trbs;
    ring->enq_seg = ring->first_seg;
    ring->dequeue = ring->enqueue;
    ring->deq_seg = ring->first_seg;
    /* The ring is initialized to 0. The producer must write 1 to the cycle
     * bit to handover ownership of the TRB, so PCS = 1.  The consumer must
     * compare CCS to the cycle bit to check ownership, so CCS = 1.
     *
     * New rings are initialized with cycle state equal to 1; if we are
     * handling ring expansion, set the cycle state equal to the old ring.
     */
    ring->cycle_state = cycle_state;
    /*
     * Each segment has a link TRB, and leave an extra TRB for SW
     * accounting purpose
     */
    ring->num_trbs_free = ring->num_segs * (TRBS_PER_SEGMENT - 1) - 1;
}

UX_XHCI_RING* _ux_hcd_xhci_ring_alloc(UX_HCD_XHCI *xhci, uint32_t num_segs, uint32_t cycle_state,
        UX_XHCI_RING_TYPE type, uint32_t max_packet)
{
    printf("Called %s(%p %u %u %u %u)\n", __FUNCTION__, xhci, num_segs, cycle_state, type, max_packet);
    UX_XHCI_RING  *ring;
    int32_t ret;
    /* Allocate the memery for Ring  */
    ring = _ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY, sizeof(*ring));
    if (!ring)
    {
#ifdef DEBUG
        printf("memory allocation failed for ring \n");
#endif
        return NULL;
    }
    if (num_segs == 0)
        return ring;
    ring->num_segs = num_segs;
    ring->bounce_buf_len = max_packet;
    INIT_LIST_HEAD(&ring->td_list);
    ring->type = type;
    ret = _ux_hcd_xhci_alloc_segments_for_ring(xhci, &ring->first_seg,
            &ring->last_seg, num_segs, cycle_state, type, max_packet);
    if (ret)
        goto fail;

    /* Only event ring  and command ring does not use link TRB */
    if (type != TYPE_EVENT)
    {
        /* See section 4.9.2.1 and 6.4.4.1 */
        ring->last_seg->trbs[TRBS_PER_SEGMENT - 1].link.control |= (LINK_TOGGLE);
    }
    _ux_hcd_xhci_initialize_ring_info(ring, cycle_state);
    return ring;

fail:
    _ux_utility_memory_free(ring);
    return NULL;
}

UX_XHCI_RING *_ux_hcd_xhci_stream_id_to_ring(UX_XHCI_VIRT_DEVICE *dev,uint32_t ep_index, uint32_t stream_id)
{
    UX_XHCI_VIRT_EP *ep = &dev->eps[ep_index];
    if (stream_id == 0)
        return ep->ring;
    if (!ep->stream_info)
        return NULL;
    if (stream_id >= ep->stream_info->num_streams)
        return NULL;
    return ep->stream_info->stream_rings[stream_id];
}

UX_XHCI_CONTAINER_CTX *_ux_hcd_xhci_alloc_container_ctx(UX_HCD_XHCI *xhci, int32_t type)
{
    printf("Called %s(%p %u)\n", __FUNCTION__, xhci, type);
    UX_XHCI_CONTAINER_CTX *ctx;
    if ((type != UX_XHCI_CTX_TYPE_DEVICE) && (type != UX_XHCI_CTX_TYPE_INPUT))
        return NULL;
    ctx = _ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY, sizeof(*ctx));
    if (!ctx)
        return NULL;
    ctx->type = type;
    ctx->size = HCC_64BYTE_CONTEXT(xhci->hcc_params1) ? 2048 : 1024;
    if (type == UX_XHCI_CTX_TYPE_INPUT)
        ctx->size += CTX_SIZE(xhci->hcc_params1);

    ctx->bytes = _ux_utility_memory_allocate(UX_ALIGN_64, UX_REGULAR_MEMORY, ctx->size);
    if (!ctx->bytes)
    {
        _ux_utility_memory_free(ctx);
        return NULL;
    }
    ctx->dma = LocalToGlobal(ctx->bytes);
    return ctx;
}

void _ux_hcd_xhci_free_device_endpoint_resources(UX_HCD_XHCI *xhci, UX_XHCI_VIRT_DEVICE *virt_dev,
        bool drop_control_ep)
{
    int32_t i;
    uint32_t num_dropped_eps = 0;
    uint32_t drop_flags = 0;
    for (i = (drop_control_ep ? 0 : 1); i < 31; i++)
    {
        if (virt_dev->eps[i].ring)
        {
            drop_flags |= 1 << i;
            num_dropped_eps++;
        }
    }
    xhci->num_active_eps -= num_dropped_eps;
    if (num_dropped_eps)
    {
#ifdef DEBUG
        printf("Dropped %u ep ctxs, flags = 0x%x, ""%u now active\n",num_dropped_eps, drop_flags,
                xhci->num_active_eps);
#endif
    }

}

UX_XHCI_INPUT_CONTROL_CTX *_ux_hcd_xhci_get_input_control_ctx(UX_XHCI_CONTAINER_CTX *ctx)
{
    if (ctx->type != UX_XHCI_CTX_TYPE_INPUT)
        return NULL;
    return (UX_XHCI_INPUT_CONTROL_CTX *)ctx->bytes;
}

UX_XHCI_COMMAND *_ux_hcd_xhci_alloc_command(UX_HCD_XHCI *xhci)
{
    UX_XHCI_COMMAND *command;
    command = _ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY, sizeof(*command));
    if (!command)
        return NULL;
    command->status = 0;
    INIT_LIST_HEAD(&command->cmd_list);
    return command;
}

UX_XHCI_COMMAND *_ux_hcd_xhci_alloc_command_with_ctx_sz(UX_HCD_XHCI *xhci)
{
    UX_XHCI_COMMAND *command;
    command = _ux_hcd_xhci_alloc_command(xhci);
    if (!command)
        return NULL;
    command->in_ctx = _ux_hcd_xhci_alloc_container_ctx(xhci, UX_XHCI_CTX_TYPE_INPUT);
    if (!command->in_ctx)
    {
        _ux_utility_memory_free(command);
        return NULL;
    }
    return command;
}

uint32_t _ux_hcd_xhci_alloc_erst(UX_HCD_XHCI *xhci, UX_XHCI_RING *evt_ring, UX_XHCI_ERST *erst)
{
    printf("Called %s(%p %p %p)\n", __FUNCTION__, xhci, evt_ring, erst);
    size_t size;
    uint32_t val;
    UX_XHCI_SEGMENT *seg;
    UX_XHCI_ERST_ENTRY *entry;
    size = sizeof(UX_XHCI_ERST_ENTRY) * evt_ring->num_segs;
    printf("size=%u, evt_ring->num_segs=%u\n", size, evt_ring->num_segs);
#ifdef USE_STATIC_RAM
    static UX_XHCI_ERST_ENTRY _entries[2]  __attribute__((section("usb_dma_buf")));
    memset(&_entries, 0, sizeof(_entries));
    erst->entries = &_entries[0];
#else
    erst->entries = _ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY, 2 * size);
#endif
    if (!erst->entries)
        return 1;
    memset (erst->entries, 0, 2 * size);
    erst->erst_dma_addr = (uint64_t)LocalToGlobal(erst->entries);
    erst->num_entries = evt_ring->num_segs;
    seg = evt_ring->first_seg;
    for (val = 0; val < evt_ring->num_segs; val++)
    {
        entry = &erst->entries[val];
        entry->seg_addr = LocalToGlobal((void *)seg->dma);
        entry->seg_size = TRBS_PER_SEGMENT;
        entry->rsvd = 0;
        seg = seg->next;
    }
    return 0;
}

void _ux_hcd_xhci_set_hc_event_deq(UX_HCD_XHCI *xhci)
{
    uint64_t reg_64 = 0;
    uint64_t deq = 0;
    deq = _ux_hcd_xhci_trb_virt_to_dma(xhci->event_ring->deq_seg, xhci->event_ring->dequeue);
    if (deq == 0)
    {
#ifdef DEBUG
        printf("WARN something wrong with SW event ring " "dequeue ptr.\n");
#endif
    }
    /* Update HC event ring dequeue pointer */
    reg_64 = xhci->run_regs->ir_set[0].ERDP;
    reg_64 &= ERST_PTR_MASK;
    /* Don't clear the EHB bit (which is RW1C) because
     * there might be more events to service.
     */
    reg_64 &= ~ERST_EHB;
    xhci->run_regs->ir_set[0].ERDP =  ((uint64_t)LocalToGlobal((void *)deq) & (uint64_t) ~ERST_PTR_MASK) | reg_64;
}

UX_XHCI_SLOT_CTX *_ux_hcd_xhci_get_slot_ctx(UX_HCD_XHCI *xhci, UX_XHCI_CONTAINER_CTX *ctx)
{
    if (ctx->type == UX_XHCI_CTX_TYPE_DEVICE)
    {
        return (UX_XHCI_SLOT_CTX *)ctx->bytes;
    }
    return (UX_XHCI_SLOT_CTX *)
        (ctx->bytes + CTX_SIZE(xhci->hcc_params1));
}

UX_XHCI_EP_CTX *_ux_hcd_xhci_get_ep_ctx(UX_HCD_XHCI *xhci,UX_XHCI_CONTAINER_CTX *ctx,
        uint32_t ep_index)
{
    /* increment ep index by offset of start of ep ctx array */
    ep_index++;
    if (ctx->type == UX_XHCI_CTX_TYPE_INPUT)
        ep_index++;

    return (UX_XHCI_EP_CTX *)
        (ctx->bytes + (ep_index * CTX_SIZE(xhci->hcc_params1)));
}

/* Set up the scratchpad buffer array and scratchpad buffers, if needed. */
static uint32_t scratchpad_alloc(UX_HCD_XHCI *xhci)
{
    int32_t i;
    int32_t num_sp = HCS_MAX_SCRATCHPAD(xhci->hcs_params2);
    if (!num_sp)
        return 0;

    xhci->scratchpad = _ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY, sizeof(*xhci->scratchpad));
    if (!xhci->scratchpad)
        goto fail_sp;
    xhci->scratchpad->sp_array = _ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY, num_sp * sizeof(uint64_t));

    if (!xhci->scratchpad->sp_array)
        goto fail_sp2;
    xhci->scratchpad->sp_dma = xhci->scratchpad->sp_array;

    xhci->scratchpad->sp_buffers = _ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY, num_sp * sizeof(void *));
    if (!xhci->scratchpad->sp_buffers)
        goto fail_sp3;

    xhci->dcbaa->dev_context_ptrs[0] = (uint64_t)(xhci->scratchpad->sp_dma);

    for (i = 0; i < num_sp; i++)
    {
        uint64_t dma;
        void *buf = _ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY, xhci->page_size);
        if (!buf)
            goto fail_sp4;

        dma = (uint64_t)buf;
        xhci->scratchpad->sp_array[i] = dma;
        xhci->scratchpad->sp_buffers[i] = buf;
    }
    return 0;

fail_sp4:
    for (i = i - 1; i >= 0; i--)
    {
        _ux_utility_memory_free(xhci->scratchpad->sp_buffers[i]);
        _ux_utility_memory_free((void *)xhci->scratchpad->sp_array[i]);
    }

    _ux_utility_memory_free(xhci->scratchpad->sp_buffers);

fail_sp3:
    _ux_utility_memory_free(xhci->scratchpad->sp_array);
    _ux_utility_memory_free(xhci->scratchpad->sp_dma);

fail_sp2:
    _ux_utility_memory_free(xhci->scratchpad);
    xhci->scratchpad = NULL;

fail_sp:
    return 1;
}

static void _ux_hcd_xhci_add_in_port(UX_HCD_XHCI *xhci,uint32_t num_ports, uint32_t offset, int32_t max_caps)
{
    uint32_t temp, port_offset, port_count;
    int32_t i;
    uint8_t major_revision, minor_revision;
    UX_XHCI_HUB *rhub;
    UX_XHCI_PORT_CAP *port_cap;

    temp = *(uint32_t *)((uint8_t *)(xhci->regs) + offset);
    major_revision = UX_XHCI_EXT_PORT_MAJOR(temp);
    minor_revision = UX_XHCI_EXT_PORT_MINOR(temp);
    if (major_revision <= 0x02)
    {
        rhub = &xhci->usb2_rhub;
    }
    else
    {
#ifdef DEBUG
        printf( "Ignoring unknown port speed, revision = 0x%x\n", major_revision);
#endif
        /* Ignoring port protocol we can't understand. FIXME */
        return;
    }
    rhub->maj_rev = UX_XHCI_EXT_PORT_MAJOR(temp);
    if (rhub->min_rev < minor_revision)
        rhub->min_rev = minor_revision;

    /* Port offset and count in the third dword, see section 7.2 */
    temp = *(uint32_t *)(( (uint8_t *)(xhci->regs) + offset) + (2*4));
    /* Compatible Port Offset  */
    port_offset = UX_XHCI_EXT_PORT_OFF(temp);
    /* Compatible Port Count  */
    port_count = UX_XHCI_EXT_PORT_COUNT(temp);

    /* Port count includes the current port offset */
    if (port_offset == 0 || (port_offset + port_count - 1) > num_ports)
        return;
    port_cap = &xhci->port_caps[xhci->num_port_caps++];
    if (xhci->num_port_caps > max_caps)
        return;

    port_cap->maj_rev = major_revision;
    port_cap->min_rev = minor_revision;
    /*  Protocol Speed ID Count (PSIC)    */
    port_cap->psi_count = UX_XHCI_EXT_PORT_PSIC(temp);

    if (port_cap->psi_count)
    {
        port_cap->psi = _ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY, port_cap->psi_count * sizeof(*port_cap->psi));
        if (!port_cap->psi)
            port_cap->psi_count = 0;

        port_cap->psi_uid_count++;
        for (i = 0; i < port_cap->psi_count; i++)
        {
            port_cap->psi[i] =  *(uint32_t *)( (uint8_t *)(xhci->regs) + offset + (4*4) + (i*4));
            /* count unique ID values, two consecutive entries can
             * have the same ID if link is asymetric
             */
            if (i && (UX_XHCI_EXT_PORT_PSIV(port_cap->psi[i]) !=
                        UX_XHCI_EXT_PORT_PSIV(port_cap->psi[i - 1])))
                port_cap->psi_uid_count++;
            {
#ifdef DEBUG
                printf("PSIV:%d PSIE:%d PLT:%d PFD:%d LP:%d PSIM:%d\n",
                        UX_XHCI_EXT_PORT_PSIV(port_cap->psi[i]),
                        UX_XHCI_EXT_PORT_PSIE(port_cap->psi[i]),
                        UX_XHCI_EXT_PORT_PLT(port_cap->psi[i]),
                        UX_XHCI_EXT_PORT_PFD(port_cap->psi[i]),
                        UX_XHCI_EXT_PORT_LP(port_cap->psi[i]),
                        UX_XHCI_EXT_PORT_PSIM(port_cap->psi[i]));
#endif
            }
        }
    }
    /* cache usb2 port capabilities */
    if (major_revision < 0x03 && xhci->num_ext_caps < max_caps)
        xhci->ext_caps[xhci->num_ext_caps++] = temp;

    if ((xhci->hci_version >= 0x100) && (major_revision != 0x03) && (temp & UX_XHCI_HLC))
    {
#ifdef DEBUG
        printf("xHCI 1.0: support USB2 hardware lpm\n");
#endif
        xhci->hw_lpm_support = 1;
    }

    port_offset--;
    for (i = port_offset; i < (port_offset + port_count); i++)
    {
        UX_XHCI_PORT *hw_port = &xhci->hw_ports[i];
        /* Duplicate entry.  Ignore the port if the revisions differ. */
        if (hw_port->rhub)
        {
#ifdef DEBUG
            printf( "Duplicate port entry, Ext Cap\n");
            printf( "Port was marked as USB %u, " "duplicated as USB %u\n",hw_port->rhub->maj_rev, major_revision);
#endif
            /* Only adjust the roothub port counts if we haven't  * found a similar duplicate.*/
            if (hw_port->rhub != rhub && hw_port->hcd_portnum != DUPLICATE_ENTRY)
            {
                hw_port->rhub->num_ports--;
                hw_port->hcd_portnum = DUPLICATE_ENTRY;
            }
            continue;
        }
        hw_port->rhub = rhub;
        hw_port->port_cap = port_cap;
        rhub->num_ports++;
    }
}

/*
 * Scan the Extended Capabilities for the "Supported Protocol Capabilities" that
 * specify what speeds each port is supposed to be.  We can't count on the port
 * speed bits in the PORTSC register being correct until a device is connected,
 * but we need to set up the two fake roothubs with the correct number of
 * USB 2.0 ports at host controller initialization time.
 */
static uint32_t _ux_hcd_xhci_setup_port_arrays(UX_HCD_XHCI *xhci)
{
    uint32_t offset;
    uint32_t num_ports;
    int32_t i, j;
    int32_t cap_count = 0;
    uint32_t cap_start;

    num_ports = HCS_MAX_PORTS(xhci->hcs_params1);
    xhci->hw_ports = _ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY, num_ports * sizeof(*xhci->hw_ports));

    if (!xhci->hw_ports)
        return 1;

    for (i = 0; i < num_ports; i++)
    {
        xhci->hw_ports[i].hw_portnum = i;
    }
    /* allocate memory for roothub bandwidth */
    xhci->rh_bw = _ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY, num_ports * sizeof(*xhci->rh_bw));
    if (!xhci->rh_bw)
        return 1;
    for (i = 0; i < num_ports; i++)
    {
        UX_XHCI_INTERVAL_BW_TABLE *bw_table;
        INIT_LIST_HEAD(&xhci->rh_bw[i].tts);  /* FIXME  */
        bw_table = &xhci->rh_bw[i].bw_table;
        for (j = 0; j < UX_XHCI_MAX_INTERVAL; j++)
            INIT_LIST_HEAD(&bw_table->interval_bw[j].endpoints);
    }
    cap_start = _ux_hcd_xhci_find_next_ext_cap(xhci, 0, UX_XHCI_EXT_CAPS_PROTOCOL);
    if (!cap_start)
    {
#ifdef DEBUG
        printf("No Extended Capability registers, unable to set up roothub\n");
#endif
        return 1;
    }

    offset = cap_start;
    /* count extended protocol capability entries for later caching */
    while (offset)
    {
        cap_count++;
        offset = _ux_hcd_xhci_find_next_ext_cap(xhci, offset, UX_XHCI_EXT_CAPS_PROTOCOL);
    }

    xhci->ext_caps = _ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY, cap_count * sizeof(*xhci->ext_caps));
    if (!xhci->ext_caps)
        return -1;

    xhci->port_caps = _ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY, cap_count * sizeof(*xhci->port_caps));
    if (!xhci->port_caps)
        return 1;

    offset = cap_start;
    while (offset)
    {
        _ux_hcd_xhci_add_in_port(xhci, num_ports, offset, cap_count);
        if (xhci->usb2_rhub.num_ports == num_ports)
            break;
        offset = _ux_hcd_xhci_find_next_ext_cap(xhci, offset, UX_XHCI_EXT_CAPS_PROTOCOL);
    }
    if (xhci->usb2_rhub.num_ports == 0)
    {
#ifdef DEBUG
        printf("No ports on the roothubs\n");
#endif
        return 1;
    }
    /* Place limits on the number of roothub ports so that the hub
     * descriptors aren't longer than the USB core will allocate.
     */
    if (xhci->usb2_rhub.num_ports > USB_MAXCHILDREN)
    {
#ifdef DEBUG
        printf("Limiting USB 2.0 roothub ports to %u.\n",USB_MAXCHILDREN);
#endif
        xhci->usb2_rhub.num_ports = USB_MAXCHILDREN;
    }
    return 0;
}

uint32_t _ux_hcd_xhci_mem_init(UX_HCD_XHCI *xhci)
{
    uint32_t page_size, hc_max_slots, i, reg;
    uint64_t reg_64 = 0;
    uint32_t ret;
    INIT_LIST_HEAD(&xhci->cmd_list);
    /* Read the page size from operational register  */
    page_size = xhci->op_regs->PAGESIZE;
    for (i = 0; i < 16; i++)
    {
        if ( page_size & XHCI_PAGE_BIT_SET)
            break;
        page_size = page_size >> 1;
    }
    if (i < 16)
    {
#ifdef DEBUG
        printf("Supported page size of %iK\n", (1 << (i+12)) / 1024);
#endif
    }
    else
    {
#ifdef DEBUG
        printf("WARN: no supported page size\n");
#endif
    }
    /*  xHC supports a page size of 2^(n+12) if bit n is Set. For example, if
        bit 0 is Set, the xHC supports 4k byte page sizes.   */
    xhci->page_shift = 12;
    xhci->page_size = 1 << xhci->page_shift;

    /* xHCI section 5.4.7
     * Program the Number of Device Slots Enabled field in the CONFIG
     * register with the max value of slots the HC can handle.
     */

    hc_max_slots = HCS_MAX_SLOTS(xhci->regs->HCSPARAMS1);
    reg = xhci->op_regs->CONFIG;

    hc_max_slots |= (reg & ~HCS_SLOTS_MASK);
    xhci->op_regs->CONFIG = hc_max_slots;

    /*
     * xHCI section 5.4.6 - doorbell array must be
     * "physically contiguous and 64-byte (cache line) aligned".
     */
#ifdef USE_STATIC_RAM
    static UX_XHCI_DEVICE_CONTEXT_ARRAY _dcbaa __attribute__((section("usb_dma_buf"))) __attribute__((aligned(128)));
    memset(&_dcbaa, 0, sizeof(_dcbaa));
    xhci->dcbaa = &_dcbaa;
#else
    xhci->dcbaa = _ux_utility_memory_allocate(UX_ALIGN_128, UX_REGULAR_MEMORY, sizeof(*xhci->dcbaa));
#endif
    if (xhci->dcbaa == NULL)
    {
#ifdef DEBUG
        printf("?DCBAA Allocation failed \n");
#endif
        return(UX_MEMORY_INSUFFICIENT);
    }
    //RTSS_CleanDCache_by_Addr(xhci->dcbaa, sizeof(*xhci->dcbaa));
    xhci->dcbaa->dma = (uint64_t)xhci->dcbaa;
    xhci->op_regs->DCBAAP = LocalToGlobal(xhci->dcbaa);

    /* Define the Command Ring Dequeue Pointer by programming the
     * Command Ring Control Register (5.4.5) with a 64-bit address pointing to
     * the starting address of the first TRB of the Command Ring   */

    /* Set up the command ring to have one segment for now. */
    xhci->cmd_ring = _ux_hcd_xhci_ring_alloc(xhci, CMD_RING_SEGS, 1, TYPE_COMMAND, 0);
    if (!xhci->cmd_ring)
    {
#ifdef DEBUG
        printf("Command ring allocation failed\n");
#endif
        goto fail;
    }

    /* Set the address in the Command Ring Control register */
    reg_64 = xhci->op_regs->CRCR;
    //RTSS_CleanDCache_by_Addr(xhci->cmd_ring->first_seg->dma, sizeof(xhci->cmd_ring->first_seg->dma));
    reg_64 = (reg_64 & (uint64_t) CMD_RING_RSVD_BITS) |
        ((uint64_t)(LocalToGlobal((void *)xhci->cmd_ring->first_seg->dma)) & (uint64_t) ~CMD_RING_RSVD_BITS) |
        xhci->cmd_ring->cycle_state;

    xhci->op_regs->CRCR = reg_64;

    /* Reserve one command ring TRB for disabling LPM.
     * Since the USB core grabs the shared usb_bus bandwidth mutex before
     * disabling LPM, we only need to reserve one TRB for all devices.
     */
    xhci->cmd_ring_reserved_trbs++;
    /*
     * Event ring setup: Allocate a normal ring, but also setup
     * the event ring segment table (ERST).  Section 4.9.3.
     */
    xhci->event_ring = _ux_hcd_xhci_ring_alloc(xhci, ERST_NUM_SEGS, 1, TYPE_EVENT, 0);

    if (!xhci->event_ring)
        goto fail;
    /* allocate the command ring segment table  */
    ret = _ux_hcd_xhci_alloc_erst(xhci, xhci->event_ring, &xhci->erst);
    if (ret)
        goto fail;

    /* set ERST count with the number of entries in the segment table */
    reg = xhci->run_regs->ir_set[0].ERSTSZ;
    reg &= ERST_SIZE_MASK;
    reg |= ERST_NUM_SEGS;
    xhci->run_regs->ir_set[0].ERSTSZ = reg;
    /* set the segment table base address */
    reg_64 = xhci->run_regs->ir_set[0].ERSTBA;
    reg_64 &= ERST_PTR_MASK;
    reg_64 |= ((uint64_t)(xhci->erst.erst_dma_addr) & (uint64_t) ~ERST_PTR_MASK);
    xhci->run_regs->ir_set[0].ERSTBA = LocalToGlobal((void *)reg_64);

    /* Set the event ring dequeue address */
    _ux_hcd_xhci_set_hc_event_deq(xhci);

    for (i = 0; i < UX_XHCI_MAX_HC_SLOTS; i++)
        xhci->devs[i] = NULL;

    if (scratchpad_alloc(xhci))
        goto fail;
    if (_ux_hcd_xhci_setup_port_arrays(xhci))
        goto fail;

    return UX_SUCCESS;

fail:
    _ux_hcd_xhci_halt(xhci);
    _ux_hcd_xhci_reset(xhci);
    return 1;
}

static void _ux_hcd_xhci_free_tt_info(UX_HCD_XHCI *xhci, UX_XHCI_VIRT_DEVICE *virt_dev,
        uint32_t slot_id)
{
    UX_HCD_XHCI_LIST *tt_list_head;
    UX_XHCI_TT_BW_INFO *tt_info, *next;
    bool slot_found = false;

    /* If the device never made it past the Set Address stage,
     * it may not have the real_port set correctly.
     */
    if (virt_dev->real_port == 0 ||
            virt_dev->real_port > HCS_MAX_PORTS(xhci->hcs_params1))
    {
#ifdef DEBUG
        printf("Bad real port.\n");
#endif
        return;
    }

    tt_list_head = &(xhci->rh_bw[virt_dev->real_port - 1].tts);
    _ux_hcd_xhci_list_for_each_entry_safe(tt_info, next, tt_list_head, tt_list, UX_XHCI_TT_BW_INFO)
    {
        /* Multi-TT hubs will have more than one entry */
        if (tt_info->slot_id == slot_id)
        {
            slot_found = true;
            _ux_hcd_xhci_list_del(&tt_info->tt_list);
            _ux_utility_memory_free(tt_info);
        }
        else if (slot_found)
        {
            break;
        }
    }
}

void _ux_hcd_xhci_update_tt_active_eps(UX_HCD_XHCI *xhci, UX_XHCI_VIRT_DEVICE *virt_dev, int32_t old_active_eps)
{
    UX_XHCI_ROOT_PORT_BW_INFO *rh_bw_info;
    if (!virt_dev->tt_info)
        return;
    rh_bw_info = &xhci->rh_bw[virt_dev->real_port - 1];
    if (old_active_eps == 0 && virt_dev->tt_info->active_eps != 0)
    {
        rh_bw_info->num_active_tts += 1;
        rh_bw_info->bw_table.bw_used += TT_HS_OVERHEAD;
    }
    else if (old_active_eps != 0 && virt_dev->tt_info->active_eps == 0)
    {
        rh_bw_info->num_active_tts -= 1;
        rh_bw_info->bw_table.bw_used -= TT_HS_OVERHEAD;
    }
}

static void _ux_hcd_xhci_free_segments_for_ring(UX_HCD_XHCI *xhci, UX_XHCI_SEGMENT *first)
{
    UX_XHCI_SEGMENT *seg;
    seg = first->next;
    while (seg != first)
    {
        UX_XHCI_SEGMENT *next = seg->next;
        _ux_hcd_xhci_segment_free(xhci, seg);
        seg = next;
    }
    _ux_hcd_xhci_segment_free(xhci, first);
}


void _ux_hcd_xhci_ring_free(UX_HCD_XHCI *xhci, UX_XHCI_RING *ring)
{
    if (!ring)
        return;
    if (ring->first_seg)
    {
        _ux_hcd_xhci_free_segments_for_ring(xhci, ring->first_seg);
    }

    _ux_utility_memory_free(ring);
}

void _ux_hcd_xhci_free_container_ctx(UX_HCD_XHCI *xhci, UX_XHCI_CONTAINER_CTX *ctx)
{
    if (!ctx)
        return;
    _ux_utility_memory_free(ctx->bytes);
    ctx->dma = NULL;
    _ux_utility_memory_free(ctx);
}

void _ux_hcd_xhci_free_command(UX_HCD_XHCI *xhci, UX_XHCI_COMMAND *command)
{

    _ux_hcd_xhci_free_container_ctx(xhci,command->in_ctx);
    _ux_utility_memory_free(command);
}

void _ux_hcd_xhci_free_stream_info(UX_HCD_XHCI *xhci, UX_XHCI_STREAM_INFO *stream_info)
{
    int32_t cur_stream;
    UX_XHCI_RING *cur_ring;

    if (!stream_info)
        return;
    for (cur_stream = 1; cur_stream < stream_info->num_streams; cur_stream++)
    {
        cur_ring = stream_info->stream_rings[cur_stream];
        if (cur_ring)
        {
            _ux_hcd_xhci_ring_free(xhci, cur_ring);
            stream_info->stream_rings[cur_stream] = NULL;
        }
    }
    _ux_hcd_xhci_free_command(xhci, stream_info->free_streams_command);
    xhci->cmd_ring_reserved_trbs--;

    _ux_utility_memory_free(stream_info->stream_rings);
    _ux_utility_memory_free(stream_info);
}

/* All the xhci_tds in the ring's TD list should be freed at this point.
 * Should be called with xhci->lock held if there is any chance the TT lists
 * will be manipulated by the configure endpoint, allocate device, or update
 * hub functions while this function is removing the TT entries from the list.
 */
void _ux_hcd_xhci_free_virt_device(UX_HCD_XHCI *xhci, uint32_t slot_id)
{
    UX_XHCI_VIRT_DEVICE *dev;
    int32_t i;
    int32_t old_active_eps = 0;

    /* Slot ID 0 is reserved */
    if (slot_id == 0 || !xhci->devs[slot_id])
        return;
    dev = xhci->devs[slot_id];
    xhci->dcbaa->dev_context_ptrs[slot_id] = 0;
    if (!dev)
        return;
    if (dev->tt_info)
        old_active_eps = dev->tt_info->active_eps;

    for (i = 0; i < 31; i++)
    {
        if (dev->eps[i].ring)
            _ux_hcd_xhci_ring_free(xhci, dev->eps[i].ring);
        if (dev->eps[i].stream_info)
            _ux_hcd_xhci_free_stream_info(xhci, dev->eps[i].stream_info);
        /* Endpoints on the TT/root port lists should have been removed
         * when usb_disable_device() was called for the device.
         * We can't drop them anyway, because the udev might have gone
         * away by this point, and we can't tell what speed it was.
         */
        if (!_ux_hcd_xhci_list_empty(&dev->eps[i].bw_endpoint_list))
        {
#ifdef DEBUG
            printf("Slot %u endpoint %u ""not removed from BW list!\n",slot_id, i);
#endif
        }
    }
    /* If this is a hub, free the TT(s) from the TT list */
    _ux_hcd_xhci_free_tt_info(xhci, dev, slot_id);
    /* If necessary, update the number of active TTs on this root port */
    _ux_hcd_xhci_update_tt_active_eps(xhci, dev, old_active_eps);

    if (dev->in_ctx)
        _ux_hcd_xhci_free_container_ctx(xhci, dev->in_ctx);
    if (dev->out_ctx)
        _ux_hcd_xhci_free_container_ctx(xhci, dev->out_ctx);

    _ux_utility_memory_free(xhci->devs[slot_id]);
    xhci->devs[slot_id] = NULL;
}

/* Copy output xhci_ep_ctx to the input xhci_ep_ctx copy.
 * Useful when you want to change one particular aspect of the endpoint and then
 * issue a configure endpoint command.
 */
void _ux_hcd_xhci_endpoint_copy(UX_HCD_XHCI *xhci, UX_XHCI_CONTAINER_CTX *in_ctx, UX_XHCI_CONTAINER_CTX *out_ctx,
        uint32_t ep_index)
{
    UX_XHCI_EP_CTX *out_ep_ctx;
    UX_XHCI_EP_CTX *in_ep_ctx;

    out_ep_ctx = _ux_hcd_xhci_get_ep_ctx(xhci, out_ctx, ep_index);
    in_ep_ctx = _ux_hcd_xhci_get_ep_ctx(xhci, in_ctx, ep_index);

    in_ep_ctx->ep_info = out_ep_ctx->ep_info;
    in_ep_ctx->ep_info2 = out_ep_ctx->ep_info2;
    in_ep_ctx->deq = out_ep_ctx->deq;
    in_ep_ctx->tx_info = out_ep_ctx->tx_info;
    in_ep_ctx->reserved[0] = out_ep_ctx->reserved[0];
    in_ep_ctx->reserved[1] = out_ep_ctx->reserved[1];
}

/* Copy output _ux_hcd_xhci_slot_ctx to the input xhci_slot_ctx.
 * Useful when you want to change one particular aspect of the endpoint and then
 * issue a configure endpoint command.  Only the context entries field matters,
 * but we'll copy the whole thing anyway.
 */
void _ux_hcd_xhci_slot_copy(
        UX_HCD_XHCI *xhci,
        UX_XHCI_CONTAINER_CTX *in_ctx,
        UX_XHCI_CONTAINER_CTX *out_ctx)
{
    UX_XHCI_SLOT_CTX *in_slot_ctx;
    UX_XHCI_SLOT_CTX *out_slot_ctx;
    in_slot_ctx = _ux_hcd_xhci_get_slot_ctx(xhci, in_ctx);
    out_slot_ctx = _ux_hcd_xhci_get_slot_ctx(xhci, out_ctx);
    in_slot_ctx->dev_info = out_slot_ctx->dev_info;
    in_slot_ctx->dev_info2 = out_slot_ctx->dev_info2;
    in_slot_ctx->tt_info = out_slot_ctx->tt_info;
    in_slot_ctx->dev_state = out_slot_ctx->dev_state;
}

/*
 * Link the ring to the new segments.
 * Set Toggle Cycle for the new ring if needed.
 */
static void _ux_hcd_xhci_link_rings(UX_HCD_XHCI *xhci, UX_XHCI_RING  *ring,
        UX_XHCI_SEGMENT *first, UX_XHCI_SEGMENT  *last, uint32_t num_segs)
{
    UX_XHCI_SEGMENT  *next;

    if (!ring || !first || !last)
        return;
    next = ring->enq_seg->next;
    _ux_hcd_xhci_link_segments(xhci, ring->enq_seg, first, ring->type);
    _ux_hcd_xhci_link_segments(xhci, last, next, ring->type);
    ring->num_segs += num_segs;
    ring->num_trbs_free += (TRBS_PER_SEGMENT - 1) * num_segs;

    if (ring->type != TYPE_EVENT && ring->enq_seg == ring->last_seg)
    {
        ring->last_seg->trbs[TRBS_PER_SEGMENT-1].link.control &= ~(LINK_TOGGLE);
        last->trbs[TRBS_PER_SEGMENT-1].link.control |= (LINK_TOGGLE);
        ring->last_seg = last;
    }
}


/*
 * Expand an existing ring.
 * Allocate a new ring which has same segment numbers and link the two rings.
 */
int32_t _ux_hcd_xhci_ring_expansion(UX_HCD_XHCI *xhci, UX_XHCI_RING *ring, uint32_t num_trbs)
{
    UX_XHCI_SEGMENT   *first;
    UX_XHCI_SEGMENT   *last;
    uint32_t num_segs;
    uint32_t num_segs_needed;
    int32_t ret;
    num_segs_needed = (num_trbs + (TRBS_PER_SEGMENT - 1) - 1) /(TRBS_PER_SEGMENT - 1);

    /* Allocate number of segments we needed, or double the ring size */
    num_segs = ring->num_segs > num_segs_needed ? ring->num_segs : num_segs_needed;

    ret = _ux_hcd_xhci_alloc_segments_for_ring(xhci, &first, &last,
            num_segs, ring->cycle_state, ring->type, ring->bounce_buf_len);
    if (ret)
        return -1;

    _ux_hcd_xhci_link_rings(xhci, ring, first, last, num_segs);
#ifdef DEBUG
    printf("ring expansion succeed, now has %d segments",ring->num_segs);
#endif
    return 0;
}

static uint32_t _ux_hcd_xhci_get_endpoint_type(UX_ENDPOINT *ep)
{
    int32_t in;
    in = (ep->ux_endpoint_descriptor.bEndpointAddress & UX_ENDPOINT_DIRECTION);

    switch (ep->ux_endpoint_descriptor.bmAttributes.xfer)
    {
        case UX_CONTROL_ENDPOINT:
            return CTRL_EP;
        case UX_BULK_ENDPOINT:
            return in ? BULK_IN_EP : BULK_OUT_EP;
        case UX_ISOCHRONOUS_ENDPOINT:
            return in ? ISOC_IN_EP : ISOC_OUT_EP;
        case UX_INTERRUPT_ENDPOINT:
            return in ? INT_IN_EP : INT_OUT_EP;
    }
    return 0;
}

/*
 * Convert bInterval expressed in microframes (in 1-255 range) to exponent of
 * microframes, rounded down to nearest power of 2.
 */
uint32_t clamp_val (uint32_t val, uint32_t min, uint32_t max)
{
    if (val < min)
        return min;
    else if (val > max)
        return max;
    else
        return val;
}
/*
 * Convert interval expressed as 2^(bInterval - 1) == interval into
 * straight exponent value 2^n == interval.
 *
 */
static uint32_t _ux_hcd_xhci_parse_exponent_interval(UX_DEVICE *udev, UX_ENDPOINT *ep)
{
    int32_t interval;
    interval = clamp_val(ep->ux_endpoint_descriptor.bInterval, 1, 16) - 1;
    if (udev->ux_device_speed == UX_FULL_SPEED_DEVICE)
    {
        interval += 3;
    }
    return interval;
}

static uint32_t _ux_hcd_xhci_microframes_to_exponent(
        UX_DEVICE *udev,
        UX_ENDPOINT *ep,
        uint32_t desc_interval,
        uint32_t min_exponent,
        uint32_t max_exponent)
{
    uint32_t interval;
    interval = desc_interval;
    interval = clamp_val(interval, min_exponent, max_exponent);
    if ((1 << interval) != desc_interval)
    {
#ifdef DEBUG
        printf("rounding interval to %d microframes, ep desc says %d microframes\n",
                1 << interval, desc_interval);
#endif
    }
    return interval;
}

static uint32_t _ux_hcd_xhci_parse_microframe_interval(UX_DEVICE *udev,
        UX_ENDPOINT *ep)
{
    if (ep->ux_endpoint_descriptor.bInterval == 0)
        return 0;
    return _ux_hcd_xhci_microframes_to_exponent(udev, ep,
            ep->ux_endpoint_descriptor.bInterval, 0, 15);
}


static uint32_t _ux_hcd_xhci_parse_frame_interval(UX_DEVICE *udev, UX_ENDPOINT *ep)
{
    return _ux_hcd_xhci_microframes_to_exponent(udev, ep,
            ep->ux_endpoint_descriptor.bInterval * 8, 3, 10);
}

/* Return the polling or NAK interval.
 *
 * The polling interval is expressed in "microframes".  If xHCI's Interval field
 * is set to N, it will service the endpoint every 2^(Interval)*125us.
 *
 * The NAK interval is one NAK per 1 to 255 microframes, or no NAKs if interval
 * is set to 0.
 */
static uint32_t _ux_hcd_xhci_get_endpoint_interval(UX_DEVICE *udev,
        UX_ENDPOINT *ep)
{
    uint32_t interval = 0;
    switch (udev->ux_device_speed)
    {
        case UX_HIGH_SPEED_DEVICE:
            /* Max NAK rate */
            if (ux_endpoint_xfer_control(&ep->ux_endpoint_descriptor) ||
                    ux_endpoint_xfer_bulk(&ep->ux_endpoint_descriptor))
            {
                interval = _ux_hcd_xhci_parse_microframe_interval(udev, ep);
            }
            break;

        case UX_FULL_SPEED_DEVICE:
            if (ux_endpoint_xfer_isoc(&ep->ux_endpoint_descriptor))
            {
                interval = _ux_hcd_xhci_parse_exponent_interval(udev, ep);
            }
            break;

        case UX_LOW_SPEED_DEVICE:
            if (ux_endpoint_xfer_int(&ep->ux_endpoint_descriptor) ||
                    ux_endpoint_xfer_isoc(&ep->ux_endpoint_descriptor))
            {
                interval = _ux_hcd_xhci_parse_frame_interval(udev, ep);
            }
            break;
        default:
#ifdef DEBUG
            printf("WARN: speed error %lu\n",udev->ux_device_speed);
#endif
            break;
    }
    return interval;
}

static uint32_t _ux_hcd_xhci_get_endpoint_max_burst(UX_DEVICE *udev, UX_ENDPOINT *ep)
{
    if (udev->ux_device_speed == UX_HIGH_SPEED_DEVICE &&
            (ux_endpoint_xfer_isoc(&ep->ux_endpoint_descriptor) ||
             ux_endpoint_xfer_int(&ep->ux_endpoint_descriptor)))
    {

        return ux_endpoint_maxp_mult(&ep->ux_endpoint_descriptor) - 1;
    }
    return 0;
}
/* Return the maximum endpoint service interval time (ESIT) payload.
 * Basically, this is the maxpacket size, multiplied by the burst size
 * and mult size.
 */

uint32_t xhci_get_max_esit_payload(UX_DEVICE *udev,UX_ENDPOINT *ep)
{
    int32_t max_burst;
    int32_t max_packet;

    /* Only applies for interrupt or isochronous endpoints */
    if (ux_endpoint_xfer_control(&ep->ux_endpoint_descriptor) ||
            ux_endpoint_xfer_bulk(&ep->ux_endpoint_descriptor))
        return 0;
    /* Get the endpoint max packet size  */
    max_packet = ux_endpoint_maxp(&ep->ux_endpoint_descriptor);
    max_burst = ux_endpoint_maxp_mult(&ep->ux_endpoint_descriptor);
    /* A 0 in max burst means 1 transfer per ESIT */
    return max_packet * max_burst;
}
/* Set up an endpoint with one ring segment. */
int32_t _ux_hcd_xhci_endpoint_init(UX_HCD_XHCI *xhci, UX_XHCI_VIRT_DEVICE *virt_dev,
        UX_DEVICE *udev, UX_ENDPOINT *ep)
{
    uint32_t ep_index;
    UX_XHCI_EP_CTX *ep_ctx;
    UX_XHCI_RING *ep_ring;
    uint32_t max_packet;
    UX_XHCI_RING_TYPE ring_type;
    uint32_t max_esit_payload;
    uint32_t endpoint_type;
    uint32_t max_burst;
    uint32_t interval;
    uint32_t mult;
    uint32_t avg_trb_len;
    uint32_t err_count = 0;
    /* Get the endpoint index    */
    ep_index = _ux_hcd_xhci_get_endpoint_index(&ep->ux_endpoint_descriptor);
    /* Get the endpoint context  */
    ep_ctx = _ux_hcd_xhci_get_ep_ctx(xhci, virt_dev->in_ctx, ep_index);
    /* Get the endpoint type  */
    endpoint_type = _ux_hcd_xhci_get_endpoint_type(ep);
    if (!endpoint_type)
    {
#ifdef DEBUG
        printf("invalid endpoint type\n");
#endif
        return -1;
    }
    ring_type = ep->ux_endpoint_descriptor.bmAttributes.xfer;

    /* Get the max endpoint service interval time  */
    max_esit_payload = xhci_get_max_esit_payload(udev, ep);
    /* Get the endpoint interval  */
    interval = _ux_hcd_xhci_get_endpoint_interval(udev, ep);

    if (ux_endpoint_xfer_int(&ep->ux_endpoint_descriptor) ||
            ux_endpoint_xfer_isoc(&ep->ux_endpoint_descriptor))
    {
        if ( udev->ux_device_speed >= UX_HIGH_SPEED_DEVICE && interval >= 7)
        {
            interval = 6;
        }
    }
    /* mult should be zero for High speed Devices   */
    mult = 0;
    /* Get the endpoint max packet size */
    max_packet = ux_endpoint_maxp(&ep->ux_endpoint_descriptor);
    /* Get the endpoint max burst size, should be applicable only Isoc and interrupt endpoints  */
    max_burst = _ux_hcd_xhci_get_endpoint_max_burst(udev, ep);
    avg_trb_len = max_esit_payload;

    /* Allow 3 retries for everything but isoc, set CErr = 3 */
    if (!ux_endpoint_xfer_isoc(&ep->ux_endpoint_descriptor))
        err_count = 3;
    /* HS bulk max packet should be 512, FS bulk supports 8, 16, 32 or 64 */
    if (ux_endpoint_xfer_bulk(&ep->ux_endpoint_descriptor))
    {
        if (udev->ux_device_speed == UX_HIGH_SPEED_DEVICE)
            max_packet = 512;
        if (udev->ux_device_speed == UX_FULL_SPEED_DEVICE)
        {
            max_packet = max_packet * max_packet;
            max_packet = clamp_val(max_packet, 8, 64);
        }
    }
    /* xHCI 1.0 and 1.1 indicates that ctrl ep avg TRB Length should be 8 */
    if (ux_endpoint_xfer_control(&ep->ux_endpoint_descriptor) && xhci->hci_version >= 0x100)
        avg_trb_len = 8;
    /* xhci 1.1 with LEC support doesn't use mult field, use RsvdZ */
    if ((xhci->hci_version > 0x100) && HCC2_LEC(xhci->hcc_params2))
        mult = 0;

    /* Set up the endpoint ring */
    virt_dev->eps[ep_index].new_ring = _ux_hcd_xhci_ring_alloc(xhci, 2/* no of seg FIXME */, 1, ring_type, max_packet);
    if (!virt_dev->eps[ep_index].new_ring)
    {
#ifdef DEBUG
        printf("Ring allocation failed\n");
#endif
        return -1;
    }

    virt_dev->eps[ep_index].skip = false;
    ep_ring = virt_dev->eps[ep_index].new_ring;

    /* Fill the endpoint context array   */
    ep_ctx->ep_info = (EP_MAX_ESIT_PAYLOAD_HI(max_esit_payload) |
            EP_INTERVAL(interval) | EP_MULT(mult));

    ep_ctx->ep_info2 = (EP_TYPE(endpoint_type) | MAX_PACKET(max_packet) |
            MAX_BURST(max_burst) | ERROR_COUNT(err_count));

    ep_ctx->deq = (ep_ring->first_seg->dma |
            ep_ring->cycle_state);

    ep_ctx->tx_info = (EP_MAX_ESIT_PAYLOAD_LO(max_esit_payload) |
            EP_AVG_TRB_LENGTH(avg_trb_len));
    return 0;
}

int32_t _ux_hcd_xhci_alloc_virt_device(UX_HCD_XHCI *xhci, uint32_t slot_id, UX_DEVICE *udev)
{
    UX_XHCI_VIRT_DEVICE *dev;
    int32_t i;

    /* Slot ID 0 is reserved */
    if (slot_id == 0 || xhci->devs[slot_id])
    {
#ifdef DEBUG
        printf("Bad Slot ID %d\n", slot_id);
#endif
        return 0;
    }

    dev = _ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY, sizeof(*dev));
    if (!dev)
    {
#ifdef DEBUG
        printf("dev Allocation failed\n");
#endif
        return 0;
    }

    /* Allocate the (output) device context that will be used in the HC. */
    dev->out_ctx = _ux_hcd_xhci_alloc_container_ctx(xhci, UX_XHCI_CTX_TYPE_DEVICE);
    if (!dev->out_ctx)
        goto fail;

    /* Allocate the (input) device context for address device command */
    dev->in_ctx = _ux_hcd_xhci_alloc_container_ctx(xhci, UX_XHCI_CTX_TYPE_INPUT);
    if (!dev->in_ctx)
        goto fail;

    /* Initialize the cancellation list for each ep */
    for (i = 0; i < 31; i++)
    {
        INIT_LIST_HEAD(&dev->eps[i].cancelled_td_list);
        INIT_LIST_HEAD(&dev->eps[i].bw_endpoint_list);
    }

    /* Allocate endpoint 0 ring */
    dev->eps[0].ring = _ux_hcd_xhci_ring_alloc(xhci, 2, 1, TYPE_CTRL, 0);
    if (!dev->eps[0].ring)
        goto fail;

    dev->udev = udev;

    /* Point to output device context in dcbaa. */
    xhci->dcbaa->dev_context_ptrs[slot_id] = (dev->out_ctx->dma);
    xhci->devs[slot_id] = dev;
    return 1;
fail:

    if (dev->in_ctx)
        _ux_hcd_xhci_free_container_ctx(xhci, dev->in_ctx);
    if (dev->out_ctx)
        _ux_hcd_xhci_free_container_ctx(xhci, dev->out_ctx);
    _ux_utility_memory_free(dev);
    return 0;
}

static bool _ux_hcd_xhci_is_async_ep(unsigned int ep_type)
{
    return (ep_type != UX_ISOCHRONOUS_ENDPOINT_OUT &&
            ep_type != UX_INTERRUPT_ENDPOINT_OUT &&
            ep_type != UX_ISOCHRONOUS_ENDPOINT_IN &&
            ep_type != UX_INTERRUPT_ENDPOINT_IN);
}

void _ux_hcd_xhci_free_endpoint_ring(UX_HCD_XHCI *xhci, UX_XHCI_VIRT_DEVICE *virt_dev, uint32_t ep_index)
{
    _ux_hcd_xhci_ring_free(xhci, virt_dev->eps[ep_index].ring);
    virt_dev->eps[ep_index].ring = NULL;
}

static void _ux_hcd_xhci_drop_ep_from_interval_table(
        UX_HCD_XHCI *xhci, UX_XHCI_BW_INFO *ep_bw, UX_XHCI_INTERVAL_BW_TABLE *bw_table,
        UX_DEVICE *udev, UX_XHCI_VIRT_EP *virt_ep, UX_XHCI_TT_BW_INFO *tt_info)
{
    UX_XHCI_INTERVAL_BW *interval_bw;
    int32_t normalized_interval;

    if (_ux_hcd_xhci_is_async_ep(ep_bw->type))
        return;
    /* SuperSpeed endpoints never get added to intervals in the table, so
     * this check is only valid for HS/FS/LS devices.
     */
    if (_ux_hcd_xhci_list_empty(&virt_ep->bw_endpoint_list))
        return;
    /* For LS/FS devices, we need to translate the interval expressed in
     * microframes to frames.
     */
    if (udev->ux_device_speed == UX_HIGH_SPEED_DEVICE)
        normalized_interval = ep_bw->ep_interval;
    else
        normalized_interval = ep_bw->ep_interval - 3;

    if (normalized_interval == 0)
        bw_table->interval0_esit_payload -= ep_bw->max_esit_payload;
    interval_bw = &bw_table->interval_bw[normalized_interval];
    interval_bw->num_packets -= ep_bw->num_packets;
    switch (udev->ux_device_speed)
    {
        case UX_LOW_SPEED_DEVICE:
            interval_bw->overhead[LS_OVERHEAD_TYPE] -= 1;
            break;
        case UX_FULL_SPEED_DEVICE:
            interval_bw->overhead[FS_OVERHEAD_TYPE] -= 1;
            break;
        case UX_HIGH_SPEED_DEVICE:
            interval_bw->overhead[HS_OVERHEAD_TYPE] -= 1;
            break;
        default:
            return;
    }
    if (tt_info)
        tt_info->active_eps -= 1;
    _ux_hcd_xhci_list_del_init(&virt_ep->bw_endpoint_list);
}

void _ux_hcd_xhci_clear_endpoint_bw_info(UX_XHCI_BW_INFO *bw_info)
{
    bw_info->ep_interval = 0;
    bw_info->mult = 0;
    bw_info->num_packets = 0;
    bw_info->max_packet_size = 0;
    bw_info->type = 0;
    bw_info->max_esit_payload = 0;
}
/*
 * This submits a Reset Device Command, which will set the device state to 0,
 * set the device address to 0, and disable all the endpoints except the default
 * control endpoint.  The USB core should come back and call
 * xhci_address_device(), and then re-set up the configuration.  If this is
 * called because of a usb_reset_and_verify_device(), then the old alternate
 * settings will be re-installed through the normal bandwidth allocation
 * functions.
 *
 * Wait for the Reset Device command to finish.  Remove all structures
 * associated with the endpoints that were disabled.  Clear the input device
 * structure? Reset the control endpoint 0 max packet size?
 *
 * If the virt_dev to be reset does not exist or does not match the udev,
 * it means the device is lost, possibly due to the xHC restore error and
 * re-initialization  In this case, call xhci_alloc_dev() to
 * re-allocate the device.
 */
int32_t _ux_hcd_xhci_reset_device(UX_HCD_XHCI *xhci, UX_DEVICE *udev)
{
    int32_t ret, i;
    uint32_t slot_id;
    UX_XHCI_VIRT_DEVICE *virt_dev;
    UX_XHCI_COMMAND *reset_device_cmd;
    UX_XHCI_SLOT_CTX *slot_ctx;
    int32_t old_active_eps = 0;
    int32_t status;

    if (!xhci || !udev)
        return 0;

    slot_id = xhci->slot_id;
    virt_dev = xhci->devs[slot_id];
    if (!virt_dev)
    {
#ifdef DEBUG
        printf("The device to be reset with slot ID %u does "
                "not exist. Re-allocate the device\n", slot_id);
#endif
        ret = _ux_hcd_xhci_alloc_dev(xhci, udev);
        if (ret == 1)
            return 0;
        else
            return -1;
    }

    if (virt_dev->tt_info)
        old_active_eps = virt_dev->tt_info->active_eps;

    if (virt_dev->udev != udev)
    {
        /* If the virt_dev and the udev does not match, this virt_dev
         * may belong to another udev.
         * Re-allocate the device.
         */
#ifdef DEBUG
        printf("The device to be reset with slot ID %u does "
                "not match the udev. Re-allocate the device\n",slot_id);
#endif
        ret = _ux_hcd_xhci_alloc_dev(xhci, udev);
        if (ret == 1)
            return 0;
        else
            return -1;
    }

    /* If device is not setup, there is no point in resetting it */
    slot_ctx = _ux_hcd_xhci_get_slot_ctx(xhci, virt_dev->out_ctx);
    if (GET_SLOT_STATE((slot_ctx->dev_state)) == SLOT_STATE_DISABLED)
        return 0;
    /* allocate the command */
    reset_device_cmd = _ux_hcd_xhci_alloc_command(xhci);
    if (!reset_device_cmd)
    {
#ifdef DEBUG
        printf("Couldn't allocate command structure.\n");
#endif
        return -1;
    }

    /* Attempt to submit the Reset Device command to the command ring */
    reset_device_cmd->fCompletion = 1;
    /* queue the reset device command trb  */
    ret = _ux_hcd_xhci_queue_reset_device(xhci, reset_device_cmd, slot_id);
    if (ret)
        goto command_cleanup;

    /* Ring the HC doorbell for command ring   */
    _ux_hcd_xhci_ring_cmd_db(xhci);
    /* Wait for the command completion */
    while(reset_device_cmd->fCompletion);

    /* The Reset Device command can't fail, according to the 0.95/0.96 spec,
     * unless we tried to reset a slot ID that wasn't enabled,
     * or the device wasn't in the addressed or configured state.
     */
    ret = reset_device_cmd->status;
    switch (ret)
    {
        case COMP_COMMAND_ABORTED:
        case COMP_COMMAND_RING_STOPPED:
#ifdef DEBUG
            printf("Timeout waiting for reset device command\n");
#endif
            ret = -1;
            goto command_cleanup;
        case COMP_SLOT_NOT_ENABLED_ERROR:
        case COMP_CONTEXT_STATE_ERROR:
            ret = 0;
            goto command_cleanup;
        case COMP_SUCCESS:
#ifdef DEBUG
            printf("Successful reset device command.\n");
#endif
            break;
        default:
            if (_ux_hcd_xhci_is_vendor_info_code(xhci, ret))
                break;
#ifdef DEBUG
            printf("Unknown completion code %u for " "reset device command.\n", ret);
#endif
            ret = -1;
            goto command_cleanup;
    }
    /* Everything but endpoint 0 is disabled, so free the rings. */
    for (i = 1; i < 31; i++)
    {
        UX_XHCI_VIRT_EP *ep = &virt_dev->eps[i];
        if (ep->ep_state & EP_HAS_STREAMS)
        {
#ifdef DEBUG
            printf("WARN: endpoint has streams on device reset, freeing streams.\n");
#endif
            _ux_hcd_xhci_free_stream_info(xhci, ep->stream_info);
            ep->stream_info = NULL;
            ep->ep_state &= ~EP_HAS_STREAMS;
        }
        if (ep->ring)
        {
            _ux_hcd_xhci_free_endpoint_ring(xhci, virt_dev, i);
        }
        if (!_ux_hcd_xhci_list_empty(&virt_dev->eps[i].bw_endpoint_list))
            _ux_hcd_xhci_drop_ep_from_interval_table(xhci, &virt_dev->eps[i].bw_info, virt_dev->bw_table,
                    udev, &virt_dev->eps[i], virt_dev->tt_info);
        _ux_hcd_xhci_clear_endpoint_bw_info(&virt_dev->eps[i].bw_info);
    }
    /* If necessary, update the number of active TTs on this root port */
    _ux_hcd_xhci_update_tt_active_eps(xhci, virt_dev, old_active_eps);
    virt_dev->flags = 0;
    ret = 0;

command_cleanup:
    _ux_hcd_xhci_free_command(xhci, reset_device_cmd);
    return ret;
}

#endif //CFG_TUH_ENABLED
