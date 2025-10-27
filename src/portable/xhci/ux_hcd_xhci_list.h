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
 * @file     ux_hcd_xhci_list.h
 * @author   Mahesh Avula
 * @email    mahesh.avula@alifsemi.com
 * @version  V1.0.0
 * @date     12-Dec-2023
 * @brief    header file Link List for XHCI.
 * @bug      None.
 * @Note     None
 ******************************************************************************/

#ifndef UX_HCD_XHCI_LIST_H
#define UX_HCD_XHCI_LIST_H

#include <stdbool.h>
#include "ux_hcd_xhci.h"

#ifdef   __cplusplus

/* Yes, C++ compiler is present.  Use standard C.  */
extern   "C" {

#endif

typedef struct UX_HCD_XHCI_LIST_STRUCT
{
    struct UX_HCD_XHCI_LIST_STRUCT   *next;
    struct UX_HCD_XHCI_LIST_STRUCT   *prev;
}UX_HCD_XHCI_LIST;

/**
 * Initialize the list as an empty list.
 * @param The list to initialized.
 */
#define UX_HCD_XHCI_LIST_INIT(name) { &(name), &(name) }

#define UX_HCD_XHCI_LIST_HEAD(name) \
   UX_HCD_XHCI_LIST name = LIST_HEAD_INIT(name)

static inline void INIT_LIST_HEAD(UX_HCD_XHCI_LIST *list)
{
    list->next = list->prev = list;
}

static inline void list_add(UX_HCD_XHCI_LIST *entry,
                UX_HCD_XHCI_LIST *prev, UX_HCD_XHCI_LIST *next)
{
    next->prev = entry;
    entry->next = next;
    entry->prev = prev;
    prev->next = entry;
}

/**
 * Insert a new element after the given list head. The new element does not
 * need to be initialised as empty list.
 * The list changes from:
 *      head ? some element ? ...
 * to
 *      head ? new element ? older element ? ...
 *
 * @param entry The new element to prepend to the list.
 * @param head The existing list.
 */
static inline void
_ux_hcd_xhci_list_add(UX_HCD_XHCI_LIST *entry, UX_HCD_XHCI_LIST *head)
{
    list_add(entry, head, head->next);
}

/**
 * Append a new element to the end of the list given with this list head.
 *
 * The list changes from:
 *      head ? some element ? ... ? lastelement
 * to
 *      head ? some element ? ... ? lastelement ? new element
 * @param entry The new element to prepend to the list.
 * @param head The existing list.
 */
static inline void
_ux_hcd_xhci_list_add_tail(UX_HCD_XHCI_LIST *entry, UX_HCD_XHCI_LIST *head)
{
    list_add(entry, head->prev, head);
}

static inline void list_del(UX_HCD_XHCI_LIST *prev, UX_HCD_XHCI_LIST *next)
{
    next->prev = prev;
    prev->next = next;
}

/**
 * Remove the element from the list it is in. Using this function will reset
 * the pointers to/from this element so it is removed from the list. It does
 * NOT free the element itself or manipulate it otherwise.
 *
 * Using list_del on a pure list head (like in the example at the top of
 * this file) will NOT remove the first element from
 * the list but rather reset the list as empty list.
 *
 *
 * @param entry The element to remove.
 */
static inline void _ux_hcd_xhci_list_del(UX_HCD_XHCI_LIST *entry)
{
    list_del(entry->prev, entry->next);
}

static inline void _ux_hcd_xhci_list_del_init(UX_HCD_XHCI_LIST *entry)
{
    list_del(entry->prev, entry->next);
    INIT_LIST_HEAD(entry);
}

/**
 * Check if the list is empty.
 * @return True if the list contains one or more elements or False otherwise.
 */
static inline bool
_ux_hcd_xhci_list_empty(UX_HCD_XHCI_LIST *head)
{
    return head->next == head;
}

static inline int
_ux_hcd_xhci_list_is_singular(UX_HCD_XHCI_LIST *head)
{
   return !_ux_hcd_xhci_list_empty(head) && (head->next == head->prev);
}

/**
 * Returns a pointer to the container of this list element.
 *
 * @param ptr Pointer to the UX_HCD_XHCI_LIST.
 * @param type Data type of the list element.
 * @param member Member name of the UX_HCD_XHCI_LIST field in the list element.
 * @return A pointer to the data struct containing the list head.
 */

#ifndef container_of
#define container_of(ptr, type, member) \
    (type *)((char *)(ptr) - (char *) &((type *)0)->member)
#endif

/**
 * Alias of container_of
 */
#define _ux_hcd_xhci_list_entry(ptr, type, member) \
    container_of(ptr, type, member)


#define _ux_hcd_xhci_list_for_each(pos, head) \
     for (pos = (head)->next; pos != (head); pos = pos->next)

/**
 * Retrieve the first list entry for the given list pointer.

 * @param ptr The list head
 * @param type Data type of the list element to retrieve
 * @param member Member name of the UX_HCD_XHCI_LIST field in the list element.
 * @return A pointer to the first list element.
 */
#define _ux_hcd_xhci_list_first_entry(ptr, type, member) \
    _ux_hcd_xhci_list_entry((ptr)->next, type, member)

/**
 * Retrieve the last list entry for the given listpointer.
 *
 * @param ptr The list head
 * @param type Data type of the list element to retrieve
 * @param member Member name of the UX_HCD_XHCI_LIST field in the list element.
 * @return A pointer to the last list element.
 */
#define _ux_hcd_xhci_list_last_entry(ptr, type, member) \
    _ux_hcd_xhci_list_entry((ptr)->prev, type, member)


/**
 *
 * This macro is not safe for node deletion. Use list_for_each_entry_safe
 * instead.
 *
 * @param pos Iterator variable of the type of the list elements.
 * @param head List head
 * @param member Member name of the UX_HCD_XHCI_LIST in the list elements.
 *
 */
#define _ux_hcd_xhci_list_for_each_entry(pos, head, member, type)     \
    for (pos = container_of((head)->next, type, member);      \
    &pos->member != (head);                                   \
    pos = container_of(pos->member.next, type, member))

/**
 * Loop through the list, keeping a backup pointer to the element. This
 * macro allows for the deletion of a list element while looping through the
 * list.
 *
 * See list_for_each_entry for more details.
 */
#define _ux_hcd_xhci_list_for_each_entry_safe(pos, tmp, head, member, type)  \
    for (pos = container_of((head)->next, type, member),                     \
    tmp = container_of(pos->member.next, type, member);                     \
    &pos->member != (head);                                                 \
    pos = tmp, tmp = container_of(pos->member.next, type, member))


#ifdef  __cplusplus
}
#endif
#endif

