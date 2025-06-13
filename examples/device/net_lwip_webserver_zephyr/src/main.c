/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2020 Peter Lawrence
 *
 * influenced by lrndis https://github.com/fetisov/lrndis
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

/*
this appears as either a RNDIS or CDC-ECM USB virtual network adapter; the OS picks its preference

RNDIS should be valid on Linux and Windows hosts, and CDC-ECM should be valid on Linux and macOS hosts

The MCU appears to the host as IP address 192.168.7.1, and provides a DHCP server, DNS server, and web server.
*/
/*
Some smartphones *may* work with this implementation as well, but likely have limited (broken) drivers,
and likely their manufacturer has not tested such functionality.  Some code workarounds could be tried:

The smartphone may only have an ECM driver, but refuse to automatically pick ECM (unlike the OSes above);
try modifying ./examples/devices/net_lwip_webserver/usb_descriptors.c so that CONFIG_ID_ECM is default.

The smartphone may be artificially picky about which Ethernet MAC address to recognize; if this happens,
try changing the first byte of tud_network_mac_address[] below from 0x02 to 0x00 (clearing bit 1).
*/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "bsp/board_api.h"
#include "tusb.h"

#include "dhserver.h"
#include "dnserver.h"
#include "httpd.h"
#include "lwip/ethip6.h"
#include "lwip/init.h"
#include "lwip/timeouts.h"

#ifdef INCLUDE_IPERF
  #include "lwip/apps/lwiperf.h"
#endif


#define USBD_STACK_SIZE     (4 * 1024)
#define BLINKY_STACK_SIZE   (2 * 1024)
#define LWIP_STACK_SIZE     (2 * 1024)

#define INIT_IP4(a, b, c, d) \
  { PP_HTONL(LWIP_MAKEU32(a, b, c, d)) }

static K_THREAD_STACK_DEFINE(usb_device_stack, USBD_STACK_SIZE);
static struct k_thread usb_device_taskdef;

static K_THREAD_STACK_DEFINE(led_blinking_stack, BLINKY_STACK_SIZE);
static struct k_thread led_blinking_taskdef;

static K_THREAD_STACK_DEFINE(lwip_stack, LWIP_STACK_SIZE);
static struct k_thread lwip_taskdef;

void usb_device_task(void *p1, void *p2, void *p3);
void lwip_task(void *p1, void *p2, void *p3);
void led_blinking_task(void *p1, void *p2, void *p3);

/* lwip context */
static struct netif netif_data;

/* shared between tud_network_recv_cb() and service_traffic() */
static struct pbuf *received_frame;

/* this is used by this code, ./class/net/net_driver.c, and usb_descriptors.c */
/* ideally speaking, this should be generated from the hardware's unique ID (if available) */
/* it is suggested that the first byte is 0x02 to indicate a link-local address */
uint8_t tud_network_mac_address[6] = {0x02, 0x02, 0x84, 0x6A, 0x96, 0x00};

/* network parameters of this MCU */
static const ip4_addr_t ipaddr = INIT_IP4(192, 168, 7, 1);
static const ip4_addr_t netmask = INIT_IP4(255, 255, 255, 0);
static const ip4_addr_t gateway = INIT_IP4(0, 0, 0, 0);

/* database IP addresses that can be offered to the host; this must be in RAM to store assigned MAC addresses */
static dhcp_entry_t entries[] = {
    /* mac ip address               lease time */
    {{0}, INIT_IP4(192, 168, 7, 2), 24 * 60 * 60},
    {{0}, INIT_IP4(192, 168, 7, 3), 24 * 60 * 60},
    {{0}, INIT_IP4(192, 168, 7, 4), 24 * 60 * 60},
};

static const dhcp_config_t dhcp_config = {
    .router = INIT_IP4(0, 0, 0, 0),  /* router address (if any) */
    .port = 67,                      /* listen port */
    .dns = INIT_IP4(192, 168, 7, 1), /* dns server (if any) */
    "usb",                           /* dns suffix */
    TU_ARRAY_SIZE(entries),          /* num entry */
    entries                          /* entries */
};

static err_t linkoutput_fn(struct netif *netif, struct pbuf *p) {
  (void) netif;

  for (;;) {
    /* if TinyUSB isn't ready, we must signal back to lwip that there is nothing we can do */
    if (!tud_ready())
      return ERR_USE;

    /* if the network driver can accept another packet, we make it happen */
    if (tud_network_can_xmit(p->tot_len)) {
      tud_network_xmit(p, 0 /* unused for this example */);
      return ERR_OK;
    }

    /* transfer execution to TinyUSB in the hopes that it will finish transmitting the prior packet */
    k_msleep( 1 );
  }
}

static err_t ip4_output_fn(struct netif *netif, struct pbuf *p, const ip4_addr_t *addr) {
  return etharp_output(netif, p, addr);
}

#if LWIP_IPV6
static err_t ip6_output_fn(struct netif *netif, struct pbuf *p, const ip6_addr_t *addr) {
  return ethip6_output(netif, p, addr);
}
#endif

static err_t netif_init_cb(struct netif *netif) {
  LWIP_ASSERT("netif != NULL", (netif != NULL));
  netif->mtu = CFG_TUD_NET_MTU;
  netif->flags = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_LINK_UP | NETIF_FLAG_UP;
  netif->state = NULL;
  netif->name[0] = 'E';
  netif->name[1] = 'X';
  netif->linkoutput = linkoutput_fn;
  netif->output = ip4_output_fn;
#if LWIP_IPV6
  netif->output_ip6 = ip6_output_fn;
#endif
  return ERR_OK;
}

static void init_lwip(void) {
  struct netif *netif = &netif_data;

  lwip_init();

  /* the lwip virtual MAC address must be different from the host's; to ensure this, we toggle the LSbit */
  netif->hwaddr_len = sizeof(tud_network_mac_address);
  memcpy(netif->hwaddr, tud_network_mac_address, sizeof(tud_network_mac_address));
  netif->hwaddr[5] ^= 0x01;

  netif = netif_add(netif, &ipaddr, &netmask, &gateway, NULL, netif_init_cb, ip_input);
#if LWIP_IPV6
  netif_create_ip6_linklocal_address(netif, 1);
#endif
  netif_set_default(netif);
}

/* handle any DNS requests from dns-server */
bool dns_query_proc(const char *name, ip4_addr_t *addr) {
  if (0 == strcmp(name, "tiny.usb")) {
    *addr = ipaddr;
    return true;
  }
  return false;
}

bool tud_network_recv_cb(const uint8_t *src, uint16_t size) {
  /* this shouldn't happen, but if we get another packet before
  parsing the previous, we must signal our inability to accept it */
  if (received_frame) return false;

  if (size) {
    struct pbuf *p = pbuf_alloc(PBUF_RAW, size, PBUF_POOL);

    if (p) {
      /* pbuf_alloc() has already initialized struct; all we need to do is copy the data */
      memcpy(p->payload, src, size);

      /* store away the pointer for service_traffic() to later handle */
      received_frame = p;
    }
  }

  return true;
}

uint16_t tud_network_xmit_cb(uint8_t *dst, void *ref, uint16_t arg) {
  struct pbuf *p = (struct pbuf *) ref;

  (void) arg; /* unused for this example */

  return pbuf_copy_partial(p, dst, p->tot_len, 0);
}

static void service_traffic(void) {
  /* handle any packet received by tud_network_recv_cb() */
  if (received_frame) {
    // Surrender ownership of our pbuf unless there was an error
    // Only call pbuf_free if not Ok else it will panic with "pbuf_free: p->ref > 0"
    // or steal it from whatever took ownership of it with undefined consequences.
    // See: https://savannah.nongnu.org/patch/index.php?10121
    if (ethernet_input(received_frame, &netif_data)!=ERR_OK) {
        pbuf_free(received_frame);
    }
    received_frame = NULL;
    tud_network_recv_renew();
  }

  sys_check_timeouts();
}

void tud_network_init_cb(void) {
  /* if the network is re-initializing and we have a leftover packet, we must do a cleanup */
  if (received_frame) {
    pbuf_free(received_frame);
    received_frame = NULL;
  }
}

int main(void) {
  /* initialize TinyUSB */
  board_init();

  // Create task for: tinyusb, lwip, blinky

  k_thread_create(&usb_device_taskdef, usb_device_stack, USBD_STACK_SIZE,
                  usb_device_task, NULL, NULL, NULL,
                  K_HIGHEST_APPLICATION_THREAD_PRIO-1, 0, K_NO_WAIT);
  k_thread_name_set(&usb_device_taskdef, "usbd");

  k_thread_create(&lwip_taskdef, usb_device_stack, LWIP_STACK_SIZE,
                  lwip_task, NULL, NULL, NULL,
                  K_HIGHEST_APPLICATION_THREAD_PRIO-2, 0, K_NO_WAIT);
  k_thread_name_set(&lwip_taskdef, "lwip");

  k_thread_create(&led_blinking_taskdef, led_blinking_stack, BLINKY_STACK_SIZE,
                  led_blinking_task, NULL, NULL, NULL,
                  K_HIGHEST_APPLICATION_THREAD_PRIO-3, 0, K_NO_WAIT);
  k_thread_name_set(&led_blinking_taskdef, "blinky");

  return 0;
}

#if TUSB_MCU_VENDOR_ESPRESSIF
void app_main(void) {
    main();
}
#endif

void usb_device_task(void *p1, void *p2, void *p3)
{
  (void) p1;
  (void) p2;
  (void) p3;

  // init device stack on configured roothub port
  tusb_rhport_init_t dev_init = {
    .role = TUSB_ROLE_DEVICE,
    .speed = TUSB_SPEED_AUTO
  };
  tusb_init(BOARD_TUD_RHPORT, &dev_init);

  if (board_init_after_tusb) {
    board_init_after_tusb();
  }

  while (1) {
    tud_task();

    // following code only run if tud_task() process at least 1 event
    //tud_cdc_write_flush();
  }
}

void lwip_task(void *p1, void *p2, void *p3)
{
    (void) p1;
    (void) p2;
    (void) p3;

    /* initialize lwip, dhcp-server, dns-server, and http */
    init_lwip();
    while (!netif_is_up(&netif_data));
    while (dhserv_init(&dhcp_config) != ERR_OK);
    while (dnserv_init(IP_ADDR_ANY, 53, dns_query_proc) != ERR_OK);
    httpd_init();

#ifdef INCLUDE_IPERF
    // test with: iperf -c 192.168.7.1 -e -i 1 -M 5000 -l 8192 -r
    lwiperf_start_tcp_server_default(NULL, NULL);
#endif

    while (1) {
        service_traffic();
        k_msleep( 1 );
  }
}

//--------------------------------------------------------------------+
// BLINKING TASK
//--------------------------------------------------------------------+
void led_blinking_task(void *p1, void *p2, void *p3)
{
    (void) p1;
    (void) p2;
    (void) p3;

    static bool led_state = false;

    while (1) {
        // Blink every interval ms
        k_msleep( 500 );

        board_led_write(led_state);
        led_state = 1 - led_state; // toggle
    }
}

/* lwip has provision for using a mutex, when applicable */
sys_prot_t sys_arch_protect(void) {
  return 0;
}
void sys_arch_unprotect(sys_prot_t pval) {
  (void) pval;
}

/* lwip needs a millisecond time source, and the TinyUSB board support code has one available */
uint32_t sys_now(void) {
  return board_millis();
}
