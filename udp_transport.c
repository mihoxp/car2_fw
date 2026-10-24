#include <string.h>
#include <time.h>

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/sync.h"
#include "lwip/pbuf.h"
#include "lwip/udp.h"

#include <uxr/client/profile/transport/custom/custom_transport.h>

#include "udp_transport.h"
#include "wifi_config.h"

#define RX_SLOTS    8
#define RX_SLOT_LEN 512

static struct udp_pcb *pcb;
static ip_addr_t agent_addr;
static bool wifi_initialized;

// Single-producer (lwIP callback) / single-consumer (read) packet queue
static uint8_t rx_buf[RX_SLOTS][RX_SLOT_LEN];
static uint16_t rx_len[RX_SLOTS];
static volatile uint32_t rx_head, rx_tail;

void usleep(uint64_t us)
{
    sleep_us(us);
}

int clock_gettime(clockid_t unused, struct timespec *tp)
{
    uint64_t m = time_us_64();
    tp->tv_sec = m / 1000000;
    tp->tv_nsec = (m % 1000000) * 1000;
    return 0;
}

static void udp_recv_cb(void *arg, struct udp_pcb *upcb, struct pbuf *p,
                        const ip_addr_t *addr, u16_t port)
{
    if (p == NULL) {
        return;
    }
    uint32_t head = rx_head;
    if (head - rx_tail < RX_SLOTS) {
        uint32_t slot = head % RX_SLOTS;
        rx_len[slot] = pbuf_copy_partial(p, rx_buf[slot], RX_SLOT_LEN, 0);
        __dmb();
        rx_head = head + 1;
    }
    // Queue full: drop the packet, XRCE-DDS will retransmit reliable data
    pbuf_free(p);
}

bool wifi_connect(void)
{
    if (!wifi_initialized) {
        if (cyw43_arch_init()) {
            printf("[wifi] cyw43 init failed\n");
            return false;
        }
        cyw43_arch_enable_sta_mode();
        // Default power save adds latency that, together with BLE sharing
        // the radio, made agent pings time out
        cyw43_wifi_pm(&cyw43_state, CYW43_PERFORMANCE_PM);
        wifi_initialized = true;
    }
    if (cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA) == CYW43_LINK_UP) {
        return true;
    }
    printf("[wifi] connecting to %s ...\n", WIFI_SSID);
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD,
                                           CYW43_AUTH_WPA2_AES_PSK, 30000)) {
        printf("[wifi] connect failed\n");
        return false;
    }
    printf("[wifi] connected, ip: %s\n", ip4addr_ntoa(netif_ip4_addr(netif_list)));
    return true;
}

const char *wifi_ip_str(void)
{
    if (!wifi_initialized ||
        cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA) != CYW43_LINK_UP) {
        return NULL;
    }
    return ip4addr_ntoa(netif_ip4_addr(netif_list));
}

bool pico_udp_transport_open(struct uxrCustomTransport *transport)
{
    if (!wifi_connect()) {
        return false;
    }
    if (pcb != NULL) {
        return true;
    }
    ipaddr_aton(AGENT_IP, &agent_addr);

    cyw43_arch_lwip_begin();
    pcb = udp_new_ip_type(IPADDR_TYPE_V4);
    if (pcb != NULL) {
        udp_bind(pcb, IP_ANY_TYPE, 0);
        udp_recv(pcb, udp_recv_cb, NULL);
    }
    cyw43_arch_lwip_end();
    return pcb != NULL;
}

bool pico_udp_transport_close(struct uxrCustomTransport *transport)
{
    cyw43_arch_lwip_begin();
    if (pcb != NULL) {
        udp_remove(pcb);
        pcb = NULL;
    }
    cyw43_arch_lwip_end();
    rx_tail = rx_head;
    return true;
}

size_t pico_udp_transport_write(struct uxrCustomTransport *transport,
                                const uint8_t *buf, size_t len, uint8_t *errcode)
{
    size_t sent = 0;
    cyw43_arch_lwip_begin();
    struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, len, PBUF_RAM);
    if (p != NULL) {
        memcpy(p->payload, buf, len);
        if (udp_sendto(pcb, p, &agent_addr, AGENT_PORT) == ERR_OK) {
            sent = len;
        }
        pbuf_free(p);
    }
    cyw43_arch_lwip_end();
    if (sent == 0) {
        *errcode = 1;
    }
    return sent;
}

size_t pico_udp_transport_read(struct uxrCustomTransport *transport,
                               uint8_t *buf, size_t len, int timeout, uint8_t *errcode)
{
    absolute_time_t deadline = make_timeout_time_ms(timeout);
    while (rx_tail == rx_head) {
        if (absolute_time_diff_us(get_absolute_time(), deadline) <= 0) {
            *errcode = 1;
            return 0;
        }
        sleep_us(200);
    }
    uint32_t slot = rx_tail % RX_SLOTS;
    size_t n = rx_len[slot] < len ? rx_len[slot] : len;
    memcpy(buf, rx_buf[slot], n);
    __dmb();
    rx_tail = rx_tail + 1;
    return n;
}
