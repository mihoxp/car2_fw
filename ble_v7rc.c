#include <stdio.h>
#include <string.h>

#include "btstack.h"
#include "ble/gatt-service/nordic_spp_service_server.h"

#include "ble_v7rc.h"
#include "car2_ble.h"   // generated from car2_ble.gatt

#define FRAME_LEN 20    // "SRT" + 4 x 4 digits + "#"

// Advertising data is limited to 31 bytes, so the name goes in the scan
// response (like YAHBOOM's advertising_resp_payload)
static const uint8_t adv_data[] = {
    // Flags: general discoverable, BR/EDR not supported
    2, BLUETOOTH_DATA_TYPE_FLAGS, 0x06,
    // Nordic UART service UUID 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
    17, BLUETOOTH_DATA_TYPE_COMPLETE_LIST_OF_128_BIT_SERVICE_CLASS_UUIDS,
    0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0, 0x93, 0xf3, 0xa3, 0xb5, 0x01, 0x00, 0x40, 0x6e,
};
static const uint8_t scan_resp_data[] = {
    10, BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME, 'p', 'i', 'c', 'o', '_', 'c', 'a', 'r', '2',
};

static hci_con_handle_t con_handle = HCI_CON_HANDLE_INVALID;
static btstack_packet_callback_registration_t hci_cb;
static char buf[FRAME_LEN + 1];
static int buf_len;
static volatile bool frame_ready;
static volatile int frame_ch[V7RC_CHANNELS];

static int parse4(const char *s)
{
    int v = 0;
    for (int i = 0; i < 4; i++) {
        if (s[i] < '0' || s[i] > '9') {
            return -1;
        }
        v = v * 10 + (s[i] - '0');
    }
    return v;
}

static void handle_frame(void)
{
    if (buf_len != FRAME_LEN || buf[0] != 'S' || buf[1] != 'R' ||
            (buf[2] != 'T' && buf[2] != 'V')) {
        return;
    }
    int ch[V7RC_CHANNELS];
    for (int i = 0; i < V7RC_CHANNELS; i++) {
        ch[i] = parse4(buf + 3 + 4 * i);
        if (ch[i] < 0) {
            return;
        }
    }
    for (int i = 0; i < V7RC_CHANNELS; i++) {
        frame_ch[i] = ch[i];
    }
    frame_ready = true;
}

static void rx_bytes(const uint8_t *data, uint16_t size)
{
    for (uint16_t i = 0; i < size; i++) {
        char c = (char)data[i];
        if (c == 'S') {
            buf_len = 0;    // a frame always starts with 'S'
        }
        if (buf_len < FRAME_LEN) {
            buf[buf_len++] = c;
        }
        if (c == '#') {
            handle_frame();
            buf_len = 0;
        }
    }
}

static void spp_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size)
{
    (void)channel;
    switch (packet_type) {
    case HCI_EVENT_PACKET:
        if (hci_event_packet_get_type(packet) != HCI_EVENT_GATTSERVICE_META) break;
        switch (hci_event_gattservice_meta_get_subevent_code(packet)) {
        case GATTSERVICE_SUBEVENT_SPP_SERVICE_CONNECTED:
            con_handle = gattservice_subevent_spp_service_connected_get_con_handle(packet);
            printf("ble: V7RC connected\n");
            break;
        case GATTSERVICE_SUBEVENT_SPP_SERVICE_DISCONNECTED:
            con_handle = HCI_CON_HANDLE_INVALID;
            buf_len = 0;
            printf("ble: V7RC disconnected\n");
            break;
        default:
            break;
        }
        break;
    case RFCOMM_DATA_PACKET:
        rx_bytes(packet, size);
        break;
    default:
        break;
    }
}

static void hci_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size)
{
    (void)channel;
    (void)size;
    if (packet_type != HCI_EVENT_PACKET) return;
    switch (hci_event_packet_get_type(packet)) {
    case BTSTACK_EVENT_STATE:
        printf("ble: state %u%s\n", btstack_event_state_get_state(packet),
               btstack_event_state_get_state(packet) == HCI_STATE_WORKING ? " (working, advertising)" : "");
        if (btstack_event_state_get_state(packet) == HCI_STATE_WORKING) {
            bd_addr_t addr;
            gap_local_bd_addr(addr);
            printf("ble: address %s\n", bd_addr_to_str(addr));
        }
        break;
    case HCI_EVENT_COMMAND_COMPLETE: {
        uint8_t status = hci_event_command_complete_get_return_parameters(packet)[0];
        if (status != 0) {
            printf("ble: command 0x%04x failed, status 0x%02x\n",
                   hci_event_command_complete_get_command_opcode(packet), status);
        }
        break;
    }
    default:
        break;
    }
}

void ble_v7rc_init(void)
{
    printf("ble: init\n");
    hci_cb.callback = &hci_handler;
    hci_add_event_handler(&hci_cb);
    l2cap_init();
    sm_init();
    att_server_init(profile_data, NULL, NULL);
    nordic_spp_service_server_init(&spp_handler);

    bd_addr_t null_addr = {0};
    // 30 ms advertising interval, connectable undirected
    gap_advertisements_set_params(0x0030, 0x0030, 0, 0, null_addr, 0x07, 0x00);
    gap_advertisements_set_data(sizeof(adv_data), (uint8_t *)adv_data);
    gap_scan_response_set_data(sizeof(scan_resp_data), (uint8_t *)scan_resp_data);
    gap_advertisements_enable(1);
    int rc = hci_power_control(HCI_POWER_ON);
    printf("ble: power on rc=%d\n", rc);
}

int ble_v7rc_state(void)
{
    return hci_get_state();
}

bool ble_v7rc_get(int ch[V7RC_CHANNELS])
{
    if (!frame_ready) {
        return false;
    }
    frame_ready = false;
    for (int i = 0; i < V7RC_CHANNELS; i++) {
        ch[i] = frame_ch[i];
    }
    return true;
}

bool ble_v7rc_connected(void)
{
    return con_handle != HCI_CON_HANDLE_INVALID;
}
