#include <stdio.h>
#include <errno.h>
#include <string.h>

#include "cJSON.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"

#include <DW1000Ng.hpp>
#include <DW1000NgRTLS.hpp>
#include <DW1000NgRanging.hpp>
#include <DW1000NgUtils.hpp>

#include "config.h"
#include "led.h"
#include "wifi.h"

static const char *TAG = "ANCHOR";

device_configuration_t DEFAULT_CONFIG = {
    false,
    true,
    true,
    true,
    false,
    SFDMode::STANDARD_SFD,
    Channel::CHANNEL_5,
    DataRate::RATE_850KBPS,
    PulseFrequency::FREQ_16MHZ,
    PreambleLength::LEN_256,
    PreambleCode::CODE_3
};

static char uniq[16] = "";

static uint16_t antenna_delay = 16436;

static char packetBuffer[255];
static char message[128];

static int udp_socket = -1;
static struct sockaddr_in server_addr;
static bool server_resolved = false;

void receiver() {
    DW1000Ng::forceTRxOff();
    DW1000Ng::startReceive();
}

static void transmit() {
    char payload[sizeof(uniq) + 1];
    snprintf(payload, sizeof(payload), "A%s", uniq);

    DW1000Ng::setTransmitData(payload);
    delay(200);
    DW1000Ng::startTransmit(TransmitMode::IMMEDIATE);
    delay(200);
    DW1000Ng::clearTransmitStatus();
    delay(200);
    DW1000Ng::startReceive();
}

/* "esp_server" is looked up lazily and cached, the same way WiFiUDP resolved the
   host on every beginPacket() call. */
static bool resolveServer() {
    char port[8];
    snprintf(port, sizeof(port), "%d", UDP_PORT);

    struct addrinfo hints = {};
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_DGRAM;

    struct addrinfo *result = nullptr;
    const int err = getaddrinfo(SERVER_HOST_NAME, port, &hints, &result);
    if (err != 0 || result == nullptr) {
        ESP_LOGW(TAG, "cannot resolve %s (%d)", SERVER_HOST_NAME, err);
        return false;
    }

    memcpy(&server_addr, result->ai_addr, sizeof(server_addr));
    freeaddrinfo(result);
    server_resolved = true;
    ESP_LOGI(TAG, "server %s is %s:%d", SERVER_HOST_NAME, inet_ntoa(server_addr.sin_addr), UDP_PORT);
    return true;
}

/** Takes ownership of `data`. */
static void sendDataToServer(const char *type, cJSON *data) {
    cJSON *doc = cJSON_CreateObject();
    cJSON_AddStringToObject(doc, "uniq", uniq);
    cJSON_AddStringToObject(doc, "type", type);
    cJSON_AddItemToObject(doc, "data", data);

    char *payload = cJSON_PrintUnformatted(doc);
    cJSON_Delete(doc);
    if (payload == nullptr) {
        return;
    }

    if (server_resolved || resolveServer()) {
        const int sent = sendto(udp_socket, payload, strlen(payload), 0,
                                (struct sockaddr *)&server_addr, sizeof(server_addr));
        if (sent < 0) {
            ESP_LOGW(TAG, "sendto failed: errno %d", errno);
            server_resolved = false;
        }
    }

    cJSON_free(payload);
}

static void getMac() {
    uint8_t mac[6];
    ESP_ERROR_CHECK(wifi_get_mac(mac));
    /* Arduino's String(byte, HEX) drops leading zeros; keep that so the id the
       server already knows does not change. */
    snprintf(uniq, sizeof(uniq), "%x%x%x%x%x%x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

static void openSocket() {
    udp_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    ESP_ERROR_CHECK(udp_socket >= 0 ? ESP_OK : ESP_FAIL);

    struct sockaddr_in local = {};
    local.sin_family = AF_INET;
    local.sin_port = htons(UDP_PORT);
    local.sin_addr.s_addr = htonl(INADDR_ANY);
    ESP_ERROR_CHECK(bind(udp_socket, (struct sockaddr *)&local, sizeof(local)) == 0 ? ESP_OK : ESP_FAIL);

    /* the main loop polls the socket between DW1000 reads, so it must not block */
    int nonblocking = 1;
    lwip_ioctl(udp_socket, FIONBIO, &nonblocking);
}

static void setup() {
    ESP_ERROR_CHECK(wifi_start(SSID, PASSWORD));

    getMac();
    ESP_LOGI(TAG, "anchor %s", uniq);

    ESP_ERROR_CHECK(led_init(LED_GPIO, LED_COUNT));
    led_set_brightness(LED_BRIGHTNESS);
    led_set_pixel(0, LED_RED);

    if (!wifi_wait_connected(10000)) {
        ESP_LOGW(TAG, "no ip yet, carrying on");
    }
    led_set_pixel(0, LED_BLUE);

    openSocket();
    sendDataToServer("anchor", cJSON_CreateObject());

    ESP_LOGI(TAG, "### DW1000Ng receiver ###");
    DW1000Ng::initializeNoInterrupt(DW1000_PIN_SS, DW1000_PIN_RST);
    ESP_LOGI(TAG, "DW1000Ng initialized ...");

    DW1000Ng::applyConfiguration(DEFAULT_CONFIG);

    DW1000Ng::setDeviceAddress(6);
    DW1000Ng::setNetworkId(10);

    DW1000Ng::setAntennaDelay(antenna_delay);
    ESP_LOGI(TAG, "Committed configuration ...");

    char msg[128];
    DW1000Ng::getPrintableDeviceIdentifier(msg);
    ESP_LOGI(TAG, "Device ID: %s", msg);
    DW1000Ng::getPrintableExtendedUniqueIdentifier(msg);
    ESP_LOGI(TAG, "Unique ID: %s", msg);
    DW1000Ng::getPrintableNetworkIdAndShortAddress(msg);
    ESP_LOGI(TAG, "Network ID & Device Address: %s", msg);
    DW1000Ng::getPrintableDeviceMode(msg);
    ESP_LOGI(TAG, "Device mode: %s", msg);

    delay(1000);

    transmit();

    led_set_pixel(0, LED_GREEN);
}

static void loop() {
    struct sockaddr_storage source;
    socklen_t source_len = sizeof(source);
    const int packetSize = recvfrom(udp_socket, packetBuffer, sizeof(packetBuffer) - 1, 0,
                                    (struct sockaddr *)&source, &source_len);

    if (packetSize > 0) {
        packetBuffer[packetSize] = '\0';
        ESP_LOGI(TAG, "received packet from %s, size %d",
                 inet_ntoa(((struct sockaddr_in *)&source)->sin_addr), packetSize);

        cJSON *doc = cJSON_Parse(packetBuffer);
        if (doc == nullptr) {
            ESP_LOGW(TAG, "cJSON_Parse() failed");
            return;
        }
        const bool reboot = cJSON_IsTrue(cJSON_GetObjectItem(doc, "reboot"));
        cJSON_Delete(doc);

        if (reboot) {
            esp_restart();
        }
    }

    if (DW1000Ng::isReceiveDone()) {
        led_set_pixel(1, LED_GREEN);
        DW1000Ng::getReceivedData(message, sizeof(message));

        cJSON *data = cJSON_CreateObject();
        cJSON_AddStringToObject(data, "uniq", message);
        cJSON_AddNumberToObject(data, "quality", DW1000Ng::getReceiveQuality());
        cJSON_AddNumberToObject(data, "power", DW1000Ng::getReceivePower());
        sendDataToServer("range", data);

        DW1000Ng::startReceive();
        led_set_pixel(1, LED_RED);
    }
}

extern "C" void app_main(void) {
    setup();
    for (;;) {
        loop();
        vTaskDelay(1);
    }
}
