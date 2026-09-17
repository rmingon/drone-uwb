#include <errno.h>
#include <stdio.h>
#include <string.h>

#include "cJSON.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"
#include "nvs.h"

#include "config.h"
#include "led.h"
#include "ranging.h"
#include "uwb_profile.h"
#include "wifi.h"

static const char *TAG = "ANCHOR";

static char uniq[16] = "";

static char packetBuffer[255];

static int udp_socket = -1;
static struct sockaddr_in server_addr;
static bool server_resolved = false;

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

    /* the main loop polls the socket between ranging exchanges, so it must not block */
    int nonblocking = 1;
    lwip_ioctl(udp_socket, FIONBIO, &nonblocking);
}

/* ------------------------------------------------- antenna delay storage -- */

#define NVS_NAMESPACE "anchor"
/* The antenna delay depends on the pulse frequency, so a value calibrated on
   one radio profile is wrong on the other. Key it by profile so switching
   never silently reuses a stale calibration. */
#define NVS_KEY_ANT_DLY "ant_dly_" UWB_PROFILE_KEY

static uint16_t loadAntennaDelay() {
    nvs_handle_t nvs;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs) != ESP_OK) {
        return DW1000_ANTENNA_DELAY;
    }
    uint16_t stored = 0;
    const esp_err_t err = nvs_get_u16(nvs, NVS_KEY_ANT_DLY, &stored);
    nvs_close(nvs);
    return err == ESP_OK ? stored : DW1000_ANTENNA_DELAY;
}

static void storeAntennaDelay(uint16_t antenna_delay) {
    nvs_handle_t nvs;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs) != ESP_OK) {
        ESP_LOGW(TAG, "cannot open nvs to store the antenna delay");
        return;
    }
    if (nvs_set_u16(nvs, NVS_KEY_ANT_DLY, antenna_delay) == ESP_OK) {
        nvs_commit(nvs);
    }
    nvs_close(nvs);
}

/* -------------------------------------------------------------- reporting -- */

static void reportNewTags() {
    ranging_tag_t tag;
    while (ranging_take_new_tag(&tag)) {
        cJSON *data = cJSON_CreateObject();
        cJSON_AddStringToObject(data, "eui", tag.eui);
        cJSON_AddNumberToObject(data, "address", tag.address);
        sendDataToServer("tag", data);
        ESP_LOGI(TAG, "tag %s is now address %u", tag.eui, tag.address);
    }
}

static void reportRange(const ranging_measure_t &measure) {
    cJSON *data = cJSON_CreateObject();
    cJSON_AddNumberToObject(data, "address", measure.tag_address);
    cJSON_AddNumberToObject(data, "range", measure.range);
    cJSON_AddNumberToObject(data, "raw_range", measure.raw_range);
    cJSON_AddNumberToObject(data, "rx_power", measure.rx_power);
    cJSON_AddNumberToObject(data, "fp_power", measure.fp_power);
    cJSON_AddBoolToObject(data, "los", measure.line_of_sight);
    sendDataToServer("range", data);
}

/* ------------------------------------------------------------------------- */

static void setup() {
    ESP_ERROR_CHECK(wifi_start(SSID, PASSWORD));

    getMac();
    ESP_LOGI(TAG, "anchor %s, short address %d", uniq, ANCHOR_ADDRESS);

    ESP_ERROR_CHECK(led_init(LED_GPIO, LED_COUNT));
    led_set_brightness(LED_BRIGHTNESS);
    led_set_pixel(0, LED_RED);

    if (!wifi_wait_connected(10000)) {
        ESP_LOGW(TAG, "no ip yet, carrying on");
    }
    led_set_pixel(0, LED_BLUE);

    openSocket();

    cJSON *hello = cJSON_CreateObject();
    cJSON_AddNumberToObject(hello, "address", ANCHOR_ADDRESS);
    cJSON_AddBoolToObject(hello, "main", ANCHOR_IS_MAIN);
    sendDataToServer("anchor", hello);

    ESP_LOGI(TAG, "### DW1000Ng TWR anchor ###");
    ranging_init(loadAntennaDelay());

    led_set_pixel(0, LED_GREEN);
}

static void handleCommand() {
    struct sockaddr_storage source;
    socklen_t source_len = sizeof(source);
    const int packetSize = recvfrom(udp_socket, packetBuffer, sizeof(packetBuffer) - 1, 0,
                                    (struct sockaddr *)&source, &source_len);
    if (packetSize <= 0) {
        return;
    }

    packetBuffer[packetSize] = '\0';
    ESP_LOGI(TAG, "received packet from %s, size %d",
             inet_ntoa(((struct sockaddr_in *)&source)->sin_addr), packetSize);

    cJSON *doc = cJSON_Parse(packetBuffer);
    if (doc == nullptr) {
        ESP_LOGW(TAG, "cJSON_Parse() failed");
        return;
    }

    const cJSON *antenna_delay = cJSON_GetObjectItem(doc, "antenna_delay");
    if (cJSON_IsNumber(antenna_delay)) {
        const uint16_t value = (uint16_t)antenna_delay->valuedouble;
        storeAntennaDelay(value);
        ranging_set_antenna_delay(value);
    }

    const bool reboot = cJSON_IsTrue(cJSON_GetObjectItem(doc, "reboot"));
    cJSON_Delete(doc);

    if (reboot) {
        esp_restart();
    }
}

static void loop() {
    handleCommand();
    reportNewTags();

    ranging_measure_t measure;
    if (ranging_poll(&measure)) {
        led_set_pixel(1, measure.line_of_sight ? LED_GREEN : LED_RED);
        reportRange(measure);
        ESP_LOGI(TAG, "tag %u at %.2f m (raw %.2f, rx %.1f dBm, fp %.1f dBm%s)",
                 measure.tag_address, measure.range, measure.raw_range,
                 measure.rx_power, measure.fp_power, measure.line_of_sight ? "" : ", nlos");
    }
}

extern "C" void app_main(void) {
    setup();
    for (;;) {
        loop();
        vTaskDelay(1);
    }
}
