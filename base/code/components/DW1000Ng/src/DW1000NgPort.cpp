/*
 * ESP-IDF implementation of the Arduino API subset used by DW1000Ng.
 */

#include "DW1000NgPort.hpp"

#include "esp_err.h"
#include "esp_log.h"
#include "freertos/semphr.h"

static const char *TAG = "dw1000ng-port";

void pinMode(uint8_t pin, uint8_t mode) {
    gpio_config_t cfg = {};
    cfg.pin_bit_mask = 1ULL << pin;
    cfg.mode         = (mode == OUTPUT) ? GPIO_MODE_OUTPUT : GPIO_MODE_INPUT;
    cfg.pull_up_en   = GPIO_PULLUP_DISABLE;
    cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    cfg.intr_type    = GPIO_INTR_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&cfg));
}

void digitalWrite(uint8_t pin, uint8_t value) {
    gpio_set_level((gpio_num_t)pin, value ? 1 : 0);
}

int digitalRead(uint8_t pin) {
    return gpio_get_level((gpio_num_t)pin);
}

void delay(uint32_t ms) {
    if (ms == 0) {
        return;
    }
    const uint32_t tick_ms = portTICK_PERIOD_MS;
    const uint32_t ticks   = ms / tick_ms;
    const uint32_t rest_ms = ms % tick_ms;
    if (ticks > 0) {
        vTaskDelay(ticks);
    }
    if (rest_ms > 0) {
        esp_rom_delay_us(rest_ms * 1000);
    }
}

/* ----------------------------------------------- deferred pin interrupts -- */

namespace {

struct IrqDispatch {
    void (*handler)(void) = nullptr;
    SemaphoreHandle_t signal = nullptr;
    TaskHandle_t task = nullptr;
    int pin = -1;
};

IrqDispatch _irq;

void IRAM_ATTR _irqIsr(void *arg) {
    BaseType_t higherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(_irq.signal, &higherPriorityTaskWoken);
    if (higherPriorityTaskWoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

void _irqTask(void *arg) {
    for (;;) {
        if (xSemaphoreTake(_irq.signal, portMAX_DELAY) == pdTRUE && _irq.handler != nullptr) {
            _irq.handler();
        }
    }
}

} // namespace

void attachInterrupt(int pin, void (*handler)(void), int mode) {
    (void)mode;

    if (_irq.pin >= 0 && _irq.pin != pin) {
        ESP_LOGE(TAG, "only one DW1000 interrupt pin is supported (already using %d)", _irq.pin);
        return;
    }

    _irq.handler = handler;

    if (_irq.signal == nullptr) {
        _irq.signal = xSemaphoreCreateBinary();
        ESP_ERROR_CHECK(_irq.signal != nullptr ? ESP_OK : ESP_ERR_NO_MEM);
        BaseType_t created = xTaskCreate(_irqTask, "dw1000ng_irq", 4096, nullptr, 10, &_irq.task);
        ESP_ERROR_CHECK(created == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
    }

    gpio_config_t cfg = {};
    cfg.pin_bit_mask = 1ULL << pin;
    cfg.mode         = GPIO_MODE_INPUT;
    cfg.pull_up_en   = GPIO_PULLUP_DISABLE;
    cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    cfg.intr_type    = GPIO_INTR_POSEDGE;
    ESP_ERROR_CHECK(gpio_config(&cfg));

    esp_err_t err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(err);
    }
    ESP_ERROR_CHECK(gpio_isr_handler_add((gpio_num_t)pin, _irqIsr, nullptr));

    _irq.pin = pin;
}

void detachInterrupt(int pin) {
    if (_irq.pin != pin) {
        return;
    }
    ESP_ERROR_CHECK(gpio_isr_handler_remove((gpio_num_t)pin));
    _irq.handler = nullptr;
    _irq.pin     = -1;
}
