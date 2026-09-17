/*
 * MIT License
 *
 * Copyright (c) 2018 Michele Biondi, Andrea Salvatori
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * @file SPIporting.cpp
 * ESP-IDF porting for the SPI interface.
*/

#include "SPIporting.hpp"
#include "DW1000NgConstants.hpp"
#include "DW1000NgRegisters.hpp"

#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_log.h"

static const char *TAG = "dw1000ng-spi";

namespace SPIporting {

	namespace {

		constexpr uint32_t EspSPImaximumSpeed = 20000000; //20MHz
		constexpr uint32_t SPIminimumSpeed = 2000000; //2MHz

		/* The largest single burst the driver asks for is the 1024 byte receive
		   buffer; a little headroom covers the 3 byte transaction header. */
		constexpr size_t BounceBufferSize = 1088;

		spi_device_handle_t _fastSPI = nullptr;
		spi_device_handle_t _slowSPI = nullptr;
		spi_device_handle_t _currentSPI = nullptr;

		/* spi_master requires DMA capable memory; the driver hands us plain stack
		   arrays, so everything goes through this buffer. */
		uint8_t* _bounce = nullptr;

		bool _initialized = false;

		void _transfer(const uint8_t* tx, uint8_t* rx, size_t len) {
			size_t done = 0;
			while(done < len) {
				const size_t chunk = (len - done) > BounceBufferSize ? BounceBufferSize : (len - done);
				if(tx != nullptr) {
					memcpy(_bounce, tx + done, chunk);
				} else {
					memset(_bounce, 0x00, chunk);
				}

				spi_transaction_t transaction = {};
				transaction.length = chunk * 8;
				transaction.tx_buffer = _bounce;
				transaction.rx_buffer = rx != nullptr ? _bounce : nullptr;
				ESP_ERROR_CHECK(spi_device_polling_transmit(_currentSPI, &transaction));

				if(rx != nullptr) {
					memcpy(rx + done, _bounce, chunk);
				}
				done += chunk;
			}
		}

		void _openSPI(uint8_t slaveSelectPIN) {
			ESP_ERROR_CHECK(spi_device_acquire_bus(_currentSPI, portMAX_DELAY));
			digitalWrite(slaveSelectPIN, LOW);
		}

		void _closeSPI(uint8_t slaveSelectPIN) {
			digitalWrite(slaveSelectPIN, HIGH);
			spi_device_release_bus(_currentSPI);
		}

		spi_device_handle_t _addDevice(uint32_t clockSpeedHz) {
			spi_device_interface_config_t devcfg = {};
			devcfg.clock_speed_hz = clockSpeedHz;
			devcfg.mode = 0;                 // CPOL = 0, CPHA = 0
			devcfg.spics_io_num = -1;        // chip select is driven by hand, see _openSPI
			devcfg.queue_size = 1;
			spi_device_handle_t device = nullptr;
			ESP_ERROR_CHECK(spi_bus_add_device((spi_host_device_t)DW1000NG_SPI_HOST, &devcfg, &device));
			return device;
		}

	}

	void SPIinit() {
		if(_initialized) {
			return;
		}

		_bounce = (uint8_t*)heap_caps_malloc(BounceBufferSize, MALLOC_CAP_DMA);
		ESP_ERROR_CHECK(_bounce != nullptr ? ESP_OK : ESP_ERR_NO_MEM);

		spi_bus_config_t buscfg = {};
		buscfg.mosi_io_num = DW1000NG_PIN_MOSI;
		buscfg.miso_io_num = DW1000NG_PIN_MISO;
		buscfg.sclk_io_num = DW1000NG_PIN_SCK;
		buscfg.quadwp_io_num = -1;
		buscfg.quadhd_io_num = -1;
		buscfg.max_transfer_sz = BounceBufferSize;
		ESP_ERROR_CHECK(spi_bus_initialize((spi_host_device_t)DW1000NG_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO));

		_fastSPI = _addDevice(EspSPImaximumSpeed);
		_slowSPI = _addDevice(SPIminimumSpeed);
		_currentSPI = _fastSPI;
		_initialized = true;

		ESP_LOGI(TAG, "bus %d up (sck=%d miso=%d mosi=%d)",
				 (int)DW1000NG_SPI_HOST, DW1000NG_PIN_SCK, DW1000NG_PIN_MISO, DW1000NG_PIN_MOSI);
	}

	void SPIend() {
		if(!_initialized) {
			return;
		}
		ESP_ERROR_CHECK(spi_bus_remove_device(_fastSPI));
		ESP_ERROR_CHECK(spi_bus_remove_device(_slowSPI));
		ESP_ERROR_CHECK(spi_bus_free((spi_host_device_t)DW1000NG_SPI_HOST));
		heap_caps_free(_bounce);
		_bounce = nullptr;
		_fastSPI = _slowSPI = _currentSPI = nullptr;
		_initialized = false;
	}

	void SPIselect(uint8_t slaveSelectPIN, uint8_t irq) {
		(void)irq; /* the interrupt line is wired up by DW1000Ng::initialize() */
		pinMode(slaveSelectPIN, OUTPUT);
		digitalWrite(slaveSelectPIN, HIGH);
	}

	void writeToSPI(uint8_t slaveSelectPIN, uint8_t headerLen, byte header[], uint16_t dataLen, byte data[]) {
		_openSPI(slaveSelectPIN);
		_transfer(header, nullptr, headerLen);
		_transfer(data, nullptr, dataLen);
		delayMicroseconds(5);
		_closeSPI(slaveSelectPIN);
	}

	void readFromSPI(uint8_t slaveSelectPIN, uint8_t headerLen, byte header[], uint16_t dataLen, byte data[]){
		_openSPI(slaveSelectPIN);
		_transfer(header, nullptr, headerLen);
		_transfer(nullptr, data, dataLen);
		delayMicroseconds(5);
		_closeSPI(slaveSelectPIN);
	}

	void setSPIspeed(SPIClock speed) {
		if(speed == SPIClock::FAST) {
			_currentSPI = _fastSPI;
		 } else if(speed == SPIClock::SLOW) {
			_currentSPI = _slowSPI;
		 }
	}

}
