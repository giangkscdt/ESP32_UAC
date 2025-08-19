#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <stdarg.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "usb_device_uac.h"
#include "freertos/ringbuf.h"
#include "I2SSampler.h"
#include "driver/gpio.h"
#include "highpass_filter.h"
#include "notch_filter.h"  // Thêm dòng này ở đầu file
#include <limits> // đảm bảo có INT16_MAX và INT16_MIN
#define TOTAL_MS 100
#define SAMPLES_PER_MS 48
//#define BYTES_PER_SAMPLE 2
#define BUFFER_SAMPLES (TOTAL_MS * SAMPLES_PER_MS)
#define BUFFER_BYTES   (BUFFER_SAMPLES * BYTES_PER_SAMPLE)
#define UAC_PACKET_SIZE 96


#define SAMPLE_RATE_HZ     48000
#define BITS_PER_SAMPLE    16
#define BYTES_PER_SAMPLE   (BITS_PER_SAMPLE / 8)
#define CHANNELS           1

#define BLOCK_SAMPLES      48                      // 1ms = 48 samples @ 48kHz
#define NUM_BLOCKS         200                     // ~200ms buffer

#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif

static I2SSampler *i2s_sampler = NULL; 
/*static int16_t audio_buffer[BUFFER_SAMPLES];
static volatile size_t write_index = 0;
static volatile size_t read_index = 0;
static int64_t last_callback_time_us = 0;*/
static int16_t audio_buffers[NUM_BLOCKS][BLOCK_SAMPLES];
static volatile size_t write_block_index = 0;
static volatile size_t read_block_index = 0;
static int64_t last_callback_time_us = 0;

static i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = 48000,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = 48,
    .use_apll = true,
    .tx_desc_auto_clear = true,
    .fixed_mclk = 0
};

static i2s_pin_config_t i2s_pins = {
    .bck_io_num = GPIO_NUM_4,
    .ws_io_num = GPIO_NUM_5,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = GPIO_NUM_6
};

extern "C" void ets_delay_us(int us);
/*
static void i2sSenderTask(void *param) {
    I2SSampler *sampler = (I2SSampler *)param;
    const TickType_t wait_ms = pdMS_TO_TICKS(100);

    while (true) {
        uint32_t notified = ulTaskNotifyTake(pdTRUE, wait_ms);
        if (notified > 0) {
            const int16_t *data = (const int16_t *)sampler->getCapturedAudioBuffer();
            size_t sample_count = sampler->getBufferSizeInBytes() / sizeof(int16_t);

            for (size_t i = 0; i < sample_count; i++) {
                audio_buffer[write_index] = data[i];
                write_index = (write_index + 1) % BUFFER_SAMPLES;
            }
        }
    }
}*/
// usb_uac_main.cpp

void i2sReaderTask(void *param) {
    I2SSampler *sampler = static_cast<I2SSampler *>(param);

    while (true) {
        uint8_t i2s_data[BLOCK_SAMPLES * 4];  // I2S đọc 32bit → 4 byte/sample

        size_t bytes_read = sampler->readSamples(i2s_data, sizeof(i2s_data));
        if (bytes_read == 0) continue;

        int16_t *dst = audio_buffers[write_block_index];
        int32_t *src = (int32_t *)i2s_data;

        for (size_t i = 0; i < BLOCK_SAMPLES; ++i) {
            dst[i] = (int16_t)(src[i] >> 11);  // scale 24/32-bit to 16-bit
        }

        size_t next_index = (write_block_index + 1) % NUM_BLOCKS;
        if (next_index == read_block_index) {
            read_block_index = (read_block_index + 1) % NUM_BLOCKS;
        }
        write_block_index = next_index;
    }
}


/*
static void i2sSenderTask(void *param) {
    I2SSampler *sampler = (I2SSampler *)param;
    const TickType_t wait_ms = pdMS_TO_TICKS(100);

    while (true) {
        // Chờ tín hiệu từ I2SSampler::addSample()
        uint32_t notified = ulTaskNotifyTake(pdTRUE, wait_ms);
        if (notified > 0) {
            const int16_t *data = (const int16_t *)sampler->getCapturedAudioBuffer();
            size_t sample_count = sampler->getBufferSizeInBytes() / sizeof(int16_t);

            for (size_t i = 0; i < sample_count; i++) {
                // Chống tràn: nếu write_index đuổi kịp read_index thì bỏ mẫu cũ
                size_t next_write_index = (write_index + 1) % BUFFER_SAMPLES;
                if (next_write_index == read_index) {
                    // Buffer đầy, bỏ mẫu cũ nhất
                    read_index = (read_index + 1) % BUFFER_SAMPLES;
                }

                // Ghi mẫu mới
                audio_buffer[write_index] = data[i];
                write_index = next_write_index;
            }
        }
    }
}
*/

static esp_err_t uac_device_output_cb(uint8_t *buf, size_t len, void *arg) {
    return ESP_OK;
}

esp_err_t uac_device_input_cb(uint8_t *buf, size_t len, size_t *bytes_read, void *arg) {
    int16_t *out = (int16_t *)buf;
    size_t samples = len / sizeof(int16_t);

    static int current_block = -1;
    static size_t sample_pos = 0;

    for (size_t i = 0; i < samples; ++i) {
        if (current_block == -1 || sample_pos >= BLOCK_SAMPLES) {
            // Load block mới nếu có dữ liệu
            if (read_block_index != write_block_index) {
                current_block = read_block_index;
                read_block_index = (read_block_index + 1) % NUM_BLOCKS;
                sample_pos = 0;
            } else {
                current_block = -1;
            }
        }

        if (current_block != -1) {
            //out[i] = audio_buffers[current_block][sample_pos++];
						int16_t sample = audio_buffers[current_block][sample_pos++];
sample = apply_highpass_filter(sample);  // nếu có HPF
sample = apply_lowpass_filter(sample);   // sau đó LPF
sample = apply_notch_filter(sample);
out[i] = sample;
        } else {
            out[i] = 0; // fallback: không có dữ liệu
        }
    }

    *bytes_read = len;

    // pacing 1ms nếu cần
    int64_t now = esp_timer_get_time();
    int64_t elapsed = now - last_callback_time_us;
    if (elapsed < 1000) ets_delay_us(1000 - elapsed);
    last_callback_time_us = esp_timer_get_time();

    return ESP_OK;
}


static void uac_device_set_mute_cb(uint32_t mute, void *arg) {
    ESP_LOGI("usb_uac_main", "Mute: %s", mute ? "ON" : "OFF");
}

static void uac_device_set_volume_cb(uint32_t volume, void *arg) {
    ESP_LOGI("usb_uac_main", "Volume: %d", volume);
}
/*
extern "C" void app_main() {
    i2s_sampler = new I2SSampler();
    TaskHandle_t sender_task;
    xTaskCreate(i2sSenderTask, "i2sSenderTask", 4096, i2s_sampler, 5, &sender_task);
    i2s_sampler->start(I2S_NUM_1, i2s_pins, i2s_config, 96, sender_task);

    uac_device_config_t config = {
        .output_cb = uac_device_output_cb,
        .input_cb = uac_device_input_cb,
        .set_mute_cb = uac_device_set_mute_cb,
        .set_volume_cb = uac_device_set_volume_cb,
    };
    uac_device_init(&config);
}*/
extern "C" void app_main() {
    I2SSampler *sampler = new I2SSampler();
    //TaskHandle_t sender_task;
    // Khởi tạo task đọc I2S
	 sampler->start(I2S_NUM_1, i2s_pins, i2s_config, BLOCK_SAMPLES, nullptr);
    xTaskCreatePinnedToCore(i2sReaderTask, "i2sReaderTask", 4096, sampler, 5, nullptr, 0);

    // Khởi tạo I2S
    init_highpass_filter(200.0f, 48000.0f);  // 🔧 HPF Init here
	init_lowpass_filter(15000.0f, 48000.0f);  // Cutoff = 15kHz, Sample rate = 48kHz
	init_notch_filter(48000.0f, 75.0f, 30.0f);  // Fs, notch_freq, Q

    uac_device_config_t config = {
        .output_cb = uac_device_output_cb,
        .input_cb = uac_device_input_cb,
        .set_mute_cb = uac_device_set_mute_cb,
        .set_volume_cb = uac_device_set_volume_cb,
    };
    uac_device_init(&config);
}
