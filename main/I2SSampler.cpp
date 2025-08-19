// === CONVERSION FROM ORIGINAL ARDUINO CODE TO ESP-IDF ===
// Original Arduino line: #include <Arduino.h>
// [REMOVED] Replaced because ESP-IDF doesn't use Arduino framework
#include "driver/i2s.h"               // [UNCHANGED] I2S driver for handling audio streams
#include "freertos/FreeRTOS.h"        // [ADDED] Required for task creation and RTOS functions
#include "freertos/task.h"            // [ADDED] For xTaskCreatePinnedToCore, TaskHandle_t
#include "freertos/queue.h"           // [ADDED] For I2S event queue support
#include <algorithm>                   // [UNCHANGED] For std::swap used in buffer swapping

#include "I2SSampler.h"               // [UNCHANGED] Include the class declaration

// [REVISED] Split assignment and increment for clarity and correctness
void I2SSampler::addSample(int16_t sample)
{
    // Original (condensed): m_current_audio_buffer[m_audio_buffer_pos++] = sample;
    // Revised for clarity and correct evaluation order:
    m_current_audio_buffer[m_audio_buffer_pos] = sample;      // [REVISED] Assign sample to buffer at current index
    m_audio_buffer_pos++;                                     // [REVISED] Increment buffer position after assignment

    if (m_audio_buffer_pos == m_buffer_size_in_samples)      // [UNCHANGED] Check if buffer is full
    {
        std::swap(m_current_audio_buffer, m_captured_audio_buffer); // [UNCHANGED] Swap buffers
        m_audio_buffer_pos = 0;                                      // [UNCHANGED] Reset index
        xTaskNotify(m_writer_task_handle, 1, eIncrement);           // [UNCHANGED] Notify writer task
    }
}

// [UNCHANGED] Convert 32-bit I2S data to 16-bit samples and push to buffer
void I2SSampler::processI2SData(uint8_t *i2s_data, size_t bytes_read)
{
    int32_t *samples = (int32_t *)i2s_data; // [UNCHANGED] reinterpret data buffer as 32-bit integers
    for (size_t i = 0; i < bytes_read / 4; i++) // [UNCHANGED] Read each 4-byte (32-bit) sample
    {
        addSample(samples[i] >> 11);         // [UNCHANGED] Scale down and add to buffer
    }
}

// Original: void i2sReaderTask(void *param)
// [REVISED] → Converted to static function for ESP-IDF C++ scope
/*
void i2sReaderTask(void *param)
{
    I2SSampler *sampler = static_cast<I2SSampler *>(param);

    while (true)
    {
        i2s_event_t evt;
        if (xQueueReceive(sampler->m_i2s_queue, &evt, portMAX_DELAY) == pdPASS)
        {
            if (evt.type == I2S_EVENT_RX_DONE)
            {
                uint8_t i2s_data[96];  // đúng 48 mẫu (1ms tại 48kHz)
                size_t bytes_read = 0;

                esp_err_t res = i2s_read(sampler->getI2SPort(), i2s_data, sizeof(i2s_data), &bytes_read, portMAX_DELAY);
                if (res == ESP_OK && bytes_read == sizeof(i2s_data)) {
                    sampler->processI2SData(i2s_data, bytes_read);
                }
            }
        }
    }
}
*/




// Original: void I2SSampler::start(i2s_port_t i2s_port, i2s_pin_config_t &i2s_pins, i2s_config_t &i2s_config, ...)
// [REVISED] Use const reference for i2s_config and i2s_pins for safety
// I2SSampler.cpp
void I2SSampler::start(i2s_port_t i2s_port,
                       const i2s_pin_config_t &i2s_pins,
                       const i2s_config_t &i2s_config,
                       int32_t buffer_size_in_bytes,
                       TaskHandle_t writer_task_handle)

{
    m_i2s_port = i2s_port;                           // [UNCHANGED] Store I2S port
    m_writer_task_handle = writer_task_handle;       // [UNCHANGED] Save writer task handle
    m_buffer_size_in_samples = buffer_size_in_bytes / sizeof(int16_t); // [UNCHANGED]
    m_buffer_size_in_bytes = buffer_size_in_bytes;                     // [UNCHANGED]

    m_audio_buffer1 = (int16_t *)malloc(buffer_size_in_bytes);  // [UNCHANGED] Allocate buffer 1
    m_audio_buffer2 = (int16_t *)malloc(buffer_size_in_bytes);  // [UNCHANGED] Allocate buffer 2
    m_current_audio_buffer = m_audio_buffer1;                   // [UNCHANGED]
    m_captured_audio_buffer = m_audio_buffer2;                  // [UNCHANGED]

    i2s_driver_install(m_i2s_port, &i2s_config, 4, &m_i2s_queue); // [UNCHANGED] Install driver with queue
    i2s_set_pin(m_i2s_port, &i2s_pins);                          // [UNCHANGED] Configure I2S pins

    //TaskHandle_t reader_task_handle;
    //xTaskCreatePinnedToCore(i2sReaderTask, "i2s Reader Task", 4096, this, 1, &reader_task_handle, 0); // [UNCHANGED]
}

