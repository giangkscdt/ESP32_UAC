// === PORTED TO ESP-IDF WITH EXPLANATIONS ===
// [REVISED] Header guard to avoid multiple inclusion
#ifndef __I2S_SAMPLER_H__
#define __I2S_SAMPLER_H__

// [UNCHANGED] Include ESP-IDF I2S driver
#include "driver/i2s.h"
// [ADDED] Include FreeRTOS headers needed for tasks and queues
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <stdint.h>
#include <stddef.h>

// [UNCHANGED] Class to manage I2S data acquisition using double-buffering
class I2SSampler
{
private:
    int16_t *m_audio_buffer1;         // [UNCHANGED] First audio buffer for double buffering
    int16_t *m_audio_buffer2;         // [UNCHANGED] Second audio buffer
    int32_t m_audio_buffer_pos = 0;   // [UNCHANGED] Current position in buffer (number of samples written)
    int16_t *m_current_audio_buffer;  // [UNCHANGED] Currently active buffer being written
    int16_t *m_captured_audio_buffer; // [UNCHANGED] Buffer containing captured data ready for processing
    int32_t m_buffer_size_in_bytes;   // [UNCHANGED] Size of buffer in bytes
    int32_t m_buffer_size_in_samples; // [UNCHANGED] Size of buffer in samples (not bytes)
    TaskHandle_t m_reader_task_handle; // [UNCHANGED] Handle for I2S reader task
    TaskHandle_t m_writer_task_handle; // [UNCHANGED] Handle for processing task that consumes captured buffer
    QueueHandle_t m_i2s_queue;         // [UNCHANGED] Event queue for I2S driver
    i2s_port_t m_i2s_port;             // [UNCHANGED] Which I2S port is used (I2S_NUM_0 or I2S_NUM_1)

protected:
    void configureI2S(); // [UNCHANGED] Internal function to configure I2S (you'll define this in .cpp)
    void processI2SData(uint8_t *i2sData, size_t bytes_read); // [UNCHANGED] Parse I2S byte stream and extract audio samples
    void addSample(int16_t sample); // [UNCHANGED] Add sample to current buffer, handle buffer full switch

    // [UNCHANGED] Accessor for I2S port used
    i2s_port_t getI2SPort()
    {
        return m_i2s_port;
    }

public:
    // [UNCHANGED] Return buffer size in bytes (useful for writing to disk or over USB)
    int32_t getBufferSizeInBytes()
    {
        return m_buffer_size_in_bytes;
    }

    // [UNCHANGED] Return pointer to buffer that has completed sampling and ready for processing
    int16_t *getCapturedAudioBuffer()
    {
        return m_captured_audio_buffer;
    }

    size_t readSamples(uint8_t* dest, size_t len) {
        size_t bytes_read = 0;
        esp_err_t ret = i2s_read(m_i2s_port, dest, len, &bytes_read, portMAX_DELAY);
        return (ret == ESP_OK) ? bytes_read : 0;
    }


    // [REVISED] Added const references to config structs for ESP-IDF safety & efficiency
    void start(i2s_port_t i2sPort,
               const i2s_pin_config_t &i2s_pins,
               const i2s_config_t &i2s_config,
               int32_t buffer_size_in_bytes,
               TaskHandle_t writer_task_handle);

    // [UNCHANGED] Declare the reader task as friend so it can access private members
    friend void i2sReaderTask(void *param);
};

#endif // __I2S_SAMPLER_H__
