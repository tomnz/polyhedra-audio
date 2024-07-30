#ifndef AUDIO_H
#define AUDIO_H

// ESP32-specific code for ingesting audio data from the ADC, before passing it to the
// generalized audio processing code in audio_fft.h.

#include <algorithm>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/adc.h"
#include "driver/i2s.h"

#include "audio_fft.h"
#include "iaudio.h"

#include "config.h"

using std::vector;

const char *tagAudio = "audio";

///////////////////////
// AUDIO CONFIGURATION
// Number of samples to read from the ADC at once.
// At 8000Hz sampling, we will read 125 times per second (and compute this many FFTs) for
// high responsiveness to audio changes (8ms interval).
#define SAMPLES (64)
// In order to provide greater resolution in the FFT, we chain multiple recent samples
// together to give the algorithm more information. The tradeoff is that this increases
// the effective latency of the audio processing - 32ms in this case.
#define SAMPLE_CHAIN_NUM 4
#define TOTAL_CHAIN (SAMPLE_CHAIN_NUM * SAMPLES)

// This is a fixed property of the FFT algorithm; do not modify.
#define FFT_BUCKETS ((TOTAL_CHAIN - 1) / 2 + 1)

///////////////////
// ADVANCED AUDIO

// Should not need to modify these.

#define I2S_NUM (I2S_NUM_0)
// At 8000Hz sampling, the maximum frequency the FFT can capture is 4000Hz. This is not
// super high, but plenty for capturing mids (vocals and melodies) and bass. There is a
// soft cap on sampling rate with the FFT computation; too high and we drop data because
// we max out the ESP32 core.
#define I2S_SAMPLE_RATE (8000)
#define I2S_SAMPLE_BITS (16)
// Read buffer length
#define SAMPLE_BYTES (SAMPLES * 2)
// Pull mono audio only
#define I2S_FORMAT (I2S_CHANNEL_FMT_ONLY_LEFT)
#define I2S_ADC_UNIT ADC_UNIT_1

// ADC and microphone hardware has inherent bias and is never perfectly centered
// in the ADC range, so we need to learn the signal midpoint over time. We do this
// by sampling a certain number of frames at startup, then continuously find a
// moving average over time.

// Number of frames of data to record before establishing a baseline midpoint
#define AUDIO_SIGNAL_LEARNING_FRAMES 100

// Historical fraction for raw data input, used to slowly find the signal midpoint
// over time
#define AUDIO_MIDPOINT_FACTOR 0.999

////////////////////////
// INPUT CONFIGURATION
// In some hardware versions, we support multiple audio inputs (mic + aux) and can
// switch processing between them.
#ifdef AUDIO_INPUT_2
#define NUM_INPUTS 2
#else
#define NUM_INPUTS 1
#endif

struct Input {
    adc1_channel_t channel;
    FLOAT_T noiseFloor[FFT_BUCKETS];
};

const struct Input inputs[NUM_INPUTS] = {AUDIO_INPUT_1
#ifdef AUDIO_INPUT_2
        ,
        AUDIO_INPUT_2
#endif
};

// For diagnostic purposes, we keep track of when the audio is clipping. This is
// used to light a single LED so we can monitor when it happens. Useful for tuning
// microphone gain.
bool audioClipping = false;

class Band : public IBand<FFT_BUCKETS> {
public:
    Band(FLOAT_T minFreq, FLOAT_T maxFreq, FLOAT_T avgSmoothFactor,
         FLOAT_T sanitizedLevelDecay)
            : IBand<FFT_BUCKETS>(minFreq, maxFreq, avgSmoothFactor,
                                 sanitizedLevelDecay) {
        this->valSemaphore = xSemaphoreCreateMutex();
    }

    void update(FLOAT_T buckets[FFT_BUCKETS]) override {
        FLOAT_T maxVal = 0;
        for (size_t i = minBucket; i <= maxBucket; i++) {
            if (buckets[i] > maxVal) {
                maxVal = buckets[i];
            }
        }

        float newVal = this->processSample(maxVal);
        if (xSemaphoreTake(valSemaphore, 1) == pdTRUE) {
            this->val = newVal;
            xSemaphoreGive(valSemaphore);
        } else {
            ESP_LOGE(tagAudio, "Couldn't get semaphore to set audio value!");
        }
    }

    float value() override {
        return this->val;
    }

protected:
    xSemaphoreHandle valSemaphore = nullptr;
};

class Audio : public AudioFFT<SAMPLES, SAMPLE_CHAIN_NUM, I2S_SAMPLE_RATE> {
public:
    explicit Audio()
            : AudioFFT<SAMPLES, SAMPLE_CHAIN_NUM, I2S_SAMPLE_RATE>(inputs[0].noiseFloor, bandModeTopN) {

        this->inputSemaphore = xSemaphoreCreateMutex();
        this->initI2SAdc();
    };

    // Responsible for:
    // - Reading audio data from the I2S ADC
    // - Normalizing the raw sample data from the ADC range [0, 4095] to [-1.0, 1.0]
    // - Passing the normalized data into the rest of FFT processing (this->ingest())
    [[noreturn]] void audioTask() {
        size_t bytesRead;
        uint32_t adcValue = 0;
        uint64_t adcValueFrameTotal = 0;
        double frameAvg = 0;

        char *i2sReadBuf = (char *) calloc(SAMPLE_BYTES, sizeof(char));
        i2s_adc_enable(I2S_NUM);
        i2s_read(I2S_NUM, (void *) i2sReadBuf, SAMPLE_BYTES, &bytesRead, 2);

        while (true) {
            if (xSemaphoreTake(inputSemaphore, 100 / portTICK_RATE_MS) ==
                pdTRUE) {
                // Read audio data from I2S into the buffer
                i2s_read(I2S_NUM, (void *) i2sReadBuf, SAMPLE_BYTES,
                         &bytesRead, 2);

                xSemaphoreGive(inputSemaphore);

                // Total across all samples; used to find an average below for midpoint learning
                adcValueFrameTotal = 0;

                FLOAT_T buffer[SAMPLES] = {};
                if (bytesRead == SAMPLE_BYTES) {
                    int clips = 0;
                    for (size_t i = 0; i < bytesRead / 2; i++) {
                        // Unpack 12-bit ADC value from 16-bit buffer (two bytes per sample)
                        adcValue = ((
                                ((uint16_t) (i2sReadBuf[i * 2 + 1] & 0xf) << 8) |
                                ((i2sReadBuf[i * 2]))));

                        adcValueFrameTotal += adcValue;

                        if (adcValue < 2 || adcValue > 4093) {
                            clips++;
                        }

                        // Only bother with computing bands once we have learned the signal
                        // characteristics (midpoint/divisor)
                        if (signalLearningFrames <= 0) {
                            // Normalize [0, 4095] to [-1.0, 1.0]
                            buffer[i] = ((FLOAT_T) adcValue - signalMidpoint) /
                                        signalDivisor;
                        }
                    }

                    frameAvg = (double) adcValueFrameTotal / (double) SAMPLES;

                    if (signalLearningFrames > 0) {
                        signalLearningTotal += frameAvg;
                        signalLearningFrames--;
                        if (signalLearningFrames == 0) {
                            // We have enough data to establish a baseline midpoint
                            signalMidpoint = signalLearningTotal /
                                             AUDIO_SIGNAL_LEARNING_FRAMES;
                            signalLearningTotal = 0;
                            signalDivisor =
                                    std::max(signalMidpoint,
                                             (FLOAT_T) 4096.0 - signalMidpoint);
                            ESP_LOGI(tagAudio,
                                     "Learned midpoint/divisor: %0.1f, %0.1f",
                                     signalMidpoint, signalDivisor);
                        }
                    } else {
                        // After initial learning, continue to drift the midpoint
                        signalMidpoint =
                                signalMidpoint * AUDIO_MIDPOINT_FACTOR +
                                frameAvg * (1.0 - AUDIO_MIDPOINT_FACTOR);
                        signalDivisor = std::max(
                                signalMidpoint, (FLOAT_T) 4096.0 - signalMidpoint);
                    }

                    // Process the samples
                    this->ingest(buffer);

                    if (clips > SAMPLES / 8) {
                        audioClipping = true;
                    } else {
                        audioClipping = false;
                    }

                } else {
                    ESP_LOGE(tagAudio, "DIDN'T GET ALL SAMPLES!!!");
                }

            } else {
                ESP_LOGE(tagAudio,
                         "Couldn't get semaphore to read audio input!");
            }

            vTaskDelay(1);
        }

        // Never actually reached
        i2s_adc_disable(I2S_NUM);
        free(i2sReadBuf);
        i2sReadBuf = nullptr;

        vTaskDelete(nullptr);
    }

    void switchInput(size_t num) {
        num %= NUM_INPUTS;
        if (num != this->inputNum) {
            if (xSemaphoreTake(inputSemaphore, 100 / portTICK_RATE_MS) ==
                pdTRUE) {
                ESP_LOGE(tagAudio, "Switching to audio input %d!", num);

                std::fill(this->vMax, this->vMax + FFT_BUCKETS, 0);
                std::fill(this->vRealChain, this->vRealChain + TOTAL_CHAIN, 0);
                std::fill(this->vImagChain, this->vImagChain + TOTAL_CHAIN, 0);

                this->inputNum = num;
                this->input = inputs[num];

                this->signalLearningFrames = AUDIO_SIGNAL_LEARNING_FRAMES;
                this->signalLearningTotal = 0;

                memmove(this->noiseFloor, this->input.noiseFloor,
                        FFT_BUCKETS * sizeof(float));

                i2s_adc_disable(I2S_NUM);
                i2s_driver_uninstall(I2S_NUM);
                this->initI2SAdc();
                i2s_adc_enable(I2S_NUM);

                ESP_LOGE(tagAudio, "Done switching to audio input %d", num);
                xSemaphoreGive(inputSemaphore);
            } else {
                ESP_LOGE(tagAudio,
                         "Couldn't get semaphore to set audio input!");
            }
        }
    }

protected:
    // ESP32 I2S ADC configuration
    void initI2SAdc() const {
        adc1_config_channel_atten(input.channel, ADC_ATTEN_11db);
        i2s_config_t i2s_config = {
                .mode = static_cast<i2s_mode_t>(I2S_MODE_MASTER | I2S_MODE_RX |
                                                I2S_MODE_ADC_BUILT_IN),
                .sample_rate = I2S_SAMPLE_RATE,
                .bits_per_sample =
                static_cast<i2s_bits_per_sample_t>(I2S_SAMPLE_BITS),
                .channel_format = static_cast<i2s_channel_fmt_t>(I2S_FORMAT),
                .communication_format =
                static_cast<i2s_comm_format_t>(I2S_COMM_FORMAT_STAND_I2S),
                .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
                .dma_buf_count = 2,
                .dma_buf_len = SAMPLES,
                .use_apll = false,
                .tx_desc_auto_clear = false,
                .fixed_mclk = 0,
                .mclk_multiple = I2S_MCLK_MULTIPLE_256,
                .bits_per_chan = I2S_BITS_PER_CHAN_DEFAULT};
        i2s_driver_install(I2S_NUM, &i2s_config, 0, nullptr);
        i2s_set_adc_mode(I2S_ADC_UNIT, input.channel);
    }

    FLOAT_T signalMidpoint = 2048.0;
    FLOAT_T signalDivisor = 2048.0;
    uint8_t signalLearningFrames = AUDIO_SIGNAL_LEARNING_FRAMES;
    FLOAT_T signalLearningTotal = 0;

    vector<Band *> bands;

    xSemaphoreHandle inputSemaphore = nullptr;
    size_t inputNum = 0;
    struct Input input = inputs[0];
};

#endif
