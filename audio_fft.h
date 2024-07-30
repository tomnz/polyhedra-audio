#ifndef AUDIO_FFT_H
#define AUDIO_FFT_H

// Generalized code which accepts raw audio data in the range [-1.0f, 1.0f] and
// processes it into several [0.0f, 1.0f] bands with heavy smoothing and
// normalization. These bands can then be used to drive effects.

#include <algorithm>
#include <utility>
#include <vector>

#include "iaudio.h"

using std::vector;

// Set to float or double to modify FFT/audio precision - float is more
// performant, double is more precise
#define FLOAT_T float
// FFT_SAMPLE_T must also be modified in the arduinoFFT submodule
// Depends on https://www.arduino.cc/reference/en/libraries/arduinofft/
#include "arduinoFFT.h"

////////////////////////////
// AUDIO PROCESSING CONFIG

// NOTES
// Smoothing operations happen via a simple "factor", calculated as:
// (historicValue * factor + newValue) / (factor + 1)
// A higher "factor" means a slower-moving average, due to greater weighting of
// the historic value.
// "Frame" means a single FFT calculation - 125/s in the current configuration.

// BAND NORMALIZATION
// We keep track of the highest value seen in each band over time, in order to
// adjust to volume changes. This is maintained as a slow-moving average. Higher
// peaks will slowly push it upwards, while lower valleys will pull it downwards
// more quickly. Note that this is not QUITE volume normalization, but rather
// normalization of the FFT values.

// How quickly to push the average up/down when the current level is higher/lower
// Higher number means lower sensitivity to volume changes
#define HISTORIC_SMOOTH_FACTOR_UP 15000.0f
#define HISTORIC_SMOOTH_FACTOR_DOWN 6000.0f

// SMOOTHING
// Apply a slight smooth to the final output level.
#define SMOOTH_FACTOR 3.0f

// BAND RANGE
// Historic band frames to keep to determine full high/low range - this is used to
// set the minimum and maximum levels for the band, in order to have the current
// level be a percentage of this range.
// This is 2.4s of data at 125 frames/s.
#define HISTORIC_FRAMES 300

// SANITIZED BAND LEVEL
// Sanitized level is designed to be a less "jumpy" level source. It will always
// be in the range [0.0, 1.0]. It operates in a peak/decay fashion - if the
// current level is higher, it will jump to that level, but then decays at a set
// rate if the level drops again.
#define SANITIZED_LEVEL_SCALE 1.7f
// The incoming level signal must be greater than this minimum to have any
// effect - helps to cut baseline flickering
#define SANITIZED_LEVEL_MIN 0.1f
// Fixed decay rate for the sanitized level if the raw level is lower
#define SANITIZED_LEVEL_DECAY 0.1f

// Minimum range for the bar - helps to prevent flickering during quiet periods
#define MIN_BAR_SIZE 0.1f

// Fraction of available FFT buckets to use within a band for top-N calculation
#define TOP_N_FRACTION 0.3f

// Band mode determines how a band's value is calculated if multiple frequency
// buckets fall within the band
enum BandMode {
    bandModeMax, bandModeAverage, bandModeTopN
};

template<int pFFTBuckets>
class IBand {
public:
    IBand(FLOAT_T minFreq, FLOAT_T maxFreq, FLOAT_T avgSmoothFactor,
          FLOAT_T sanitizedLevelDecay, BandMode bandMode)
            : minFreq(minFreq), maxFreq(maxFreq), avgSmoothFactor(avgSmoothFactor),
              sanitizedLevelDecay(sanitizedLevelDecay), bandMode(bandMode) {}

    IBand(FLOAT_T minFreq, FLOAT_T maxFreq, FLOAT_T avgSmoothFactor,
          FLOAT_T sanitizedLevelDecay)
            : minFreq(minFreq), maxFreq(maxFreq), avgSmoothFactor(avgSmoothFactor),
              sanitizedLevelDecay(sanitizedLevelDecay), bandMode(bandModeMax) {}

    void init(const FLOAT_T freqs[pFFTBuckets]) {
        for (int i = 0; i < pFFTBuckets; i++) {
            if (freqs[i] > this->minFreq && minBucket == 0) {
                minBucket = i;
            }
            if (freqs[i] < maxFreq) {
                maxBucket = i;
            } else {
                break;
            }
        }
        topNCount = std::max(
                (int) ((float) (maxBucket - minBucket + 1) * TOP_N_FRACTION), 1);
    }

    // Pull the current frame's value from the FFT buckets using the band mode,
    // then pass it to processSample.
    virtual void update(FLOAT_T buckets[pFFTBuckets]) {
        if (bandMode == bandModeMax) {
            FLOAT_T maxVal = 0;
            for (int i = minBucket; i <= maxBucket; i++) {
                if (buckets[i] > maxVal) {
                    maxVal = buckets[i];
                }
            }
            this->val = this->processSample(maxVal);
        } else if (bandMode == bandModeAverage) {
            FLOAT_T avgVal = 0;
            for (int i = minBucket; i <= maxBucket; i++) {
                avgVal += std::max(std::min(buckets[i], 1.0f), 0.0f);
            }
            avgVal /= (FLOAT_T) (maxBucket - minBucket + 1);
            this->val = this->processSample(avgVal);
        } else if (bandMode == bandModeTopN) {
            vector<FLOAT_T> sortedBands;
            for (int i = minBucket; i <= maxBucket; i++) {
                sortedBands.push_back(std::max(buckets[i], 0.0f));
            }
            std::sort(sortedBands.begin(), sortedBands.end(),
                      std::greater<FLOAT_T>());
            FLOAT_T avgVal = 0;
            for (int i = 0; i < topNCount; i++) {
                avgVal += sortedBands[i];
            }
            avgVal /= topNCount;
            this->val = this->processSample(avgVal);
        }
    }

    virtual float value() { return this->val; }

protected:
    // Incorporate the current frame's sample into the band's value.
    // This is where all the smoothing and normalization logic is applied to produce
    // a clean band value for downstream effects to utilize.
    float processSample(FLOAT_T level) {
        level = std::max(std::min(level, 1.0f), 0.0f);
        // Maintain the last N frames seen for this band. Configured to be about 2.4s of
        // data at 125 frames/s.
        this->history[this->historyFrame] = level;
        this->historyFrame = (this->historyFrame + 1) % HISTORIC_FRAMES;
        // Find the min/max levels in the history window.
        FLOAT_T minLevel, maxLevel, histVal;
        minLevel = maxLevel = this->history[0];
        for (size_t i = 1; i < HISTORIC_FRAMES; i++) {
            histVal = this->history[i];
            if (histVal > maxLevel)
                maxLevel = histVal;
            if (histVal < minLevel)
                minLevel = histVal;
        }

        // Slightly smooth the min/max levels over time to prevent jumpiness of the
        // band's range.
        this->minLevelAvg =
                (this->minLevelAvg * this->avgSmoothFactor + minLevel) /
                (this->avgSmoothFactor + 1.0f);
        this->maxLevelAvg =
                (this->maxLevelAvg * this->avgSmoothFactor + maxLevel) /
                (this->avgSmoothFactor + 1.0f);
        // Forcefully push up the max if it's higher so we can quickly respond to peaks.
        if (maxLevel > this->maxLevelAvg)
            this->maxLevelAvg = maxLevel;

        // Slightly smooth the current level over time.
        this->smoothedLevel = (this->smoothedLevel * SMOOTH_FACTOR + level) /
                              (SMOOTH_FACTOR + 1.0f);
        // Apply the smoothed current level to the historic level - this is the main
        // mechanism for "volume" normalization.
        FLOAT_T historicSmoothFactor = HISTORIC_SMOOTH_FACTOR_DOWN;
        if (this->smoothedLevel > this->historicLevel) {
            historicSmoothFactor = HISTORIC_SMOOTH_FACTOR_UP;
        }
        this->historicLevel =
                (this->historicLevel * historicSmoothFactor + this->smoothedLevel) /
                (historicSmoothFactor + 1.0f);

        // Set the final min/max range to be used for computing the current frame's
        // percentage value within the band's range.
        // Use the normalized "volume" level as the hard minimum - this lets us know if
        // we are just in a temporarily quiet part of the song - but if the historic
        // minimum (last 2.4s) is higher, we use that instead to allow for the bar to
        // be more "dynamic" and touch the lower end of the range.
        this->minLevelCurrent =
                std::max(this->historicLevel * 0.3f, this->minLevelAvg * 0.7f);
        // Similar for maximum - but we also provide a fixed minimum size for the bar
        // via MIN_BAR_SIZE. This helps for relatively static environments - the level
        // should sit at the bottom of the bar. Without the MIN_BAR_SIZE, level would
        // erratically bounce between low and high in those situations. This is important
        // for keeping the effects quiet when e.g. moving between stages at a festival.
        this->maxLevelCurrent = std::max(
                this->maxLevelAvg,
                this->minLevelCurrent + std::max(MIN_BAR_SIZE * this->maxLevelAvg,
                                                 this->historicLevel * 0.6f));

        // Everything up to this point has been about finding the min/max levels
        // of the band's range over time. This final section is about creating a
        // "sanitized" CURRENT level within that band. This includes some light
        // peak/decay and smoothing logic.

        // Get the instantaneous percentage value for the current frame within the band.
        this->transformedLevel =
                (this->smoothedLevel - this->minLevelCurrent) /
                (this->maxLevelCurrent - this->minLevelCurrent);

        // Perform some light sanitization to make the level less flickery.
        FLOAT_T newSanitizedLevel =
                (this->transformedLevel - SANITIZED_LEVEL_MIN) *
                SANITIZED_LEVEL_SCALE;
        if (newSanitizedLevel > this->sanitizedLevel) {
            this->sanitizedLevel = std::min(newSanitizedLevel, 1.0f);
        } else {
            this->sanitizedLevel -= this->sanitizedLevelDecay;
        }
        this->sanitizedLevel = std::max(0.0f, this->sanitizedLevel);

        return (float) (std::max(0.0f, std::min(1.0f, this->sanitizedLevel)));
    }

    float val = 0;
    FLOAT_T minFreq = 0, maxFreq = 0, avgSmoothFactor = 0,
            sanitizedLevelDecay = 0;
    size_t minBucket = 0, maxBucket = 0;

    FLOAT_T smoothedLevel = 0;
    FLOAT_T historicLevel = 0.1;
    FLOAT_T sanitizedLevel = 0;
    FLOAT_T minLevelCurrent = 1.0, maxLevelCurrent = 1.0,
            transformedLevel = 1.0;
    FLOAT_T minLevelAvg = 0, maxLevelAvg = 0;

    FLOAT_T history[HISTORIC_FRAMES] = {};
    int historyFrame = 0;

    int topNCount = 0;

    BandMode bandMode;
};

template<int pSamples, int pSampleChain, int pSampleRate,
        int pTotalSamples = (pSamples * pSampleChain),
        int pFFTBuckets = ((pTotalSamples - 1) / 2 + 1)>
class AudioFFT : public IAudio {
public:
    AudioFFT(const FLOAT_T noiseFloor[pFFTBuckets],
             BandMode bandMode = bandModeAverage)
            : AudioFFT(bandMode) {

        memmove(this->noiseFloor, noiseFloor, pFFTBuckets * sizeof(float));
        for (int i = 0; i < pFFTBuckets; i++) {
            this->vMax[i] = this->noiseFloor[i] + 0.1f;
        }
    }


    explicit AudioFFT(BandMode bandMode = bandModeAverage)
            : IAudio() {
        this->bands = vector<IBand<pFFTBuckets> *>();
        // Four bands: bass, mid1, mid2, high
        // Note the additional smoothing and slower decay for higher bands - this gives heavy bass
        // hits more "oomph" with a slower roll-off, while the trebles are more responsive to quick
        // changes.
        this->bands.push_back(new IBand<pFFTBuckets>(
        // minFreq, maxFreq, avgSmoothFactor, sanitizedLevelDecay, bandMode
                30, 110, 50.0f, SANITIZED_LEVEL_DECAY, bandMode));
        this->bands.push_back(new IBand<pFFTBuckets>(
                110, 300, 42.0f, SANITIZED_LEVEL_DECAY * 1.2, bandMode));
        this->bands.push_back(new IBand<pFFTBuckets>(
                300, 3000, 35.0f, SANITIZED_LEVEL_DECAY * 1.6, bandMode));
        // In reality, the maximum frequency we observe at 8000 samples/s is 4000Hz, but
        // we say 22000Hz to ensure that it gets everything available at the high end.
        this->bands.push_back(new IBand<pFFTBuckets>(
                3000, 22000, 30.0f, SANITIZED_LEVEL_DECAY * 2.5, bandMode));

        this->initBuckets();
    }

    // Ingest raw audio data, apply FFT, and update bands with new FFT data.
    // Expects data in the range [-1.0f, 1.0f] (normalized from [0, 4095] in the case of ESP32 I2S)
    // Expects number of samples to equal pSamples
    void ingest(FLOAT_T *data) override {
        for (size_t i = 0; i < pSamples; i++) {
            vReal[this->chain][i] = data[i];
            vImag[this->chain][i] = 0;
        }

        this->chain = (this->chain + 1) % pSampleChain;

        // Build sample chain from all chunks.
        for (size_t i = 0; i < pSampleChain; i++) {
            int chainNum = (this->chain + i) % pSampleChain;
            std::copy_n(this->vReal[chainNum], pSamples,
                        this->vRealChain + i * pSamples);
            std::copy_n(this->vImag[chainNum], pSamples,
                        this->vImagChain + i * pSamples);
        }

        // FFT is delegated to a library...
        // An important note is that we just pass the raw audio data to the FFT without
        // doing any volume normalization at this step. FFT extracts the frequency
        // information, so the output data will have corresponding magnitudes in each
        // frequency band - but it has enough precision to retain all the information we
        // need. We then normalize the FFT output for volume in the next step.
        arduinoFFT fft(this->vRealChain, this->vImagChain,
                       (uint16_t) pTotalSamples, pSampleRate);

        // Windowing is super important.
        fft.Windowing(FFT_WIN_TYP_BLACKMAN, FFT_FORWARD);
        fft.Compute(FFT_FORWARD);
        // Discard the imaginary component.
        fft.ComplexToMagnitude();

        for (size_t i = 0; i < pFFTBuckets; i++) {
            // We keep track of the maximum value seen in each FFT bucket over time. The real
            // value is divided by the maximum value to normalize it to the range [0.0, 1.0].
            // For whatever reason, the raw FFT values can be quite high (e.g. 30). This
            // normalization is important to be able to compare values across buckets - the
            // maximum tends to trend from high in the bass frequencies to low in the trebles,
            // even for relatively flat sound.
            if (this->vRealChain[i] > this->vMax[i]) {
                this->vMax[i] = this->vRealChain[i];
            }

            // Noise floor is hard-coded/configured for the hardware - depends
            // on the quality of mic, ADC, noise in the circuit, environment, etc.
            // This is done by printing the maximum observed values for each FFT bucket
            // over a period of time in a quiet environment.
            // An example config might look like:
            //#define AUDIO_INPUT_1                                                          \
            //    {                                                                          \
            //        AUDIO_CHANNEL_1, {                                                     \
            //            0.3356, 0.3332, 0.2685, 0.2485, 0.1797, 0.1032, 0.1146, 0.1144,    \
            //                0.1202, 0.1468, 0.1613, 0.1250, 0.0928, 0.0973, 0.0922,        \
            //                0.0733, 0.0828, 0.0651, 0.0470, 0.0486, 0.0456, 0.0472,        \
            //                0.0494, 0.0413, 0.0352, 0.0377, 0.0373, 0.0356, 0.0329,        \
            //                0.0405, 0.0418, 0.0349, 0.0387, 0.0377, 0.0340, 0.0381,        \
            //                0.0319, 0.0347, 0.0349, 0.0315, 0.0318, 0.0312, 0.0289,        \
            //                0.0261, 0.0251, 0.0259, 0.0235, 0.0316, 0.0302, 0.0336,        \
            //                0.0357, 0.0258, 0.0278, 0.0282, 0.0341, 0.0259, 0.0294,        \
            //                0.0282, 0.0292, 0.0309, 0.0361, 0.0384, 0.0293, 0.0286,        \
            //                0.0282, 0.0412, 0.0398, 0.0297, 0.0317, 0.0252, 0.0296,        \
            //                0.0295, 0.0292, 0.0304, 0.0305, 0.0325, 0.0277, 0.0251,        \
            //                0.0252, 0.0291, 0.0313, 0.0305, 0.0311, 0.0319, 0.0335,        \
            //                0.0324, 0.0335, 0.0308, 0.0285, 0.0326, 0.0384, 0.0341,        \
            //                0.0304, 0.0319, 0.0294, 0.0330, 0.0340, 0.0324, 0.0370,        \
            //                0.0335, 0.0334, 0.0327, 0.0337, 0.0320, 0.0328, 0.0347,        \
            //                0.0361, 0.0379, 0.0375, 0.0514, 0.0469, 0.0378, 0.0459,        \
            //                0.0446, 0.0413, 0.0420, 0.0457, 0.0461, 0.0482, 0.0395,        \
            //                0.0361, 0.0318, 0.0298, 0.0300, 0.0319, 0.0367, 0.0342, 0.0281 \
            //        }                                                                      \
            //    }
            // (Note the higher baseline values for low frequencies - this is typical)

            FLOAT_T fftNoiseFloor = this->noiseFloor[i];
            this->vRealChain[i] = (this->vRealChain[i] - fftNoiseFloor) /
                                  (this->vMax[i] - fftNoiseFloor);
        }

        for (size_t i = 0; i < this->bands.size(); i++) {
            this->bands[i]->update(this->vRealChain);
        }
    }

    vector<float> bandVals() override {
        vector<float> vals = vector<float>();
        for (size_t i = 0; i < this->bands.size(); i++) {
            vals.push_back(this->bands[i]->value());
        }
        return vals;
    }

protected:
    void initBuckets() {
        FLOAT_T freqs[pFFTBuckets] = {};
        // Compute the frequency (Hz) of each FFT bucket so that the bands know
        // which buckets to pull from.
        FLOAT_T val =
                1.0f / ((FLOAT_T) (pSampleChain * pSamples) / (FLOAT_T) pSampleRate);
        for (int i = 0; i < pFFTBuckets; i++) {
            freqs[i] = (FLOAT_T) i * val;
        }
        for (int i = 0; i < this->bands.size(); i++) {
            this->bands[i]->init(freqs);
        }
    }

    vector<IBand<pFFTBuckets> *> bands;

    FLOAT_T vReal[pSampleChain][pSamples] = {};
    FLOAT_T vImag[pSampleChain][pSamples] = {};
    FLOAT_T vMax[pFFTBuckets] = {};
    FLOAT_T vMax2[pFFTBuckets] = {};

    int chain = 0;

    FLOAT_T vRealChain[pTotalSamples] = {};
    FLOAT_T vImagChain[pTotalSamples] = {};

    FLOAT_T noiseFloor[pFFTBuckets] = {};
};

#endif
