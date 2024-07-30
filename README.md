Audio processing code for `polyhedra`.

This is part of a wider codebase, so would need to be plumbed in. It is designed
to accept audio input from a fixed gain microphone or clean input (e.g. line
in)- the mic should NOT use AGC, as the code will try to account for volume
changes on its own.

The interface is fairly straightforward:

- `AudioFFT` is a generalized (platform agnostic) class for processing raw
  incoming audio data with samples in the range [-1.0, 1.0]. It applies an FFT,
  then processes and normalizes the FFT buckets into several "bands". Each band
  then provides a clean value between [0.0, 1.0] which indicates the relative
  volume of the band, after applying volume normalization, smoothing, etc. These
  bands can be used to drive effects.
    - `AudioFFT::ingest()` accepts this data.
    - `AudioFFT::bandVals()` returns the current band values.
- `Audio` is an ESP32-specific class that extends `AudioFFT`. It includes a
  [FreeRTOS](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/freertos_idf.html)
  task which reads audio data from I2S. This is designed to be run on its own
  ESP32 core. FFTs are relatively expensive, and I wouldn't recommend running
  this serially with animation code, as it could kill framerate pretty quickly,
  and wind up dropping audio data if the processor hits full utilization.
- Alternatively, `AudioFFT` can be extended and passed data from ANY source -
  e.g. I use this to pass a loopback audio stream when simulating the code on a
  PC.

The messiest part is by far the normalization code in `Band::processSample()`. I
have done a ton of iterating on this to try to perfect the behavior in loud
festival environments, e.g. walking between stages. The code also has to account
for variances in hardware, which takes two forms:

- Automatic midpoint setting. ADC inputs tend to have bias - a flat line in
  might not perfectly align with the center of the ADC range. Accounting for
  this seems to help out the FFT algorithm.
- Noise floor - this is a hard-coded/configured array of values that capture the
  noise floor for the current hardware. It is determined by running the code in
  a quiet environment and observing the minimum values of each band over a few
  seconds (this debugging code has been removed).

The [arduinoFFT library](https://github.com/kosme/arduinoFFT) is utilized. FHT
might be ever so slightly more performant with the right library, but FFT has
worked fine.
