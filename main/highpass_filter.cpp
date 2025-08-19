#include "highpass_filter.h"
#include <math.h>

typedef struct {
    float b0, b1, b2;
    float a1, a2;
    float input_prev1;
    float input_prev2;
    float output_prev1;
    float output_prev2;
} BiquadFilter;

static BiquadFilter hp_filter;
static BiquadFilter lp_filter;

void init_highpass_filter(float cutoff_hz, float sample_rate) {
    float omega = 2.0f * M_PI * cutoff_hz / sample_rate;
    float cos_omega = cosf(omega);
    float sin_omega = sinf(omega);
    float Q = sqrtf(0.5f);  // Butterworth Q

    float alpha = sin_omega / (2.0f * Q);

    float b0 =  (1 + cos_omega) / 2;
    float b1 = -(1 + cos_omega);
    float b2 =  (1 + cos_omega) / 2;
    float a0 =  1 + alpha;
    float a1 = -2 * cos_omega;
    float a2 =  1 - alpha;

    hp_filter.b0 = b0 / a0;
    hp_filter.b1 = b1 / a0;
    hp_filter.b2 = b2 / a0;
    hp_filter.a1 = a1 / a0;
    hp_filter.a2 = a2 / a0;

    hp_filter.input_prev1 = 0;
    hp_filter.input_prev2 = 0;
    hp_filter.output_prev1 = 0;
    hp_filter.output_prev2 = 0;
}

int16_t apply_highpass_filter(int16_t input_sample) {
    float x = (float)input_sample;

    float y = hp_filter.b0 * x +
              hp_filter.b1 * hp_filter.input_prev1 +
              hp_filter.b2 * hp_filter.input_prev2 -
              hp_filter.a1 * hp_filter.output_prev1 -
              hp_filter.a2 * hp_filter.output_prev2;

    // Cập nhật bộ nhớ
    hp_filter.input_prev2 = hp_filter.input_prev1;
    hp_filter.input_prev1 = x;
    hp_filter.output_prev2 = hp_filter.output_prev1;
    hp_filter.output_prev1 = y;

    // Clamp
    if (y > 32767.0f) y = 32767.0f;
    if (y < -32768.0f) y = -32768.0f;

    return (int16_t)y;
}

void init_lowpass_filter(float cutoff_hz, float sample_rate) {
    float omega = 2.0f * M_PI * cutoff_hz / sample_rate;
    float cos_omega = cosf(omega);
    float sin_omega = sinf(omega);
    float Q = sqrtf(0.5f);  // Butterworth

    float alpha = sin_omega / (2.0f * Q);

    float b0 = (1 - cos_omega) / 2;
    float b1 = 1 - cos_omega;
    float b2 = (1 - cos_omega) / 2;
    float a0 = 1 + alpha;
    float a1 = -2 * cos_omega;
    float a2 = 1 - alpha;

    lp_filter.b0 = b0 / a0;
    lp_filter.b1 = b1 / a0;
    lp_filter.b2 = b2 / a0;
    lp_filter.a1 = a1 / a0;
    lp_filter.a2 = a2 / a0;

    lp_filter.input_prev1 = 0;
    lp_filter.input_prev2 = 0;
    lp_filter.output_prev1 = 0;
    lp_filter.output_prev2 = 0;
}

int16_t apply_lowpass_filter(int16_t input_sample) {
    float x = (float)input_sample;

    float y = lp_filter.b0 * x +
              lp_filter.b1 * lp_filter.input_prev1 +
              lp_filter.b2 * lp_filter.input_prev2 -
              lp_filter.a1 * lp_filter.output_prev1 -
              lp_filter.a2 * lp_filter.output_prev2;

    lp_filter.input_prev2 = lp_filter.input_prev1;
    lp_filter.input_prev1 = x;
    lp_filter.output_prev2 = lp_filter.output_prev1;
    lp_filter.output_prev1 = y;

    if (y > 32767.0f) y = 32767.0f;
    if (y < -32768.0f) y = -32768.0f;

    return (int16_t)y;
}
