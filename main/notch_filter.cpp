#include "notch_filter.h"
#include <math.h>

// Internal state variables with safe prefixes
static float z_x1 = 0.0f, z_x2 = 0.0f;
static float z_y1 = 0.0f, z_y2 = 0.0f;

// Notch filter coefficients for ~76 Hz, Q ~30, fs = 48000 Hz
static float a0 = 0.978030f;
static float a1 = -1.902113f;
static float a2 = 0.978030f;
static float b1 = -1.902113f;
static float b2 = 0.956060f;

int16_t apply_notch_filter(int16_t input) {
    float x0 = (float)input;
    float y0 = a0 * x0 + a1 * z_x1 + a2 * z_x2 - b1 * z_y1 - b2 * z_y2;

    // Update history
    z_x2 = z_x1;
    z_x1 = x0;
    z_y2 = z_y1;
    z_y1 = y0;

    // Clamp result to int16_t
    if (y0 > INT16_MAX) y0 = INT16_MAX;
    if (y0 < INT16_MIN) y0 = INT16_MIN;

    return (int16_t)y0;
}

void init_notch_filter(float fs, float notch_freq, float q) {
    float omega = 2.0f * M_PI * notch_freq / fs;
    float alpha = sinf(omega) / (2.0f * q);

    float b0 = 1.0f;
    float b1_ = -2.0f * cosf(omega);
    float b2_ = 1.0f;
    float a0_ = 1.0f + alpha;
    float a1_ = -2.0f * cosf(omega);
    float a2_ = 1.0f - alpha;

    a0 = b0 / a0_;
    a1 = b1_ / a0_;
    a2 = b2_ / a0_;
    b1 = a1_ / a0_;
    b2 = a2_ / a0_;
}