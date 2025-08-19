#ifndef HIGHPASS_FILTER_H
#define HIGHPASS_FILTER_H

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void init_highpass_filter(float cutoff_hz, float sample_rate);
int16_t apply_highpass_filter(int16_t input_sample);

void init_lowpass_filter(float cutoff_hz, float sample_rate);
int16_t apply_lowpass_filter(int16_t input_sample);

#ifdef __cplusplus
}
#endif


#endif // HIGHPASS_FILTER_H