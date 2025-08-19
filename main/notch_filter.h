#ifndef NOTCH_FILTER_H
#define NOTCH_FILTER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif
void init_notch_filter(float fs, float notch_freq, float q);

int16_t apply_notch_filter(int16_t input);

#ifdef __cplusplus
}
#endif

#endif // NOTCH_FILTER_H