/*
 * functions.h
 *
 * Created: 03/05/2019 14:20:19
 *  Author: D�niel Buga
 */


#ifndef UTILS_FUNCTIONS_H_
#define UTILS_FUNCTIONS_H_

#include <stdio.h>
#include <stdint.h>
#include <float.h>

float constrain_f32(float in, float min, float max);
int32_t constrain_int32(int32_t value, int32_t min, int32_t max);
int16_t constrain_int16(int16_t value, int16_t min, int16_t max);
int8_t constrain_int8(int8_t value, int8_t min, int8_t max);

int32_t sgn_int32(int32_t value);
float sgn_float(float value);

float map(float in, float min_in, float max_in, float min_out, float max_out);
float map_constrained(float in, float min_in, float max_in, float min_out, float max_out);
uint32_t SwapEndian(uint32_t dig);

#endif /* UTILS_FUNCTIONS_H_ */