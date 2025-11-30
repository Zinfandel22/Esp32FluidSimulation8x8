#ifndef UTILS_H
#define UTILS_H

#include "config.h"

namespace utils {

// clamp value between min and max inlined for performance
// flaot version
inline float __attribute__((always_inline)) clamp(float value, float min_val, float max_val) {
  if (value < min_val) return min_val;
  if (value > max_val) return max_val;
  return value;
}
//int version
inline int __attribute__((always_inline)) clamp(int value, int min_val, int max_val) {
  if (value < min_val) return min_val;
  if (value > max_val) return max_val;
  return value;
}

// quake III fast inverse square root algorithm
// inlined for performance
inline float __attribute__((always_inline)) fastInvSqrt(float number) {
  long i;
  float x2, y;
  const float threehalfs = 1.5F;

  x2 = number * 0.5F;
  y = number;
  i = *(long*)&y;                             // treat float bits directly as an integer to allow bit manipulation
  i = 0x5f3759df - (i >> 1);                  // generate initial approximation using the magic number constant
  y = *(float*)&i;                            // convert the integer bits back into a floating point number
  y = y * (threehalfs - (x2 * y * y));        // perform one iteration of newton's method to improve precision
  
  return y;
}

// qmi8658 sensor functions
bool initQMI8658();
void readQMI8658Accel(float* x, float* y, float* z);
void writeRegister(uint8_t reg, uint8_t value);
uint8_t readRegister(uint8_t reg);

}  // namespace utils

#endif