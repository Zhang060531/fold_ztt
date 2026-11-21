#ifndef USER_LIB_H
#define USER_LIB_H

#include "stdint.h"
#include "math.h"
#include "stdbool.h"

#include "Motor.h"

extern  float ramp_speed(float last_speed, float target_speed, float max_step);
extern  float F_Loop_Constrain(float Input, float Min_Value, float Max_Value);
extern  void ConstrainLoop(float* output, float input, float min, float max);
extern  float uint_to_float(int X_int, float X_min, float X_max, int Bits);
extern  int float_to_uint(float x, float x_min, float x_max, int bits);
extern  void SectionTransform(float* output, float input, float out_min, float out_zero, float out_max, float in_min, float in_zero, float in_max);
float filter_LPF1(float temp,float alpha);
float PreventFullCircleConstrain(float set, float fdb);
void MinMaxConstrain(float* input, float min, float max);

#endif


