#ifndef USER_LIB_H
#define USER_LIB_H


#include "stdint.h"


extern  float filter(float temp, float alpha);
extern  float F_Loop_Constrain(float Input, float Min_Value, float Max_Value);  


/* int16 -> float 转换(处理符号位，等价于简单强转) */
float Int16ToFloat(int16_t data);
/* int16 -> int32 转换(处理符号位) */
int Int16ToInt32(int16_t data);

#endif
