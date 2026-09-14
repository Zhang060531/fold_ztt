#ifndef RLS_H
#define RLS_H


#include "Config.h"
#include "stdint.h"
#include "arm_math.h"

/* ==== 文件说明 ================================================================
 * RLS.h —— 递推最小二乘（Recursive Least Squares）头文件
 * ---------------------------------------------------------------------------
 * 作用：在线估计"输入 X 到输出 Y 的线性权重 W"（即 Y ≈ W·X），并且能随新数据
 *       不断更新 W，无需保存历史数据。常用于系统参数辨识（如功率/电流模型）。
 *
 * 核心思想：
 *   误差  E(n) = Y(n) - W(n-1)·X(n)       （预测差了多少）
 *   增益  K(n) = P(n-1)·X / (λ + XT·P·X)  （λ 是遗忘因子）
 *   权重  W(n) = W(n-1) + K(n)·E(n)       （按误差修正权重）
 *   协方差 P(n) = (I - K·XT)·P(n-1) / λ   （更新"记忆"，供下一轮用）
 *
 * 遗忘因子 Lamda(λ)：0<λ≤1。λ 越接近 1，记得越久、估计越稳但收敛慢；
 *   λ 越小，越"健忘"、能快速跟踪时变参数但容易抖。
 * 在整车里的用途：辨识云台/底盘的功率、电流与负载之间的关系模型。
 * ========================================================================== */

#define Matrix             arm_matrix_instance_f32
#define Matrix_64          arm_matrix_instance_f64
#define Matrix_Init        arm_mat_init_f32
#define Matrix_Add         arm_mat_add_f32
#define Matrix_Subtract    arm_mat_sub_f32
#define Matrix_Multiply    arm_mat_mult_f32
#define Matrix_Transpose   arm_mat_trans_f32
#define Matrix_Inverse     arm_mat_inverse_f32
#define Matrix_Inverse_64  arm_mat_inverse_f64


/* 递推最小二乘句柄。字段含义：
 *   X / Y : 输入（回归向量）/ 输出（测量）—— 我们要找 Y ≈ W·X 里的 W
 *   W     : 权重矩阵，即要在线估计的参数
 *   P     : 协方差矩阵（算法内部维护的"记忆"，越大说明估计越不确定）
 *   E     : 误差 = Y - W·X
 *   K     : 增益
 *   Lamda : 遗忘因子 λ（0<λ≤1，越小越健忘、越能跟踪时变参数）
 * Mat 存矩阵结构体实例，Data 存对应内存指针（真正存数的数组）。 */
typedef struct
{

  uint8_t sizeof_float;
	
	uint8_t X_Size;
	uint8_t Y_Size;
	uint8_t P_Size;
	float Lamda; //forgetting factor 遗忘因子 λ（0~1，越小越健忘、收敛越快但越抖）

 struct 
  {
    Matrix X; 
    Matrix XT; 
    Matrix Lamda; 		
    Matrix E;                 
    Matrix Z;
    Matrix K;  		
    Matrix W;         		
    Matrix P;
    Matrix Y;
    Matrix U;		
    Matrix K_Numerator;                 
    Matrix K_Denominator;    
    Matrix Cache_Matrix[2];   
    Matrix Cache_Vector[2];   
		Matrix Output;  
  }Mat;
	
	 arm_status MatStatus;
	
	 struct 
  {
    float *X;
    float	*XT;
    float *Lamda;	
    float *E;                 
    float *Z;
    float *K;		
    float *W; 
		float *Y;		
    float *U;                 
    float *P;                 
    float *K_Numerator;                 
    float *K_Denominator;    
    float *Cache_Matrix[2];   
    float *Cache_Vector[2];  
    float *Output;
 		
  }Data;

	
	
	
}Recursive_Least_Squares_Info_TypeDef;


/* 全局唯一的功率控制 RLS 句柄。 */
extern Recursive_Least_Squares_Info_TypeDef Power_Control_RLS_Info;

/* 执行一次递推最小二乘更新（调用前需填好 RLS->Y、RLS->X、RLS->Lamda）。 */
extern void RLS_Update(Recursive_Least_Squares_Info_TypeDef *RLS);
/* 初始化 RLS：X_Size=输入维数，P_Size=协方差维数，Y_Size=输出维数。 */
extern void RLS_Init(Recursive_Least_Squares_Info_TypeDef *RLS,uint8_t X_Size,uint8_t P_Size,uint8_t Y_Size);




#endif