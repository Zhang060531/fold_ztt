#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
#include "arm_math.h"
#include "Config.h"

#ifndef user_malloc
#ifdef _CMSIS_OS_H
	#define user_malloc pvPortMalloc
#else
	#define User_malloc malloc
#endif
#endif

/* ==== 文件说明 ================================================================
 * Kalman_Filter.h —— 通用（线性）卡尔曼滤波器头文件
 * ---------------------------------------------------------------------------
 * 作用：实现标准卡尔曼滤波。它把"模型预测"和"传感器测量"按各自的不确定度
 *       （噪声 Q、R）加权融合，得到比单一来源更平滑、更可信的状态估计。
 *
 * 核心思路（5 个经典步骤，对应 Kalman_Filter.c 里的函数）：
 *   1) 预测状态   Xhatminus = A·Xhat + B·U
 *   2) 预测协方差 Pminus    = A·P·AT + Q
 *   3) 计算增益   K         = Pminus·HT / (H·Pminus·HT + R)
 *   4) 更新状态   Xhat      = Xhatminus + K·(Z - H·Xhatminus)
 *   5) 更新协方差 P         = (I - K·H)·Pminus
 *
 * 关键矩阵含义：
 *   A 状态转移矩阵（描述系统自身如何演化）、B 控制矩阵（外部输入如何影响状态）、
 *   H 测量矩阵（状态 -> 测量的映射）、Q 过程噪声协方差（模型多不可信）、
 *   R 测量噪声协方差（传感器多不可信）、P 估计误差协方差、K 卡尔曼增益。
 *
 * 新手口诀：Q 越大越信测量，R 越大越信模型；K 就是"模型与测量之间的权衡权重"。
 * 在整车里的用途：姿态解算的 EKF（Quaternion.c）就是在本文件基础上扩展而来。
 * 注意：本文件依赖 CMSIS-DSP 库（arm_math.h）做矩阵运算，需开启 FPU。
 * ========================================================================== */

/**
 * @brief macro definition of the matrix calculation.
 * 中文：把 CMSIS-DSP 的矩阵类型/运算函数起个短别名，方便书写。
 */
#define Matrix             arm_matrix_instance_f32
#define Matrix_64          arm_matrix_instance_f64
#define Matrix_Init        arm_mat_init_f32
#define Matrix_Add         arm_mat_add_f32
#define Matrix_Subtract    arm_mat_sub_f32
#define Matrix_Multiply    arm_mat_mult_f32
#define Matrix_Transpose   arm_mat_trans_f32
#define Matrix_Inverse     arm_mat_inverse_f32
#define Matrix_Inverse_64  arm_mat_inverse_f64

/* 中文：以上宏只是"别名"——例如 Matrix_Multiply(A,B,C) 实际调用 arm_mat_mult_f32(A,B,C)。
 * 注意：CMSIS 矩阵运算要求维度严格匹配，否则返回 ARM_MATH_SIZE_MISMATCH（运行出错）。 */

/* Exported types ------------------------------------------------------------*/
/**
 * @brief typedef structure that contains the information  for the Chi Square Test.
 * 中文：卡方检验结构体，用来检测"测量值是否明显偏离模型预测"（即异常测量/野值检测）。
 */
typedef struct
{
  bool TestFlag;    /*!< Enable/Disable Flag 本次检验是否启用（满足条件才置 true） */
  Matrix ChiSquare_Matrix;   /*!< chi square test matrix 卡方检验结果矩阵（1x1） */
  float ChiSquare_Data[1];    /*!< chi square test matrix data 卡方检验矩阵的数据存储 */
  float ChiSquareTestThresholds;    /*!< chi square test matrix Thresholds 检验阈值 */
  uint8_t ChiSquareCnt;   /*!< chi square test count 连续异常的计数 */
  bool Result;   /*!< chi square test result 检验结果（true=测量可信） */
}ChiSquareTest_Typedef;

/* 卡尔曼滤波器句柄：一次 Init 创建内存，之后填好 A/B/H/Q/R 等矩阵，再周期性调用 Update。
 * Xhat 就是"我们想估计的状态向量"，例如姿态解算里就是 [四元数, 陀螺零偏] 那一串。 */
typedef struct KF_Info_TypeDef
{
  uint16_t sizeof_float, sizeof_double; /*!< size of float/double 记录 float/double 各占几个字节 */

  uint8_t Xhat_Size;   /*!< state vector dimension 状态向量维度（姿态解算是 6） */
  uint8_t U_Size;      /*!< control vector dimension 控制向量维度（无控制输入填 0） */
  uint8_t Z_Size;      /*!< measurement vector dimension 测量向量维度（如加速度计 3 轴=3） */

  float dt;   /*!< update cycle 滤波周期，单位秒（每控制周期调一次 Update） */
  float *MeasuredVector; /*!< external measure vector pointer 外部测量输入（调用前填好） */
  float *ControlVector;  /*!< external control vector pointer 外部控制输入（无则不用） */

  ChiSquareTest_Typedef ChiSquareTest;  /*!< Chi Square Test */

  /**
   * @brief Instance structure for the floating-point matrix structure.
   * 中文：这里放所有矩阵的"结构体实例"。Xhatminus=先验(预测)估计，Xhat=后验(修正)估计。
   * 下面 Data 结构体放的是这些矩阵对应的"内存指针"（真正存数据的数组）。
   */
  struct 
  {
    Matrix Xhat;              /*!< posteriori estimate Matrix 后验估计（最终输出的状态） */
    Matrix Xhatminus;         /*!< priori estimate Matrix 先验估计（模型预测出来的状态） */
    Matrix U;                 /*!< control vector */
    Matrix Z;                 /*!< measurement vector */
    Matrix B;                 /*!< control Matrix */ 
    Matrix A,AT;              /*!< state transition Matrix 状态转移矩阵（描述系统自身如何演化） */
    Matrix H,HT;              /*!< measurement Matrix 测量矩阵（把状态映射到测量量） */
    Matrix P;                 /*!< posteriori covariance Matrix 后验误差协方差 */
    Matrix Pminus;            /*!< priori covariance Matrix 先验误差协方差 */
    Matrix Q;                 /*!< process noise covariance Matrix 过程噪声协方差（模型可信度，越小越信模型） */ 
    Matrix R;                 /*!< measurement noise covariance Matrix 测量噪声协方差（传感器可信度，越小越信传感器） */ 
    Matrix K;                 /*!< kalman gain Matrix 卡尔曼增益（模型/测量之间的权衡权重） */
    Matrix K_denominator;     /*!< K_denominator Matrix (K_denominator = H Pminus HT + R) */
    Matrix Cache_Matrix[2];   /*!< calculate cache Matrix */
    Matrix Cache_Vector[2];   /*!< calculate cache vector */
  }Mat;

  arm_status MatStatus;   /*!< Error status. */

  /**
   * @brief Instance structure for the floating-point matrix data pointer.
   */
  struct 
  {
    float *Xhat,*Xhatminus;   /*!< posteriori/priori estimate matrix memory pointer */
    float *U;                 /*!< control vector memory pointer */
    float *Z;                 /*!< measurement vector memory pointer */
    float *B;                 /*!< control matrix memory pointer */ 
    float *A,*AT;             /*!< state transition matrix memory pointer */
    float *H,*HT;             /*!< measurement matrix memory pointer */
    float *P;                 /*!< posteriori covariance matrix memory pointer */
    float *Pminus;            /*!< priori covariance matrix memory pointer */
    float *Q;                 /*!< process noise covariance matrix memory pointer */ 
    float *R;                 /*!< measurement noise covariance matrix memory pointer */ 
    float *K;                 /*!< kalman gain matrix memory pointer */
    float *K_denominator;     /*!< K_denominator matrix memory pointer */
    float *Cache_Matrix[2];   /*!< calculate cache matrix memory pointer */
    float *Cache_Vector[2];   /*!< calculate cache vector memory pointer */
  }Data;

  uint8_t SkipStep1 : 1;  /*!< flag to skip the first step of kalman filter updating 置1则跳过第1步(预测状态) */
  uint8_t SkipStep2 : 1;  /*!< flag to skip the second step of kalman filter updating 置1则跳过第2步(预测协方差) */
  uint8_t SkipStep3 : 1;  /*!< flag to skip the third step of kalman filter updating 置1则跳过第3步(计算增益) */
  uint8_t SkipStep4 : 1;  /*!< flag to skip the fourth step of kalman filter updating 置1则跳过第4步(更新状态) */
  uint8_t SkipStep5 : 1;  /*!< flag to skip the fifth step of kalman filter updating 置1则跳过第5步(更新协方差) */
  uint8_t reserve   : 3;  /*!< 保留位。上面是 C 语言"位域"：一个 uint8_t 被拆成 5+3 个位来省内存 */

  /**
   * @brief user functions that can replace any step of kalman filter updating.
   * 中文：用户回调函数指针（钩子）。若不为 NULL，则在对应步骤前后被调用，
   * 允许用户替换/介入滤波的某一步（Quaternion.c 就用它自定义 A、H 矩阵和状态更新）。
   */
  void (*User_Function0)(struct KF_Info_TypeDef *KF);
  void (*User_Function1)(struct KF_Info_TypeDef *KF);
  void (*User_Function2)(struct KF_Info_TypeDef *KF);
  void (*User_Function3)(struct KF_Info_TypeDef *KF);
  void (*User_Function4)(struct KF_Info_TypeDef *KF);
  void (*User_Function5)(struct KF_Info_TypeDef *KF);
  void (*User_Function6)(struct KF_Info_TypeDef *KF);

  float *Output;  /*!< kalman filter output 滤波结果输出数组指针（即 Xhat 的拷贝） */

}KalmanFilter_Info_TypeDef;

/* Exported functions prototypes ---------------------------------------------*/
/**
  * @brief Initializes the kalman filter according to the specified parameters in the KalmanFilter_Info_TypeDef.
  * 中文：初始化卡尔曼滤波器。按 Xhat/U/Z 的维度为所有矩阵申请内存并清零。
  * Xhat_Size=状态维数，U_Size=控制维数(无控制填0)，Z_Size=测量维数。
  */
extern void Kalman_Filter_Init(KalmanFilter_Info_TypeDef *KF,uint8_t Xhat_Size,uint8_t U_Size,uint8_t Z_Size);
/**
  * @brief Update the Kalman Filter according to the specified parameters in the KalmanFilter_Info_TypeDef.
  * 中文：执行一次完整的卡尔曼滤波更新（5 步），返回指向输出数组 Output 的指针。
  */
extern float *Kalman_Filter_Update(KalmanFilter_Info_TypeDef *KF);

#endif //KALMAN_FILTER_H




