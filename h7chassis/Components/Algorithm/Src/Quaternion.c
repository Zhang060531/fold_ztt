#include "Quaternion.h"
#include "Kalman_Filter.h"
#include "arm_math.h"

/* ==== 文件说明 ================================================================
 * Quaternion.c —— 四元数 + 扩展卡尔曼滤波（EKF）姿态解算实现
 * ---------------------------------------------------------------------------
 * 作用：这是云台姿态解算的"心脏"。它把两个传感器融合成稳定的姿态：
 *   - 陀螺仪：短时很准，但长期会漂移（积分误差累积）；
 *   - 加速度计：能测出重力方向（绝对参考），但震动大、高频噪声大；
 *   EKF 用陀螺仪"预测"姿态、用加速度计"修正"姿态，取长补短。
 *
 * 状态向量 Xhat = [q0,q1,q2,q3, 陀螺零偏x, 陀螺零偏y]（共 6 维）
 *   前 4 个是姿态四元数，后 2 个是陀螺零偏（估计出来后用来补偿陀螺漂移）。
 *
 * 一次 Update 的流程：
 *   1) 用零偏补偿陀螺 -> 计算 0.5·gyro·dt（半角增量）
 *   2) 组装状态转移矩阵 A（四元数运动学对状态求偏导的雅可比）
 *   3) 把加速度计归一化成"重力方向"作为测量 Z
 *   4) 卡方检验判断这次测量是否可信（加速度过大/过小时不可信，跳过修正）
 *   5) 填入 Q、R，调用 Kalman_Filter_Update 完成 EKF
 *   6) 把四元数换算成欧拉角输出
 *
 * 被谁调用：云台姿态解算任务，每控制周期调用一次 QuaternionEKF_Update。
 * 新手提示：看不懂 A、H 矩阵里的系数没关系，它们都是"四元数姿态微分方程"的导数，
 *   记住结论即可：陀螺积分出姿态，加速度计拉回真实重力方向。
 * ========================================================================== */

/* 本文件的内部函数声明（static = 只在本文件内使用，外部不可见） */
static float Fast_InverseSqrt(float x);

static void QuaternionEKF_A_Update(KalmanFilter_Info_TypeDef *KF);

static void QuaternionEKF_H_Update(KalmanFilter_Info_TypeDef *KF);

static bool QuaternionEKF_ChiSqrtTest(KalmanFilter_Info_TypeDef *KF);

static void QuaternionEKF_xhat_Update(KalmanFilter_Info_TypeDef *KF);

/* 全局唯一的姿态解算句柄，其他模块通过它读取当前姿态/欧拉角。 */
Quaternion_Info_Typedef Quaternion_Info;

/* QuaternionEKF_Init —— 初始化 EKF
 * 参数：Quat 句柄；Process_Noise1 四元数过程噪声 Q1；Process_Noise2 零偏过程噪声 Q2；
 *       Measure_Noise 测量噪声 R；QuaternionEKF_A_Data/P_Data 是外部提供的 A、P 矩阵初值内存。
 * 说明：状态维 6（四元数4 + 零偏2）、控制维 0、测量维 3（加速度计三轴）。
 *       把自定义的 A、H、状态更新函数挂到卡尔曼框架的 User_Function 钩子上，
 *       同时跳过框架默认的第3、4步（改由 QuaternionEKF_xhat_Update 实现）。
 * 初值：四元数设为 [1,0,0,0]（表示"无旋转"的初始姿态）。
 */
void QuaternionEKF_Init(Quaternion_Info_Typedef *Quat,float Process_Noise1,float Process_Noise2,float Measure_Noise,float *QuaternionEKF_A_Data,float *QuaternionEKF_P_Data)
{

  Quat->Q1 = Process_Noise1;
  Quat->Q2 = Process_Noise2;
  Quat->R  = Measure_Noise;
                     
  Quat->QuaternionEKF_A_Data = QuaternionEKF_A_Data;
  Quat->QuaternionEKF_P_Data = QuaternionEKF_P_Data;

  Kalman_Filter_Init(&Quat->QuaternionEKF,6,0,3);
	
	Quat->QuaternionEKF.ChiSquareTest.TestFlag = false;
  Quat->QuaternionEKF.ChiSquareTest.Result = false;
  Quat->QuaternionEKF.ChiSquareTest.ChiSquareTestThresholds = 1e-8f;
  Quat->QuaternionEKF.ChiSquareTest.ChiSquareCnt = 0;
	
	Quat->QuaternionEKF.Data.Xhat[0] = 1.f;
  Quat->QuaternionEKF.Data.Xhat[1] = 0.f;
  Quat->QuaternionEKF.Data.Xhat[2] = 0.f;
  Quat->QuaternionEKF.Data.Xhat[3] = 0.f;
  
  Quat->QuaternionEKF.User_Function1 = QuaternionEKF_A_Update;
  Quat->QuaternionEKF.User_Function2 = QuaternionEKF_H_Update;
  Quat->QuaternionEKF.User_Function3 = QuaternionEKF_xhat_Update;

  Quat->QuaternionEKF.SkipStep3 = true;
  Quat->QuaternionEKF.SkipStep4 = true;

  memcpy(Quat->QuaternionEKF.Data.A,Quat->QuaternionEKF_A_Data,Quat->QuaternionEKF.sizeof_float * Quat->QuaternionEKF.Xhat_Size * Quat->QuaternionEKF.Xhat_Size);
  memcpy(Quat->QuaternionEKF.Data.P,Quat->QuaternionEKF_P_Data,Quat->QuaternionEKF.sizeof_float * Quat->QuaternionEKF.Xhat_Size * Quat->QuaternionEKF.Xhat_Size);

}

/* QuaternionEKF_Update —— 执行一次姿态解算（核心入口，每控制周期调用一次）
 * 参数：Gyro[3] 陀螺角速度；Accel[3] 加速度计读数（单位 m/s²）；dt 周期（秒）。
 * 流程：
 *   1) 用估计的零偏补偿陀螺，算出 0.5·gyro·dt（四元数半角增量）；
 *   2) 组装 A 矩阵（四元数运动学雅可比）；
 *   3) 把加速度计归一化成单位向量，当作"重力方向"测量 Z；
 *   4) 卡方门槛判断：陀螺慢速 + 加速度模长≈重力 时才用测量修正，否则只靠陀螺积分；
 *   5) 更新 Q、R 噪声矩阵，调用 Kalman_Filter_Update 完成 EKF；
 *   6) 取出四元数，换算成欧拉角（yaw/pitch/roll）。
 */
void QuaternionEKF_Update(Quaternion_Info_Typedef *Quat,float Gyro[3],float Accel[3],float dt)
{
  /* update cycle */
  Quat->QuaternionEKF.dt = dt;
  
	/* Apply accelerometer feedback to gyroscope —— 用估计出的零偏补偿陀螺，得到更准的角速度 */
	Quat->Gyro[0] = Gyro[0] - Quat->Deviate[0];
  Quat->Gyro[1] = Gyro[1] - Quat->Deviate[1];
  Quat->Gyro[2] = Gyro[2] - Quat->Deviate[2];
	
	Quat->GyroInvNorm = Fast_InverseSqrt(Quat->Gyro[0]*Quat->Gyro[0] + Quat->Gyro[1]*Quat->Gyro[1] + Quat->Gyro[2]*Quat->Gyro[2]);
	
  /* Convert gyroscope to radians per second scaled by 0.5 —— 乘 0.5 是因为四元数旋转用"半角"表示 */	
	Quat->Half_Gyro_dt[0] = 0.5f * Quat->Gyro[0] * Quat->QuaternionEKF.dt;
	Quat->Half_Gyro_dt[1] = 0.5f * Quat->Gyro[1] * Quat->QuaternionEKF.dt;
  Quat->Half_Gyro_dt[2] = 0.5f * Quat->Gyro[2] * Quat->QuaternionEKF.dt;

 /**
   * @brief A = \frac{\partial f}{\partial x}
   * 中文：A 是"四元数状态转移矩阵"，即四元数运动学对状态求偏导的雅可比矩阵。
   * 左上 4x4 描述陀螺如何推动四元数旋转，右上 4x2 是零偏对四元数变化率的影响。
   *        (1,        -halfgxdt,  -halfgydt,  -halfgzdt,)   0.5f*q1*dt,  0.5f*q2*dt
   *        (halfgxdt,  1,          halfgzdt,  -halfgydt,)  -0.5f*q0*dt,  0.5f*q3*dt
   *        (halfgydt, -halfgzdt,   1,          halfgxdt,)  -0.5f*q3*dt, -0.5f*q0*dt
   *        (halfgzdt,  halfgydt,  -halfgxdt,   1,       )   0.5f*q2*dt, -0.5f*q1*dt
   *        (0,         0,          0,          0,         1,            0 )
   *        (0,         0,          0,          0,         0,            1 )
 */

  memcpy(Quat->QuaternionEKF.Data.A,Quat->QuaternionEKF_A_Data,Quat->QuaternionEKF.sizeof_float * Quat->QuaternionEKF.Xhat_Size * Quat->QuaternionEKF.Xhat_Size);

  Quat->QuaternionEKF.Data.A[1]  =  -Quat->Half_Gyro_dt[0];
	Quat->QuaternionEKF.Data.A[2]  =  -Quat->Half_Gyro_dt[1];
	Quat->QuaternionEKF.Data.A[3]  =  -Quat->Half_Gyro_dt[2];
	
	Quat->QuaternionEKF.Data.A[6]  =   Quat->Half_Gyro_dt[0];
	Quat->QuaternionEKF.Data.A[8]  =   Quat->Half_Gyro_dt[2];
	Quat->QuaternionEKF.Data.A[9]  =  -Quat->Half_Gyro_dt[1];

  Quat->QuaternionEKF.Data.A[12] =   Quat->Half_Gyro_dt[1];
	Quat->QuaternionEKF.Data.A[13] =  -Quat->Half_Gyro_dt[2];
	Quat->QuaternionEKF.Data.A[15] =   Quat->Half_Gyro_dt[0]; 

  Quat->QuaternionEKF.Data.A[18] =   Quat->Half_Gyro_dt[2];
	Quat->QuaternionEKF.Data.A[19] =   Quat->Half_Gyro_dt[1];
	Quat->QuaternionEKF.Data.A[20] =  -Quat->Half_Gyro_dt[0];

  memcpy(Quat->Accel,Accel,sizeof(Quat->Accel));

 /* Calculate direction of gravity indicated by measurement */
  Quat->AccelInvNorm = Fast_InverseSqrt(Quat->Accel[0] * Quat->Accel[0] + Quat->Accel[1] * Quat->Accel[1] + Quat->Accel[2] * Quat->Accel[2]);

  Quat->QuaternionEKF.MeasuredVector[0] =  Quat->Accel[0] * Quat->AccelInvNorm;
  Quat->QuaternionEKF.MeasuredVector[1] =  Quat->Accel[1] * Quat->AccelInvNorm;
  Quat->QuaternionEKF.MeasuredVector[2] =  Quat->Accel[2] * Quat->AccelInvNorm;
 
  /* chi square test 卡方门槛判断：只有陀螺慢速(<0.3rad/s)且加速度模长≈重力(GravityAccel±0.5)
   * 时才认为测量可信、可以做姿态修正；否则（剧烈加速/碰撞/快速转动）跳过修正，只靠陀螺积分。 */
  if(1.f/Quat->GyroInvNorm < 0.3f && 1.f/Quat->AccelInvNorm  > (GravityAccel-0.5f) && 1.f/Quat->AccelInvNorm < (GravityAccel+0.5f))
	{
		Quat->QuaternionEKF.ChiSquareTest.TestFlag = true;
  }
  else
  {
    Quat->QuaternionEKF.ChiSquareTest.TestFlag = false;
  }
  
  /* update the process/measurement noise covariance matrix
   * 更新 Q、R：Q 对角线 = 噪声系数 × dt（过程噪声随周期累积，dt 越大越不确定）；
   * 前 4 项对应四元数(Q1)，后 2 项对应陀螺零偏(Q2)；R 对角线直接填测量噪声 R。 */
  Quat->QuaternionEKF.Data.Q[0]  = Quat->Q1 * Quat->QuaternionEKF.dt;
  Quat->QuaternionEKF.Data.Q[7]  = Quat->Q1 * Quat->QuaternionEKF.dt;
  Quat->QuaternionEKF.Data.Q[14] = Quat->Q1 * Quat->QuaternionEKF.dt;
  Quat->QuaternionEKF.Data.Q[21] = Quat->Q1 * Quat->QuaternionEKF.dt;
  Quat->QuaternionEKF.Data.Q[28] = Quat->Q2 * Quat->QuaternionEKF.dt;
  Quat->QuaternionEKF.Data.Q[35] = Quat->Q2 * Quat->QuaternionEKF.dt;
  Quat->QuaternionEKF.Data.R[0]  = Quat->R;
  Quat->QuaternionEKF.Data.R[4]  = Quat->R;
  Quat->QuaternionEKF.Data.R[8]  = Quat->R;

  /* update the kalman filter */
  Kalman_Filter_Update(&Quat->QuaternionEKF);

  /* Update the Euler angle in radians */
  Quat->quat[0]    = Quat->QuaternionEKF.Output[0];
  Quat->quat[1]    = Quat->QuaternionEKF.Output[1];
  Quat->quat[2]    = Quat->QuaternionEKF.Output[2];
  Quat->quat[3]    = Quat->QuaternionEKF.Output[3];
  Quat->Deviate[0] = Quat->QuaternionEKF.Output[4];
  Quat->Deviate[1] = Quat->QuaternionEKF.Output[5];
  Quat->Deviate[2] = 0.f;  /* 零偏只估计 x/y 两轴，z 轴(偏航)固定为 0 */
	
	
	/* Update the Euler angle in radians */
	Quat->EulerAngle[0] = atan2f(2.f*(Quat->quat[0]*Quat->quat[3] + Quat->quat[1]*Quat->quat[2]), 1.f - 2.f*(Quat->quat[2]*Quat->quat[2] + Quat->quat[3]*Quat->quat[3]));  /* yaw 偏航 */
  Quat->EulerAngle[1] = asinf( 2.f*(Quat->quat[0]*Quat->quat[2]) - 2.f*(Quat->quat[1]*Quat->quat[3] ));  /* pitch 俯仰 */
	Quat->EulerAngle[2] = atan2f(2.f*(Quat->quat[0]*Quat->quat[1] + Quat->quat[2]*Quat->quat[3]), 1.f - 2.f*(Quat->quat[1]*Quat->quat[1] + Quat->quat[2]*Quat->quat[2]));  /* roll 横滚 */
	
}

/* QuaternionEKF_A_Update —— 自定义"状态转移矩阵 A"（挂在 User_Function1）
 * 作用：1) 把预测出的四元数归一化（保证它仍代表纯旋转，模长=1）；
 *       2) 补上 A 矩阵里"零偏列"的系数（四元数变化率对零偏的偏导）；
 *       3) 限制 P 矩阵的零偏项别发散（防止积分爆掉）。
 */
static void QuaternionEKF_A_Update(KalmanFilter_Info_TypeDef *KF){
   /* Normalise quaternion */
	memset(KF->Data.Cache_Vector[0],0,KF->sizeof_float * KF->Xhat_Size);
	KF->Data.Cache_Vector[0][0] = Fast_InverseSqrt(KF->Data.Xhatminus[0]*KF->Data.Xhatminus[0] \
                                                +KF->Data.Xhatminus[1]*KF->Data.Xhatminus[1] \
                                                +KF->Data.Xhatminus[2]*KF->Data.Xhatminus[2] \
                                                +KF->Data.Xhatminus[3]*KF->Data.Xhatminus[3]);
	
	KF->Data.Xhatminus[0] *= KF->Data.Cache_Vector[0][0];
  KF->Data.Xhatminus[1] *= KF->Data.Cache_Vector[0][0];
  KF->Data.Xhatminus[2] *= KF->Data.Cache_Vector[0][0];
  KF->Data.Xhatminus[3] *= KF->Data.Cache_Vector[0][0];
	
	KF->Data.A[4]  =  0.5f*KF->Data.Xhatminus[1]*KF->dt;
  KF->Data.A[5]  =  0.5f*KF->Data.Xhatminus[2]*KF->dt;

  KF->Data.A[10] = -0.5f*KF->Data.Xhatminus[0]*KF->dt;
  KF->Data.A[11] =  0.5f*KF->Data.Xhatminus[3]*KF->dt;

  KF->Data.A[16] = -0.5f*KF->Data.Xhatminus[3]*KF->dt;
  KF->Data.A[17] = -0.5f*KF->Data.Xhatminus[0]*KF->dt;

  KF->Data.A[22] =  0.5f*KF->Data.Xhatminus[2]*KF->dt;
  KF->Data.A[23] = -0.5f*KF->Data.Xhatminus[1]*KF->dt;

  /* Limit the P data */
  VAL_LIMIT(KF->Data.P[28],-10000,10000);
  VAL_LIMIT(KF->Data.P[35],-10000,10000);
	
}

/* QuaternionEKF_H_Update —— 自定义"测量矩阵 H"（挂在 User_Function2）
 * 作用：H 是"重力方向测量对状态(四元数)的偏导"，由旋转矩阵对四元数求导得到。
 *       这些 2.0/-2.0 系数都是四元数微分的结果，直接把当前四元数代入即可。 */
static void QuaternionEKF_H_Update(KalmanFilter_Info_TypeDef *KF){
  
	memset(KF->Data.H,0,KF->sizeof_float * KF->Z_Size * KF->Xhat_Size);

  KF->Data.H[0]   = -2.f*KF->Data.Xhatminus[2];
  KF->Data.H[1]   =  2.f*KF->Data.Xhatminus[3];
  KF->Data.H[2]   = -2.f*KF->Data.Xhatminus[0];
  KF->Data.H[3]   =  2.f*KF->Data.Xhatminus[1];

  
	KF->Data.H[6]   =  2.f*KF->Data.Xhatminus[1];
  KF->Data.H[7]   =  2.f*KF->Data.Xhatminus[0];
  KF->Data.H[8]   =  2.f*KF->Data.Xhatminus[3];
  KF->Data.H[9]   =  2.f*KF->Data.Xhatminus[2];

	
	KF->Data.H[12]  =  2.f*KF->Data.Xhatminus[0];
  KF->Data.H[13]  = -2.f*KF->Data.Xhatminus[1];
  KF->Data.H[14]  = -2.f*KF->Data.Xhatminus[2];
  KF->Data.H[15]  =  2.f*KF->Data.Xhatminus[3];

	
	
}

/* QuaternionEKF_ChiSqrtTest —— 卡方检验（异常测量检测）
 * 作用：计算"新息平方/协方差"这个量（近似卡方分布），若它超出阈值说明测量可疑。
 *   连续可疑超过 50 次则判定测量长期失效(Result=0，恢复 P 更新)；
 *   偶尔可疑则跳过本次 P 更新(SkipStep5=true)并把状态退回先验值，防止被野值带偏。
 * 返回 true 表示本次测量被判异常、已做特殊处理（调用方据此提前 return）。 */
static bool QuaternionEKF_ChiSqrtTest(KalmanFilter_Info_TypeDef *KF){
 
  // Cache_Matrix[0] = inverse(H·Pminus·HT + R)·(z(k) - h(Xhatminus))
  KF->Mat.Cache_Matrix[0].numRows = KF->Mat.Cache_Matrix[1].numRows;
  KF->Mat.Cache_Matrix[0].numCols = 1;
  KF->MatStatus = Matrix_Multiply(&KF->Mat.Cache_Matrix[1], &KF->Mat.Cache_Vector[1], &KF->Mat.Cache_Matrix[0]);

 // Cache_Vector[0] = (z(k) - h(Xhatminus)'
	KF->Mat.Cache_Vector[0].numRows = 1;
  KF->Mat.Cache_Vector[0].numCols = KF->Mat.Cache_Matrix[1].numRows;
  KF->MatStatus = Matrix_Transpose(&KF->Mat.Cache_Matrix[1], &KF->Mat.Cache_Vector[0]);
	
// ChiSquare_Matrix = (z(k) - h(Xhatminus)'·inverse(H·Pminus·HT + R)·(z(k) - h(Xhatminus)) */
  KF->MatStatus = Matrix_Multiply(&KF->Mat.Cache_Vector[0], &KF->Mat.Cache_Matrix[0], &KF->ChiSquareTest.ChiSquare_Matrix);
	
	if (KF->ChiSquareTest.ChiSquare_Data[0] < 0.5f * KF->ChiSquareTest.ChiSquareTestThresholds)
  {
      KF->ChiSquareTest.Result = true;
  }
	
	  if (KF->ChiSquareTest.ChiSquare_Data[0] > KF->ChiSquareTest.ChiSquareTestThresholds && KF->ChiSquareTest.Result)
  {
      if (KF->ChiSquareTest.TestFlag)
      {
          KF->ChiSquareTest.ChiSquareCnt++;
      }
      else
      {
          KF->ChiSquareTest.ChiSquareCnt = 0;
      }

      if (KF->ChiSquareTest.ChiSquareCnt > 50)  /* 连续 50 次可疑 => 判定测量长期失效 */
      {
          KF->ChiSquareTest.Result = 0;
          KF->SkipStep5 = false; // step-5 is cov mat P updating
      }
      else
      {
          /* xhat(k) = xhat'(k) */
          /* P(k) = P'(k) */
          memcpy(KF->Data.Xhat, KF->Data.Xhatminus, KF->sizeof_float * KF->Xhat_Size);
          memcpy(KF->Data.P, KF->Data.Pminus, KF->sizeof_float * KF->Xhat_Size * KF->Xhat_Size);

          /* skip the P update */
          KF->SkipStep5 = true;
          return true;
      }
  }else{
			
		 /* The smaller the rk , the greater the gain */
			if(KF->ChiSquareTest.ChiSquare_Data[0] > 0.1f * KF->ChiSquareTest.ChiSquareTestThresholds && KF->ChiSquareTest.Result)
			{
				KF->Data.Cache_Vector[0][0] = (KF->ChiSquareTest.ChiSquareTestThresholds - KF->ChiSquareTest.ChiSquare_Data[0]) / (0.9f * KF->ChiSquareTest.ChiSquareTestThresholds);
			}
			else
			{
				KF->Data.Cache_Vector[0][0] = 1.f;
			}
			
      KF->ChiSquareTest.ChiSquareCnt = 0;
      KF->SkipStep5 = false;
  }
  return false;
}


/* QuaternionEKF_xhat_Update —— 自定义"状态更新"（替代框架默认的第3、4步）
 * 作用：1) 算重力方向的预测值 h(xhat')；2) 卡方检验；
 *       3) 算增益 K 并按"姿态与重力方向的夹角"缩放零偏修正项；
 *       4) 限制零偏修正量，最后 Xhat = Xhatminus + K·(Z - h)。
 * 直白理解：姿态用加速度计强力修正，零偏只用一小部分修正（避免剧烈抖动）。 */
static void QuaternionEKF_xhat_Update(KalmanFilter_Info_TypeDef *KF){

	  /* HT */
  KF->MatStatus = Matrix_Transpose(&KF->Mat.H,&KF->Mat.HT);

  /* Cache_Matrix[0] = H·Pminus */
  KF->Mat.Cache_Matrix[0].numRows = KF->Mat.H.numRows;
  KF->Mat.Cache_Matrix[0].numCols = KF->Mat.Pminus.numCols;
  KF->MatStatus = Matrix_Multiply(&KF->Mat.H, &KF->Mat.Pminus, &KF->Mat.Cache_Matrix[0]);

  /* Cache_Matrix[1] = H·Pminus·HT */
  KF->Mat.Cache_Matrix[1].numRows = KF->Mat.Cache_Matrix[0].numRows;
  KF->Mat.Cache_Matrix[1].numCols = KF->Mat.HT.numCols;
  KF->MatStatus = Matrix_Multiply(&KF->Mat.Cache_Matrix[0], &KF->Mat.HT, &KF->Mat.Cache_Matrix[1]); 
  
  /* K_denominator = H Pminus HT + R */
  KF->Mat.K_denominator.numRows = KF->Mat.R.numRows;
  KF->Mat.K_denominator.numCols = KF->Mat.R.numCols;
  KF->MatStatus = Matrix_Add(&KF->Mat.Cache_Matrix[1], &KF->Mat.R, &KF->Mat.K_denominator);

  /* Cache_Matrix[1] = inverse(H·Pminus·HT + R) */
  KF->MatStatus = Matrix_Inverse(&KF->Mat.K_denominator, &KF->Mat.Cache_Matrix[1]);
	
  KF->Mat.Cache_Vector[0].numRows = KF->Mat.H.numRows;
  KF->Mat.Cache_Vector[0].numCols = 1;
  KF->Data.Cache_Vector[0][0] = 2.f * (KF->Data.Xhatminus[1] * KF->Data.Xhatminus[3] - KF->Data.Xhatminus[0] * KF->Data.Xhatminus[2]);
  KF->Data.Cache_Vector[0][1] = 2.f * (KF->Data.Xhatminus[0] * KF->Data.Xhatminus[1] + KF->Data.Xhatminus[2] * KF->Data.Xhatminus[3]);
  KF->Data.Cache_Vector[0][2] =   KF->Data.Xhatminus[0] * KF->Data.Xhatminus[0] \
                                - KF->Data.Xhatminus[1] * KF->Data.Xhatminus[1] \
                                - KF->Data.Xhatminus[2] * KF->Data.Xhatminus[2] \
                                + KF->Data.Xhatminus[3] * KF->Data.Xhatminus[3];
																
	/* the cosine of three axis orientation */
	float OrientationCosine[3];
	
  /* calculate the cosine of three axis orientation */
	for (uint8_t i = 0; i < 3; i++)
	{
		OrientationCosine[i] = acosf(fabsf(KF->Data.Cache_Vector[0][i]));
	}
																
	 /* Cache_Vector[1] = z(k) - h(xhat'(k)) */
  KF->Mat.Cache_Vector[1].numRows = KF->Mat.Z.numRows;
  KF->Mat.Cache_Vector[1].numCols = 1;
  KF->MatStatus = Matrix_Subtract(&KF->Mat.Z, &KF->Mat.Cache_Vector[0], &KF->Mat.Cache_Vector[1]);

  /* Chi Square root Test */
  if(QuaternionEKF_ChiSqrtTest(KF)==true)
  {
    return;
  }

  /* Cache_Matrix[0] = Pminus·HT */
  KF->Mat.Cache_Matrix[0].numRows = KF->Mat.Pminus.numRows;
  KF->Mat.Cache_Matrix[0].numCols = KF->Mat.HT.numCols;
  KF->MatStatus = Matrix_Multiply(&KF->Mat.Pminus, &KF->Mat.HT, &KF->Mat.Cache_Matrix[0]);

  /* k = Pminus·HT·inverse(H·Pminus·HT + R) */
  KF->MatStatus = Matrix_Multiply(&KF->Mat.Cache_Matrix[0], &KF->Mat.Cache_Matrix[1], &KF->Mat.K);
	
	/* The smaller the rk , the greater the gain */
	for(uint8_t i = 0; i < KF->Mat.K.numCols*KF->Mat.K.numRows; i++)
	{
		KF->Data.K[i] *= KF->Data.Cache_Vector[0][0];
	}

  /**
   * @brief K = \frac {P·minus·HT}{H·Pminus·HT + V·R·VT}
   *          = [  0,  1,  2,
   *               3,  4,  5,
   *               6,  7,  8,
   *               9, 10, 11, 
   *             (12, 13, 14,)
   *             (15, 16, 17,)]
   * @note  K[12..17] *=  cos(axis)/(PI/2.f)
   */
  for (uint8_t i = 4; i < 6; i++)
  {
    for (uint8_t j = 0; j < 3; j++)
    {
        KF->Data.K[i * 3 + j] *= OrientationCosine[i - 4] / 1.5707963f; // 1 rad  1.5707963≈PI/2，按"该轴与重力夹角"缩放零偏增益
    }
  }

  /* Cache_Vector[0] = K(k)·(z(k) - H·xhat'(k)) */
  KF->Mat.Cache_Vector[0].numRows = KF->Mat.K.numRows;
  KF->Mat.Cache_Vector[0].numCols = 1;
  KF->MatStatus = Matrix_Multiply(&KF->Mat.K, &KF->Mat.Cache_Vector[1], &KF->Mat.Cache_Vector[0]);

  if(KF->ChiSquareTest.Result)
  {
    VAL_LIMIT(KF->Data.Cache_Vector[0][4],-1e-2f*KF->dt,1e-2f*KF->dt);
    VAL_LIMIT(KF->Data.Cache_Vector[0][5],-1e-2f*KF->dt,1e-2f*KF->dt);
  }
  KF->Data.Cache_Vector[0][3] = 0;

  KF->MatStatus = Matrix_Add(&KF->Mat.Xhatminus, &KF->Mat.Cache_Vector[0], &KF->Mat.Xhat);															
																											
}

/* Fast_InverseSqrt —— 快速求 1/sqrt(x)（著名的 Quake III 平方根倒数速算）
 * 原理：用位运算 0x5f375a86 - (i>>1) 先猜一个近似初值，再用一次牛顿迭代精化。
 * 好处：比标准 sqrtf+除法快很多，姿态解算每周期要算好多次平方根倒数，收益明显。
 * 注意：这是近似算法，精度够用即可；0x5f375a86 是经验魔数（经典值是 0x5f3759df）。 */
static float Fast_InverseSqrt(float x)
{

    float halfx = 0.5f * x;
    float y = x;
    long i = *(long *)&y;

    i = 0x5f375a86 - (i >> 1);  /* 魔数：对浮点数的位模式做"魔法减法"，得到接近 1/sqrt(x) 的初值 */
    y = *(float *)&i;
    y = y * (1.5f - (halfx * y * y));  /* 一次牛顿迭代：让初值更接近真实 1/sqrt(x) */
    return y;
}