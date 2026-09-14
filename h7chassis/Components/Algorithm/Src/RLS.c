#include "RLS.h"

/* ==== 文件说明 ================================================================
 * RLS.c —— 递推最小二乘（RLS）实现
 * ---------------------------------------------------------------------------
 * 作用：在线估计线性模型 Y ≈ W·X 中的权重 W，随新数据不断修正，无需存历史。
 * 一次 RLS_Update 做的事（对应代码里的英文注释）：
 *   1) E = Y - U                       计算预测误差
 *   2) K = P·X / (λ + XT·P·X)          计算增益
 *   3) P = (P - K·XT·P) / λ            更新协方差（除以 λ 实现"遗忘"）
 *   4) W = W + K·E                     修正权重
 * 被谁调用：功率/电流等参数在线辨识（Power_Control_RLS_Info）。
 * 注意：本文件用裸 malloc 分配内存（未用 FreeRTOS 的 pvPortMalloc）。
 * ========================================================================== */

/* RLS_Init —— 初始化
 * 参数：X_Size 输入向量维度，P_Size 协方差矩阵维度，Y_Size 输出向量维度。
 * 作用：为所有矩阵申请内存并清零；调用后还需手动给 P 设初值（通常设成大对角阵，
 *       表示一开始"完全不确定"，让算法能快速收敛）。 */
void RLS_Init(Recursive_Least_Squares_Info_TypeDef *RLS,uint8_t X_Size,uint8_t P_Size,uint8_t Y_Size){


	 RLS->sizeof_float = sizeof(float);
	
   RLS->X_Size = X_Size;
	 RLS->P_Size = P_Size;
   RLS->Y_Size = Y_Size;
   
	 RLS->Data.W = (float *)malloc(RLS->sizeof_float * X_Size);
   memset(RLS->Data.W, 0, RLS->sizeof_float * X_Size);
   Matrix_Init(&RLS->Mat.W, X_Size, 1, (float *)RLS->Data.W);
	
   RLS->Data.X = (float *)malloc(RLS->sizeof_float * X_Size);
   memset(RLS->Data.X, 0, RLS->sizeof_float * X_Size);
   Matrix_Init(&RLS->Mat.X, X_Size, 1, (float *)RLS->Data.X);
	
	 RLS->Data.XT = (float *)malloc(RLS->sizeof_float * X_Size);
   memset(RLS->Data.XT, 0, RLS->sizeof_float * X_Size);
   Matrix_Init(&RLS->Mat.XT,1, X_Size, (float *)RLS->Data.XT);
	
	 RLS->Data.Lamda = (float *)malloc(RLS->sizeof_float);
   memset(RLS->Data.Lamda, 0, RLS->sizeof_float);
   Matrix_Init(&RLS->Mat.Lamda,1, 1, (float *)RLS->Data.Lamda);
	
	 RLS->Data.P = (float *)malloc(RLS->sizeof_float * P_Size * P_Size);
   memset(RLS->Data.P, 0, RLS->sizeof_float * P_Size * P_Size);
   Matrix_Init(&RLS->Mat.P, P_Size, P_Size, (float *)RLS->Data.P);
    	
   RLS->Data.Y = (float *)malloc(RLS->sizeof_float * Y_Size);
   memset(RLS->Data.Y, 0, RLS->sizeof_float * Y_Size);
   Matrix_Init(&RLS->Mat.Y, Y_Size, 1, (float *)RLS->Data.Y);      

   RLS->Data.U = (float *)malloc(RLS->sizeof_float * Y_Size);
   memset(RLS->Data.U, 0, RLS->sizeof_float * Y_Size);
   Matrix_Init(&RLS->Mat.U, Y_Size, 1, (float *)RLS->Data.U);   

   RLS->Data.E = (float *)malloc(RLS->sizeof_float * Y_Size);
   memset(RLS->Data.E, 0, RLS->sizeof_float * Y_Size);
   Matrix_Init(&RLS->Mat.E, Y_Size, 1, (float *)RLS->Data.E);     
	 
	 RLS->Data.K_Numerator = (float *)malloc(RLS->sizeof_float * P_Size * P_Size);
   memset(RLS->Data.K_Numerator,0,RLS->sizeof_float * P_Size * P_Size);
   Matrix_Init(&RLS->Mat.K_Numerator, P_Size, P_Size, (float *)RLS->Data.K_Numerator);
	 
	 RLS->Data.K_Denominator = (float *)malloc(RLS->sizeof_float );
   memset(RLS->Data.K_Denominator,0,RLS->sizeof_float);
   Matrix_Init(&RLS->Mat.K_Denominator, 1, 1, (float *)RLS->Data.K_Denominator);
	 
	 RLS->Data.K = (float *)malloc(RLS->sizeof_float );
   memset(RLS->Data.K,0,RLS->sizeof_float);
   Matrix_Init(&RLS->Mat.K, X_Size, 1, (float *)RLS->Data.K);
	 
	 RLS->Data.Cache_Matrix[0] = (float *)malloc(RLS->sizeof_float * X_Size * X_Size);
   memset(RLS->Data.Cache_Matrix[0],0,RLS->sizeof_float * X_Size * X_Size);
   Matrix_Init(&RLS->Mat.Cache_Matrix[0], X_Size, X_Size, (float *)RLS->Data.Cache_Matrix[0]);

	 RLS->Data.Cache_Matrix[1] = (float *)malloc(RLS->sizeof_float * X_Size * X_Size);
   memset(RLS->Data.Cache_Matrix[1],0,RLS->sizeof_float * X_Size * X_Size);
   Matrix_Init(&RLS->Mat.Cache_Matrix[1], X_Size, X_Size, (float *)RLS->Data.Cache_Matrix[1]);
	 
	 RLS->Data.Output = (float *)malloc(RLS->sizeof_float * X_Size);
   memset(RLS->Data.Output, 0, RLS->sizeof_float * X_Size);
   Matrix_Init(&RLS->Mat.Output, RLS->X_Size, 1, (float *)RLS->Data.Output);
}

/* RLS_Update —— 执行一次递推最小二乘更新
 * 输入：RLS->Y（测量输出）、RLS->X（回归向量）、RLS->Lamda（遗忘因子）需在调用前填好。
 * 输出：更新后的权重存在 RLS->Data.W（这里只回写了 W[0]、W[1] 两个元素）。
 * 步骤见文件头说明。 */
void RLS_Update(Recursive_Least_Squares_Info_TypeDef *RLS){
 
    
  //E(n) = Y(n) - U(n)  误差 = 测量输出 - 当前权重预测的输出
	RLS->MatStatus = Matrix_Subtract(&RLS->Mat.Y,&RLS->Mat.U,&RLS->Mat.E);

  //P(n-1)*X(n) 
	RLS->Mat.K_Numerator.numRows = RLS->Mat.P.numRows;
  RLS->Mat.K_Numerator.numCols = RLS->Mat.X.numCols;
	RLS->MatStatus = Matrix_Multiply(&RLS->Mat.P,& RLS->Mat.X,&RLS->Mat.K_Numerator);
	
	//XT(n) 
  RLS->MatStatus = Matrix_Transpose(&RLS->Mat.X,&RLS->Mat.XT);	
	
	//XT(n)*P(n-1)
	RLS->Mat.Cache_Matrix[0].numRows = RLS->Mat.XT.numRows;
  RLS->Mat.Cache_Matrix[0].numCols = RLS->Mat.P.numCols;
	RLS->MatStatus = Matrix_Multiply(&RLS->Mat.XT,& RLS->Mat.P,&RLS->Mat.Cache_Matrix[0]);
	
	
	//XT(n)*P(n-1)*X(n)
  RLS->Mat.Cache_Matrix[1].numRows = 1;
  RLS->Mat.Cache_Matrix[1].numCols = 1;
	RLS->MatStatus = Matrix_Multiply(&RLS->Mat.Cache_Matrix[0],& RLS->Mat.X,&RLS->Mat.Cache_Matrix[1]);
	
	//lamda + XT(n)*P(n-1)*X(n)
	RLS->Mat.Cache_Matrix[0].numRows = 1;
  RLS->Mat.Cache_Matrix[0].numCols = 1;
	RLS->MatStatus = Matrix_Add(&RLS->Mat.Lamda,&RLS->Mat.Cache_Matrix[1],&RLS->Mat.Cache_Matrix[0]);

  //1/(lamda + XT(n)*P(n-1)*X(n))
  RLS->MatStatus = Matrix_Inverse(&RLS->Mat.Cache_Matrix[0],&RLS->Mat.K_Denominator);


 //K = P(n-1)*X(n) / lamda + XT(n)*P(n-1)*X(n)  增益 = 协方差·输入 / (遗忘因子 + 输入二次型)
	RLS->Mat.K.numRows = RLS->Mat.K_Numerator.numRows;
  RLS->Mat.K.numCols = RLS->Mat.K_Denominator.numCols;;
	RLS->MatStatus = Matrix_Multiply(&RLS->Mat.K_Numerator,& RLS->Mat.K_Denominator,&RLS->Mat.K);
 

// K * XT(n) 	
  RLS->Mat.Cache_Matrix[0].numRows = RLS->Mat.K.numRows;
  RLS->Mat.Cache_Matrix[0].numCols = RLS->Mat.XT.numCols;
	RLS->MatStatus = Matrix_Multiply(&RLS->Mat.K,& RLS->Mat.XT,&RLS->Mat.Cache_Matrix[0]);

// K * XT(n) * P(n-1)
  RLS->Mat.Cache_Matrix[1].numRows = RLS->Mat.Cache_Matrix[0].numRows;
  RLS->Mat.Cache_Matrix[1].numCols = RLS->Mat.P.numCols;
	RLS->MatStatus = Matrix_Multiply(&RLS->Mat.Cache_Matrix[0],&RLS->Mat.P,&RLS->Mat.Cache_Matrix[1]);

// P(n-1) -  K * XT(n) * P(n-1)

  RLS->Mat.Cache_Matrix[0].numRows = RLS->Mat.P.numRows;
  RLS->Mat.Cache_Matrix[0].numCols = RLS->Mat.P.numCols;
	RLS->MatStatus = Matrix_Subtract(&RLS->Mat.P,&RLS->Mat.Cache_Matrix[1],&RLS->Mat.Cache_Matrix[0]);

// 1/lamda
  RLS->Mat.Cache_Matrix[1].numRows = 1;
  RLS->Mat.Cache_Matrix[1].numCols = 1;
	RLS->Data.Cache_Matrix[1][0] = 1/RLS->Data.Lamda[0];  /* 除以 λ：λ 越小，协方差 P 越大，越"健忘" */

//P(n) = 1/lamda *  (P(n-1) -  K * XT(n) * P(n-1))  更新协方差（除以 λ 实现遗忘）
	RLS->MatStatus = Matrix_Multiply(&RLS->Mat.Cache_Matrix[1],&RLS->Mat.Cache_Matrix[0],&RLS->Mat.P);

//W(n) = W(n-1) + K*E(n)  用增益 K 把误差 E 反馈回权重 W

  RLS->Mat.Cache_Matrix[0].numRows = RLS->Mat.K.numRows;
  RLS->Mat.Cache_Matrix[0].numCols = RLS->Mat.E.numCols;
	RLS->MatStatus = Matrix_Multiply(&RLS->Mat.K,&RLS->Mat.E,&RLS->Mat.Cache_Matrix[0]);

  RLS->MatStatus = Matrix_Add(&RLS->Mat.W,&RLS->Mat.Cache_Matrix[0],&RLS->Mat.Output);

  RLS->Data.W[0] = RLS->Data.Output[0];
  RLS->Data.W[1] = RLS->Data.Output[1];


}
	






