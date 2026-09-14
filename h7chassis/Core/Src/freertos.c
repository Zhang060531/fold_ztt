/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "SEGGER_SYSVIEW_FreeRTOS.h"

/* USER CODE END Includes */

/* ===== 文件说明：FreeRTOS 启动文件 (freertos.c) —— 创建所有任务 =====
 * 作用：MX_FREERTOS_Init() 里用 osThreadNew() 创建 7 个任务，每个任务都
 *       指定了「任务入口函数 + 优先级 + 栈大小」。这是整机系统的启动/调度骨架。
 *
 * 启动流程(结合 main.c)：
 *   main() -> 外设初始化(MX_xxx_Init) -> MCU_Init()(CAN/串口/IMU 等) ->
 *   osKernelInitialize() -> MX_FREERTOS_Init()(创建下面 7 个任务) -> osKernelStart()。
 *   之后 CPU 交给 FreeRTOS，按优先级轮流跑各任务。
 *
 * 任务清单(优先级数值越大越高，FreeRTOS 约定)：
 *   任务名                入口函数          优先级             周期    作用
 *   Start_INS_Task        INS_Task         High(高)           ~1ms     IMU 姿态解算(BMI088 + 四元数 EKF)
 *   Start_Control_Task    Control_Task     AboveNormal(次高)  ~1ms     底盘速度环 + 功率限制
 *   Start_CAN_Task        CAN_Task         Normal(普通)       ~1ms     发送电机电流指令
 *   Start_UI_Task         UI_Task          Normal(普通)       ~1ms     裁判系统 UI 界面
 *   Start_Test_Task       Test_Task        Normal(普通)       ~1ms     测试(当前为空)
 *   Start_LED_Task        LED_Task         Normal(普通)       ~1ms     WS2812 灯带灯效
 *   Start_Detect_Task     Detect_Task      BelowNormal(低)    ~1ms     遥控器掉线监测
 *
 * 重要说明：本文件里的 INS_Task/Control_Task/... 都是 __weak 弱定义，只有空循环。
 *   真正的任务实现在 Application/Task/Src/ 下的同名 .c 文件里(强定义)，链接时会
 *   覆盖这里的弱定义。所以实际跑的是那些 .c 里的实现。
 *   其中弱定义 INS_Task 里的 MX_USB_DEVICE_Init() 实际不生效(USB 已在 MCU_Init 初始化)。
 *
 * ===== 调车关注 =====
 *  - 每个任务栈大小都是 uint32_t Buffer[2048] = 8KB，若栈溢出会进 HardFault。
 *  - 所有任务主循环都是 osDelay(1)，即约 1ms 周期(前提 SysTick 为 1kHz)。
 *  - 想加新任务：在 MX_FREERTOS_Init 里加 osThreadNew + 在 Application/Task 下建 .c。
 * ===== ===================================================================== */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;   /* 静态任务控制块类型(任务用静态分配内存) */
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */

/* ---- 各任务的句柄 / 栈缓冲 / 控制块 / 属性定义 ---- */

/* Definitions for Start_INS_Task  惯性导航(IMU 姿态解算)任务 */
osThreadId_t Start_INS_TaskHandle;                 /* 任务句柄 */
uint32_t Start_INS_TaskBuffer[ 2048 ];             /* 任务栈：2048 字 = 8KB */
osStaticThreadDef_t Start_INS_TaskControlBlock;    /* 静态任务控制块 */
const osThreadAttr_t Start_INS_Task_attributes = { /* 任务属性(静态创建) */
  .name = "Start_INS_Task",
  .cb_mem = &Start_INS_TaskControlBlock,
  .cb_size = sizeof(Start_INS_TaskControlBlock),
  .stack_mem = &Start_INS_TaskBuffer[0],
  .stack_size = sizeof(Start_INS_TaskBuffer),
  .priority = (osPriority_t) osPriorityHigh,       /* 最高优先级：姿态解算优先 */
};
/* Definitions for Start_Control_Task  底盘控制任务 */
osThreadId_t Start_Control_TaskHandle;
uint32_t Start_Control_TaskBuffer[ 2048 ];
osStaticThreadDef_t Start_Control_TaskControlBlock;
const osThreadAttr_t Start_Control_Task_attributes = {
  .name = "Start_Control_Task",
  .cb_mem = &Start_Control_TaskControlBlock,
  .cb_size = sizeof(Start_Control_TaskControlBlock),
  .stack_mem = &Start_Control_TaskBuffer[0],
  .stack_size = sizeof(Start_Control_TaskBuffer),
  .priority = (osPriority_t) osPriorityAboveNormal,/* 次高优先级：控制闭环 */
};
/* Definitions for Start_CAN_Task  CAN 发送任务 */
osThreadId_t Start_CAN_TaskHandle;
uint32_t Start_CAN_TaskBuffer[ 2048 ];
osStaticThreadDef_t Start_CAN_TaskControlBlock;
const osThreadAttr_t Start_CAN_Task_attributes = {
  .name = "Start_CAN_Task",
  .cb_mem = &Start_CAN_TaskControlBlock,
  .cb_size = sizeof(Start_CAN_TaskControlBlock),
  .stack_mem = &Start_CAN_TaskBuffer[0],
  .stack_size = sizeof(Start_CAN_TaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,     /* 普通优先级 */
};
/* Definitions for Start_Detect_Task  掉线检测任务 */
osThreadId_t Start_Detect_TaskHandle;
uint32_t Start_Detect_TaskBuffer[ 2048 ];
osStaticThreadDef_t Start_Detect_TaskControlBlock;
const osThreadAttr_t Start_Detect_Task_attributes = {
  .name = "Start_Detect_Task",
  .cb_mem = &Start_Detect_TaskControlBlock,
  .cb_size = sizeof(Start_Detect_TaskControlBlock),
  .stack_mem = &Start_Detect_TaskBuffer[0],
  .stack_size = sizeof(Start_Detect_TaskBuffer),
  .priority = (osPriority_t) osPriorityBelowNormal,/* 低优先级：检测任务最后跑 */
};
/* Definitions for Start_UI_Task  裁判系统 UI 任务 */
osThreadId_t Start_UI_TaskHandle;
uint32_t Start_UI_TaskBuffer[ 2048 ];
osStaticThreadDef_t Start_UI_TaskControlBlock;
const osThreadAttr_t Start_UI_Task_attributes = {
  .name = "Start_UI_Task",
  .cb_mem = &Start_UI_TaskControlBlock,
  .cb_size = sizeof(Start_UI_TaskControlBlock),
  .stack_mem = &Start_UI_TaskBuffer[0],
  .stack_size = sizeof(Start_UI_TaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,     /* 普通优先级 */
};
/* Definitions for Start_Test_Task  测试任务(当前为空) */
osThreadId_t Start_Test_TaskHandle;
uint32_t Start_Test_TaskBuffer[ 2048 ];
osStaticThreadDef_t Start_Test_TaskControlBlock;
const osThreadAttr_t Start_Test_Task_attributes = {
  .name = "Start_Test_Task",
  .cb_mem = &Start_Test_TaskControlBlock,
  .cb_size = sizeof(Start_Test_TaskControlBlock),
  .stack_mem = &Start_Test_TaskBuffer[0],
  .stack_size = sizeof(Start_Test_TaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,     /* 普通优先级 */
};
/* Definitions for Start_LED_Task  LED 灯效任务 */
osThreadId_t Start_LED_TaskHandle;
uint32_t Start_Led_TaskBuffer[ 2048 ];
osStaticThreadDef_t Start_Led_TaskControlBlock;
const osThreadAttr_t Start_LED_Task_attributes = {
  .name = "Start_LED_Task",
  .cb_mem = &Start_Led_TaskControlBlock,
  .cb_size = sizeof(Start_Led_TaskControlBlock),
  .stack_mem = &Start_Led_TaskBuffer[0],
  .stack_size = sizeof(Start_Led_TaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,     /* 普通优先级 */
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

/* 各任务入口函数原型(实现在 Application/Task/Src 下) */
void INS_Task(void *argument);      /* 惯性导航任务(INS_Task.c) */
void Control_Task(void *argument);  /* 底盘控制任务(Control_Task.c) */
void CAN_Task(void *argument);      /* CAN 发送任务(CAN_Task.c) */
void Detect_Task(void *argument);   /* 掉线检测任务(Detect_Task.c) */
void UI_Task(void *argument);       /* UI 界面任务(UI_Task.c) */
void Test_Task(void *argument);     /* 测试任务(Test_Task.c) */
void LED_Task(void *argument);      /* LED 灯效任务(LED_Task.c) */

extern void MX_USB_DEVICE_Init(void);   /* USB 设备初始化(在 MCU_Init 里已调用) */
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS 初始化：创建 7 个任务(由 main() 在 osKernelInitialize 后调用)
  * @param  无
  * @retval 无
  * @note   本工程未使用队列/信号量/定时器(相关 USER CODE 段为空)，只创建任务。
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s)  依次创建 7 个任务 */
  /* creation of Start_INS_Task  惯性导航任务(最高优先级) */
  Start_INS_TaskHandle = osThreadNew(INS_Task, NULL, &Start_INS_Task_attributes);

  /* creation of Start_Control_Task  底盘控制任务(次高优先级) */
  Start_Control_TaskHandle = osThreadNew(Control_Task, NULL, &Start_Control_Task_attributes);

  /* creation of Start_CAN_Task  CAN 发送任务 */
  Start_CAN_TaskHandle = osThreadNew(CAN_Task, NULL, &Start_CAN_Task_attributes);

  /* creation of Start_Detect_Task  掉线检测任务 */
  Start_Detect_TaskHandle = osThreadNew(Detect_Task, NULL, &Start_Detect_Task_attributes);

  /* creation of Start_UI_Task  UI 界面任务 */
  Start_UI_TaskHandle = osThreadNew(UI_Task, NULL, &Start_UI_Task_attributes);

  /* creation of Start_Test_Task  测试任务 */
  Start_Test_TaskHandle = osThreadNew(Test_Task, NULL, &Start_Test_Task_attributes);

  /* creation of Start_LED_Task  LED 灯效任务 */
  Start_LED_TaskHandle = osThreadNew(LED_Task, NULL, &Start_LED_Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* ===== 以下是各任务的弱定义(__weak)空壳 =====
 * 实际任务实现在 Application/Task/Src/任务中.c(强定义)，链接时会覆盖这里的弱定义。 
 * 下面这些循环体只会在「没有找到强定义」时才执行，本工程中基本不会跑到。 */

/* USER CODE BEGIN Header_INS_Task */
/**
  * @brief  惯性导航任务弱定义(实际实现在 INS_Task.c)
  * @param  argument: 未使用
  * @retval 无
  */
/* USER CODE END Header_INS_Task */
__weak void INS_Task(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();   /* 注意: 仅弱定义里调用，实际 USB 已在 MCU_Init 初始化 */
  /* USER CODE BEGIN INS_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END INS_Task */
}

/* USER CODE BEGIN Header_Control_Task */
/**
* @brief 底盘控制任务弱定义(实际实现在 Control_Task.c)
* @param argument: 未使用
* @retval None
*/
/* USER CODE END Header_Control_Task */
__weak void Control_Task(void *argument)
{
  /* USER CODE BEGIN Control_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Control_Task */
}

/* USER CODE BEGIN Header_CAN_Task */
/**
* @brief CAN 发送任务弱定义(实际实现在 CAN_Task.c)
* @param argument: 未使用
* @retval None
*/
/* USER CODE END Header_CAN_Task */
__weak void CAN_Task(void *argument)
{
  /* USER CODE BEGIN CAN_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END CAN_Task */
}

/* USER CODE BEGIN Header_Detect_Task */
/**
* @brief 掉线检测任务弱定义(实际实现在 Detect_Task.c)
* @param argument: 未使用
* @retval None
*/
/* USER CODE END Header_Detect_Task */
__weak void Detect_Task(void *argument)
{
  /* USER CODE BEGIN Detect_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Detect_Task */
}

/* USER CODE BEGIN Header_UI_Task */
/**
* @brief UI 界面任务弱定义(实际实现在 UI_Task.c)
* @param argument: 未使用
* @retval None
*/
/* USER CODE END Header_UI_Task */
__weak void UI_Task(void *argument)
{
  /* USER CODE BEGIN UI_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END UI_Task */
}

/* USER CODE BEGIN Header_Test_Task */
/**
* @brief 测试任务弱定义(实际实现在 Test_Task.c)
* @param argument: 未使用
* @retval None
*/
/* USER CODE END Header_Test_Task */
__weak void Test_Task(void *argument)
{
  /* USER CODE BEGIN Test_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Test_Task */
}

/* USER CODE BEGIN Header_LED_Task */
/**
* @brief LED 灯效任务弱定义(实际实现在 LED_Task.c)
* @param argument: 未使用
* @retval None
*/
/* USER CODE END Header_LED_Task */
__weak void LED_Task(void *argument)
{
  /* USER CODE BEGIN LED_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END LED_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
