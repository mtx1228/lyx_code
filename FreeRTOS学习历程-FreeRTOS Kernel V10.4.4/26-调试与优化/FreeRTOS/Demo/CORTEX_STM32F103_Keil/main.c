/* Standard includes. */
#include <stdio.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "event_groups.h"
#include "timers.h"



/* Library includes. */
#include "stm32f10x_it.h"
#include "stm32f10x_tim.h"

/* Demo app includes. */
#include "lcd.h"
#include "LCD_Message.h"
#include "BlockQ.h"
#include "death.h"
#include "integer.h"
#include "blocktim.h"
#include "partest.h"
#include "semtest.h"
#include "PollQ.h"
#include "flash.h"
#include "comtest2.h"

#include "serial.h"


/*-----------------------------------------------------------*/

/*
 * Configure the clocks, GPIO and other peripherals as required by the demo.
 */
static void prvSetupHardware(void);

/*
 * Retargets the C library printf function to the USART.
 */
int fputc(int ch, FILE *f);

/*
 * Configures the timers and interrupts for the fast interrupt test as
 * described at the top of this file.
 */
extern void vSetupTimerTest(void);

/*-----------------------------------------------------------*/

static int timerFlag = 0;/*定时器中断变量*/
static int timerIntFlag = 0;

static uint32_t g_timer_cnt;/*获取时间*/

/***
 * 使用volatile时，修改变量i时，要读、修改、写内存
 * 不使用volatile的话，修改变量i时可能会优化，比如把它的值放在CPU寄存器中，累加时不涉及内存，这样速度会加快
 * 要访问硬件寄存器的话，要加上volatile避免被优化
 * volatile的含义是		易变的，这是提醒编译器，不要轻易优化我
 */
TimerHandle_t xTimerHandleTest;/*定时器句柄 */


void Timer3_Init(void)/*定时器2初始化*/
{	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;/*时基单元初始化结构体*/
	NVIC_InitTypeDef NVIC_InitStruct;/*NVIC结构体*/

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;/*不分频*/
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;/*向上计数*/
	TIM_TimeBaseInitStruct.TIM_Period = 100-1;/*自动重装值*/
	TIM_TimeBaseInitStruct.TIM_Prescaler = 72-1;/*预分频值*/
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStruct);/*时基单元初始化*/

	NVIC_InitStruct.NVIC_IRQChannel = TIM3_IRQChannel; /*TIM2的中断线*/
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;/*中断使能*/
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;/*抢占优先级最高*/
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;/*从优先级最高*/
	NVIC_Init(&NVIC_InitStruct);/*NVIC初始化*/

	//TIM_ClearFlag(TIM2, TIM_FLAG_Update);	/*防止一开时就记一次数*/
	//TIM_ClearITPendingBit(TIM2, TIM_IT_Update); // 清除更新中断请求位		  //这句话一定要放在使能更新中断前面，否则定时器初始化会直接进入一次更新中断
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);/*定时器中断映射*/

	TIM_Cmd(TIM3, ENABLE);/*开启定时器*/
}

/*中断的次数*/
uint32_t TimerGetCount(void)
{
	return g_timer_cnt;
}

void TIM3_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM3, TIM_IT_Update)==SET)
	{
		timerIntFlag=!timerIntFlag;
		g_timer_cnt++;	/*中断的次数*/
		//printf("TIM3_IRQHandler is Running!\r\n");
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
	}
}

/*实现获取时间的函数*/

char pcWriteBuffer[200];/*存放系统任务的统计信息*/

void Task1Function(void *param)
{
	volatile int i; /*让系统不再优化此变量*/

	while (1)
	{
		//printf("Task1Function .....\r\n");
		
		/*获得任务的统计信息*/
		//vTaskList(pcWriteBuffer);
		/*获得CPU资源占用率*/
		vTaskGetRunTimeStats(pcWriteBuffer);
		printf(pcWriteBuffer);/*打印统计信息*/
		vTaskDelay(1000);/*延时*/
	}
}

void Task2Function(void *param)/*累减*/
{
	volatile int i; /*让系统不再优化此变量*/

	while (1)
	{
		
	}
}


		/*定时器中断回调函数*/
void MyTimerCallbackFunction( TimerHandle_t xTimer )
{
	static int cnt;
	timerFlag = !timerFlag;
	printf("MyTimerCallbackFunction  cnt = %d\r\n",cnt++);
}


/*-----------------------------------------------------------*/

int main(void)
{
	TaskHandle_t xHandleTask1; // 任务句柄 以后使用它来操作这个任务

#ifdef DEBUG
	debug();
#endif

	prvSetupHardware(); // 硬件设置

	printf("Hello,World!\r\n");

	//Timer3_Init(72-1, 100-1);/*TIM2初始化 定时0.1ms*/
	
	/* 创建定时器 */
	/**
	* 第一个参数是定时器的名字，这个随意
	* 第二个参数是定时器的定时周期 这里定时100ms 100个tick
	* 第三个参数是定时器是否  ，  重复运行  ， 单次执行还是重复执行  ， 
		pdFALSE - 重复执行     pdTRUE	- 单次执行
	* 第四个参数是定时器的ID，回调函数可以使用这个参数，来分辨是哪个定时器
	* 第五个参数是定时器中断回调函数
	*/
	//xTimerHandleTest = xTimerCreate("Timer", 20000, pdFALSE, NULL, MyTimerCallbackFunction);

	/*动态分配task*/
	xTaskCreate(Task1Function, "task1", 100, NULL, 2, &xHandleTask1); // 任务1 打印1
	xTaskCreate(Task2Function, "task2", 100, NULL, 1, NULL);		  // 任务2 打印2

	/* Start the scheduler. */
	vTaskStartScheduler(); // 设置周期

	/* Will only get here if there was not enough heap space to create the
	idle task. */
	return 0;
}
/*-----------------------------------------------------------*/

/*-----------------------------------------------------------*/

/*-----------------------------------------------------------*/

static void prvSetupHardware(void)
{
	/* Start with the clocks in their expected state. */
	RCC_DeInit();

	/* Enable HSE (high speed external clock). */
	RCC_HSEConfig(RCC_HSE_ON);

	/* Wait till HSE is ready. */
	while (RCC_GetFlagStatus(RCC_FLAG_HSERDY) == RESET)
	{
	}

	/* 2 wait states required on the flash. */
	*((unsigned long *)0x40022000) = 0x02;

	/* HCLK = SYSCLK */
	RCC_HCLKConfig(RCC_SYSCLK_Div1);

	/* PCLK2 = HCLK */
	RCC_PCLK2Config(RCC_HCLK_Div1);

	/* PCLK1 = HCLK/2 */
	RCC_PCLK1Config(RCC_HCLK_Div2);

	/* PLLCLK = 8MHz * 9 = 72 MHz. */
	RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);

	/* Enable PLL. */
	RCC_PLLCmd(ENABLE);

	/* Wait till PLL is ready. */
	while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
	{
	}

	/* Select PLL as system clock source. */
	RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

	/* Wait till PLL is used as system clock source. */
	while (RCC_GetSYSCLKSource() != 0x08)
	{
	}

	/* Enable GPIOA, GPIOB, GPIOC, GPIOD, GPIOE and AFIO clocks */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE | RCC_APB2Periph_AFIO, ENABLE);

	/* SPI2 Periph clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

	/* Set the Vector Table base address at 0x08000000 */
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	/* Configure HCLK clock as SysTick clock source. */
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);

	SerialPortInit();
}
/*-----------------------------------------------------------*/

/*-----------------------------------------------------------*/

/*-----------------------------------------------------------*/

#ifdef DEBUG
/* Keep the linker happy. */
void assert_failed(unsigned char *pcFile, unsigned long ulLine)
{
	for (;;)
	{
	}
}
#endif
