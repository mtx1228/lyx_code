/*
 * FreeRTOS V202107.00
 * Copyright (C) 2020 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */

/*
 * Creates all the demo application tasks, then starts the scheduler.  The WEB
 * documentation provides more details of the standard demo application tasks.
 * In addition to the standard demo tasks, the following tasks and tests are
 * defined and/or created within this file:
 *
 * "Fast Interrupt Test" - A high frequency periodic interrupt is generated
 * using a free running timer to demonstrate the use of the
 * configKERNEL_INTERRUPT_PRIORITY configuration constant.  The interrupt
 * service routine measures the number of processor clocks that occur between
 * each interrupt - and in so doing measures the jitter in the interrupt timing.
 * The maximum measured jitter time is latched in the ulMaxJitter variable, and
 * displayed on the LCD by the 'Check' task as described below.  The
 * fast interrupt is configured and handled in the timertest.c source file.
 *
 * "LCD" task - the LCD task is a 'gatekeeper' task.  It is the only task that
 * is permitted to access the display directly.  Other tasks wishing to write a
 * message to the LCD send the message on a queue to the LCD task instead of
 * accessing the LCD themselves.  The LCD task just blocks on the queue waiting
 * for messages - waking and displaying the messages as they arrive.
 *
 * "Check" task -  This only executes every five seconds but has the highest
 * priority so is guaranteed to get processor time.  Its main function is to
 * check that all the standard demo tasks are still operational.  Should any
 * unexpected behaviour within a demo task be discovered the 'check' task will
 * write an error to the LCD (via the LCD task).  If all the demo tasks are
 * executing with their expected behaviour then the check task writes PASS
 * along with the max jitter time to the LCD (again via the LCD task), as
 * described above.
 *
 */

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

/***
 * 使用volatile时，修改变量i时，要读、修改、写内存
 * 不使用volatile的话，修改变量i时可能会优化，比如把它的值放在CPU寄存器中，累加时不涉及内存，这样速度会加快
 * 要访问硬件寄存器的话，要加上volatile避免被优化
 * volatile的含义是		易变的，这是提醒编译器，不要轻易优化我
 */
TimerHandle_t xTimerHandleTest;/*定时器句柄 */


void Key_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStruct;/*GPIO结构体*/
	EXTI_InitTypeDef EXTI_InitStruct;/*EXTI结构体*/
	NVIC_InitTypeDef  NVIC_InitStruct;/*NVIC结构体*/
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); /*开启GPIOA时钟*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE); /*开启AFIO时钟复用*/

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;/*PA0*/
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;/*上拉输入*/
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;/*50MHZ*/
	GPIO_Init(GPIOA, &GPIO_InitStruct);/*GPIO初始化*/

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0);/*开启PA0的中断线*/

	EXTI_InitStruct.EXTI_Line = EXTI_Line0;			/*开启中断线*/
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;			/*开启中断*/
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt; /*中断模式*/
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising_Falling ; /*双边沿触发 上升沿/下升沿都触发*/
	EXTI_Init(&EXTI_InitStruct);/*外部中断初始化*/

	NVIC_InitStruct.NVIC_IRQChannel = EXTI0_IRQChannel;		/*中断处理*/
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;			/*使能NVIC*/
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;	/*抢占优先级*/
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;			/*从优先级*/
	NVIC_Init(&NVIC_InitStruct);
}



void EXTI0_IRQHandler(void)/*外部中断0服务函数*/
{
	static int cnt = 0;
	/* xHigherPriorityTaskWoken的含义是：是否有更高优先级的任务被唤醒了。
	 * 如果为pdTRUE，则意味着后面要进行任务切换。
	 */
	BaseType_t pxHigherPriorityTaskWoken = pdFALSE;
	
	if(EXTI_GetFlagStatus(EXTI_Line0) !=RESET)
	{
		printf("EXTI0_IRQChannel is Running, cnt = %d!\r\n",cnt++);  /*测试中断服务函数*/

		/* 使用定时器消除抖动 */
		/**
		* 注意，这次创建的定时器是单次触发的定时器
		* 当定时时间到了之后，就会触发一次回调函数，然后进入阻塞状态
		* 只有当再次启动唤醒定时器才可以执行 
		* 比如这里的xTimerReset   ， 
		*	-会重新设置他的超时时间，这个超时时间等于当前时刻+定时周期
		*	-当到达这个时刻，回调函数才会被调用
		*/
		//xTimerReset(xTimerHandleTest, 0);   /*不可以直接使用这个函数 能够使用是因为将等待时间改为了0*/

		/**
		* 这些中断ISR函数并不会发起调度
		* 会唤醒任务，但是不会引起调度
		* 将会在这个变量pxHigherPriorityTaskWoken 记录是否需要调度
		* 这个变量初始值应该为pdFLASE
		*/
		xTimerResetFromISR(xTimerHandleTest, &pxHigherPriorityTaskWoken); 
		portEND_SWITCHING_ISR( pxHigherPriorityTaskWoken );
		/**
		 * 使用这个宏来进行任务调度  ， 
		 * 如果xHigherPriorityTaskWoken 的值为pdTRUE的话
		 * 将会触发任务调度，否则将不会触发任务调度
		 * 触发一个中断，当当前中断运行完后，立马进入这个中断，处理任务调度
		 */
		
		EXTI_ClearITPendingBit(EXTI_Line0);/*清除中断标志位*/
	}
}

void Task1Function(void *param)
{
	volatile int i; /*让系统不再优化此变量*/

	/* 启动定时器 */
	/**
	* 实质上是向队列中写入命令数据
	* 如果队列满了的话 需要等待阻塞
	* 这里不等
	*/
	xTimerStart(xTimerHandleTest, 0);
	/**
	* 虽然定时器是在任务1中启动的
	* 但是定时器并不是任务1 的定时器
	* 定时器的中断回调函数不是由任务1来执行，而是由守护任务来执行
	* 守护任务一执行， 就会把task1打断，执行中断回调函数
	* 中断回调函数执行完后， 再次等待定时周期的到来
	* task1继续运行
	* 如此往复
	*/

	while (1)
	{
		//printf("Task1Function .....\r\n");
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
	printf("Get Key INT cnt = %d\r\n",cnt++);
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

	/*按键初始化*/
	Key_Init();

	/*创建定时器   , 只触发一次, 周期为2s*/
	xTimerHandleTest = xTimerCreate("Timer", 2000, pdFALSE, NULL, MyTimerCallbackFunction);

	/*创建task1*/
	xTaskCreate(Task1Function, "task1", 100, NULL, 1, &xHandleTask1);

	/* Start the scheduler. */
	vTaskStartScheduler(); /*启动任务调度器*/

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
