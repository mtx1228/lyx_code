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
#include "serial.h"

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"	/*队列头文件*/
#include "semphr.h" /*信号量头文件*/

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
static int sum = 0;

/***
 * 使用volatile时，修改变量i时，要读、修改、写内存
 * 不使用volatile的话，修改变量i时可能会优化，比如把它的值放在CPU寄存器中，累加时不涉及内存，这样速度会加快
 * 要访问硬件寄存器的话，要加上volatile避免被优化
 * volatile的含义是		易变的，这是提醒编译器，不要轻易优化我
 */


static volatile int flagCalcEnd = 0; /*task1计算结束 为1*/

static volatile int flagUARTUsed = 0; /*串口是否使用*/

SemaphoreHandle_t xSemaphoreHandleTest; /*计数型信号量的句柄*/

SemaphoreHandle_t xSemaphoreHandleBinary; /*二进制型信号量的句柄*/

static TaskHandle_t xHandleTask2; // task2的任务句柄


void Task1Function(void *param)
{
	volatile int i; /*让系统不再优化此变量*/

	while (1)
	{
		for (i = 0; i < 10000; i++)
		{
			sum++;
		}
		for(i=0;i<10;i++)
			xTaskNotifyGive(xHandleTask2); /*此时计算结束     ， 给task2发送任务通知，让通知值 +1*/
		/***
		 * 发送任务通知给目标任务task2
		 * 通知值+10次之后，让通知值+1
		 * 当计算完后，自杀一下 让task2得以运行
		 */
		vTaskDelete(NULL); /*自杀  task1删除*/

		// printf("1");
	}
}

void Task2Function(void *param)
{
	static int i = 0;
	int val;
	while (1)
	{
		flagCalcEnd = 0;
		/**
		*** xClearCountOnExit 的参数选择
		* pdTRUE   - 在接收到任务通知值时将会清零任务通知值
		* pdFALSE  - 在接收到任务通知值时，不会清零任务通知值
		*** 函数的返回值
		* 大于0  ， 返回任务通知值
		* 等于0， 一直没有被其他task通知，超时返回
		*/
		val = ulTaskNotifyTake(pdTRUE,portMAX_DELAY); 
		/***
		 * 一开始，task2的任务通知值为0   无法take 所以task2进入阻塞状态
		 * task1计算完成，给task2发送任务通知，让通知值+1
		 * task2表示接收到了数据，待处理
		 * 这时task2由阻塞状态进入就绪状态，从而进入Running状态
		 * 打印出数据
		 */
		flagCalcEnd = 1; /*task1计算完成 task2处于就绪状态 进而转入运行态 从而打印数据*/

		printf("sum = %d, val = %d, i = %d\r\n", sum,val,i++);
	}
}

void TaskGeneralFunction(void *param)
{
	while (1)
	{
		xSemaphoreTake(xSemaphoreHandleBinary, portMAX_DELAY); /*take信号量  此时信号量-1   为0*/
		printf("%s\r\n", (char *)param);
		xSemaphoreGive(xSemaphoreHandleBinary); /*give信号量  信号量+1 为1*/
		/***
		 * 二进制信号量give  最大值为1
		 * 就无法+了,会进入阻塞状态
		 * 等待信号量计数值变为0  然后再give
		 */
		vTaskDelay(1); /*防止task4一直独占串口的情况  blocked后task3才可以执行*/
	}
}

void Task3Function(void *param)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStruct);

	while (1)
	{
		GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_RESET);
		vTaskDelay(100);
		GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_SET);
		vTaskDelay(100);
	}
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

	/*使用计数型的信号量 实现同步功能*/
	/*创建一个计数型的信号量  此函数内部会分配信号量结构体*/
	/***
	 * 第一个参数是信号量 最大计数值  此时信号量最大计数值为10
	 * 第二个参数是信号量 初始计数值  此时信号量初始计数值为0
	 */
	xSemaphoreHandleTest = xSemaphoreCreateCounting(10, 0);

	/*二进制型的信号量  实现互斥功能*/
	/*创建一个二进制型的信号量*/
	xSemaphoreHandleBinary = xSemaphoreCreateBinary(); /*这个信号量的初始值为0*/
	/*需要give一次 说明串口是可以使用的  不然会阻塞*/
	xSemaphoreGive(xSemaphoreHandleBinary); /*二进制信号量计数值 +1 等待take*/

	// 动态分配task
	xTaskCreate(Task1Function, "task1", 100, NULL, 1, &xHandleTask1); // 任务1 打印1
	xTaskCreate(Task2Function, "task2", 100, NULL, 1, &xHandleTask2);		  // 任务2 打印2

	//xTaskCreate(TaskGeneralFunction, "task3", 100, "Task3 is Running!", 1, NULL); // 任务1 打印1
	//xTaskCreate(TaskGeneralFunction, "task4", 100, "Task4 is Running!", 1, NULL); // 任务1 打印1

	//  xTaskCreate(Task3Function, "task3", 100, NULL, 1, NULL);		  // 任务3 点灯

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
