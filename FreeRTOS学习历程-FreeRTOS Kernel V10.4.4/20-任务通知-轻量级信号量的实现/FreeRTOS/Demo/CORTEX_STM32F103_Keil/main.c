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
#include "queue.h"	/*����ͷ�ļ�*/
#include "semphr.h" /*�ź���ͷ�ļ�*/

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
 * ʹ��volatileʱ���޸ı���iʱ��Ҫ�����޸ġ�д�ڴ�
 * ��ʹ��volatile�Ļ����޸ı���iʱ���ܻ��Ż������������ֵ����CPU�Ĵ����У��ۼ�ʱ���漰�ڴ棬�����ٶȻ�ӿ�
 * Ҫ����Ӳ���Ĵ����Ļ���Ҫ����volatile���ⱻ�Ż�
 * volatile�ĺ�����		�ױ�ģ��������ѱ���������Ҫ�����Ż���
 */


static volatile int flagCalcEnd = 0; /*task1������� Ϊ1*/

static volatile int flagUARTUsed = 0; /*�����Ƿ�ʹ��*/

SemaphoreHandle_t xSemaphoreHandleTest; /*�������ź����ľ��*/

SemaphoreHandle_t xSemaphoreHandleBinary; /*���������ź����ľ��*/

static TaskHandle_t xHandleTask2; // task2��������


void Task1Function(void *param)
{
	volatile int i; /*��ϵͳ�����Ż��˱���*/

	while (1)
	{
		for (i = 0; i < 10000; i++)
		{
			sum++;
		}
		for(i=0;i<10;i++)
			xTaskNotifyGive(xHandleTask2); /*��ʱ�������     �� ��task2��������֪ͨ����ֵ֪ͨ +1*/
		/***
		 * ��������֪ͨ��Ŀ������task2
		 * ֵ֪ͨ+10��֮����ֵ֪ͨ+1
		 * �����������ɱһ�� ��task2��������
		 */
		vTaskDelete(NULL); /*��ɱ  task1ɾ��*/

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
		*** xClearCountOnExit �Ĳ���ѡ��
		* pdTRUE   - �ڽ��յ�����ֵ֪ͨʱ������������ֵ֪ͨ
		* pdFALSE  - �ڽ��յ�����ֵ֪ͨʱ��������������ֵ֪ͨ
		*** �����ķ���ֵ
		* ����0  �� ��������ֵ֪ͨ
		* ����0�� һֱû�б�����task֪ͨ����ʱ����
		*/
		val = ulTaskNotifyTake(pdTRUE,portMAX_DELAY); 
		/***
		 * һ��ʼ��task2������ֵ֪ͨΪ0   �޷�take ����task2��������״̬
		 * task1������ɣ���task2��������֪ͨ����ֵ֪ͨ+1
		 * task2��ʾ���յ������ݣ�������
		 * ��ʱtask2������״̬�������״̬���Ӷ�����Running״̬
		 * ��ӡ������
		 */
		flagCalcEnd = 1; /*task1������� task2���ھ���״̬ ����ת������̬ �Ӷ���ӡ����*/

		printf("sum = %d, val = %d, i = %d\r\n", sum,val,i++);
	}
}

void TaskGeneralFunction(void *param)
{
	while (1)
	{
		xSemaphoreTake(xSemaphoreHandleBinary, portMAX_DELAY); /*take�ź���  ��ʱ�ź���-1   Ϊ0*/
		printf("%s\r\n", (char *)param);
		xSemaphoreGive(xSemaphoreHandleBinary); /*give�ź���  �ź���+1 Ϊ1*/
		/***
		 * �������ź���give  ���ֵΪ1
		 * ���޷�+��,���������״̬
		 * �ȴ��ź�������ֵ��Ϊ0  Ȼ����give
		 */
		vTaskDelay(1); /*��ֹtask4һֱ��ռ���ڵ����  blocked��task3�ſ���ִ��*/
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
	TaskHandle_t xHandleTask1; // ������ �Ժ�ʹ�����������������

#ifdef DEBUG
	debug();
#endif

	prvSetupHardware(); // Ӳ������

	printf("Hello,World!\r\n");

	/*ʹ�ü����͵��ź��� ʵ��ͬ������*/
	/*����һ�������͵��ź���  �˺����ڲ�������ź����ṹ��*/
	/***
	 * ��һ���������ź��� ������ֵ  ��ʱ�ź���������ֵΪ10
	 * �ڶ����������ź��� ��ʼ����ֵ  ��ʱ�ź�����ʼ����ֵΪ0
	 */
	xSemaphoreHandleTest = xSemaphoreCreateCounting(10, 0);

	/*�������͵��ź���  ʵ�ֻ��⹦��*/
	/*����һ���������͵��ź���*/
	xSemaphoreHandleBinary = xSemaphoreCreateBinary(); /*����ź����ĳ�ʼֵΪ0*/
	/*��Ҫgiveһ�� ˵�������ǿ���ʹ�õ�  ��Ȼ������*/
	xSemaphoreGive(xSemaphoreHandleBinary); /*�������ź�������ֵ +1 �ȴ�take*/

	// ��̬����task
	xTaskCreate(Task1Function, "task1", 100, NULL, 1, &xHandleTask1); // ����1 ��ӡ1
	xTaskCreate(Task2Function, "task2", 100, NULL, 1, &xHandleTask2);		  // ����2 ��ӡ2

	//xTaskCreate(TaskGeneralFunction, "task3", 100, "Task3 is Running!", 1, NULL); // ����1 ��ӡ1
	//xTaskCreate(TaskGeneralFunction, "task4", 100, "Task4 is Running!", 1, NULL); // ����1 ��ӡ1

	//  xTaskCreate(Task3Function, "task3", 100, NULL, 1, NULL);		  // ����3 ���

	/* Start the scheduler. */
	vTaskStartScheduler(); // ��������

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
