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

/* Task priorities. */
#define mainQUEUE_POLL_PRIORITY (tskIDLE_PRIORITY + 2)
#define mainCHECK_TASK_PRIORITY (tskIDLE_PRIORITY + 3)
#define mainSEM_TEST_PRIORITY (tskIDLE_PRIORITY + 1)
#define mainBLOCK_Q_PRIORITY (tskIDLE_PRIORITY + 2)
#define mainCREATOR_TASK_PRIORITY (tskIDLE_PRIORITY + 3)
#define mainFLASH_TASK_PRIORITY (tskIDLE_PRIORITY + 1)
#define mainCOM_TEST_PRIORITY (tskIDLE_PRIORITY + 1)
#define mainINTEGER_TASK_PRIORITY (tskIDLE_PRIORITY)

/* Constants related to the LCD. */
#define mainMAX_LINE (240)
#define mainROW_INCREMENT (24)
#define mainMAX_COLUMN (20)
#define mainCOLUMN_START (319)
#define mainCOLUMN_INCREMENT (16)

/* The maximum number of message that can be waiting for display at any one
time. */
#define mainLCD_QUEUE_SIZE (3)

/* The check task uses the sprintf function so requires a little more stack. */
#define mainCHECK_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE + 50)

/* Dimensions the buffer into which the jitter time is written. */
#define mainMAX_MSG_LEN 25

/* The time between cycles of the 'check' task. */
#define mainCHECK_DELAY ((TickType_t)5000 / portTICK_PERIOD_MS)

/* The number of nano seconds between each processor clock. */
#define mainNS_PER_CLOCK ((unsigned long)((1.0 / (double)configCPU_CLOCK_HZ) * 1000000000.0))

/* Baud rate used by the comtest tasks. */
#define mainCOM_TEST_BAUD_RATE (115200)

/* The LED used by the comtest tasks. See the comtest.c file for more
information. */
#define mainCOM_TEST_LED (3)

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
volatile int i; /*��ϵͳ�����Ż��˱���*/

static volatile int flagCalcEnd = 0; /*task1������� Ϊ1*/

static volatile int flagUARTUsed = 0; /*�����Ƿ�ʹ��*/

static QueueHandle_t xQueueCalHandle; /*���еľ��*/

static QueueHandle_t xQueueUARTHandle; /*���еľ��*/

int InitUATtLock(void) /*��ʼ�� ���� ��*/
{
	int val;
	/*����Queue*/
	/*����2��ʾ���еĳ��ȣ�sizeof(int)��ʾ���������ݵĴ�С*/
	xQueueUARTHandle = xQueueCreate(1, sizeof(int));
	if (xQueueUARTHandle == NULL) /*��������ʧ��*/
		return -1;

	xQueueSend(xQueueUARTHandle, &val, portMAX_DELAY); /*�������д������*/
	return 0;

	/****
	 * �����������ݣ���˵�����˿��ԴӶ����ж�ȡ����
	 */
}

void GetUATtLock(void)
{
	int val;
	xQueueReceive(xQueueUARTHandle, &val, portMAX_DELAY); /*��ȡ�����е�����*/

	/****
	 * ����ĳ��taskʹ�ô���
	 * ����ִ�д˺������������
	 * ��ȡ���У��õ����ݾ�˵���õ��˴��ڵ�ʹ��Ȩ
	 */
}

void PutUATtLock(void) /*�ͷŴ���*/
{
	int val;
	xQueueSend(xQueueUARTHandle, &val, portMAX_DELAY); /*�������д������*/

	/*****
	 * ʹ���괮�ں󣬾������������д��һ������
	 * ��ʾ�Ѿ�ʹ���괮�ڣ���ʹ��Ȩ����
	 */
}

void Task1Function(void *param)
{
	while (1)
	{
		for (i = 0; i < 10000000; i++)
		{
			sum++;
		}
		// flagCalcEnd = 1;   /*��ʱ�������*/
		// vTaskDelete(NULL); /*��ɱ  task1ɾ��*/
		// printf("1");

		/*task1��Queue��д������*/
		/***
		 * �������д������
		 * ��һ�������Ƕ��о����Ҫд���ĸ�����
		 * �ڶ�������������ָ�룬������ݵ�ֵ���ᱻ���ƽ�����   �Ǵ�sum�ĵ�ַ���п�����������
		 * �����������ǵȴ�ʱ��
						0 	�޷�д������ʱ����������
						portMAX_DELAY	һֱ������ֱ���пռ��д
		 */
		xQueueSend(xQueueCalHandle, &sum, portMAX_DELAY);

		sum = 1; /*�޸�sum��ֵҲ����Ӱ�� �����е�ֵ*/
	}
}

void Task2Function(void *param)
{
	int val; /*��ȡ���е�buff������*/
	while (1)
	{
		// if (flagCalcEnd) /*task2 �ڵȴ�task1��ɼ���  ֮����ͬ���Ĺ�ϵ*/
		// 	printf("sum=%d\r\n", sum);

		flagCalcEnd = 0; /*����task1���ڼ�������*/

		/*task2��ȡQueue*/
		/***
		* ������ж�ȡ����
		 * ��һ�������Ƕ��о����Ҫ���ĸ�����
		 * �ڶ�������������ָ�룬������ݵ�ֵ���ᱻ��ȡ��������
		 * �����������ǵȴ�ʱ��
						0 	�޷�д������ʱ����������
						portMAX_DELAY	һֱ������ֱ���пռ��д
		 */
		xQueueReceive(xQueueCalHandle, &val, portMAX_DELAY);
		flagCalcEnd = 1; /*����task2�Ѿ��Ӷ����л�ȡ������*/
		printf("sum=%d\r\n", val);
	}
}

void TaskGeneralFunction(void *param)
{
	while (1)
	{
		GetUATtLock(); /*���UART��  ��ȡ��������*/
		printf("%s\r\n", (char *)param);

		// ���ͷ������֮ǰ task3�ڵȴ�����
		PutUATtLock(); /*�ͷ�UART��  д���������  task3���뵽Ready״̬  */
					   /***
						* task4ͨ��д���еķ�ʽ����task3������״̬blocked�ŵ��˾���״̬ready
						* ����task4��running���ٴν���whileѭ�����Ӷ�����������ж�ȡ����
						*/

		vTaskDelay(1); /*task4  ������ʱ����blocked״̬  ����task�л���ִ��*/
	}
}

void Task3Function(void *param) /*���task*/
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

	/*��ʼ��������*/
	InitUATtLock();

	/*����Queue*/
	/*����2��ʾ���еĳ��ȣ�sizeof(int)��ʾ���������ݵĴ�С*/
	xQueueCalHandle = xQueueCreate(2, sizeof(int));
	if (xQueueCalHandle == NULL)
		printf("can not create the Queue!\r\n");

	/*��̬����task*/
	xTaskCreate(Task1Function, "task1", 100, NULL, 1, &xHandleTask1); // ����1 ��ӡ1
	xTaskCreate(Task2Function, "task2", 100, NULL, 1, NULL);		  // ����2 ��ӡ2

	xTaskCreate(TaskGeneralFunction, "task3", 100, "Task3 is Running!", 1, NULL); // ����1 ��ӡ1
	xTaskCreate(TaskGeneralFunction, "task4", 100, "Task4 is Running!", 1, NULL); // ����1 ��ӡ1

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
