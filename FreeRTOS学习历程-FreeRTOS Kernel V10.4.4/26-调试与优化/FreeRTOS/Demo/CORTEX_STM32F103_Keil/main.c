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

static int timerFlag = 0;/*��ʱ���жϱ���*/
static int timerIntFlag = 0;

static uint32_t g_timer_cnt;/*��ȡʱ��*/

/***
 * ʹ��volatileʱ���޸ı���iʱ��Ҫ�����޸ġ�д�ڴ�
 * ��ʹ��volatile�Ļ����޸ı���iʱ���ܻ��Ż������������ֵ����CPU�Ĵ����У��ۼ�ʱ���漰�ڴ棬�����ٶȻ�ӿ�
 * Ҫ����Ӳ���Ĵ����Ļ���Ҫ����volatile���ⱻ�Ż�
 * volatile�ĺ�����		�ױ�ģ��������ѱ���������Ҫ�����Ż���
 */
TimerHandle_t xTimerHandleTest;/*��ʱ����� */


void Timer3_Init(void)/*��ʱ��2��ʼ��*/
{	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;/*ʱ����Ԫ��ʼ���ṹ��*/
	NVIC_InitTypeDef NVIC_InitStruct;/*NVIC�ṹ��*/

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;/*����Ƶ*/
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;/*���ϼ���*/
	TIM_TimeBaseInitStruct.TIM_Period = 100-1;/*�Զ���װֵ*/
	TIM_TimeBaseInitStruct.TIM_Prescaler = 72-1;/*Ԥ��Ƶֵ*/
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStruct);/*ʱ����Ԫ��ʼ��*/

	NVIC_InitStruct.NVIC_IRQChannel = TIM3_IRQChannel; /*TIM2���ж���*/
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;/*�ж�ʹ��*/
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;/*��ռ���ȼ����*/
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;/*�����ȼ����*/
	NVIC_Init(&NVIC_InitStruct);/*NVIC��ʼ��*/

	//TIM_ClearFlag(TIM2, TIM_FLAG_Update);	/*��ֹһ��ʱ�ͼ�һ����*/
	//TIM_ClearITPendingBit(TIM2, TIM_IT_Update); // ��������ж�����λ		  //��仰һ��Ҫ����ʹ�ܸ����ж�ǰ�棬����ʱ����ʼ����ֱ�ӽ���һ�θ����ж�
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);/*��ʱ���ж�ӳ��*/

	TIM_Cmd(TIM3, ENABLE);/*������ʱ��*/
}

/*�жϵĴ���*/
uint32_t TimerGetCount(void)
{
	return g_timer_cnt;
}

void TIM3_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM3, TIM_IT_Update)==SET)
	{
		timerIntFlag=!timerIntFlag;
		g_timer_cnt++;	/*�жϵĴ���*/
		//printf("TIM3_IRQHandler is Running!\r\n");
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
	}
}

/*ʵ�ֻ�ȡʱ��ĺ���*/

char pcWriteBuffer[200];/*���ϵͳ�����ͳ����Ϣ*/

void Task1Function(void *param)
{
	volatile int i; /*��ϵͳ�����Ż��˱���*/

	while (1)
	{
		//printf("Task1Function .....\r\n");
		
		/*��������ͳ����Ϣ*/
		//vTaskList(pcWriteBuffer);
		/*���CPU��Դռ����*/
		vTaskGetRunTimeStats(pcWriteBuffer);
		printf(pcWriteBuffer);/*��ӡͳ����Ϣ*/
		vTaskDelay(1000);/*��ʱ*/
	}
}

void Task2Function(void *param)/*�ۼ�*/
{
	volatile int i; /*��ϵͳ�����Ż��˱���*/

	while (1)
	{
		
	}
}


		/*��ʱ���жϻص�����*/
void MyTimerCallbackFunction( TimerHandle_t xTimer )
{
	static int cnt;
	timerFlag = !timerFlag;
	printf("MyTimerCallbackFunction  cnt = %d\r\n",cnt++);
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

	//Timer3_Init(72-1, 100-1);/*TIM2��ʼ�� ��ʱ0.1ms*/
	
	/* ������ʱ�� */
	/**
	* ��һ�������Ƕ�ʱ�������֣��������
	* �ڶ��������Ƕ�ʱ���Ķ�ʱ���� ���ﶨʱ100ms 100��tick
	* �����������Ƕ�ʱ���Ƿ�  ��  �ظ�����  �� ����ִ�л����ظ�ִ��  �� 
		pdFALSE - �ظ�ִ��     pdTRUE	- ����ִ��
	* ���ĸ������Ƕ�ʱ����ID���ص���������ʹ��������������ֱ����ĸ���ʱ��
	* ����������Ƕ�ʱ���жϻص�����
	*/
	//xTimerHandleTest = xTimerCreate("Timer", 20000, pdFALSE, NULL, MyTimerCallbackFunction);

	/*��̬����task*/
	xTaskCreate(Task1Function, "task1", 100, NULL, 2, &xHandleTask1); // ����1 ��ӡ1
	xTaskCreate(Task2Function, "task2", 100, NULL, 1, NULL);		  // ����2 ��ӡ2

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
