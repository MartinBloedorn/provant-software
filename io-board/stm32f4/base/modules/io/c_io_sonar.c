/**
  ******************************************************************************
  * @file    modules/io/c_io_sonar.c
  * @author  Patrick Jose Pereira
  * @version V1.0.0
  * @date    11-Fevereiro-2013
  * @brief   Implementação da leitura do sonar XL-MaxSonar-EZ MB1200.
  ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "c_io_sonar.h"

/** @addtogroup Module_IO
  * @{
  */

/** @addtogroup Module_IO_Component_Sonar
  *	\brief Componente para a leitura do sonar XL-MaxSonar-EZ MB1200.
  *
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
#define SONAR_BAUDRATE  9600
#ifdef STM32F4_H407
  #define SONAR_USART     USART6
#else
  #define SONAR_USART     USART3
#endif
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions definitions --------------------------------------------*/

void adc_configure()
{
  ADC_InitTypeDef       ADC_InitStructure;
  ADC_CommonInitTypeDef ADC_CommonInitStructure;
  DMA_InitTypeDef       DMA_InitStructure;
  GPIO_InitTypeDef      GPIO_InitStructure;

  /* Enable ADCx, DMA and GPIO clocks ****************************************/ 
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3, ENABLE);
  
  /* Configure ADC3 Channel7 pin as analog input ******************************/
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOF, &GPIO_InitStructure);

  /* ADC Common Init **********************************************************/
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
  ADC_CommonInit(&ADC_CommonInitStructure);

  /* ADC3 Init ****************************************************************/
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_10b;
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfConversion = 1;
  ADC_Init(ADC3, &ADC_InitStructure);

  /* ADC3 regular channel7 configuration *************************************/
  ADC_RegularChannelConfig(ADC3, ADC_Channel_4, 1, ADC_SampleTime_3Cycles);

  /* Enable ADC3 */
  ADC_Cmd(ADC3, ENABLE);
}

int adc_convert()
{
 ADC_SoftwareStartConv(ADC3);//Start the conversion
 while(!ADC_GetFlagStatus(ADC3, ADC_FLAG_EOC));//Processing the conversion
 return ADC_GetConversionValue(ADC3); //Return the converted data
}

/**
  * @brief  Initializes the sonar module according to the specified
  *         parameters in the define area .
  * @retval None
  */
void c_io_sonar_init()
{
  #if SERIAL
    #ifdef STM32F4_H407
      c_common_usart6_init(9600); //change this command if SONAR_USART change
    #else
      c_common_usart3_init(9600); //change this command if SONAR_USART change
    #endif
  #else
    adc_configure();
  #endif
}

/**
  * @brief  return the distance between the sonar and floor
  * @retval the distance in centimeters
  */

float  c_io_sonar_read()
{
  #if SERIAL
    char dist[3];
    while(c_common_usart_read(SONAR_USART)!='R'){}
    dist[0]=c_common_usart_read(SONAR_USART);
    dist[1]=c_common_usart_read(SONAR_USART);
    dist[2]=c_common_usart_read(SONAR_USART);
    return (float)atoi(dist);
  #else
    return (float)adc_convert();
  #endif
}

/* IRQ handlers ------------------------------------------------------------- */

/**
  * @}
  */

/**
  * @}
  */

