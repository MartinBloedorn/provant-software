/**
  ******************************************************************************
  * @file    modules/io/c_io_blctrl.c
  * @author  Patrick Jose Pereira
  * @version V1.0.0
  * @date    06-Dezembro-2014
  * @brief   Implementação do esc BL-Ctrl 2.0.
  *****************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "c_io_blctrl.h"

#include "c_common_i2c.h"

#include <math.h>

/** @addtogroup Module_IO
  * @{
  */

/** @addtogroup Module_IO_Component_BLCTRL
  *	\brief Componente para o Esc Mikrokopter BL-Ctrl.
  *
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
//#define I2Cx_blctrl             I2C2// i2c of blctrl

#define BLCTRL_NUM_OF_MAGNETS   14 // numero de polos do motor
#define BLCTRL_BUFFER_SIZE      0x18 // tamanho do buffer
#define BLCTRL_BUFFER_L         4 // tamanho do buffer
#define BLCTRL_CURRENT          0 // local da memoria onde esta o valor de corrente
#define BLCTRL_TEMPERATURE      2 // local da memoria onde esta o valor de temperatura
#define BLCTRL_MAH              2 // local da memoria onde esta o valor da corrente  em mAh
#define BLCTRL_SPEED            3 // local da memoria onde esta a velocidade
#define BLCTRL_VOLTAGE          5 // local da memoria onde esta o valor de tensao

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
bool ppm;
char blctrl_buffer[BLCTRL_BUFFER_L][BLCTRL_BUFFER_SIZE]={};  // buffer para a leitura do esc
I2C_TypeDef* I2Cx_blctrl; // I2C do blctrl

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions definitions --------------------------------------------*/

/** \brief Inicializa ppm se nao tiver sido inicializado antes
  *
  *
  * @param  None
  * @retval None
  */

void c_io_blctrl_init_ppm()
{
    ppm=1;
    #ifdef STM32F4_H407
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
    TIM_OCInitTypeDef TIM_OCInitStruct;

    /////////////////////////////////////////////////////////////////PF6 TIMER10
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10, ENABLE);

    
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6; //PF6 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOF, &GPIO_InitStructure);

    GPIO_PinAFConfig(GPIOF, GPIO_PinSource6, GPIO_AF_TIM10); 

    // Let PWM frequency equal 100Hz.
    // Let period equal 19000us. Therefore, timer runs from zero to 19000us. 
    // Solving for prescaler gives 168 for 168 MHz device

    //timer 168Mhz
    TIM_TimeBaseInitStruct.TIM_Prescaler = (SystemCoreClock/1000000); // 100 KHz 
    TIM_TimeBaseInitStruct.TIM_Period = 20000;   // 0..999, 100 Hz (us)
    TIM_TimeBaseInitStruct.TIM_ClockDivision = 0;
    TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM10, &TIM_TimeBaseInitStruct);

    TIM_OCStructInit(&TIM_OCInitStruct);
    TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
    // Initial duty cycle equals 0%. Value can range from zero to 1000.
    TIM_OCInitStruct.TIM_Pulse = 0; // 0 .. 1000 (0=Always Off, 1000=Always On)
    TIM_OC1Init(TIM10, &TIM_OCInitStruct);
    TIM_OC2Init(TIM10, &TIM_OCInitStruct);
    TIM_Cmd(TIM10, ENABLE);

    /////////////////////////////////////////////////////////////////PF9 TIMER11
    GPIO_InitTypeDef GPIO_InitStructure2;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct2;
    TIM_OCInitTypeDef TIM_OCInitStruct2;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14, ENABLE);

    
    GPIO_InitStructure2.GPIO_Pin =  GPIO_Pin_9; //PF9
    GPIO_InitStructure2.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure2.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure2.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_InitStructure2.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOF, &GPIO_InitStructure2);

    GPIO_PinAFConfig(GPIOF, GPIO_PinSource9, GPIO_AF_TIM14); 

    // Let PWM frequency equal 100Hz.
    // Let period equal 19000. Therefore, timer runs from zero to 19000.
    // Solving for prescaler gives 168 for 84 MHz device

    //timer 84Mhz
    TIM_TimeBaseInitStruct2.TIM_Prescaler = (SystemCoreClock/2000000); // 100 KHz 
    TIM_TimeBaseInitStruct2.TIM_Period = 20000;   // 0..999, 100 Hz (us)
    TIM_TimeBaseInitStruct2.TIM_ClockDivision = 0;
    TIM_TimeBaseInitStruct2.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM14, &TIM_TimeBaseInitStruct2);

    TIM_OCStructInit(&TIM_OCInitStruct2);
    TIM_OCInitStruct2.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStruct2.TIM_OCMode = TIM_OCMode_PWM1;
    // Initial duty cycle equals 0%. Value can range from zero to 1000.
    TIM_OCInitStruct2.TIM_Pulse = 0; // 0 .. 1000 (0=Always Off, 1000=Always On)
    TIM_OC1Init(TIM14, &TIM_OCInitStruct2);
    TIM_OC2Init(TIM14, &TIM_OCInitStruct2);
    TIM_Cmd(TIM14, ENABLE);
  #endif
}

/** \brief Inicializa I2CX se nao tiver sido inicializada antes
  *
  * \todo Fazer a funcao e evitar conflitos com a inicializacao normal da I2X.
  *
  * @param  None
  * @retval None
  */
void c_io_blctrl_init_i2c(I2C_TypeDef* I2Cx)
{
  ppm=0;
  I2Cx_blctrl=I2Cx;
}

/** \brief Seta a velocidade desejada, em rpm.
  * Retorna \b 1 em caso de sucesso.
  * Retorna \b -1 em caso de falha.
  *
  * @param  ID ID do esc.
  * @param  speed velocidade em rpm.
  * @retval sucesso ou nao.
  * @todo Existe um erro relacionado ao I2C na funcao init.
  */
int  c_io_blctrl_setSpeed(uint8_t ID, unsigned char speed)
{
  //RPM = raw * 780 / no. of magnets
  //int raw = (int) speed*BLCTRL_NUM_OF_MAGNETS/780;
  if(speed<=255 && speed>=0)
  {
    if(ppm)
    {
      if(ID==0)
      {
        TIM10->CCR1 = 1160 + speed*3.92;
        TIM10->CCR2 = 1160 - speed*3.92;
      }
      if(ID==1)
      {
        TIM14->CCR1 = 1090 + speed*3.92;
        TIM14->CCR2 = 1090 - speed*3.92;
      }
      return 1;
    }
    else
    {
      c_common_i2c_start(I2Cx_blctrl, (BLCTRL_ADDR+ID)<<1, I2C_Direction_Transmitter);
      c_common_i2c_write(I2Cx_blctrl, (uint8_t) speed);
      c_common_i2c_stop(I2Cx_blctrl);
      return 1;
    }
  }
  else
  {
    return -1;
  }
}


/** \brief Lê o valor dado um endereço de memoria, entre 0 e 0x18.
  * Retorna o valor em caso de sucesso.
  *
  * @param  ID ID do esc.
  * @retval Local da memoria.
  */
int  c_io_blctrl_read(uint8_t ID, int local)
{
  return (int) blctrl_buffer[ID][local];
}

/** \brief Lê a velocidade atual do esc, em rpm.
  * Retorna o valor em caso de sucesso.
  *
  * @param  ID ID do esc.
  * @retval Velocidade em rpm.
  */
int  c_io_blctrl_readSpeed(uint8_t ID)
{
  return (int) blctrl_buffer[ID][BLCTRL_SPEED]*780/BLCTRL_NUM_OF_MAGNETS;
}

/** \brief Lê o valor de tensao atual do esc, em 10*V.
  * Retorna o valor em caso de sucesso.
  *
  * @param  ID ID do esc.
  * @retval Tensao em 10*V
  */
int  c_io_blctrl_readVoltage(uint8_t ID)
{
  return (int) blctrl_buffer[ID][BLCTRL_VOLTAGE];
}

/** \brief Lê a memoria do esc.
  * Retorna o \b 1 caso de sucesso.
  *
  * @param  ID ID do esc.
  * @retval \b 1 se sucesso.
  */
int  c_io_blctrl_updateBuffer(uint8_t ID)
{  
  c_common_i2c_readBytes(I2Cx_blctrl, BLCTRL_ADDR + ID, 0x00, 4, blctrl_buffer[ID]);
  return 1;
}

/* IRQ handlers ------------------------------------------------------------- */

/**
  * @}
  */

/**
  * @}
  */

