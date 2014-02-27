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
//#define I2Cx_blctrl             I2C1// i2c of blctrl

#define BLCTRL_NUM_OF_MAGNETS   14 // numero de polos do motor
#define BLCTRL_BUFFER_SIZE      10 // tamanho do buffer
#define BLCTRL_BUFFER_L         4 // tamanho do buffer
#define BLCTRL_CURRENT          0 // local da memoria onde esta o valor de corrente
#define BLCTRL_TEMPERATURE      2 // local da memoria onde esta o valor de temperatura
#define BLCTRL_MAH              2 // local da memoria onde esta o valor da corrente  em mAh
#define BLCTRL_SPEED            3 // local da memoria onde esta a velocidade
#define BLCTRL_VOLTAGE          5 // local da memoria onde esta o valor de tensao

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
char blctrl_buffer[BLCTRL_BUFFER_L][BLCTRL_BUFFER_SIZE]={};  // buffer para a leitura do esc
I2C_TypeDef* I2Cx_blctrl; // I2C do blctrl

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions definitions --------------------------------------------*/

/** \brief Inicializa I2CX se nao tiver sido inicializada antes
  *
  * \todo Fazer a funcao e evitar conflitos com a inicializacao normal da I2X.
  *
  * @param  None
  * @retval None
  */
void c_io_blctrl_init(I2C_TypeDef* I2Cx)
{
  I2Cx_blctrl=I2Cx;
}

/** \brief Seta a velocidade desejada, em rpm.
  * Retorna \b 1 em caso de sucesso.
  * Retorna \b -1 em caso de falha.
  *
  * @param  ID ID do esc.
  * @param  speed velocidade em rpm.
  * @retval sucesso ou nao.
  */
int  c_io_blctrl_setSpeed(uint8_t ID, int speed)
{
  //RPM = raw * 780 / no. of magnets
  int raw = (int) speed*BLCTRL_NUM_OF_MAGNETS/780;

  if(raw<=255 && raw>=0)
  {
    c_common_i2c_start(I2Cx_blctrl, (BLCTRL_ADDR+ID)<<1, I2C_Direction_Transmitter);
    c_common_i2c_write(I2Cx_blctrl, (uint8_t) raw);
    c_common_i2c_stop(I2Cx_blctrl);
    return 1;
  }
  else
  {
    return -1;
  }

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
  for(int i=0;i<BLCTRL_BUFFER_SIZE;i++)
  {
    c_common_i2c_start(I2Cx_blctrl, (BLCTRL_ADDR+ID)<<1, I2C_Direction_Receiver);
    blctrl_buffer[ID][i]=c_common_i2c_readNack(I2Cx_blctrl);
    for(int b=0;b<1000;b++); // gastar tempo para conseguir ler varios enderecos
    c_common_i2c_stop(I2Cx_blctrl);
  }
  
  return 1;
}

/* IRQ handlers ------------------------------------------------------------- */

/**
  * @}
  */

/**
  * @}
  */

