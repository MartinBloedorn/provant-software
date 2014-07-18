/**
  ******************************************************************************
  * @file    modules/datapr/c_datapr_MultWii.c
  * @author  Patrick José Pereira
  * @version V1.0.0
  * @date    16-Jul-2014
  * @brief   Funções para envio de dados para a interface de do Multwii
  *****************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "c_datapr_MultWii.h"

/** @addtogroup Module_DATAPR
  * @{
  */

/** @addtogroup Module_DATAPR_Component_MultWii
  * \brief Componente para tratamento de dados e envio para a interface multwii.
  *
  * 
  * 
  * @{
  */


/* Private variables ---------------------------------------------------------*/
static const int _multwiisize=1024;;
uint16_t multwii_msgindex = 0;
uint8_t multwii_checksum = 0;

/* Private functions ---------------------------------------------------------*/

/**
 * \brief Funcao para serializar os bytes.
 * @param a byte de entrada
 */
void serialize8(uint8_t a)
{
  //Serial.write(a);
  multwii_msg[multwii_msgindex++] = a;
  multwii_msg[multwii_msgindex] = '\0';
  multwii_checksum ^= a &0x00FF;
  if (multwii_msgindex >= _multwiisize)
  {
    multwii_msgindex = 0;
  }
}

/**
 * Retorna o tamanho da pilha 
 * @return Retorna o tamanho da pilha
 */
int get_raw_size()
{
  return multwii_msgindex;
}

/**
 * Retorna a pilha
 * @return Retorna a pilha
 */
char* get_raw_String()
{
  multwii_msgindex = 0;
  return multwii_msg;
}

/**
 * Serializa dois bytes na pilha
 * @param a Qualquer representação de 2 bytes.
 */
void serialize16(int16_t a)
{
  serialize8((a   ) & 0xFF);
  serialize8((a>>8) & 0xFF);
}

/**
 * Serializa 4 bytes como se fosse 2.
 * @param a Qualquer representação de 4 bytes.
 */
void serialize32_as16(int32_t a)
{
  serialize8(((int16_t)a   ) & 0xFF);
  serialize8(((int16_t)a>>8) & 0xFF);
}

/**
 * Serializa 4 bytes na pilha.
 * @param a Qualquer representação de 4 bytes.
 */
void serialize32(uint32_t a) 
{
  serialize8((a    ) & 0xFF);
  serialize8((a>> 8) & 0xFF);
  serialize8((a>>16) & 0xFF);
  serialize8((a>>24) & 0xFF);
}

/**
 * Monta o header da mensagem.
 * @param size        tamanho da mensagem em bytes.
 * @param multwii_msg tipo da mensagem
 */
void headSerialResponse(uint8_t size, uint8_t multwii_msg)
{
  serialize8('$');
  serialize8('M');
  serialize8('>');
  multwii_checksum = 0; // start calculating a new multwii_checksum
  serialize8(size);
  serialize8(multwii_msg);
}

/**
 * Finaliza o pacote a ser enviado montando o checksum.
 */
void tailSerialReply()
{
  serialize8(multwii_checksum);	
}

/*********************End of serial comm************************************/

/* Exported functions ------------------------------------------------------- */
/*********************Begin of ... something************************************/

/**
 * Envia a attitude para a pilha.
 * @param x Roll
 * @param y Pitch
 * @param z Yaw
 */
void c_common_datapr_multwii_attitude(float x,float y,float z)
{
  headSerialResponse(6,MSP_ATTITUDE);
  serialize32_as16(x*10);
  serialize32_as16(y*10);
  serialize32_as16(z);
  tailSerialReply();
}

/**
 * Envia os dados da imu para a pilha.
 * @param acc x,y,z do acelerometro
 * @param gyr x,y,y do giroscopio
 * @param mag x,y,z do magnetometro
 */
void c_common_datapr_multwii_raw_imu(float* acc,float* gyr, float* mag)
{
  headSerialResponse(18,MSP_RAW_IMU);
  serialize32_as16(acc[0]);
  serialize32_as16(acc[1]);
  serialize32_as16(acc[2]);
  serialize32_as16(gyr[0]);

  serialize32_as16(gyr[1]);
  serialize32_as16(gyr[2]);
  serialize32_as16(mag[0]);
  serialize32_as16(mag[1]);
  serialize32_as16(mag[2]);
  tailSerialReply();
}

/**
 * Envia a altitude para a pilha.
 * @param alt   altitude
 * @param vario a derivada da altitude, usado em planadores como feedback pra saber se esta numa termica.
 */
void c_common_datapr_multwii_altitude(float alt, float vario)
{
  headSerialResponse(6,MSP_ALTITUDE);
  serialize32(alt);
  serialize16(vario);
  tailSerialReply();
}

/**
 * Manda para a pilha a identificação do vant como tiltrotor.
 */
void c_common_datapr_multwii_bicopter_identifier()
{
  headSerialResponse(7,MSP_IDENT);
  serialize8(32);
  serialize8(4); //codigo do bicoptero
  serialize8(0); //not used
  serialize32(0);
  tailSerialReply();
}

/**
 * Envia a posição dos pinos para saber a posição dos motores na imagem do multwii
 */
void c_common_datapr_multwii_motor_pins()
{
  headSerialResponse(8, MSP_MOTOR_PINS);
  serialize8(1);
  serialize8(2);
  serialize8(0);
  serialize8(0);

  serialize8(0);
  serialize8(0);
  serialize8(0);
  serialize8(0);
  tailSerialReply();
}

/**
 * Envia os angulos dos motores para a pilha
 * @param angle1 Angulo do servo esquerdo
 * @param angle2 Angulo do servo direito
 * /Todo existe um problema no angle1
 */
void c_common_datapr_multwii_servos(float angle1,float angle2)
{
  headSerialResponse(16,MSP_SERVO);
  serialize16(0);
  serialize16(0);
  serialize16(0);
  serialize16(0);

  serialize16((int)(1500+8.333*angle1));
  serialize16((int)(1500+8.333*angle2));
  serialize16(0);
  serialize16(0);
  tailSerialReply();
}

/**
 * Envia a forca dos motores para a pilha.
 * @param forca_esquerdo forca do motor esquerdo
 * @param forca_direito  forca do motor direito
 */
void c_common_datapr_multwii_motor(float forca_esquerdo,float forca_direito)
{
  headSerialResponse(16, MSP_MOTOR);
  serialize32_as16((int)((forca_esquerdo*100)));
  serialize32_as16((int)((forca_direito*100)));
  serialize32_as16(0);
  serialize32_as16(0);

  serialize32_as16(0);
  serialize32_as16(0);
  serialize32_as16(0);
  serialize32_as16(0);
  tailSerialReply();
}

/**
 * Envia os dados de debug para a pilha.
 * @param debug1 debug1
 * @param debug2 debug2
 * @param debug3 debug3
 * @param debug4 debug4
 */
void c_common_datapr_multwii_debug(float debug1,float  debug2,float  debug3,float debug4)
{
  headSerialResponse(8, MSP_DEBUG);
  serialize32_as16((int)((debug1)));
  serialize32_as16((int)((debug2)));
  serialize32_as16((int)((debug3)));
  serialize32_as16((int)((debug4)));
  tailSerialReply();
}

void c_common_datapr_multwii_sendstack(USART_TypeDef* USARTx)
{
  for (int i = 0; i <  get_raw_size(); ++i)
    c_common_usart_putchar(USARTx,multwii_msg[i]);
  get_raw_String(); // limpa a pilha;
}

/**
  * @}
  */

/**
  * @}
  */