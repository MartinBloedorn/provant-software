/**
  ******************************************************************************
  * @file    modules/common/pv_typedefs.h
  * @author  Martin Vincent Bloedorn
  * @version V1.0.0
  * @date    10-February-2014
  * @brief   Definições de tipos e estruturas de mensagens para o projeto.
  *
  *****************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef PV_TYPEDEFS_H
#define PV_TYPEDEFS_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_conf.h"

#ifdef __cplusplus
 extern "C" {
#endif

 /** @addtogroup Common_Components
   * @{
   */

 /** @addtogroup PV_Typedefs
   *
   * Definições de tipagem e estruras de mensagem para o projeto.
   * @{
   */

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

/** \brief Um par de floats. */
typedef struct {
	int channel[8];
} pv_type_receiverChannels;

/** \brief Tipo do ESC com informações.*/
typedef struct {
	char  ID;
	float angularSpeed;
	float current;
	float tension;
} pv_type_io_esc;

/* Exported messages ---------------------------------------------------------*/

/** \brief Estrutura de mensagens para os dois RX24f e ESCs.
 *
 * Pacote prevê passagem de referências de velocidade para os ESCs em <b>rad/s</b>,
 * referências agulares em <b>rad</b> e torque em <b>N.m</b>. Para o servo, o módulo de
 * IO automaticamente alternará entre referências de ângulo para torque se a flag
 *  \b servoTorqueControlEnable estiver setada.
 */
typedef struct {
	bool  servoTorqueControlEnable;
	float servoLeft;
	float servoRight;
	float escRightSpeed;
	float escLeftSpeed;
} pv_msg_io_actuation;

/** \brief Estrutura de para orientação do VANT.*/
typedef struct {
	float roll, pitch, yaw;
	float dotRoll, dotPitch, dotYaw;
} pv_msg_datapr_attitude;

/** \brief Estrutura de para posição do VANT.*/
typedef struct {
	float x, y, z;
	float dotX, dotY, dotZ;
} pv_msg_datapr_position;

/** \brief Estrutura para informacões relativos a tempo dos sensores*/
typedef struct {
	float IMU_sample_time; //tempo entre uma aquisicão de dado e outra. Utilizado para integracão. Valor variável.
} pv_msg_datapr_sensor_time;


/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
/* Header-defined wrapper functions ----------------------------------------- */

 /**
   * @}
   */
 /**
   * @}
   */

#ifdef __cplusplus
}
#endif

#endif //PV_TYPEDEFS_H

