/**
  ******************************************************************************
  * @file    modules/rc/pv_module_rc.h
  * @author  Martin Vincent Bloedorn
  * @version V1.0.0
  * @date    30-November-2013
  * @brief   Definição do módulo de controle e comunicação via rádio manual.
  *****************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef PV_MODULE_RC_H
#define PV_MODULE_RC_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

/* FreeRTOS kernel includes */
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

/* proVANT includes */
#include "c_common_gpio.h"
#include "c_rc_control.h"
#include "c_rc_receiver.h"

#include "pv_typedefs.h"

/* Exported types ------------------------------------------------------------*/

 /** \brief Ponteiro de in- e outboxes do módulo de RC.
   *
   * Armazena e envia dados brutos ou pré-processados de sensores, e recebe
   * setpoints ou comandos para atuadores.
   */
struct pv_interface_rc {
	xQueueHandle oActuation;	/** Sinais de atuação para módulo de IO. **/
	xQueueHandle iSensorTime;	/** Sinais de atuação para módulo de IO. **/
	xQueueHandle iAttitude;		/** Feedback da orientação do VANT. **/
} pv_interface_rc;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
void module_rc_init();
void module_rc_run();

#ifdef __cplusplus
}
#endif

#endif //PV_MODULE_RC_H
