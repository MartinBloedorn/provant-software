/**
  ******************************************************************************
  * @file    modules/rc/c_rc_control.h
  * @author  Martin Vincent Bloedorn
  * @version V1.0.0
  * @date    30-November-2013
  * @brief   Controle de estabilizacao para vôo com usando controle remoto manual.
  *****************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef C_RC_CONTROL_H
#define C_RC_CONTROL_H

#define ARM_MATH_CM4
#include "arm_math.h"

#include "pv_typedefs.h"

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
void c_rc_control_init();
pv_msg_io_actuation RC_controller(pv_msg_datapr_attitude attitude,
				  pv_msg_datapr_attitude attitude_reference,
				  pv_msg_datapr_position position,
				  pv_msg_datapr_position position_reference,
				  pv_msg_datapr_sensor_time sensor_time,
				  int throttle_control);

#ifdef __cplusplus
}
#endif

#endif //C_RC_CONTROL_H
