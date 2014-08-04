/**
  ******************************************************************************
  * @file    modules/io/c_io_blctrl.h
  * @author  Patrick Jose Pereira
  * @version V1.0.0
  * @date    06-Dezembro-2014
  * @brief   Implementação do esc BL-Ctrl 2.0.
  *****************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef C_IO_BLCTRL_H
#define C_IO_BLCTRL_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_conf.h"
#include "c_common_utils.h"

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define BLCTRL_ADDR  0x29

/* Exported macro ------------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
void c_io_blctrl_init_ppm();
void c_io_blctrl_init_i2c();
int  c_io_blctrl_read(uint8_t ID, int local);
int  c_io_blctrl_setSpeed(uint8_t ID, unsigned char speed);
int  c_io_blctrl_readSpeed(uint8_t ID);
int  c_io_blctrl_readVoltage(uint8_t ID);
int  c_io_blctrl_updateBuffer(uint8_t ID);

#ifdef __cplusplus
}
#endif

#endif //C_IO_BLCTRL20_H
