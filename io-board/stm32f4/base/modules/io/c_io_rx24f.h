/**
  ******************************************************************************
  * @file    modules/io/c_io_rx24f.h
  * @author  Martin Vincent Bloedorn
  * @version V1.0.0
  * @date    02-Dezember-2013
  * @brief   Implementação do servo RX-24F.
  *****************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef C_IO_RX24F_H
#define C_IO_RX24F_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_conf.h"

#include "c_common_gpio.h"
#include "c_common_uart.h"
#include "c_common_utils.h"

#include <math.h>

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define STARTX_ERROR 			-1
#define CHECKSUM_ERROR 			-2
#define INPUT_VOLTAGE_ERROR		-3
#define OVERHEATING_ERROR		-4
#define RANGE_ERROR				-5
#define CHECKSUM_ERROR_SERVO	-6
#define OVERLOAD_ERROR			-7
#define INSTRUCTION_ERROR		-8
#define UNDEFINED_ERROR			-9
#define ANGLE_LIMIT_ERROR	    -10
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
void c_io_rx24f_init(int baudrate);
int  c_io_rx24f_move(unsigned char ID, int position);
int  c_io_rx24f_write(unsigned char ID, unsigned char address,unsigned char value);
int  c_io_rx24f_readPosition(unsigned char ID);
int  c_io_rx24f_setLed(unsigned char ID, unsigned char value);
int  c_io_rx24f_setSpeed(unsigned char ID, float speed_in_rpm);

#ifdef __cplusplus
}
#endif

#endif //C_IO_RX24F_H
