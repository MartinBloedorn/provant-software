/**
  ******************************************************************************
  * @file    modules/io/c_io_imu.h
  * @author  Martin Vincent Bloedorn
  * @version V1.0.0
  * @date    11-February-2014
  * @brief   Funções para IMUs (incialmente baseadas no CIs ITG3205 e ADXL345).
  *****************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef C_IO_IMU_H
#define C_IO_IMU_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_conf.h"

#include "c_common_i2c.h"
#include "c_common_utils.h"

#define ARM_MATH_CM4
#include "arm_math.h"
#include <math.h>

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define PV_IMU_ROLL        0
#define PV_IMU_PITCH       1
#define PV_IMU_YAW         2

#define PV_IMU_X           0
#define PV_IMU_Y           1
#define PV_IMU_Z           2

/* Exported macro ------------------------------------------------------------*/
#define C_IO_IMU_USE_ITG_ADXL_HMC
//#define C_IO_IMU_USE_MPU6050_HMC5883

/* Exported functions ------------------------------------------------------- */
void c_io_imu_init(I2C_TypeDef* I2Cx);
void c_io_imu_getRaw(float  * accRaw, float * gyrRaw, float * magRaw);
void c_io_imu_getComplimentaryRPY(float * rpy);
void c_io_imu_calibrate();

#ifdef __cplusplus
}
#endif

#endif //C_IO_IMU_H
