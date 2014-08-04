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
#include "c_common_gpio.h"

#define ARM_MATH_CM4
#include "arm_math.h"
#include <math.h>

/* FreeRTOS kernel includes */
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define PV_IMU_ROLL        0
#define PV_IMU_PITCH       1
#define PV_IMU_YAW         2
#define PV_IMU_DROLL       3
#define PV_IMU_DPITCH      4
#define PV_IMU_DYAW        5

#define PV_IMU_X           0
#define PV_IMU_Y           1
#define PV_IMU_Z           2

/* Dados do Giroscopio encontrados experimentalmente pelo metodo da variancia de allan */
#define POL_GYRO_X		   	-0.051135
#define POL_GYRO_Y		   	-0.014749
#define POL_GYRO_Z			0.0095847
#define VAR_POL_GYRO_X		0.00000000056929960
#define VAR_POL_GYRO_Y		0.00000000099609672
#define VAR_POL_GYRO_Z		0.0000000000083024651
#define VAR_GYRO_X			0.000000020733120
#define VAR_GYRO_Y			0.000000019290432
#define VAR_GYRO_Z			0.000000115015940

 /* Dados do Acelerometro encontrados experimentalmente pelo metodo da variancia de allan */
#define VAR_ACCL_X			0.00000355775044
#define VAR_ACCL_Y			0.00000414651769
#define VAR_ACCL_Z			0.00001150159396

 /* Dados do Magnetometro encontrados experimentalmente pelo metodo da variancia de allan */
#define VAR_MAGN_X			0.2951857561 // ou  0.0763637956
#define VAR_MAGN_Y			0.1793861316 // ou  0.0522899689
#define VAR_MAGN_Z			4.40454169   // ou  1.0497846681

#define G				   	9.81 //Ver se nao esta definindo denovo

/* Exported macro ------------------------------------------------------------*/
#define C_IO_IMU_USE_ITG_ADXL_HMC
//#define C_IO_IMU_USE_MPU6050_HMC5883

/* Exported functions ------------------------------------------------------- */
void c_io_imu_init(I2C_TypeDef* I2Cx);
void c_io_imu_getRaw(float  * accRaw, float * gyrRaw, float * magRaw);
void c_io_imu_getComplimentaryRPY(float * acce_raw, float * gyro_raw, float * magn_raw, float sample_time, float * rpy);
void c_io_imu_getKalmanFilterRPY(float * rpy, float * acce_raw, float * gyro_raw, float * magn_raw);
void c_io_imu_initKalmanFilter();
void c_io_imu_calibrate();
void c_io_imu_Quaternion2Euler(float * q, float * rpy);
void c_io_imu_EulerMatrix(float * rpy, float * velAngular);

#ifdef __cplusplus
}
#endif

#endif //C_IO_IMU_H
