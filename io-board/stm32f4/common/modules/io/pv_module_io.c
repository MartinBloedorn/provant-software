/**
  ******************************************************************************
  * @file    modules/io/pv_module_io.c
  * @author  Martin Vincent Bloedorn
  * @version V1.0.0
  * @date    02-Dezember-2013
  * @brief   Implementação do módulo de gerenciamento de sensores e atuadores.
  ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "pv_module_io.h"

/** @addtogroup ProVANT_Modules
  * @{
  */

/** @addtogroup Module_IO
  * \brief Componentes para atuação e sensoriamento do VANT.
  *
  * Reunião de todos os componentes relacionados às operações de I/O do VANT.
  * Leituras de todos os sensores, comandos para atuadores. O processamento destes
  * dados brutos NÃO é feito neste módulo.
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define MODULE_PERIOD	   10//ms

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
portTickType lastWakeTime;
//int  accRaw[3], gyroRaw[3], magRaw[3];
char str[256];

/* Inboxes buffers */
pv_msg_io_actuation    iActuation;

/* Outboxes buffers*/
pv_msg_datapr_attitude oAttitude;
pv_msg_datapr_position oPosition;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions definitions --------------------------------------------*/

/** \brief Inicializacao componentes de IO.
  *
  * Incializa o hardware para comunicar com os sensores e atuadores. Rotinas de teste
  * ainda precisam ser executadas.
  * @param  None
  * @retval None
  */
void module_io_init() {
//	float accRaw[3], gyrRaw[3], magRaw[3]; // TODO Tirar daqui junto com o init

	/* Inicialização do hardware do módulo */
	c_common_i2c_init(I2C1); //imu 
	//c_common_i2c_init(I2C2); //esc

	c_common_usart2_init(460800);

	/* Inicializar do sonar */
	c_io_sonar_init();

	/* Inicializar os servos */
	c_io_rx24f_init(1000000);
	c_common_utils_delayms(2);
	c_io_rx24f_setSpeed(1, 20);
	c_io_rx24f_setSpeed(2, 20);
	c_common_utils_delayms(2);
	/* CCW Compliance Margin e CCW Compliance margin */
	c_io_rx24f_write(1, 0x1A,0x03);
	c_io_rx24f_write(1, 0x1B,0x03);
	c_io_rx24f_write(2, 0x1A,0x03);
	c_io_rx24f_write(2, 0x1B,0x03);
	c_common_utils_delayms(2);
	c_io_rx24f_move(1, 130);
	c_io_rx24f_move(2, 150);
	c_common_utils_delayms(100);

	c_io_imu_init(I2C1);   
	//c_io_blctrl_init(I2C1);

	// TODO Tirar daqui junto com o init
//	taskENTER_CRITICAL();
//	c_io_imu_getRaw(accRaw, gyrRaw, magRaw);
//	taskEXIT_CRITICAL();
//	c_io_imu_initKalmanFilter(accRaw, gyrRaw, magRaw); // Inicia o filtro de Kalman
	// END TODO

	//inicializacao dos PPM
	c_io_blctrl_init_ppm();

	/* Inicialização das filas do módulo. Apenas inboxes (i*!) são criadas! */
	pv_interface_io.iActuation = xQueueCreate(1, sizeof(pv_msg_io_actuation));

	/* Inicializando outboxes em 0 */
	pv_interface_io.oAttitude = 0;
	pv_interface_io.oPosition = 0;

	/* Verificação de criação correta das filas */
	if(pv_interface_io.iActuation == 0) {
		vTraceConsoleMessage("Could not create queue in pv_interface_io!");
		while(1);
	}
}

/** \brief Função principal do módulo de IO.
  * @param  None
  * @retval None
  *
  * Loop que amostra sensores e escreve nos atuadores como necessário.
  *
  */
/// SERVOS
		float servoRightFiltrado=0;
		float servoLeftFiltrado=0;
		float alpha_servo=0.07;
		float alpha_Esc=0.95;
		float velo_rightFiltrado=0;
		float velo_leftFiltrado=0;
void module_io_run() 
{
	float accRaw[3], gyrRaw[3], magRaw[3];
	char  ax[16], ay[16], az[16], r[16], p[16], y[16], dr[16], dp[16], dy[16];
	float rpy[] = {0,0,0,0,0,0};
	int counte=0;
	float cond;

	while(1)
	{
		lastWakeTime = xTaskGetTickCount();

		xQueueReceive(pv_interface_io.iActuation, &iActuation, 0);

    	/*
    	TIM11->CCR1 = 512; 
    	TIM11->CCR2 = 1024 - velo_left*4; 
    	*/
		
		/// IMU DATA
		#if 1
		 	taskENTER_CRITICAL();
		 	c_io_imu_getRaw(accRaw, gyrRaw, magRaw);
//			c_io_imu_getComplimentaryRPY(rpy);
		 	//c_io_imu_serialPrintData();
			taskEXIT_CRITICAL();
			//c_io_imu_getKalmanFilterRPY(rpy, accRaw, gyrRaw, magRaw);
			c_io_imu_getComplimentaryRPY(accRaw, gyrRaw, magRaw, rpy);
		#endif

		/// SERVOS
		#if 0
			// filtro de referencia
			servoRightFiltrado = servoRightFiltrado + alpha_servo*(iActuation.servoRight-servoRightFiltrado);
			servoLeftFiltrado = servoLeftFiltrado + alpha_servo*(iActuation.servoLeft-servoLeftFiltrado);

			if(servoRightFiltrado*RAD_TO_DEG<60 || servoRightFiltrado*RAD_TO_DEG>-60)
				c_io_rx24f_move(2, 150+servoRightFiltrado*RAD_TO_DEG);
			if(servoLeftFiltrado*RAD_TO_DEG<60 || servoLeftFiltrado*RAD_TO_DEG>-60)
				c_io_rx24f_move(1, 130+servoLeftFiltrado*RAD_TO_DEG);
		#endif
//		#if 0
//			if(iActuation.servoRight*RAD_TO_DEG<60 || iActuation.servoRight*RAD_TO_DEG>-60)
//				c_io_rx24f_move(2, 150+iActuation.servoRight*RAD_TO_DEG);
//			if(iActuation.servoLeft*RAD_TO_DEG<60 || iActuation.servoLeft*RAD_TO_DEG>-60)
//				c_io_rx24f_move(1, 130+iActuation.servoLeft*RAD_TO_DEG);
//		#endif



unsigned char velo_right, velo_left;
		/// ESCS
		#if 0
			// TODO Esse 8.2 É só para testes! TIRAR
			if ((iActuation.escRightSpeed-7.5f) < 0)
				iActuation.escRightSpeed = 0.0f;
			else
				iActuation.escRightSpeed=iActuation.escRightSpeed-7.5f;

			if ((iActuation.escLeftSpeed-7.5f) < 0)
				iActuation.escLeftSpeed = 0.0f;
			else
				iActuation.escLeftSpeed=iActuation.escLeftSpeed-7.5f;


			velo_rightFiltrado = velo_rightFiltrado + alpha_Esc*(iActuation.escRightSpeed-velo_rightFiltrado);
			velo_leftFiltrado = velo_leftFiltrado + alpha_Esc*(iActuation.escLeftSpeed-velo_leftFiltrado);
//			velo_rightFiltrado = 10;
//			velo_leftFiltrado = 10;
			/* força para char
			 *  Foram retirados 2 retas, uma para valores baixos (<6) e uma para valores >6. Na verdade pode-se aproximar a curva inteira
			 *  por um polinomio de maior ordem
			 */

			if (velo_rightFiltrado < 6)
				velo_right = (int)(20.332*velo_rightFiltrado +1.7466);
			else
				velo_right = (int)(12.256*velo_rightFiltrado - 39.441);

			if (velo_leftFiltrado < 6)
				velo_left = (int)(20.332*velo_leftFiltrado +1.7466);
			else
				velo_left = (int)(12.256*velo_leftFiltrado - 39.441);


			taskENTER_CRITICAL();
			//if(counte>2500)
			//{
				c_io_blctrl_setSpeed(0, velo_right );
				c_common_utils_delayus(10);
				c_io_blctrl_setSpeed(1, velo_left );
			//}
			//else   //inicializacao do esc
			//{
			//	c_io_blctrl_setSpeed(0, (char)(counte/10));
			//	c_common_utils_delayus(10);
			//	c_io_blctrl_setSpeed(1, (char)(counte/10));
	    	//}
			taskEXIT_CRITICAL();
		#endif
		
		/// SONAR
		#if 0
			c_common_utils_floatToString(c_io_sonar_read(), r,  3);
			sprintf(str, "Distance: %s \n\r",r );
	    	c_common_usart_puts(USART2, str);
    	#endif


		/// DEBUG
		#if 1
			//c_common_utils_floatToString(rpy[PV_IMU_ROLL  ]*RAD_TO_DEG, r,  4);
			//c_common_utils_floatToString(rpy[PV_IMU_PITCH ]*RAD_TO_DEG, p,  4);
			//c_common_utils_floatToString(rpy[PV_IMU_YAW   ]*RAD_TO_DEG, y,  4);
			//c_common_utils_floatToString(rpy[PV_IMU_DROLL ]*RAD_TO_DEG, dr, 4);
			//c_common_utils_floatToString(rpy[PV_IMU_DPITCH]*RAD_TO_DEG, dp, 4);
			//c_common_utils_floatToString(rpy[PV_IMU_DYAW  ]*RAD_TO_DEG, dy, 4);
			sprintf(str, "imu -> \t %d \t %d \t %d \t %d \t %d \t %d \t %d \t %d \t %d \t %d \t %d \t %d \n\r" ,(int)(rpy[PV_IMU_ROLL  ]*RAD_TO_DEG),
					(int)(rpy[PV_IMU_PITCH  ]*RAD_TO_DEG), (int)(rpy[PV_IMU_YAW  ]*RAD_TO_DEG), (int)(rpy[PV_IMU_DROLL  ]*RAD_TO_DEG),
					(int)(rpy[PV_IMU_DPITCH  ]*RAD_TO_DEG), (int)(rpy[PV_IMU_DYAW  ]*RAD_TO_DEG),(int)(servoLeftFiltrado*RAD_TO_DEG),
					(int)(servoRightFiltrado*RAD_TO_DEG),(int)velo_left,(int)velo_right,(int)(velo_leftFiltrado*100),(int)(velo_rightFiltrado*100));
			c_common_usart_puts(USART2, str);
			counte++;

	    	//cond=(rpy[0]*rpy[0])+(rpy[1]*rpy[1])+(rpy[2]*rpy[2])+(rpy[3]*rpy[3]);
//	    	sprintf(str, "test -> \t %d \t %d \t %d \t %d \t %d \t %d \n\r" ,(int)(rpy[0]),(int)(rpy[1]),(int)(rpy[2]),(int)(rpy[3]*1000),(int)(rpy[4]),(int)(rpy[5]));
	    	//sprintf(str, "test -> \n\r");
//	    	c_common_usart_puts(USART2, str);

		#endif

		/// DADOS OUT
		oAttitude.roll     = rpy[PV_IMU_ROLL  ];
		oAttitude.pitch    = rpy[PV_IMU_PITCH ];
		oAttitude.yaw      = rpy[PV_IMU_YAW   ];
		oAttitude.dotRoll  = rpy[PV_IMU_DROLL ];
		oAttitude.dotPitch = rpy[PV_IMU_DPITCH];
		oAttitude.dotYaw   = rpy[PV_IMU_DYAW  ];
		if(pv_interface_io.oAttitude != 0)
      		xQueueOverwrite(pv_interface_io.oAttitude, &oAttitude);

		//vTaskDelayUntil( &lastWakeTime, (MODULE_PERIOD / portTICK_RATE_MS));
	}
}
/* IRQ handlers ------------------------------------------------------------- */

/**
  * @}
  */

/**
  * @}
  */

