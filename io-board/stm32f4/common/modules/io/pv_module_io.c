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
#define MODULE_PERIOD	   100//ms

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
portTickType lastWakeTime;
int  accRaw[3], gyroRaw[3], magRaw[3];
char str[64];

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
	/* Inicialização do hardware do módulo */
	c_common_i2c_init();

	c_common_usart2_init(115200);

	/* Inicializar os servos */
	c_io_rx24f_init(1000000);
	c_io_rx24f_setSpeed(1, 20);
	c_io_rx24f_setSpeed(2, 20);
	c_common_utils_delayms(1);
	c_io_rx24f_move(1, 150);
	c_io_rx24f_move(2, 140);
	c_common_utils_delayms(1);
	c_io_rx24f_setSpeed(1, 70);
	c_io_rx24f_setSpeed(2, 70);

	c_common_utils_delayms(100);
	c_io_imu_init();

	c_io_blctrl_init();

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
void module_io_run() {
	float accRaw[3], gyrRaw[3], magRaw[3];
	char  ax[16], ay[16], az[16], r[16], p[16];
	float rpy[] = {0,0,0};
	char str[64];

	while(1) {
		lastWakeTime = xTaskGetTickCount();

		xQueueReceive(pv_interface_io.iActuation, &iActuation, 0);

		c_io_blctrl_setSpeed(0, 700);//1700-iActuation.escLeftSpeed);
		c_io_blctrl_setSpeed(1, 700);//1700-iActuation.escLeftSpeed);

		//c_io_imu_getRaw(accRaw, gyrRaw, magRaw);

		//c_common_utils_floatToString(accRaw[0], ax, 4);
		//c_common_utils_floatToString(accRaw[1], ay, 4);
		//c_common_utils_floatToString(accRaw[2], az, 4);

		c_io_imu_getComplimentaryRPY(rpy);

		c_common_utils_floatToString(RAD_TO_DEG*rpy[PV_IMU_ROLL ], r, 4);
		c_common_utils_floatToString(RAD_TO_DEG*rpy[PV_IMU_PITCH], p, 4);

		sprintf(str, "Time: %ld \t %s \t\t %s\n\r", c_common_utils_millis(), r, p);
		c_common_usart_puts(USART2, str);
		//if(iActuation.servoLeft > 60) iActuation.servoLeft = 60.0;
		//if(iActuation.servoLeft < 0)  iActuation.servoLeft =  0.0;
		//if(iActuation.servoRight > 60) iActuation.servoRight = 60.0;
		//if(iActuation.servoRight < 0)  iActuation.servoRight =  0.0;

		//c_io_rx24f_move(2, iActuation.servoLeft);
		//c_io_rx24f_move(1, iActuation.servoRight-10);

		vTaskDelayUntil( &lastWakeTime, (MODULE_PERIOD / portTICK_RATE_MS));
	}
}
/* IRQ handlers ------------------------------------------------------------- */

/**
  * @}
  */

/**
  * @}
  */
