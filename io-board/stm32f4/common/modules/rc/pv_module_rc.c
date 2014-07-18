/**
  ******************************************************************************
  * @file    modules/rc/pv_module_rc.c
  * @author  Martin Vincent Bloedorn
  * @version V1.0.0
  * @date    30-November-2013
  * @brief   ...
  ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "pv_module_rc.h"

/** @addtogroup ProVANT_Modules
  * @{
  */

/** @addtogroup Module_RC
  * \brief Módulo com as principais funcionalidades para operação em modo rádio controlado.
  *
  * Definição do módulo de controle e comunicação via rádio manual.
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define MODULE_PERIOD	   10//ms

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
pv_msg_io_actuation actuation;
pv_type_receiverChannels receiver;
portTickType lastWakeTime;
char str[64];
GPIOPin LED_builtin_rc;

/* Inboxes buffers */
pv_msg_datapr_attitude iAttitude;
pv_msg_datapr_sensor_time iSensorTime;

/* Outboxes buffers*/
pv_msg_io_actuation    oActuation;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions definitions --------------------------------------------*/

/** \brief Inicializacao do módulo de RC.
  *
  * Instancia as Queues de comunicação inter-thread, inicializa a pinagem necessária para
  * o controle remoto e aloca o que for necessário para as equações de controle.
  * @param  None
  * @retval None
  */
void module_rc_init() {
	/* Inicialização do hardware do módulo */
	c_rc_receiver_init();
  LED_builtin_rc = c_common_gpio_init(GPIOB, GPIO_Pin_15, GPIO_Mode_OUT);

	/* Inicialização das filas do módulo. Apenas inboxes (i*!) são criadas! */
	pv_interface_rc.iAttitude  = xQueueCreate(1, sizeof(pv_msg_datapr_attitude));
	pv_interface_rc.iSensorTime = xQueueCreate(1, sizeof(pv_msg_datapr_sensor_time));

	/* Inicializando outboxes em 0 */
	pv_interface_rc.oActuation = 0;

	/* Verificação de criação correta das filas */
	if(pv_interface_rc.iAttitude == 0 || pv_interface_rc.iSensorTime == 0) {
		vTraceConsoleMessage("Could not create queue in pv_interface_io!");
		while(1);
	}
}

/** \brief Função principal do módulo de RC.
  * @param  None
  * @retval None
  *
  * Interpreta o recebimento de PPM, calcula sinais de controle e os envia
  * via interface.
  * Devido as diferenças do modelo matematica com a construção mecanica o sinal do angulo do servo direito deve 
  * ser adaptado.
  *
  */
void module_rc_run() {
  char rc_channel[4][16];
  int throttle_control;

	while(1) {
		lastWakeTime = xTaskGetTickCount();
    c_common_gpio_toggle(LED_builtin_rc);

    xQueueReceive(pv_interface_rc.iAttitude, &iAttitude, 0);
    xQueueReceive(pv_interface_rc.iSensorTime, &iSensorTime, 0);
    
    /// Controle
    #if 1
      pv_msg_io_actuation    actuation = {0,0.0f,0.0f,0.0f,0.0f};
      pv_msg_datapr_attitude attitude  = {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};
      pv_msg_datapr_attitude attitude_reference = {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};
      pv_msg_datapr_position position  = {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};
      pv_msg_datapr_position position_reference = {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};

      // Limita a velocidade dos ESCs a partir do RC
      //throttle_control = c_rc_receiver_getChannel(C_RC_CHANNEL_PITCH);
//      if (throttle_control < 0)
    	  throttle_control = -throttle_control;

      oActuation = RC_controller(iAttitude,attitude_reference,position,position_reference,iSensorTime,throttle_control);

      // Ajusta o eixo de referencia do servo (montado ao contrario)
      oActuation.servoLeft = -oActuation.servoLeft;

      // Modulando sinal do ESC para testes
//      oActuation.escLeftSpeed  = (oActuation.escLeftSpeed*throttle_control)/100;
//      oActuation.escRightSpeed = (oActuation.escRightSpeed*throttle_control)/100;
      //oActuation.escLeftSpeed  = throttle_control;

    #endif
    
    /// Receiver debug
    #if 0
      c_common_utils_floatToString(c_rc_receiver_getChannel(C_RC_CHANNEL_THROTTLE), rc_channel[0],  2);
      c_common_utils_floatToString(c_rc_receiver_getChannel(C_RC_CHANNEL_ROLL), rc_channel[1],  2);
      c_common_utils_floatToString(c_rc_receiver_getChannel(C_RC_CHANNEL_YAW), rc_channel[2],  2);
      c_common_utils_floatToString(c_rc_receiver_getChannel(C_RC_CHANNEL_PITCH), rc_channel[3],  2);
      sprintf(str, "receiver -> \t %s \t %s \t %s \t %s\n\r", rc_channel[0], rc_channel[1], rc_channel[2], rc_channel[3]);
      c_common_usart_puts(USART2, str);

    /// Receiver control
    #endif

    #if 0
      taskENTER_CRITICAL();
      oActuation.servoRight = c_rc_receiver_getChannel(C_RC_CHANNEL_PITCH);
      oActuation.servoLeft = 0;
      taskEXIT_CRITICAL();
    #endif




    if(pv_interface_rc.oActuation != 0)
      xQueueOverwrite(pv_interface_rc.oActuation, &oActuation);

    vTaskDelayUntil( &lastWakeTime, MODULE_PERIOD / portTICK_RATE_MS);
	}
}
/* IRQ handlers ------------------------------------------------------------- */

/**
  * @}
  */

/**
  * @}
  */
