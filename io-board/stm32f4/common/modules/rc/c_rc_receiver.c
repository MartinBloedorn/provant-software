/**
  ******************************************************************************
  * @file    modules/rc/c_rc_receiver.c
  * @author  Martin Vincent Bloedorn
  * @version V1.0.0
  * @date    30-November-2013
  * @brief   Implementação do receiver do controle de rádio manual.
  * 		 Implementa as funções de recebimento, detecção e interpretação do
  * 		 receiver configurado em modo PPM.
  ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "c_rc_receiver.h"

/** @addtogroup Module_RC
  * @{
  */

/** @addtogroup Module_RC_Component_Receiver
  * \brief Funções para inicialização e recebimento de sinais PPM do receiver do controle remoto.
  *
  * @{
  */


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define 	EXTI_PORT		EXTI_PortSourceGPIOG
#define 	EXTI_SOURCE		EXTI_PinSource13		// PD5 na placa STM32F4-H407
#define 	EXTI_LINE		EXTI_Line13				// Tem que ser a mesma do pino
#define 	PULSE_INTERVAL 	400						// us
#define 	SYNC_WIDTH		2500					// us
#define	 	NUM_OF_CHANNELS 6

#define 	RECV_MINIMUM	450   /** Largura mínima de um pulso válido */
#define 	RECV_MAXIMUM	1800  /** Largura máxima de um pulso válido */
#define 	RECV_MIN_THROTTLE 700 /** Minima largura de pulso para um throttle válido */

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
long int 	channels[12];
long int 	last_channels[12];
int 		channel_index = 0;
long int   channel_center[4]; /** Largura média do pulso em repouso, apenas para canais R-P-Y. */

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/** \brief Amostra os canais de Roll, Pitch e Yaw (com o controle supostamente em
 * repouso) e mede a largura média de pulso.
 *
 * AVISO: Nesta versão, os centros são hard-coded!
 */
void c_rc_calibrateCenters() {
	int iterations = 10;

	for(int i=0; i<4; i++)
		channel_center[i] = 0;
	/*
	for(int i=0; i<iterations; i++) {
		channel_center[C_RC_CHANNEL_PITCH] += c_rc_receiver_getChannel(C_RC_CHANNEL_PITCH);
		channel_center[C_RC_CHANNEL_ROLL]  += c_rc_receiver_getChannel(C_RC_CHANNEL_ROLL);
		channel_center[C_RC_CHANNEL_YAW]   += c_rc_receiver_getChannel(C_RC_CHANNEL_YAW);
		channel_center[C_RC_CHANNEL_THROTTLE]+= c_rc_receiver_getChannel(C_RC_CHANNEL_THROTTLE);
		for(int i=0; i<0xFFFFFF; i++) { __asm("NOP"); } //delay para nova amostra
	}
	*/
	for(int i=0; i<4; i++)
		channel_center[i] = channel_center[i]/iterations;

	/* Forçando canal de Throttle para o minimo predefinido */
	channel_center[C_RC_CHANNEL_THROTTLE] = RECV_MIN_THROTTLE;
	channel_center[C_RC_CHANNEL_PITCH] += 1100;
	channel_center[C_RC_CHANNEL_ROLL]  += 1100;
	channel_center[C_RC_CHANNEL_YAW]   += 1100;
}
/* Exported functions definitions --------------------------------------------*/

/** \brief Inicializa a leitura de PPM num pino predefinido via DEFINE.
  * Inicializa o TIM2 e a interrupção EXTI. A partir deste momento, a contagem
  * de pulsos já está ocorrendo no background.
  *
  * @param  None
  * @retval None
  */
void c_rc_receiver_init() {
	 /* zerando os contadores */
	 for(int i=0; i<12; i++)
		 channels[i] = 0;

	 /* Enable SYSCFG clock */
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	 /* Connect EXTI Line to appropriate GPIO Pin */
	 SYSCFG_EXTILineConfig(EXTI_PORT, EXTI_SOURCE);

	 NVIC_InitTypeDef   NVIC_InitStructure;
	 EXTI_InitTypeDef   EXTI_InitStructure;

	 /* Configure EXTI Line */
	 EXTI_InitStructure.EXTI_Line = EXTI_LINE;
	 EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	 EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising; // Apenas borda de subida
	 EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	 EXTI_Init(&EXTI_InitStructure);

	 /* Enable and set EXTI Line Interrupt */
	 NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
	 NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
	 NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
	 NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	 NVIC_Init(&NVIC_InitStructure);

	 /* TIM2 clock enable */
	 TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	 RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	 /* Time base configuration */
	 TIM_TimeBaseStructure.TIM_Prescaler = (SystemCoreClock / 2000000) - 1; // a cada us
	 TIM_TimeBaseStructure.TIM_Period = 0xFFFFFFFF;// 1 us
	 TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	 TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	 TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	 /* TIM IT enable */
	 TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	 /* TIM2 enable counter */
	 TIM_Cmd(TIM2, ENABLE);

	 // Calibra os centros
	 c_rc_calibrateCenters();
}

/** \brief Retorna a largura do pulso em \em us do canal selecionado.
  *
  * Retorna -1 caso o canal desejado não seja válido (por ex., não exista), ou se a largura
  * do pulso estiver fora do range [ RECV_MINIMUM , RECV_MAXIMUM ] (para evitar que eventuais
  * erros de leitura sejam passados adiante).
  *
  * @param  int Canal a ser lido (começando em 0)
  * @retval int Duração em \em us do pulso no canal selecionado.
  */
float  c_rc_receiver_getChannel(int channel_n) {
	if(channel_n < NUM_OF_CHANNELS && channels[channel_n] > RECV_MINIMUM && channels[channel_n] < RECV_MAXIMUM) {

  /** Maximos e minimos de cada canal
    * temosque encontrar os zeros do mag

        C_RC_CHANNEL_THROTTLE   |    C_RC_CHANNEL_ROLL    |    C_RC_CHANNEL_YAW   |   C_RC_CHANNEL_PITCH
    ----------------------------|-------------------------|-----------------------|----------------------
            706/1581            |        713/1542         |        692/1523       |       687/1513
    *****************************************************************************************************/
        if(channel_n==C_RC_CHANNEL_THROTTLE)
            last_channels[channel_n] = channels[channel_n] = c_common_utils_map(channels[channel_n], 706, 1581, -100, +100);
       	if(channel_n==C_RC_CHANNEL_ROLL)
            last_channels[channel_n] = channels[channel_n] = c_common_utils_map(channels[channel_n], 713, 1542, -100, +100);
        if(channel_n==C_RC_CHANNEL_YAW)
            last_channels[channel_n] = channels[channel_n] = c_common_utils_map(channels[channel_n], 692, 1523, -100, +100);
        if(channel_n==C_RC_CHANNEL_PITCH)
            last_channels[channel_n] = channels[channel_n] = c_common_utils_map(channels[channel_n], 687, 1513, -100, +100);
		return channels[channel_n];
	}
	else
		return last_channels[channel_n];
}

/** \brief Retorna a largura do pulso em \em us do canal selecionado, em relação ao centro do canal.
  *
  * Roll, Pitch e Yaw são subtraídos dos centros dos canais. Throttle é subtraído do mínimo throttle válido.
  *
  * \todo Não tá uma beleza. O Throttle e o Pitch precisam ser invertidos (estão decrescendo pra cima)
  * \bug  Os valores estão flutuando as vezes, pulam de algo OK pra -1000 ou algo assim.
  *
  * @param  int Canal a ser lido (começando em 0)
  * @retval int Duração em \em us do pulso no canal selecionado em relação ao centro.
  */
int  c_rc_receiver_getCenteredChannel(int channel_n) {
	int ch2ret, attempts;

	switch(channel_n) {
	case C_RC_CHANNEL_ROLL:
	case C_RC_CHANNEL_PITCH:
	case C_RC_CHANNEL_YAW:
	case C_RC_CHANNEL_THROTTLE:
		attempts = 10;
		do {
			ch2ret = c_rc_receiver_getChannel(channel_n);
			attempts--;
			if(ch2ret == -1)
				c_common_utils_delayms(1);
		}
		while((ch2ret == -1) && attempts);
		return ch2ret - channel_center[channel_n];
		break;

	default:
		/* Chamado um canal indefinido */
		return -1;
		break;
	}
}



/* IRQ handlers ------------------------------------------------------------- */

/** \brief Detecta pulso de sincronização do PPM e lê os canais do receiver.
 *
 */
void  EXTI15_10_IRQHandler()
{
	EXTI_ClearITPendingBit(EXTI_LINE); // clear interrupt
	int pulse_width = TIM_GetCounter(TIM2) - PULSE_INTERVAL;

	if(pulse_width > SYNC_WIDTH) //sync pulse
		channel_index = 0;
	else {
		if(channel_index < NUM_OF_CHANNELS) {
			channels[channel_index] = pulse_width;
			channel_index++;
		}
	}
	TIM_SetCounter(TIM2, 0);
}

/**
  * @}
  */

/**
  * @}
  */

