/**
  ******************************************************************************
  * @file    modules/common/c_common_uart.c
  * @author  Martin Vincent Bloedorn
  * @version V1.0.0
  * @date    30-November-2013
  * @brief   Implmentacão das funções de UART.
  ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "c_common_uart.h"

/** @addtogroup Common_Components
  * @{
  */

/** @addtogroup Common_Components_UART
  *
  * \brief Implementa as funções básicas de envio e recebimento de UART/USART.
  *
  * Para cada uma, é instalado o tratador de interrupções respectivo, e um buffer circular que armazena
  * o que vai sendo recebido. Enquanto o buffer não tiver sido completamente lido, uma flag é mantida
  * setada.
  *
  * \bug A flag de recebimento (ex.: usart2_available_flag) é setada quando o indexador do buffer circular
  * (usart2_rb_in) está a frente do indexador de leitura do buffer (usart2_rb_out). Caso o buffer tenha
  * dado "uma volta completa", a flag não é mantida setada, mesmo havendo 64 caracteres não tratados.
  *
  * \todo Implementar UART3 - disponível na placa STM32F407
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define RECV_BUFFER_SIZE	64

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
unsigned char usart2_recv_buffer[RECV_BUFFER_SIZE]; //! Ring-Buffer de recebimento de USART2.
unsigned char usart6_recv_buffer[RECV_BUFFER_SIZE]; //! Ring-Buffer de recebimento de USART6.

int usart2_rb_in  = 0; //! Index do Ring-Buffer para recebimento na USART2.
int usart2_rb_out = 0; //! Index para leitura do Ring-Buffer da USART2.

int usart6_rb_in  = 0; //! Index do Ring-Buffer para recebimento na USART2.
int usart6_rb_out = 0; //! Index para leitura do Ring-Buffer da USART2.

bool usart2_available_flag = 0;	//! Flag de recebimento de USART2.
bool usart6_available_flag = 0;	//! Flag de recebimento de USART6.

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions definitions --------------------------------------------*/

/** \brief Inicializa a USART6 com o Baurate desejado em modo 8-N-1.
  *	Instala USART2 nos pinos PA6 e PA7 (TX e RX, respectivamente) - pinos 1 e 2
  *	do conector UEXT (10 vias).
  * Tratador de interrupções para recebimento já é instalado automaticamente.
  *
  * @param  baudrate a ser inicializado.
  * @retval None
  */
void c_common_usart6_init(int baudrate) {
	GPIO_InitTypeDef GPIO_InitStruct; // this is for the GPIO pins used as TX and RX
	USART_InitTypeDef USART_InitStruct; // this is for the USART6 initilization
	NVIC_InitTypeDef NVIC_InitStructure; // this is used to configure the NVIC (nested vector interrupt controller)

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);

	/* enable the peripheral clock for the pins used by
	* USART6, PB6 for TX and PB7 for RX
	*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; // Pins 6 (TX) and 7 (RX) are used
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;       // the pins are configured as alternate function so the USART peripheral has access to them
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;    // this defines the IO speed and has nothing to do with the baudrate!
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;      // this defines the output type as push pull mode (as opposed to open drain)
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;      // this activates the pullup resistors on the IO pins
	GPIO_Init(GPIOC, &GPIO_InitStruct);          // now all the values are passed to the GPIO_Init() function which sets the GPIO registers

	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6); //
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_USART6);

	USART_InitStruct.USART_BaudRate = baudrate;        // the baudrate is set to the value we passed into this init function
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;// we want the data frame size to be 8 bits (standard)
	USART_InitStruct.USART_StopBits = USART_StopBits_1;    // we want 1 stop bit (standard)
	USART_InitStruct.USART_Parity = USART_Parity_No;    // we don't want a parity bit (standard)
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // we don't want flow control (standard)
	USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; // we want to enable the transmitter and the receiver
	USART_Init(USART6, &USART_InitStruct);          // again all the properties are passed to the USART_Init function which takes care of all the bit setting

	USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);       // enable the USART6 receive interrupt

	NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;     // we want to configure the USART6 interrupts
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;// this sets the priority group of the USART6 interrupts
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;     // this sets the subpriority inside the group
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;       // the USART6 interrupts are globally enabled
	NVIC_Init(&NVIC_InitStructure);               // the properties are passed to the NVIC_Init function which takes care of the low level stuff

	// finally this enables the complete USART6 peripheral
	USART_Cmd(USART6, ENABLE);
}

/** \brief Inicializa a USART2 com o Baurate desejado em modo 8-N-1.
 * 	Instala USART2 nos pinos PA2 e PA3 (TX e RX, respectivamente) - pinos D1 e D0
 * 	do layout do Arduino.
  * Tratador de interrupções para recebimento já é instalado automaticamente.
  *
  * @param  baudrate a ser inicializado.
  * @retval None
  */
void c_common_usart2_init(int baudrate) {
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	/* enable peripheral clock for USART2 */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	/* GPIOA clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	/* GPIOA Configuration:  USART2 TX on PA2, RX on PA3 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Connect USART2 pins to AF2 */
	// TX = PA2
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);

	USART_InitStructure.USART_BaudRate = baudrate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_Init(USART2, &USART_InitStructure);

	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); // enable the USART1 receive interrupt

	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;		 // we want to configure the USART1 interrupts
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;// this sets the priority group of the USART1 interrupts
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		 // this sets the subpriority inside the group
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			 // the USART1 interrupts are globally enabled
	NVIC_Init(&NVIC_InitStructure);							 // the properties are passed to the NVIC_Init function which takes care of the low level stuff

	USART_Cmd(USART2, ENABLE); // enable USART2
}

/** \brief Enviar uma string.
  *
  * @param  USARTx USART usada.
  * @retval s String a ser enviada.
  */
void c_common_usart_puts(USART_TypeDef* USARTx, volatile char *s){

	while(*s){
		// wait until data register is empty
		while( !(USARTx->SR & 0x00000040) );
		USART_SendData(USARTx, *s);
		*s++;
	}
}

/** \brief Envia apenas um \em char pela USART desejada.
  *
  * @param  USARTx USART usada.
  * @retval c Caracter a ser enviado.
  */
void c_common_usart_putchar(USART_TypeDef* USARTx, volatile char c){
	// wait until data register is empty
	while( !(USARTx->SR & 0x00000040) );
	USART_SendData(USARTx, c);
}

/** \brief Retorna se existe algum caracter não lido no Ring-Buffer da USART escolhida.
 *
 * 	@param USARTx USART a verificar.
 * 	@return 1 caso haja um caracter não lido, 0 caso contrário.
 */
bool c_common_usart_available(USART_TypeDef* USARTx) {
	if(USARTx == USART2)
		return usart2_available_flag;
	else if(USARTx == USART6)
		return usart6_available_flag;
	else
		return 0;
}

/** \brief Retorna o caracter recebido e não-lido mais recente do Ring-Buffer.
 *
 * 	@param USARTx USART a verificar.
 * 	@return Caracter recebido e não-lido mais recente.
 */
unsigned char c_common_usart_read(USART_TypeDef* USARTx) {
	if(USARTx == USART2) {
		uint8_t ret = usart2_recv_buffer[usart2_rb_out];
		if(usart2_rb_out < RECV_BUFFER_SIZE-1) usart2_rb_out++;
		else	usart2_rb_out = 0;
		if(usart2_rb_in == usart2_rb_out) usart2_available_flag = false;
		return ret;
	}
	else if(USARTx == USART6) {
		uint8_t ret = usart6_recv_buffer[usart6_rb_out];
		if(usart6_rb_out < RECV_BUFFER_SIZE-1) usart6_rb_out++;
		else	usart6_rb_out = 0;
		if(usart6_rb_in == usart6_rb_out) usart6_available_flag = false;
		return ret;
	}
	else return 0;
}

/* IRQ handlers ------------------------------------------------------------- */

/** \brief Tratador de interrupção para o recebimento de um byte em USART2.
  * Armazena os bytes lidos no buffer usart2_recv_buffer e seta o flag
  * usart2_available_flag .
  *
  * @param  None
  * @retval None
  */
void USART2_IRQHandler(void){
	// check if the USART1 receive interrupt flag was set
	if( USART_GetITStatus(USART2, USART_IT_RXNE) ){
		usart2_available_flag = 1;
		usart2_recv_buffer[usart2_rb_in] = USART_ReceiveData(USART2);
		if(usart2_rb_in < RECV_BUFFER_SIZE-1) usart2_rb_in++;
		else	usart2_rb_in = 0;

		USART_ClearFlag(USART2, USART_IT_RXNE);
		USART_ClearITPendingBit(USART2, USART_IT_RXNE);
	}
}

/** \brief Tratador de interrupção para o recebimento de um byte em USART6.
  * Armazena os bytes lidos no buffer usart2_recv_buffer e seta o flag
  * usart2_available_flag .
  *
  * @param  None
  * @retval None
  */
void USART6_IRQHandler(void){
	// check if the USART1 receive interrupt flag was set
	if( USART_GetITStatus(USART6, USART_IT_RXNE) ){
		usart6_available_flag = 1;
		usart6_recv_buffer[usart6_rb_in] = USART_ReceiveData(USART6);
		if(usart6_rb_in < RECV_BUFFER_SIZE-1) usart6_rb_in++;
		else	usart6_rb_in = 0;

		USART_ClearFlag(USART6, USART_IT_RXNE);
		USART_ClearITPendingBit(USART6, USART_IT_RXNE);
	}
}

/**
  * @}
  */

/**
  * @}
  */

