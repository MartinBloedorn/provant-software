/**
  ******************************************************************************
  * @file    main/main.c
  * @author  Martin Vincent Bloedorn
  * @version V1.0.0
  * @date    30-November-2013
  * @brief   Startup do projeto.
  *
  *	TODO
  *
  * \todo	 1 - Terminar gpio_common.
  * \todo	 2 - Implementar comunicação com servos.
  * \todo	 3 - Implementar comunicação com ESCs.
  *****************************************************************************/

/* Includes ------------------------------------------------------------------*/

/* Std includes */
#include <stdio.h>
#include <math.h>
#include "inttypes.h"

/* Hardware includes. */
#include "stm32f4xx_conf.h"

/* FreeRTOS kernel includes */
#include "FreeRTOS.h"
#include "queue.h"
#include "timers.h"
#include "task.h"

/* FreeRTOS+Trace Includes */
#include "trcUser.h"

/* ProVANT Modules */
#include "pv_module_rc.h"
#include "pv_module_io.h"

/* Common Components, FOR TESTING */
#include "c_rc_receiver.h"
#include "c_common_uart.h"
#include "c_common_gpio.h"
#include "c_common_i2c.h"

/** @addtogroup ProVANT_Modules
  * \brief Ponto de entrada do software geral do VANT.
  *
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
GPIOPin LED;

/* Private define ------------------------------------------------------------*/
#define ITG3205_ADDR 0x68    // The address of ITG3205
#define ADXL345_ADDR 0x53    // The adress of ADXL345
#define ITG3205_X_ADDR 0x1D  // Start address for x-axis
#define ADXL345_X_ADDR 0x32  // Start address for x-axis

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
unsigned char ITG3205_ID = 0;
unsigned char ADXL345_ID = 0;
uint8_t sensorBuffer[8];
int accRaw[3], gyroRaw[3];

/* Private function prototypes -----------------------------------------------*/
void vApplicationTickHook() {};
void vApplicationIdleHook() {};
void vApplicationStackOverflowHook() {};
void vApplicationMallocFailedHook() {};

/* Tasks ----------------------------------------------------------------------*/

// `I'm alive` kind of task
void blink_led_task(void *pvParameters)
{
    while(1) {
    	//c_io_rx24f_readPosition(0x01);
    	//c_io_rx24f_setLed(0x01, 0x01);
    	c_io_rx24f_move(0x01, 0);
    	c_common_gpio_toggle(LED);
        vTaskDelay(500/portTICK_RATE_MS);

        //c_io_rx24f_readPosition(0x01);
        //c_io_rx24f_setLed(0x01, 0x00);
        c_io_rx24f_move(0x01, 10);
        c_common_gpio_toggle(LED);
        vTaskDelay(500/portTICK_RATE_MS);
    }
}

// Reads receiver inputs and drives the servo accordingly
void rc_servo_task(void *pvParameters)
{
	int angle = 0;

    while(1) {
    	angle = c_rc_receiver_get_channel(2) - 700;
    	angle = round(map(angle, 0, 1000, 0, 300));
    	c_io_rx24f_move(0x01, angle);
    	vTaskDelay(25/portTICK_RATE_MS);
    }
}

// Echoes anything received via UART2 using interrupts
void echo_task(void *pvParameters)
{
	while(1) {
		while(!c_common_usart_available(USART2)) { vTaskDelay(10/portTICK_RATE_MS); }
		c_common_usart_puts(USART2, "Got: ");
		c_common_usart_putchar(USART2, c_common_usart_read(USART2));
		c_common_usart_puts(USART2, " \n\r");
	}
}

// Prints what the receiver gets from the remote
void uart_task(void *pvParameters)
{
	char str[32];

    while(1) {
    	c_common_usart_puts(USART2, "\n\r---------------------\n\r");
    	for(int i=0; i<6; i++) {
    		sprintf(str, "Canal %d : %d\n\r", i, c_rc_receiver_get_channel(i));
    		c_common_usart_puts(USART2, str);
    	}

        vTaskDelay(1000/portTICK_RATE_MS);
    }
}

// Periodically reads the IMU and outputs the raw content via UART2
void i2c_task(void *pvParameters)
{
	char str[64];
	c_common_i2c_readBytes(ADXL345_ADDR, 0x00, 1, &ADXL345_ID);

	// Accelerometer increase G-range (+/- 16G)
	c_common_i2c_writeByte(ADXL345_ADDR, 0x31, 0b00001011);

    //  ADXL345 POWER_CTL
    c_common_i2c_writeByte(ADXL345_ADDR, 0x2D, 0);
    c_common_i2c_writeByte(ADXL345_ADDR, 0x2D, 16);
    c_common_i2c_writeByte(ADXL345_ADDR, 0x2D, 8);

	while(1) {
	    // Read x, y, z acceleration, pack the data.
		c_common_i2c_readBytes(ADXL345_ADDR, ADXL345_X_ADDR, 6, sensorBuffer);
	    accRaw[0] = ((int)sensorBuffer[0] | ((int)sensorBuffer[1] << 8)) * -1;
	    accRaw[1] = ((int)sensorBuffer[2] | ((int)sensorBuffer[3] << 8)) * -1;
	    accRaw[2] = (int)sensorBuffer[4] | ((int)sensorBuffer[5] << 8);

	    sprintf(str, "Accel: %d %d %d\n\r", accRaw[0], accRaw[1], accRaw[2]);
	    c_common_usart_puts(USART2, str);

		vTaskDelay(100/portTICK_RATE_MS);
	}
}


/* PRV -----------------------------------------------------------------------*/
void prvHardwareInit()
{
	c_common_i2c_init();
	c_common_usart2_init(9600);
	c_io_rx24f_init(1000000);
	c_rc_receiver_init();
	LED = c_common_gpio_init(GPIOC, GPIO_Pin_13, GPIO_Mode_OUT);
}

/* Main ----------------------------------------------------------------------*/
int main(void)
{
	SystemInit();
	prvHardwareInit();

	vTraceInitTraceData();

	c_common_usart_puts(USART2, "Programa iniciado!\n\r");
	vTraceConsoleMessage("Starting application...");

	//sprintf(str, "Line, %d \n\r", __LINE__);
	//c_common_usart_puts(USART2, str);

	if (! uiTraceStart() )
		vTraceConsoleMessage("Could not start recorder!");

	/* create tasks */
	xTaskCreate(blink_led_task, (signed char *)"Blink led", configMINIMAL_STACK_SIZE, (void *)NULL, tskIDLE_PRIORITY+1, NULL);
	//xTaskCreate(echo_task, (signed char *)"Echo task", configMINIMAL_STACK_SIZE, (void *)NULL, tskIDLE_PRIORITY+1, NULL);
	xTaskCreate(i2c_task,  (signed char *)"I2C task" , configMINIMAL_STACK_SIZE, (void *)NULL, tskIDLE_PRIORITY+1, NULL);
	//xTaskCreate(uart_task	  , (signed char *)"UART task", configMINIMAL_STACK_SIZE, (void *)NULL, tskIDLE_PRIORITY+1, NULL);
	//xTaskCreate(rc_servo_task , (signed char *)"Servo task", configMINIMAL_STACK_SIZE, (void *)NULL, tskIDLE_PRIORITY+1, NULL);

	/* Start the scheduler. */
	vTaskStartScheduler();

	/* should never reach here! */
	for(;;);
}

/**
  * @}
  */
