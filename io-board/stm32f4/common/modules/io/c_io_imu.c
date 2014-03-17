/**
  ******************************************************************************
  * @file    modules/io/c_io_imu.c
  * @author  Martin Vincent Bloedorn
  * @version V1.0.0
  * @date    11-February-2014
  * @brief   Funções para IMUs (incialmente baseadas no CIs ITG3205 e ADXL345).
  *****************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "c_io_imu.h"

/** @addtogroup Module_IO
  * @{
  */

/** @addtogroup Module_IO_Component_IMU
  *	\brief Componente para IMU.
  *
  *	Este componente é projetado para implementar funções da IMU da aeronave - leitura
  *	e pré-processamento. A IMU suportada é a baseada nos CIs ITG3205 e ADXL345, mas
  *	outros modelos podem ser incorporados via #define.
  *
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
//#define I2Cx_imu      I2C2 // i2c of imu

#ifdef C_IO_IMU_USE_ITG_ADXL_HMC
	#define GYRO_ADDR   0x68 // The address of ITG3205
	#define ACCL_ADDR   0x53 // The address of ADXL345
	#define MAGN_ADDR   0x1E // The address of HMC5883
	#define GYRO_X_ADDR 0x1D // Start address for x-axis
	#define ACCL_X_ADDR 0x32 // Start address for x-axis
	#define MAGN_X_ADDR 0x03 // Start address for x-axis
#elif defined C_IO_IMU_USE_MPU6050_HMC5883
	#include "c_io_imu_MPU6050.h"

	#define HMC58X3_ADDR 0x1E // 7 bit address of the HMC58X3
	#define HMC_POS_BIAS 1
	#define HMC_NEG_BIAS 2

	// HMC58X3 register map. For details see HMC58X3 datasheet
	#define HMC58X3_R_CONFA 0
	#define HMC58X3_R_CONFB 1
	#define HMC58X3_R_MODE 2
	#define HMC58X3_R_XM 3
	#define HMC58X3_R_XL 4
	#define HMC58X3_R_STATUS 9
	#define HMC58X3_R_IDA 10
	#define HMC58X3_R_IDB 11
	#define HMC58X3_R_IDC 12
#else
	#error "Define an IMU type in `c_io_imu.h`! C_IO_USE_ITG_ADXL, or other!"
#endif


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
I2C_TypeDef* I2Cx_imu;
uint8_t imuBuffer[16];
long lastIntegrationTime=0; /** Último valor do SysTick quando a função de filtragem foi chamada - para integracão numérica */
unsigned char ACCL_ID = 0;
unsigned char GYRO_ID = 0;
unsigned char MAGN_ID = 0;

float gyro_rpy[3], acce_rpy[3], filt_rpy[3];

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/* Exported functions definitions --------------------------------------------*/

/** \brief Inicializa a IMU.
 *
 * Seta sensibilidade do acelerômetro e liga o girscópio.
 */
void c_io_imu_init(I2C_TypeDef* I2Cx)
{

  I2Cx_imu=I2Cx;

#ifdef C_IO_IMU_USE_ITG_ADXL_HMC // Inicialização para a IMU selecionada
	// Get Accelerometer ID
	c_common_i2c_readBytes(I2Cx_imu, ACCL_ADDR, 0x00, 1, &ACCL_ID);

	// Accelerometer increase G-range (+/- 16G)
	c_common_i2c_writeByte(I2Cx_imu, ACCL_ADDR, 0x31, 0x0B);

  //  ADXL345 (Accel) POWER_CTL
  c_common_i2c_writeByte(I2Cx_imu, ACCL_ADDR, 0x2D, 8);

  // Gyro ID and setup
	c_common_i2c_readBytes(I2Cx_imu, GYRO_ADDR, 0x00, 1, &GYRO_ID);
	c_common_i2c_writeByte(I2Cx_imu, GYRO_ADDR, 0X16, 24); //24 = 0b0001 1000

  // HMC5883 (Magn) Run in continuous mode
  c_common_i2c_writeByte(I2Cx_imu, MAGN_ADDR, 0x02, 0x00);
  // configure the B register to default value of Sensor Input Field Range: 1.2Ga
  // +/- 1.2Ga <-> +/- 2047
  c_common_i2c_writeByte(I2Cx_imu, MAGN_ADDR, 0x01, 0x20);
#endif

#ifdef C_IO_IMU_USE_MPU6050_HMC5883 //Inicialização para a IMU baseada na MPU6050
  // Clear the 'sleep' bit to start the sensor.
  c_common_i2c_writeByte(I2Cx_imu, MPU6050_I2C_ADDRESS, MPU6050_PWR_MGMT_1, 0);

  // Alocar o sub i2c -> desligar o I2C Master da MPU, habilitar I2C bypass
  c_common_i2c_writeBit(I2Cx_imu, MPU6050_I2C_ADDRESS, MPU6050_USER_CTRL, MPU6050_I2C_MST_EN, 0);
  c_common_i2c_writeBit(I2Cx_imu, MPU6050_I2C_ADDRESS, MPU6050_INT_PIN_CFG, MPU6050_I2C_BYPASS_EN, 1);

  /** \todo Implementar e testar o enabling do bus secundário da MPU, para leitura do HMC.*/
  //c_common_i2c_writeByte(0x1E, 0x02, 0x00);
  //uint8_t hmcid[3];
  //c_common_i2c_readBytes(HMC58X3_ADDR, HMC58X3_R_IDA, 3, hmcid);
#endif
}

/** \brief Obtem as leituras raw do acelerômetro, giro e magnetômetro.
 *
 * O output de cada um dos sensores é escrito nos buffers passados para a função. Os buffers passados
 * devem ter tamanho mínimo 3 (tipo float), e serão escritos sempre com valores dos eixos X, Y e Z
 * de cada sensor lido.
 * Os valores retornados para cada eixo de cada sensor estão em  \f$ g \f$ para o \b acelerômetro,
 * \f$ rad/s \f$ para o \b giroscópio, e \f$ rad \f$ para o \b magnetômetro.
 *
 * @param accRaw Buffer onde serão escritos os dados do acelerômetro.
 * @param gyrRaw Buffer onde serão escritos os dados do giroscópio.
 * @param magRaw Buffer onde serão escritos os dados do magnetômetro.
 */
void c_io_imu_getRaw(float  * accRaw, float * gyrRaw, float * magRaw) {

#ifdef C_IO_IMU_USE_ITG_ADXL_HMC
    // Read x, y, z acceleration, pack the data.
    uint8_t  buffer[14];
  	c_common_i2c_readBytes(I2Cx_imu, ACCL_ADDR, ACCL_X_ADDR, 6, imuBuffer);

    accRaw[0] = (int16_t)((imuBuffer[0] | ((int16_t)imuBuffer[1] << 8)));
    accRaw[1] = (int16_t)((imuBuffer[2] | ((int16_t)imuBuffer[3] << 8)));
    accRaw[2] = (int16_t)((imuBuffer[4] | ((int16_t)imuBuffer[5] << 8)));

    // Read x, y, z from gyro, pack the data

    /** A sensitividade do acelerômetro da ITG32000 é dada pela tabela (extraída do datasheet):
     FS_SEL | Full Scale Range | LSB Sensitivity
    --------|------------------|----------------
    0       | Reservado        | Reservado
    1       | Reservado        | Reservado
    2       | Reservado        | Reservado
    3       | 2,000°/seg       | 14.375 LSBs °/S
    ***********************************************/

    float accScale =14.375f;
    accScale /= 0.0174532925;//0.0174532925 = PI/180

  	c_common_i2c_readBytes(I2Cx_imu, GYRO_ADDR, GYRO_X_ADDR, 6, imuBuffer);
  	gyrRaw[0] =  (float)((int16_t)(imuBuffer[1] | ((int16_t)imuBuffer[0] << 8)))/accScale;
  	gyrRaw[1] =  (float)((int16_t)(imuBuffer[3] | ((int16_t)imuBuffer[2] << 8)))/accScale;
  	gyrRaw[2] =  (float)((int16_t)(imuBuffer[5] | ((int16_t)imuBuffer[4] << 8)))/accScale;

    // Read x, y, z from magnetometer;
    c_common_i2c_readBytes(I2Cx_imu, MAGN_ADDR, MAGN_X_ADDR, 6, imuBuffer);
   
    magRaw[0] =  (float)((int16_t)(imuBuffer[1] | ((int16_t)imuBuffer[0] << 8)));// X
    magRaw[1] =  (float)((int16_t)(imuBuffer[5] | ((int16_t)imuBuffer[4] << 8)));// Y
    magRaw[2] =  (float)((int16_t)(imuBuffer[3] | ((int16_t)imuBuffer[2] << 8)));// Z

    /** Como dito no link:  http://www.multiwii.com/forum/viewtopic.php?f=8&t=1387&p=10658
    * temosque encontrar os zeros do mag

         X   |     Y    |     Z
    ---------|----------|----------------
    -196/607 | -488/250 | -422/263
    ***********************************************/

    // -100/100
    magRaw[1] = (magRaw[1]-(250-488)/2)/3.69;
    magRaw[0] = (magRaw[0]-(607-196)/2)/4.015;
    magRaw[2] = (magRaw[2]-(263-422)/2)/3.425;

#endif

#ifdef C_IO_IMU_USE_MPU6050_HMC5883
    uint8_t  buffer[14];
    c_common_i2c_readBytes(I2Cx_imu, MPU6050_I2C_ADDRESS, MPU6050_ACCEL_XOUT_H, 14, buffer);

    /** A sensitividade do acelerômetro da MPU6050 é dada pela tabela (extraída do datasheet):
    AFS_SEL | Full Scale Range | LSB Sensitivity
    --------|------------------|----------------
    0       | ±2g              | 16384 LSB/g
    1       | ±4g              | 8192 LSB/g
    2       | ±8g              | 4096 LSB/g
    3       | ±16g             | 2048 LSB/g
    ***********************************************/
    float accScale = 16384.0f;

    accRaw[0] = -1.0f*(float)((((signed char)buffer[0]) << 8) | ((uint8_t)buffer[1] & 0xFF))/accScale;
    accRaw[1] = -1.0f*(float)((((signed char)buffer[2]) << 8) | ((uint8_t)buffer[3] & 0xFF))/accScale;
    accRaw[2] =       (float)((((signed char)buffer[4]) << 8) | ((uint8_t)buffer[5] & 0xFF))/accScale;

    /** A sensitividade do giroscópio da MPU6050 é dada pela tabela (extraída do datasheet):
    FS_SEL | Full Scale Range | LSB Sensitivity
    -------|------------------|----------------
    0      | ± 250 °/s        | 131 LSB/°/s
    1      | ± 500 °/s        | 65.5 LSB/°/s
    2      | ± 1000 °/s       | 32.8 LSB/°/s
    3      | ± 2000 °/s       | 16.4 LSB/°/s
    ***********************************************/
    float gyrScale = 131.0f;

    gyrRaw[0] = (float)((((signed char)buffer[8])  << 8) | ((uint8_t)buffer[9]  & 0xFF))/gyrScale;
    gyrRaw[1] = (float)((((signed char)buffer[10]) << 8) | ((uint8_t)buffer[11] & 0xFF))/gyrScale;
    gyrRaw[2] = (float)((((signed char)buffer[12]) << 8) | ((uint8_t)buffer[13] & 0xFF))/gyrScale;
#endif

}

/** \brief Retorna os ângulos RPY através de um filtro complementar simples.
 *
 * Implementa apenas uma fusão simples de dados do Giroscópio e Acelerômetro usando a proposta de um filtro complementar.
 *
 * \image html complementary_filter_diagram.jpg "Diagrama de blocos simplificado para um filtro complementar." width=4cm
 */
void c_io_imu_getComplimentaryRPY(float * rpy) {
	float acce_raw[3], gyro_raw[3], magn_raw[3];
	float acce_rpy[3];

	c_io_imu_getRaw(acce_raw, gyro_raw, magn_raw);

	//gyro_raw[X] = gyro_raw[X] - mean_gyro_raw[X];
	//gyro_raw[Y] = gyro_raw[Y] - mean_gyro_raw[Y];
	//gyro_raw[Z] = gyro_raw[Z] - mean_gyro_raw[Z];

  /*
	acce_rpy[PV_IMU_PITCH] = atan2( acce_raw[PV_IMU_X], sqrt(pow(acce_raw[PV_IMU_Y],2)
			+ pow(acce_raw[PV_IMU_Z],2)));// - mean_acce_rpy[ROLL] ;
	acce_rpy[PV_IMU_ROLL ] = atan2( acce_raw[PV_IMU_Y], sqrt(pow(acce_raw[PV_IMU_X],2)
			+ pow(acce_raw[PV_IMU_Z],2)));// - mean_acce_rpy[PITCH];
  */

  acce_rpy[PV_IMU_PITCH] = atan(acce_raw[PV_IMU_X]/sqrt(pow(acce_raw[PV_IMU_Y],2) + pow(acce_raw[PV_IMU_Z],2)));
  acce_rpy[PV_IMU_ROLL ] = atan(acce_raw[PV_IMU_Y]/sqrt(pow(acce_raw[PV_IMU_X],2) + pow(acce_raw[PV_IMU_Z],2)));

  float xh = magn_raw[PV_IMU_X]*cos(acce_rpy[PV_IMU_PITCH])+magn_raw[PV_IMU_Y]*sin(acce_rpy[PV_IMU_ROLL ])*sin(acce_rpy[PV_IMU_PITCH])-magn_raw[PV_IMU_Z]*cos(acce_rpy[PV_IMU_ROLL ])*sin(acce_rpy[PV_IMU_PITCH]);
  float yh = magn_raw[PV_IMU_Y]*cos(acce_rpy[PV_IMU_ROLL ])-magn_raw[PV_IMU_Z]*sin(acce_rpy[PV_IMU_ROLL ]);
  acce_rpy[PV_IMU_YAW ] = atan2(yh, xh);
	
  //Filtro complementar
	float a = 0.93;
  long  IntegrationTime = c_common_utils_millis();
  if(lastIntegrationTime==0) lastIntegrationTime=IntegrationTime;
  float IntegrationTimeDiff=(float)(((float)IntegrationTime- (float)lastIntegrationTime)/1000.0);

	rpy[PV_IMU_PITCH] = a*(rpy[PV_IMU_PITCH] + gyro_raw[PV_IMU_PITCH]*IntegrationTimeDiff) + (1.0f - a)*acce_rpy[PV_IMU_PITCH];
	rpy[PV_IMU_ROLL ] = a*(rpy[PV_IMU_ROLL ] + gyro_raw[PV_IMU_ROLL ]*IntegrationTimeDiff) + (1.0f - a)*acce_rpy[PV_IMU_ROLL ];
  rpy[PV_IMU_YAW]   = a*(rpy[PV_IMU_YAW  ] + gyro_raw[PV_IMU_YAW  ]*IntegrationTimeDiff) + (1.0f - a)*acce_rpy[PV_IMU_YAW  ];//acce_rpy[PV_IMU_YAW];
  

	lastIntegrationTime = IntegrationTime;
}

/** \brief Calibra a IMU considerando o veículo em repouso.
 *
 *
 */
void c_io_imu_calibrate() {

}

/* IRQ handlers ------------------------------------------------------------- */

/**
  * @}
  */

/**
  * @}
  */

