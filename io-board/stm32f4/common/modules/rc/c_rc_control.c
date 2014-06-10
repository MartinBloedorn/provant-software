/**
  ******************************************************************************
  * @file    modules/rc/c_rc_receiver.c
  * @author  Martin Vincent Bloedorn
  * @version V1.0.0
  * @date    30-November-2013
  * @brief   Controle de estabilizacao para vôo com usando controle remoto manual.
  ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "c_rc_control.h"

/** @addtogroup Module_RC
  * @{
  */

/** @addtogroup Module_RC_Component_Control
  * \brief Controle de estabilização para o modo de operação RC.
  *
  * O controlador pode ser testado fazendo-se, por exemplo:
  * \code
  * GPIOPin test = c_common_gpio_init(GPIOB, GPIO_Pin_11, GPIO_Mode_OUT);
  *
  * pv_msg_io_actuation   actuation = {0,0.0f,0.0f,0.0f,0.0f};
  * pv_msg_datapr_attitude attitude = {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};
  * pv_msg_datapr_attitude attitude_reference = {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};
  * pv_msg_datapr_position position = {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};
  * pv_msg_datapr_position position_reference = {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};
  *
  * attitude_reference.roll = 1.0f;
  *
  * while(1) {
  *   	actuation = RC_controller(attitude,
  * 					attitude_reference,
  * 					position,
  * 					position_reference);
  * 	    c_common_gpio_toggle(test);
  * }
  * \endcode
  *
  * A frequência de cálculo pode ser vista então se olhando o pino comutado no osciloscópio.
  * Para o exemplo acima, gira em torno de 80us.
  * @{
  */

		//---------------------------------------------------------------------------------------------

/* Exported functions definitions --------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

// PID controller gains
#if 1
	#define KPPHI    340.0f
	#define KVPHI    32.0f
	#define KIPHI	 1200.0f
	#define KPTHETA  340.0f
	#define KVTHETA  32.0f
	#define KITHETA	 1200.0f
	#define KPPSI    100.0f
	#define KVPSI    20.0f
	#define KIPSI	 0.0f
	#define KVZ		 5.0f
	#define KPZ		 30.0f
#else
#define KPPHI    100.0f
	#define KVPHI    20.0f
	#define KIPHI	 0.0f
	#define KPTHETA  100.0f
	#define KVTHETA  20.0f
	#define KITHETA	 0.0f
	#define KPPSI    100.0f
	#define KVPSI    20.0f
	#define KIPSI	 0.0f
	#define KVZ		 5.0f
	#define KPZ		 30.0f
#endif

// Environment parameters
#define G   9.81 //gravity

// Aircraft parameters
#define M    1.672   // mass kg
#define L    0.27023 // aircraft's arm length. Y-axis distance between center of rotation(B) and rotor center of mass.
#define H    0.03349 // center of mass displacement in Z-axis
// Aircraft's Moments of Inertia km*m²
#define IXX  0.01905797115
#define IYY  0.00502396129
#define IZZ  0.01859602726

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
float32_t gamma_f32[3];
float32_t tau_f32[3];
float32_t attitudeVector_f32[3];
float32_t Mq_f32[3][3];
float32_t Cqq_f32[3][3];
float32_t Fzb;

/** \brief Integral do erro dos angulos de orientacao VANT.*/
typedef struct {
	float roll, pitch, yaw;
} c_rc_control_error;

// Variáveis criadas para guardar o erro e a integral do erro
// São globais porque precisamos do erro anterior e integracao anterior
c_rc_control_error integrated_error={0};
c_rc_control_error error={0};


/* Private function prototypes -----------------------------------------------*/
float32_t altitude_controller_step(float32_t altitude, float32_t altitude_reference, float32_t rateOfClimb, float32_t rateOfClimb_reference, pv_msg_datapr_attitude attitude);
arm_matrix_instance_f32 PD_gains_step(pv_msg_datapr_attitude attitude, pv_msg_datapr_attitude attitude_reference);
arm_matrix_instance_f32 torque_calculation_step(pv_msg_datapr_attitude attitude, arm_matrix_instance_f32 gamma);
pv_msg_io_actuation actuators_signals_step(arm_matrix_instance_f32 tau, float32_t Fzb);
arm_matrix_instance_f32 inertia_matrix(pv_msg_datapr_attitude attitude);
arm_matrix_instance_f32 coriolis_matrix(pv_msg_datapr_attitude attitude);

float integrateTrapezoidal(float last_integration, float current_value, float last_value, float sample_time);
void integrateError();

/* Private functions ---------------------------------------------------------*/

// rateOfClimb (RoC) = aircraft's vertical speed (velocity in Z-axis)
float32_t altitude_controller_step(float32_t altitude, float32_t altitude_reference, float32_t rateOfClimb, float32_t rateOfClimb_reference, pv_msg_datapr_attitude attitude)
{
	float32_t Zeta3;

	Zeta3 = -KPZ*(altitude - altitude_reference) - KVZ*(rateOfClimb - rateOfClimb_reference);

	return (M * G + M * Zeta3) / (cos(attitude.roll) * cos(attitude.pitch));
}

/**\brief Integral numérica utilizando o método trapezoidal (Tustin)
 *
 */
float integrateTrapezoidal(float last_integration, float current_value, float last_value, float sample_time){

	return last_integration + sample_time * (current_value + last_value)/2;

	//integrated_error.roll = integrated_error.roll + SAMPLE_TIME*(error_roll+error.roll);
}

void integrateError(){

}


arm_matrix_instance_f32 PD_gains_step(pv_msg_datapr_attitude attitude, pv_msg_datapr_attitude attitude_reference)
{
	arm_matrix_instance_f32 gamma;

	error.roll = attitude.roll - attitude_reference.roll;
	error.pitch = attitude.pitch - attitude_reference.pitch;
	error.yaw = attitude.yaw - attitude_reference.yaw;

	// Integra o erro, utiliza as variaveis globais integrated_error e error
	integrateError();

	//gamma1
	gamma_f32[0] = -KPPHI * (attitude.roll - attitude_reference.roll)
						- KVPHI * (attitude.dotRoll - attitude_reference.dotRoll)
						- KIPHI * integrated_error.roll;
	//gamma2
	gamma_f32[1] = -KPTHETA * (attitude.pitch - attitude_reference.pitch)
						- KVTHETA * (attitude.dotPitch - attitude_reference.dotPitch)
						- KITHETA * integrated_error.pitch;
	//gamma3
	gamma_f32[2] = -KPPSI * (attitude.yaw - attitude_reference.yaw)
						- KVPSI * (attitude.dotYaw - attitude.dotYaw)
						- KIPSI * integrated_error.yaw;

	arm_mat_init_f32(&gamma, 3, 1, (float32_t *)gamma_f32);

	return gamma;
}

arm_matrix_instance_f32 torque_calculation_step(pv_msg_datapr_attitude attitude, arm_matrix_instance_f32 gamma)
{
	float32_t r1_f32[3]; // used as intermediate variable for matrices operations
	float32_t r2_f32[3]; // used as intermediate variable for matrices operations
	arm_matrix_instance_f32 tau;
	arm_matrix_instance_f32 attitudeVector;
	arm_matrix_instance_f32 r1;
	arm_matrix_instance_f32 r2;
	arm_matrix_instance_f32 cor_att;
	arm_matrix_instance_f32 inr_att;

	arm_mat_init_f32(&r1, 3, 1, (float32_t *)r1_f32);
	arm_mat_init_f32(&r2, 3, 1, (float32_t *)r2_f32);

	attitudeVector_f32[0]= attitude.dotRoll;
	attitudeVector_f32[1]= attitude.dotPitch;
	attitudeVector_f32[2]= attitude.dotYaw;

	arm_mat_init_f32(&tau, 3, 1, (float32_t *)tau_f32);
	arm_mat_init_f32(&attitudeVector, 3, 1, (float32_t *)attitudeVector_f32);

	/*tau = ( inertia_matrix(pv_msg_datapr_attitude attitude) * gamma )
	+ ( coriolis_matrix(pv_msg_datapr_attitude attitude) * dotTheta );*/
	inr_att = inertia_matrix(attitude);
	cor_att = coriolis_matrix(attitude);

	arm_mat_mult_f32(&inr_att, &gamma, &r1);
	arm_mat_mult_f32(&cor_att, &attitudeVector, &r2);
	arm_mat_add_f32(&r1, &r2, &tau);

	return tau;
	/* tau[0] = tauRoll
	   tau[1] = tauPitch
	   tau[2] = tauYaw */
}

pv_msg_io_actuation actuators_signals_step(arm_matrix_instance_f32 tau, float32_t Fzb)
{
	/* tau[0] = tauRoll
	   tau[1] = tauPitch
	   tau[2] = tauYaw */

	pv_msg_io_actuation actuation;
	float32_t * tauData = tau.pData;

	actuation.escRightSpeed = 0.5 * sqrt(
				pow((tauData[1] / H + tauData[2] / L), 2)
				+ pow((-Fzb + tauData[0] / L), 2));

	actuation.escLeftSpeed = 0.5 * sqrt(
				pow((-tauData[1] / H + tauData[2] / L), 2)
				+ pow((Fzb + tauData[0] / L), 2));

	actuation.servoRight = atan2(
				(tauData[1] / H + tauData[2] / L)
				, (Fzb - tauData[0] / L));

	actuation.servoLeft = atan2(
				(tauData[1] / H - tauData[2] / L)
				, (Fzb + tauData[0] / L));

	// Declares that the servos will use angle control, rather than torque control
	actuation.servoTorqueControlEnable = 0;

	return actuation;
}

// Inertia matrix
arm_matrix_instance_f32 inertia_matrix(pv_msg_datapr_attitude attitude)
{
	arm_matrix_instance_f32 Mq;

	Mq_f32[0][0] = IXX;
	Mq_f32[0][1] = 0;
	Mq_f32[0][2] = -IXX * sin(attitude.pitch);
	Mq_f32[1][0] = 0;
	Mq_f32[1][1] = IYY * pow(cos(attitude.roll), 2) + IZZ * pow(sin(attitude.roll), 2);
	Mq_f32[1][2] = (IYY - IZZ) * cos(attitude.roll) * sin(attitude.roll)* cos(attitude.pitch);
	Mq_f32[2][0] = - IXX * sin(attitude.pitch);
	Mq_f32[2][1] = (IYY - IZZ) * cos(attitude.roll) * sin(attitude.roll)* cos(attitude.pitch);
	Mq_f32[2][2] = IXX * pow(sin(attitude.pitch), 2)
			+ IYY * pow(sin(attitude.roll), 2)
			* pow(cos(attitude.pitch), 2)
			+ IZZ * pow(cos(attitude.roll), 2)
			* pow(cos(attitude.pitch), 2);

	arm_mat_init_f32(&Mq, 3, 3, (float32_t *)Mq_f32);

	return Mq;
}

/*	Coriolis and Centrifugal Matrix = C(THETA,dotTHETA) - Cqq - obtained by
*  christoffel symbols of the first kind page 40 of "ROBUST CONTROL STRATEGIES
*  FOR A QUADROTOR HELICOPTER - An Underactuated Mechanical System" by Raffo G.
*  */
arm_matrix_instance_f32 coriolis_matrix(pv_msg_datapr_attitude attitude)
{
	arm_matrix_instance_f32 Cqq;

	Cqq_f32[0][0] = 0;

	Cqq_f32[0][1] = (IYY - IZZ)
			* (attitude.pitch * cos(attitude.roll)
			* sin(attitude.roll)
			+ 0.5 * attitude.dotYaw
			* pow(sin(attitude.roll), 2)
			* cos(attitude.pitch))
			+ 0.5 * (IZZ - IYY) * attitude.dotYaw
			* pow(cos(attitude.roll), 2)
			* cos(attitude.pitch)
			- 0.5 * IXX * attitude.dotYaw * cos(attitude.pitch);

	Cqq_f32[0][2] = (IZZ - IYY)
			* (attitude.dotYaw * cos(attitude.roll)
			* sin(attitude.roll)
			* pow(cos(attitude.pitch), 2)
			+ 0.5 * attitude.dotPitch * cos(attitude.pitch)
			* pow(cos(attitude.roll), 2))
			+ 0.5 * (IYY - IZZ) * attitude.dotPitch
			* cos(attitude.pitch)
			* pow(sin(attitude.roll), 2)
			- 0.5 * IXX * attitude.dotPitch * cos(attitude.pitch);

	Cqq_f32[1][0] = (IZZ - IYY)
			* (attitude.dotPitch * cos(attitude.roll)
			* sin(attitude.roll)
			+ 0.5 * attitude.dotYaw
			* pow(sin(attitude.roll), 2)
			* cos(attitude.pitch))
			+ 0.5 * (IYY - IZZ)
			* (attitude.dotYaw * pow(cos(attitude.roll), 2)
			* cos(attitude.pitch))
			+ 0.5 * IXX * attitude.dotYaw * cos(attitude.pitch);

	Cqq_f32[1][1] = (IZZ - IYY) * attitude.dotRoll * cos(attitude.roll)
			* sin(attitude.roll);

	Cqq_f32[1][2] = -IXX * attitude.dotYaw * sin(attitude.pitch)
			* cos(attitude.pitch)
			+ IYY * attitude.dotYaw * pow(sin(attitude.roll), 2)
			* cos(attitude.pitch) * sin(attitude.pitch)
			+ IZZ * attitude.dotYaw * pow(cos(attitude.roll), 2)
			* sin(attitude.pitch) * cos(attitude.pitch)
			+ 0.5 * (IYY - IZZ) * attitude.dotRoll
			* cos(attitude.pitch)
			* pow(cos(attitude.roll), 2)
			+ 0.5 * (IZZ - IYY) * attitude.dotRoll
			* cos(attitude.pitch)
			* pow(sin(attitude.roll), 2)
			+ 0.5 * IXX * attitude.dotRoll * cos(attitude.pitch);

	Cqq_f32[2][0] = (IYY - IZZ)
			* (attitude.dotYaw * pow(cos(attitude.pitch), 2)
			* sin(attitude.roll) * cos(attitude.roll)
			+ 0.5 * attitude.dotPitch * cos(attitude.pitch)
			* pow(cos(attitude.roll), 2))
			+ 0.5 * (IZZ - IYY) * attitude.dotPitch
			* cos(attitude.pitch)
			* pow(sin(attitude.roll), 2)
			- 0.5 * IXX * attitude.dotPitch * cos(attitude.pitch);

	Cqq_f32[2][1] = (IZZ - IYY)
			* (attitude.dotPitch * cos(attitude.roll)
			* sin(attitude.roll) * sin(attitude.pitch)
			+ 0.5 * attitude.dotRoll
			* pow(sin(attitude.roll), 2)
			* cos(attitude.pitch))
			+ 0.5 * (IYY - IZZ) * attitude.dotRoll
			* pow(cos(attitude.roll), 2)
			* cos(attitude.pitch)
			- 0.5 * IXX * attitude.dotRoll * cos(attitude.pitch)
			+ IXX * attitude.dotYaw * sin(attitude.pitch)
			* cos(attitude.pitch)
			- IYY * attitude.dotYaw * pow(sin(attitude.roll), 2)
			* sin(attitude.pitch) * cos(attitude.pitch)
			- IZZ * attitude.dotYaw * pow(cos(attitude.roll), 2)
			* sin(attitude.pitch) * cos(attitude.pitch);

	Cqq_f32[2][2] = (IYY - IZZ) * attitude.dotRoll * cos(attitude.roll)
			* sin(attitude.roll) * pow(cos(attitude.pitch), 2)
			- IYY * attitude.dotPitch * pow(sin(attitude.roll), 2)
			* cos(attitude.pitch) * sin(attitude.pitch)
			- IZZ * attitude.dotPitch * pow(cos(attitude.roll), 2)
			* cos(attitude.pitch) * sin(attitude.pitch)
			+ IXX * attitude.dotPitch * cos(attitude.pitch)
			* sin(attitude.pitch);

	arm_mat_init_f32(&Cqq, 3, 3, (float32_t *)Cqq_f32);

	return Cqq;
}

/* Exported functions definitions --------------------------------------------*/

/** \brief Inicilização do controle de estabilidade.
 *
 * O controlador utilza a API de DSP da CMSIS, e portanto se baseia fortemente no uso do
 * tipo arm_matrix_instance_f32. Esta \b struct contêm os valores de número de linhas e
 * colunas de matriz, além de um ponteiro para seus elementos (na forma de array).
 * Estes arrays são prealocados globalmente (ver código fonte), para evitar overhead
 * de alocação dinâmica em cada chamada e para evitar que, a cada alocação em uma função, a memória para
 * a qual o ponteiro aponta saia de escopo e seja deletada. Uma vez que as funções são privadas e chamadas
 * em ordem determinística, mutexes não são implementadas (por simplicidade apenas)
 */
void c_rc_control_init() {

}

/* \brief Controlador.
 *
 * Implemented based on the article "Back-stepping Control Strategy for Stabilization of a Tilt-rotor UAV" by Chowdhury, A. B., with some modifications.
 * It implements an estabilization controller and an altitude controller. It is meant to be used with the radio controller.
 * The struct pv_msg_io_attitude includes the angular velocity.
 */
pv_msg_io_actuation RC_controller(pv_msg_datapr_attitude attitude,
				  pv_msg_datapr_attitude attitude_reference,
				  pv_msg_datapr_position position,
				  pv_msg_datapr_position position_reference) {
	arm_matrix_instance_f32 gamma;
	arm_matrix_instance_f32 tau;

	gamma = PD_gains_step(attitude, attitude_reference);

	tau = torque_calculation_step(attitude, gamma);

	Fzb = altitude_controller_step(position.z, position_reference.z, position.dotZ, position_reference.dotZ, attitude);

	return actuators_signals_step(tau, Fzb);
}

/* IRQ handlers ------------------------------------------------------------- */

/**
  * @}
  */

/**
  * @}
  */

