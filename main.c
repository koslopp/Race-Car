/* T-106.5300 Embedded Systems

Students: 411398 – Daniel Koslopp
	  411411 – Talita Tobias Carneiro */

/* This file contains the project's code */

/* LIBRARIES */

#include <inttypes.h> 		/* definitions for uint8_t and others */
#include <avr/io.h> 		/* definitions for all PORT* and other registers */
#include <avr/interrupt.h>	/* definitions for interruptions */
#include <util/delay.h>		/* definitions to use _delay_ms(x) function */
#include <math.h>		/* Math functions */

/* PHYSICAL DEFINITIONS */

#define BUMPER_PIN	PINA	/* Read bumper sensors */
#define BUMPER_DDR 	DDRA	/* Configure DDR bumper sensors */

#define TACHO_PIN 	PINL	/* Read tachometer at T5 (PL2) */
#define TACHO_DDR 	DDRL	/* Configure DDR tachometer port */

#define POSITION_PORT 	PORTB	/* PWM for position at OC1A (PB5) */
#define POSITION_DDR 	DDRB	/* Configure DDR PWM position port */

#define MOTOR_PORT	PORTK	/* Motor port */
#define MOTOR_PIN	PINK	/* Read motor port */
#define MOTOR_DDR	DDRK	/* Configure DDR motor port */

#define VELOCITY_PORT	PORTH	/* PWM for velocity motor at OC4A (PH3) */
#define VELOCITY_DDR	DDRH	/* Configure DDR PWM motor port */

#define LCD_PORT	PORTD	/* LCD port */
#define LCD_PIN		PIND	/* LCD pin */

#define LCD_DDR		DDRD	/* Configure DDR LCD port */

#define LEDS_PORT 	PORTC	/* Leds port */
#define LEDS_PIN 	PINC	/* Leds pin */
#define LEDS_DDR 	DDRC	/* Configure DDR leds port */

#define SWITCH_PIN	PINE	/* Switch pin */
#define SWITCH_DDR	DDRE	/* Switch DDR */

#define B0 		PA0	/* Bit definitions for bumper sensors */
#define B1 		PA1
#define B2 		PA2
#define B3 		PA3
#define B4 		PA4
#define B5 		PA5
#define B6 		PA6
#define B7 		PA7

#define TACHOMETER	PL2	/* Tachometer input (T5) */

#define PWM_SERVO	PB5	/* PWM servo positioning output (OC1A) */

#define INA		PK0	/* Control motor bits definition */
#define INB		PK1
#define ENA		PK2
#define ENB		PK3
#define CS		PK4	

#define PWM_MOTOR	PH3	/* PWM velocity output (OC4A) */

#define LCD_TX		PD2	/* LCD TX */
#define LCD_RX		PD3	/* LCD RX */

#define SWITCH		PE5	/* User switch - Turn on/off*/

#define LED0		PC0	/* User LED 0 */
#define LED1		PC1	/* User LED 1 */


/* GLOBAL VARIABLES*/
 
double 	CONTROLLER_OUT_DIR = 0; /* Out value from direction control */
double 	CONTROLLER_OUT_VEL = 0; /* Out value from velocity control */
double 	PREVIOUS_ERROR = 0; /* Variable used in the direction and velocity control algorithm */
double 	PREVIOUS_ERROR_TACHO = 0; /* Variable used in the tachometer control algorithm */
double 	MEASURED_PULSE = 0; /* Used as a tachometer counter */
double 	INTEGRAL_DIR = 0; /* Direction Integrative error */
double 	DERIVATIVE_DIR = 0; /* Direction derivative error */
double 	DERIVATIVE_VEL = 0; /* Velocity derivative error */
double  VEL = 9; /* Tachometer velocity required (in pulses each velocity's sampling time) */
uint8_t TACHO_COUNTER = 0; /* Used to getting the tachometer value each velocity's sampling time */
uint8_t CALC_VEL = 0; /* Auxiliar variable to call velocity function */
uint8_t	TURN_ON_OFF = 0; /* Auxiliar variable for turn the car on/off. Start off */
uint16_t BUMPER_OFF_COUNTER = 0; /* Count time with no sensor bumper active to stop the car */

/* SETUPS */

void setup_servo(void)
{
	POSITION_DDR |= _BV(PWM_SERVO); /* Setting PWM_SERVO as output*/
	TCCR1A |= _BV(WGM11); /* Fast PWM (14), non-inverter, disconnected */
	TCCR1B |= _BV(WGM13) | _BV(WGM12) | _BV(CS10); /* Fast PWM (14), Prescale in 1 */
	ICR1 = 63999; /* Top value for 4ms*/
	OCR1A = 24000; /* Initial value for compare match. Angle = 0 */
}

void setup_sampling_timer3(void)
{
	TCCR3A = 0; /* Mode CTC (4), disconnected at start*/
	TCCR3B |= _BV(WGM32); /* Mode CTC (4) */
	OCR3A = 249; /* Sampling period of 1ms*/
}

void setup_interruption(void)
{
	SREG |= _BV(7); /* Global interruption enable*/
}

void setup_tachometer_timer5(void)
{
	TCCR5A = 0; /* Normal mode of operation */
	TCCR5B |= _BV(CS52) | _BV(CS51) | _BV(CS50); /* External clk source, Rising edge */ 
}

void setup_motor(void)
{
	MOTOR_DDR |= _BV(INA) | _BV(INB); /* Config. as output */
	VELOCITY_DDR |= _BV(PWM_MOTOR); /* Config. as output */
	/* Fast PWM Config: WGM43:0 = 0b1110; 
	Freq. 20 kHz: Prescale = 1; CS42:0 0b001; OCR4A = 799;
	Disconected: COM4A1:0 0b00; */
	TCCR4A |= _BV(WGM41);
	TCCR4B |= _BV(WGM43) | _BV(WGM42) | _BV(CS41);
	ICR4 = 799;
	OCR4A = 150; /* The initial speed of the motor is set to 12% of the max */
}

void setup_digital_IO(void)
{
	/* BUMPER_DDR = 0; Already configured as input */
	/* SWITCH_DDR &= ~(_BV(SWITCH)); Already configured as input */
	LEDS_DDR |= _BV(LED0) | _BV(LED1); /* LEDs as outputs */
}

/* INTERRUPTIONS */

ISR(TIMER3_COMPA_vect) /* Sampling time (1 ms) */
{
	/* Local variables */	
	uint8_t BUMPER_TEMP = 0; /* Temp. bumper error */
	double ERROR = 0; /* Mean error from bumper */
	double WEIGHT = -1; /* Weight for sensor position starting from (-1) */
	double MEASURED_ERROR = 0; /* Total sum of error from each active sensor */
	double SENSOR_COUNTER = 0; /* Number of active sensors to calc. mean error */
	double KP_DIR = 1; /* Proportional gain of the direction controller */
	double KI_DIR = 0.5; /* Integrative gain of the direction controller */
	double KD_DIR = 1; /* Derivative gain of the direction controller */
	double KP_VEL = 0.5; /* Proportional gain of the velocity controller */
	double KI_VEL = 0.1; /* Integrative gain of the velocity controller */
	double KD_VEL = 1.2; /* Derivative gain of the velocity controller */
	double dt = 0.001; /* Sampling time */

	/* Allocating the value of the bumper sensor */
	BUMPER_TEMP = ~(BUMPER_PIN); 	

	/* Measuring error from bumper */
	for (int i = 0; i < 8; i++) /* Read the entire byte of sensors from variable */
	{
		/* Verifies which and how many sensors are active adding its weight error */
		if (BUMPER_TEMP & _BV(i)) {MEASURED_ERROR += WEIGHT; SENSOR_COUNTER++;}
		WEIGHT+= 0.25; /* Add weight for next sensor */
	}
	if(SENSOR_COUNTER == 0) /* If sensor is not active */
	{
		ERROR = PREVIOUS_ERROR; /* If no sensor is active use the last error */
		BUMPER_OFF_COUNTER++; /* Increase sensor bumper inactive variable */
		/* If no sensor is active for more than 3 seconds, the car is turned off */
		if (BUMPER_OFF_COUNTER == 2000) turn_car_off();
	}
	else 
	{
		ERROR = - MEASURED_ERROR/SENSOR_COUNTER; /* Mean error with ref value in 0*/
		BUMPER_OFF_COUNTER = 0;
	}

	/* PID Control implementation */

	INTEGRAL_DIR += ERROR*dt; /* Calculation of integral term */
	DERIVATIVE_DIR = (ERROR - PREVIOUS_ERROR)/dt; /* Calculation of derivative term */
	PREVIOUS_ERROR = ERROR; /* Save error for next sampling */
	/* Out control value for direction */
	CONTROLLER_OUT_DIR = KP_DIR*ERROR + KI_DIR*INTEGRAL_DIR + KD_DIR*DERIVATIVE_DIR;

	ERROR = abs(ERROR); /* Velocity does not care for directional error, just is abs. value */
	DERIVATIVE_VEL = (ERROR - PREVIOUS_ERROR)/dt; /* Calculation of derivative term */
	/* Out control value for velocity */
	CONTROLLER_OUT_VEL = KP_VEL*ERROR + KD_VEL*DERIVATIVE_VEL;

	/* Allocating the value of the tachometer after 200 interruptions (200ms) */
	if (TACHO_COUNTER == 199)
	{
		MEASURED_PULSE = TCNT5; /* Save number of motor turns (from timer 5) */
		TCNT5 = 0; /* Reset timer 5 */
		TACHO_COUNTER = 0; /* Reset variable to count 200 interruptions */
		CALC_VEL = 1; /* Active velocity control function */
	}
	TACHO_COUNTER++; /* Count 200 interruptions */
}

/* FUNCTIONS */

void direction_pwm(double control_out)
{
	double CONTROL_SATURATION = 1; /* Find by simulation */
	
	/*If control signal between max acceptable value, calc. OCR1A
	to find respectively position. If not give the extreme values to direction */
	if (control_out <= CONTROL_SATURATION & control_out >= (-CONTROL_SATURATION))
	{
		OCR1A = round(23999.625 - 5599.9125*(control_out/CONTROL_SATURATION));		
	}
	else if (control_out > CONTROL_SATURATION) {OCR1A = 29600; CONTROLLER_OUT_DIR = CONTROL_SATURATION;} /* angle = 90 */
	else if (control_out < (-CONTROL_SATURATION)) {OCR1A = 18399; CONTROLLER_OUT_DIR = - CONTROL_SATURATION;} /* angle = -90 */
}

void velocity_pwm (double control_out)
{
	double CONTROL_SATURATION = 1; /* Find by simulation */
	double INTEGRAL_TACHO = 0; /* Tachometer Integrative error */
	double DERIVATIVE_TACHO = 0; /* Tachometer Derivative error */
	double ERROR_TACHO = 0; /* Tachometer error */
	double dt_tacho = 0.2; /* Tachometer sampling time */
	double CONTROLLER_OUT_TACHO = 0; /* Out tachometer control value */
	double KP_TACHO = 0.5; /* Propotional gain of the tachometer controller */
	double KI_TACHO = 0.3; /* Integrative gain of the tachometer controller */
	double KD_TACHO = 0.5; /* Derivative gain of the tachometer controller */

	/*If control signal between max acceptable value, calc. OCR4A
	to find respectively velocity. If not give the min value to motor turns (VEL) */
	if (control_out <= CONTROL_SATURATION)
	{
		VEL = round(10 - 9*(control_out/CONTROL_SATURATION));		
	}
	else if (control_out > CONTROL_SATURATION) {VEL = 1; CONTROLLER_OUT_VEL = CONTROL_SATURATION;} /* turns = min */
		
	ERROR_TACHO = VEL - MEASURED_PULSE; /* Error between turn required and measured */
	INTEGRAL_TACHO += ERROR_TACHO*dt_tacho; /* Calculation of integral term*/
	DERIVATIVE_TACHO = (ERROR_TACHO - PREVIOUS_ERROR_TACHO)/dt_tacho; /* Calculation of derivative term*/
	PREVIOUS_ERROR_TACHO = ERROR_TACHO; /* Save error for next sampling */
	
	/* Out control value for velocity pwm */
	CONTROLLER_OUT_TACHO = KP_TACHO*ERROR_TACHO + KI_TACHO*INTEGRAL_TACHO + KD_TACHO*DERIVATIVE_TACHO;
	
	/* Control Motor PWM between min and max values */
	if (CONTROLLER_OUT_TACHO <= CONTROL_SATURATION & CONTROLLER_OUT_TACHO >= (-CONTROL_SATURATION))
	{
		OCR4A = round(125 - 75*(CONTROLLER_OUT_TACHO/CONTROL_SATURATION));		
	}
	else if (CONTROLLER_OUT_TACHO > CONTROL_SATURATION) {OCR4A = 190;} /* pwm = max */
	else if (CONTROLLER_OUT_TACHO < (-CONTROL_SATURATION)) {OCR4A = 50;} /* pwm = min */
	
	CALC_VEL = 0; /* Deactivate pwm function */
}

void turn_car_on (void)
{
	TURN_ON_OFF = 1;
	TIMSK3 |= _BV(OCIE3A); /* Enable interrupt for TMR3A */ 
	TCCR3B |= _BV(CS31) | _BV(CS30); /* Prescale in 64. Start sampling */
	TCCR1A |= _BV(COM1A1); /* Turn on pwm servo */
	TCCR4A |= _BV(COM4A1); /* Turn on pwm motor */
	MOTOR_PORT |= _BV(INA); /* Turn motor on clockwise */
	LEDS_PORT |= _BV(LED0); /* Turn on led */
}

void turn_car_off (void)
{
	TURN_ON_OFF = 0;
	TIMSK3 &= ~_BV(OCIE3A); /* Desable interrupt for TMR3A */ 
	TCCR3B &= ~_BV(CS31) | _BV(CS30); /* No clock source. Stop sampling */
	TCCR1A &= ~_BV(COM1A1); /* Turn off pwm servo */
	TCCR4A &= ~_BV(COM4A1); /* Turn off pwm motor */
	MOTOR_PORT &= ~_BV(INA); /* Turn motor off */
	LEDS_PORT &= ~_BV(LED0); /* Turn off led */
		
	/*Reset of variables */				
	INTEGRAL_DIR = 0;
	DERIVATIVE_DIR = 0;
	DERIVATIVE_VEL = 0;
	PREVIOUS_ERROR = 0; 
	CONTROLLER_OUT_DIR = 0;
	CONTROLLER_OUT_VEL = 0;
}

/* MAIN ROUTINE */

int main(void)
{
	/*Setup functions */
	setup_servo();
	setup_sampling_timer3();
	setup_interruption();
	setup_tachometer_timer5();
	setup_motor();
	setup_digital_IO();

	/*Execution routine*/
	for(;;)
	{
		/* The execution starts after the SWITCH button is pressed */
		if (~(SWITCH_PIN) & _BV(SWITCH))
		{
			if (TURN_ON_OFF == 0) /* Car off. Turn it on */
			{
				_delay_ms(10);
				turn_car_on();
				while (~(SWITCH_PIN) & _BV(SWITCH));				
			}
			else /* Turn it off */
			{
				turn_car_off();
				_delay_ms(10);
				while (~(SWITCH_PIN) & _BV(SWITCH));	
			}
		}
		direction_pwm(CONTROLLER_OUT_DIR); /* Call direction control function */
		if (CALC_VEL == 1) velocity_pwm(CONTROLLER_OUT_VEL); /* If active, call velocity control function */	
	}
}
