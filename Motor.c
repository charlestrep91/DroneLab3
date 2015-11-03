/*
 * Motor.c
 *
 *  Created on: 12 sept. 2013
 *      Author: bruno
 */


#include "Motor.h"
#include <termios.h>
#include <fcntl.h>

#define TimeDelay	50000
#define SleepDelayNanos 5000000
#define MotorTaskPriority	1

#define WRITE_PWM_CMD 0b001
#define WRITE_LED_CMD 0b011

extern sem_t	MotorTimerSem;
extern int		MotorActivated;

pthread_barrier_t 	MotorStartBarrier;
//SetPwm union definition
typedef union uPwmStream
{
	char bytes[5];
	struct
	{
		char xVal		:1;
		int PwmMoteur4	:9;
		int PwmMoteur3	:9;
		int PwmMoteur2	:9;
		int PwmMoteur1	:9;
		char cmd		:3;
	}ele;
}tPwmStream;

//SetLed union definition
typedef union uLedStream
{
	char bytes[2];
	struct
	{
		char cmd		:3;
		int  RedLed		:4;
		char xVal1		:4;
		int  GrnLed		:4;
		char xVal2		:1;
	}ele;
}tLedStream;



pthread_barrier_t 	MotorStartBarrier;

void SetPWM(int file, int pwm1, int pwm2, int pwm3, int pwm4)
{
//	tPwmStream pwmData;
//	pwmData.ele.cmd=WRITE_PWM_CMD;
//	pwmData.ele.PwmMoteur1=pwm1;
//	pwmData.ele.PwmMoteur2=pwm2;
//	pwmData.ele.PwmMoteur3=pwm3;
//	pwmData.ele.PwmMoteur4=pwm4;
//    write(file,pwmData.bytes,5);
	unsigned char cmd[5];
	cmd[0] = 0x20 | ((pwm1&0x1ff)>>4);
	cmd[1] = ((pwm1&0x1ff)<<4) | ((pwm2&0x1ff)>>5);
	cmd[2] = ((pwm2&0x1ff)<<3) | ((pwm3&0x1ff)>>6);
	cmd[3] = ((pwm3&0x1ff)<<2) | ((pwm4&0x1ff)>>7);
	cmd[4] = ((pwm4&0x1ff)<<1);
	write(file, cmd, 5);
}

void SetLed(int file, char redLed, char grnLed)
{
	tLedStream ledData;
	ledData.ele.cmd=WRITE_LED_CMD;
	ledData.ele.RedLed=redLed;
	ledData.ele.GrnLed=grnLed;
    write(file,ledData.bytes,2);
}

int gpio_set (int nr, int val)  {
	char cmdline[200];

	if (val < 0)
		sprintf(cmdline, "/usr/sbin/gpio %d -d i", nr);
	else if (val > 0)
		sprintf(cmdline, "/usr/sbin/gpio %d -d ho 1", nr);
	else
		sprintf(cmdline, "/usr/sbin/gpio %d -d ho 0", nr);

	return system(cmdline);
}


int motor_open(void) {
	struct termios config;
	int uart = open(MOTOR_UART, O_RDWR | O_NOCTTY | O_NDELAY);

	if (uart < 0) {
		printf("motor_open : impossible d'ouvrir le uart du moteur\n");
		return uart;
	}

	fcntl(uart, F_SETFL, 0); //read calls are non blocking

	//set port config
	tcgetattr(uart, &config);
	cfsetspeed(&config, B115200);
	config.c_cflag |= (CLOCAL | CREAD); //Enable the receiver and set local mode
	config.c_iflag = 0; //clear input config
	config.c_lflag = 0; //clear local config
	config.c_oflag &= ~OPOST; //clear output config (raw output)
	cfmakeraw(&config);
	tcsetattr(uart, TCSANOW, &config);
	return uart;
}

int motor_cmd(int file, uint8_t cmd, uint8_t *reply, int replylen) {
	int size;

	write(file, &cmd, 1);
	fsync(file);
	usleep(TimeDelay);
	size = read(file, reply, replylen);

	return size;
}

int MotorPortInit(MotorStruct *Motor) {
	uint8_t reply[256];
	int		i;

	//open motor port
	Motor->file = motor_open();
	if (Motor->file < 0) {
		printf("motor_open: Impossible d'ouvrir le UART\n");
		return Motor->file;
	}
	//reset IRQ flipflop - this code resets GPIO_ERROR_READ to 0
	gpio_set(GPIO_ERROR_RESET, 0);
	usleep(2*TimeDelay);

	//all select lines inactive
	gpio_set(GPIO_M1, 0);
	gpio_set(GPIO_M2, 0);
	gpio_set(GPIO_M3, 0);
	gpio_set(GPIO_M4, 0);
	usleep(2*TimeDelay);

	//configure motors
	for (i = 0; i < 4; ++i) {
		gpio_set(GPIO_M1 + i, -1);
		usleep(2*TimeDelay);
		motor_cmd(Motor->file, 0xE0, reply, 2);
		motor_cmd(Motor->file, 0x91, reply, 121);
		motor_cmd(Motor->file, 0xA1, reply, 2);
		motor_cmd(Motor->file, i + 1, reply, 1);
		motor_cmd(Motor->file, 0x40, reply, 2);
		gpio_set(GPIO_M1 + i ,0);
		usleep(2*TimeDelay);
	}

	//all select lines actives
	gpio_set(GPIO_M1, -1);
	gpio_set(GPIO_M2, -1);
	gpio_set(GPIO_M3, -1);
	gpio_set(GPIO_M4, -1);
	usleep(2*TimeDelay);

	gpio_set(GPIO_ERROR_READ, -1);
	usleep(2*TimeDelay);

	return 0;
}


void motor_send(MotorStruct *Motor, int SendMode) {
/* Fonction utilitaire pour simplifier les transmissions aux moteurs */

	switch (SendMode) {
	case MOTOR_NONE : 		break;
	case MOTOR_PWM_ONLY :	/* A faire! */
							break;
	case MOTOR_LED_ONLY :	/* A faire! */
							break;
	case MOTOR_PWM_LED :	/* A faire! */
							break;
	}
}


void *MotorTask ( void *ptr )
{
/* A faire! */
/* Tache qui transmet les nouvelles valeurs de vitesse */
/* à chaque moteur à interval régulier (5 ms).         */
	struct timespec Delai;
	MotorStruct *Motor = (MotorStruct*)ptr;
	clock_gettime(CLOCK_REALTIME, &Delai);

	MotorActivated = 1;
	while (MotorActivated)
	{
		Delai.tv_nsec += SleepDelayNanos;
		if(Delai.tv_nsec >= 1000000000)
		{
			Delai.tv_sec += 1;
			Delai.tv_nsec -= 1000000000;
		}
		clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &Delai, NULL);
//		printf("MotorTask time: %d seconds and %d ns\n", Delai.tv_sec, Delai.tv_nsec);
//		SetPWM(Motor->file, 500 , 500 , 500 , 500);
		SetLed(Motor->file,0xF,0);

	}
	pthread_exit(0); /* exit thread */
}


int MotorInit (MotorStruct *Motor) {
/* A faire! */
/* Ici, vous devriez faire l'initialisation des moteurs.   */
/* C'est-à-dire initialiser le Port des moteurs avec la    */
/* fonction MotorPortInit() et créer la Tâche MotorTask()  */
/* qui va s'occuper des mises à jours des moteurs en cours */ 
/* d'exécution.                                            */
	int result;

	MotorPortInit(Motor);
//	if(result == 0)
//	{
		pthread_create(&Motor->MotorThread, PTHREAD_CREATE_JOINABLE, MotorTask, (void *)Motor);
		printf("MotorTask created...\n");
//	}
//	else
//		return result;

	return 0;
}



int MotorStart (void) {
	int retval = -1;
/* A faire! */
/* Ici, vous devriez démarrer la mise à jour des moteurs (MotorTask).    */ 
/* Tout le système devrait être prêt à faire leur travail et il ne reste */
/* plus qu'à tout démarrer.                                              */
	return retval;
}



int MotorStop (MotorStruct *Motor) {
/* A faire! */
/* Ici, vous devriez arrêter les moteurs et fermer le Port des moteurs. */ 
	return 0;
}

