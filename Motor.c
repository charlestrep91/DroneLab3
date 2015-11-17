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
#define MotorTaskPriority	1
#define POLICY SCHED_RR

#define WRITE_PWM_CMD 0b001
#define WRITE_LED_CMD 0b011

extern sem_t	MotorTimerSem;
extern int		MotorActivated;
int pwmVal=0;

pthread_barrier_t 	MotorStartBarrier;

void SetPWM(int file, uint16_t *pwm)
{
	unsigned char cmd[5];

	cmd[0] = 0x20|((pwm[0]&0x1ff)>>4);
	cmd[1] = ((pwm[0]&0x1ff)<<4) | ((pwm[1]&0x1ff)>>5);
	cmd[2] = ((pwm[1]&0x1ff)<<3) | ((pwm[2]&0x1ff)>>6);
	cmd[3] = ((pwm[2]&0x1ff)<<2) | ((pwm[3]&0x1ff)>>7);
	cmd[4] = ((pwm[3]&0x1ff)<<1);
    write(file,cmd,5);
}

void SetLed(int file, u_int16_t * led)
{
	unsigned char cmd[2];
	unsigned char i;
	unsigned char red=0;
	unsigned char grn=0;

	for (i=0;i<4;i++)
	{
		if(led[i]&MOTOR_LEDRED)
			red|=0x1<<i;
		if(led[i]&MOTOR_LEDGREEN)
			grn|=0x1<<i;
	}

	cmd[0] = 0x60 | ((red&0x0f)<<1);
	cmd[1] = ((grn&0x0f)<<1);
    write(file,cmd,2);
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


void motor_send(MotorStruct *Motor, int SendMode)
{
/* Fonction utilitaire pour simplifier les transmissions aux moteurs */
	switch (SendMode)
	{
		case MOTOR_NONE :

		break;

		case MOTOR_PWM_ONLY :
			SetPWM( Motor->file, Motor->pwm);
		break;

		case MOTOR_LED_ONLY :
			SetLed( Motor->file, Motor->led);
		break;

		case MOTOR_PWM_LED :
			SetPWM( Motor->file, Motor->pwm);
			SetLed( Motor->file, Motor->led);
		break;
	}
}



void *MotorTask ( void *ptr )
{
/* A faire! */
/* Tache qui transmet les nouvelles valeurs de vitesse */
/* à chaque moteur à interval régulier (5 ms).*/
	struct timespec Delai;
	uint16_t	led[4] = {MOTOR_LEDGREEN, MOTOR_LEDGREEN, MOTOR_LEDGREEN, MOTOR_LEDGREEN};
	MotorStruct *Motor = (MotorStruct*)ptr;
	pthread_barrier_wait(&(MotorStartBarrier));
	while (MotorActivated)
	{
		sem_wait(&MotorTimerSem);
		pthread_spin_lock(&(Motor->MotorLock));
		motor_send(Motor,MOTOR_PWM_ONLY);
		pthread_spin_unlock(&(Motor->MotorLock));
		SetLed(Motor->file, led);

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
	int minprio,maxprio;
	struct sched_param param;
	pthread_attr_t attr;
	sem_init(&MotorTimerSem,0,0);
	result=MotorPortInit(Motor);

	if(result == 0)
	{	pthread_attr_init(&attr);
		pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
		pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
		pthread_attr_setscope(&attr, PTHREAD_SCOPE_SYSTEM);
		minprio = sched_get_priority_min(POLICY);
		maxprio = sched_get_priority_max(POLICY);
		pthread_attr_setschedpolicy(&attr, POLICY);
		param.sched_priority = minprio + (maxprio - minprio)/2;
		pthread_attr_setstacksize(&attr, THREADSTACK);
		pthread_attr_setschedparam(&attr, &param);
		pthread_barrier_init(&MotorStartBarrier, NULL, 2);
		pthread_create(&Motor->MotorThread, &attr, MotorTask, (void *)Motor);
		printf("MotorThread created...");
	}
	else
		return result;

	return 0;
}



int MotorStart (void) {
/* A faire! */
/* Ici, vous devriez démarrer la mise à jour des moteurs (MotorTask).    */ 
/* Tout le système devrait être prêt à faire leur travail et il ne reste */
/* plus qu'à tout démarrer.   */
	MotorActivated = 1;
	pthread_barrier_wait(&(MotorStartBarrier));
	pthread_barrier_destroy(&MotorStartBarrier);
	printf("%s MotorStart démarré\n", __FUNCTION__);
	return 0;
}



int MotorStop (MotorStruct *Motor) {
/* A faire! */
/* Ici, vous devriez arrêter les moteurs et fermer le Port des moteurs. */ 
	MotorActivated = 0;
	sem_destroy(&MotorTimerSem);
	if(close(Motor->file)!=0)
		return -1;
	else
		return  0;
}

