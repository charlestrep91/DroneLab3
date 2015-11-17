/*
 * Sensor.c
 *
 *  Created on: 12 sept. 2013
 *      Author: bruno
 */

#include "Sensor.h"

#define ABS(x) (((x) < 0.0) ? -(x) : (x))

#define MAX_TOT_SAMPLE 1000
#define RATE_NANOS 5000000		//number of nanoseconds per rate
#define POLICY SCHED_RR

extern SensorStruct	SensorTab[NUM_SENSOR];

pthread_barrier_t   SensorStartBarrier;
pthread_barrier_t   LogStartBarrier;
pthread_mutex_t 	  Log_Mutex;

extern uint8_t  SensorsActivated;
uint8_t  LogActivated  	  	= 0;
uint8_t  numLogOutput 	  	= 0;

/*
 * Tache generique pour la lecture des capteurs ansi que leur conversion.
 * Informe les autres taches lorsqu'une nouvelle donnee a ete convertie.
 */
void *SensorTask ( void *ptr )
{
	SensorStruct *Sensor = (SensorStruct*)ptr;
	int result;
	int i;
	double tempData[3];
	uint32_t tempTimeDelay;

	printf("SensorTask: %s task running...\n",Sensor->Name);
	pthread_barrier_wait(&SensorStartBarrier);							//on attend a la barriere pour un start

	while(SensorsActivated)
	{
		sem_wait(&Sensor->DataSem);																	//attend le semaphore qui gere le timing dans DroneFirmware.c
		pthread_spin_lock(&Sensor->DataLock);														//capture le spinlock pour que les donnees ne soient pas modifiees par une autre tache
		result = read(Sensor->File, &(Sensor->RawData[Sensor->DataIdx]), sizeof(SensorRawData));	//fait la lecture des donnees brutes du sensor et les place dans le tableau RawData
		pthread_spin_unlock(&Sensor->DataLock);														//libere le spinlock

		if(result == sizeof(SensorRawData))															//verifie si la structure complete a bien ete copiee
		{
			if(Sensor->RawData[Sensor->DataIdx].status == NEW_SAMPLE)								//verifie si la donnee est nouvelle
			{
				//check if RawData samples have been missed
				//TODO check sample #
//				if((Sensor->RawData[Sensor->DataIdx].ech_num - Sensor->RawData[(Sensor->DataIdx - 1)].ech_num) != 1)
//					printf("SensorTask: RawData sample missed: Old sample: #%u New sample: #%u Offset: %u\n", Sensor->RawData[(Sensor->DataIdx - 1)].ech_num, Sensor->RawData[Sensor->DataIdx].ech_num, (Sensor->RawData[Sensor->DataIdx].ech_num - Sensor->RawData[(Sensor->DataIdx - 1)].ech_num));

				for(i=0; i<3; i++)
				{
					tempData[i] = (double)Sensor->RawData[Sensor->DataIdx].data[i];					//fait une copie dans une variable locale
					tempData[i] = tempData[i] * Sensor->Param->Conversion;							//applique le facteur de conversion du sensor
				}

				//calcule le delai entre deux echantillons en utilisant les timestamps
				tempTimeDelay = ((Sensor->RawData[Sensor->DataIdx].timestamp_s - Sensor->RawData[(Sensor->DataIdx - 1)].timestamp_s) * 1000000000) + (Sensor->RawData[Sensor->DataIdx].timestamp_n - Sensor->RawData[(Sensor->DataIdx - 1)].timestamp_n);

				pthread_spin_lock(&Sensor->DataLock);												//capture le spinlock
				Sensor->Data[Sensor->DataIdx].Data[0] = tempData[0];								//met a jour les donnees converties du sensor
				Sensor->Data[Sensor->DataIdx].Data[1] = tempData[1];
				Sensor->Data[Sensor->DataIdx].Data[2] = tempData[2];
				Sensor->Data[Sensor->DataIdx].TimeDelay = tempTimeDelay;							//met a jour le delai entre les echantillons
				pthread_spin_unlock(&Sensor->DataLock);												//libere le semaphore

//				if((Sensor->DataIdx % 25) == 0 && Sensor->type == ACCELEROMETRE)					//sert a imprimer une donnee
//				{
//					printf("SensorTask: TimeDelay# = %u\n", tempTimeDelay);
//					printf("SensorTask: 	Data0: %f	Data1: %f 	Data2: %f\n", Sensor->Data[Sensor->DataIdx].Data[0], Sensor->Data[Sensor->DataIdx].Data[1], Sensor->Data[Sensor->DataIdx].Data[2]);
//					printf("SensorTask: 	Data0: %d	Data1: %d 	Data2: %d\n", Sensor->RawData[Sensor->DataIdx].data[0], Sensor->RawData[Sensor->DataIdx].data[1], Sensor->RawData[Sensor->DataIdx].data[2]);
//				}

				Sensor->DataIdx = ((Sensor->DataIdx+1) + DATABUFSIZE) % DATABUFSIZE;				//incremente l'index du buffer de donnees

				pthread_mutex_lock(&(Sensor->DataSampleMutex));										//capture le mutex
				pthread_cond_broadcast(&(Sensor->DataNewSampleCondVar));							//informe les autres taches qu'une nouvelle donnee a ete convertie
				pthread_mutex_unlock(&(Sensor->DataSampleMutex));									//libere le mutex
			}
			else if(Sensor->RawData[Sensor->DataIdx].status == NEW_SAMPLE)
			{
				printf("SensorTask: RawData #%u has an invalid checksum\n", Sensor->RawData[Sensor->DataIdx].ech_num);
			}
		}
		else
		{
			printf("SensorTask: invalid data struct size copy\n");
		}

	}
	pthread_exit(0); /* exit thread */
}

/*
 * Creation des taches de chaque capteur ainsi que l'initialisation de leurs
 * spinlocks et semaphores.
 */
int SensorsInit (SensorStruct SensorTab[NUM_SENSOR])
{
	int i;
	int minprio,maxprio;
	struct sched_param param;
	pthread_attr_t attr;

	printf("SensorInit...\n");
	pthread_barrier_init(&SensorStartBarrier, NULL, NUM_SENSOR+1);

	//configuration des attribus de la tache
	pthread_attr_init(&attr);
	pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
	pthread_attr_setscope(&attr, PTHREAD_SCOPE_SYSTEM);
	minprio = sched_get_priority_min(POLICY);
	maxprio = sched_get_priority_max(POLICY);
	pthread_attr_setschedpolicy(&attr, POLICY);
	param.sched_priority = minprio + (maxprio - minprio)/2;
	pthread_attr_setstacksize(&attr, THREADSTACK);
	pthread_attr_setschedparam(&attr, &param);

	for(i=0; i<NUM_SENSOR; i++)																	//Selon le nombre de capteurs, cree une tache pour chacun d'eux
	{
		SensorTab[i].File = open(SensorTab[i].DevName, O_RDONLY);								//attribue le fichier de lecture du capteur a sa structure
		if (SensorTab[i].File < 0)
		{
			printf("SensorInit: Impossible d'ouvrir le fichier %s\n",SensorTab[i].DevName);
			return SensorTab[i].File;
		}
	sem_init(&SensorTab[i].DataSem,0,0);														//initialisation du semaphore du capteur
		pthread_spin_init(&SensorTab[i].DataLock,0);											//initialisation du spinlock du capteur
		pthread_create(&SensorTab[i].SensorThread, &attr, SensorTask, (void *)&SensorTab[i]);	//creation de la tache du capteur
		SensorTab[i].DataIdx = 0;
	}
	printf("SensorInit finished...\n");

	return 0;
};

/*
 * Active la variable d'activation des capteurs.
 * Attends les taches des capteurs a l'aide de la barriere.
 * Detruit la barriere.
 */
int SensorsStart (void) {

	SensorsActivated = 1;
	pthread_barrier_wait(&SensorStartBarrier);
	pthread_barrier_destroy(&SensorStartBarrier);
	printf("%s Sensors démarré\n", __FUNCTION__);
	return 0;
}

/*
 * Desactive la variable d'activation des capteurs.
 * Pour chaque capteur, detruit le spinlock et le semaphore
 * puis ferme le fichier de lecture du capteur.
 */
int SensorsStop (SensorStruct SensorTab[NUM_SENSOR]) {
	int i, err;

	err = 0;
	SensorsActivated = 0;
	for(i=0; i<NUM_SENSOR; i++)
	{
		pthread_spin_destroy(&SensorTab[i].DataLock);
		sem_destroy(&SensorTab[i].DataSem);
		err += close(SensorTab[i].File);
	}
	if(err != 0)
		return -1;
	else
		return 0;
}



/* Le code ci-dessous est un CADEAU !!!	*/
/* Ce code permet d'afficher dans la console les valeurs reçues des capteurs.               */
/* Évidemment, celà suppose que les acquisitions sur les capteurs fonctionnent correctement. */
/* Donc, dans un premier temps, ce code peut vous servir d'exemple ou de source d'idées.     */
/* Et dans un deuxième temps, peut vous servir pour valider ou vérifier vos acquisitions.    */
/*                                                                                           */
/* NOTE : Ce code suppose que les échantillons des capteurs sont placés dans un tampon       */
/*        circulaire de taille DATABUFSIZE, tant pour les données brutes (RawData) que       */
/*        les données converties (NavData) (voir ci-dessous)                                 */
void *SensorLogTask ( void *ptr ) {
	SensorStruct	*Sensor    = (SensorStruct *) ptr;
	uint16_t		*Idx       = &(Sensor->DataIdx);
	uint16_t		LocalIdx   = DATABUFSIZE;
	SensorData	 	*NavData   = NULL;
	SensorRawData	*RawData   = NULL;
	SensorRawData   tpRaw;
	SensorData 	    tpNav;
	double			norm;

	printf("%s : Log de %s prêt à démarrer\n", __FUNCTION__, Sensor->Name);
	pthread_barrier_wait(&(LogStartBarrier));

	while (LogActivated) {
		pthread_mutex_lock(&(Sensor->DataSampleMutex));
		while (LocalIdx == *Idx)
			pthread_cond_wait(&(Sensor->DataNewSampleCondVar), &(Sensor->DataSampleMutex));
	    pthread_mutex_unlock(&(Sensor->DataSampleMutex));

	   	pthread_spin_lock(&(Sensor->DataLock));
    	NavData   = &(Sensor->Data[LocalIdx]);
    	RawData   = &(Sensor->RawData[LocalIdx]);
		memcpy((void *) &tpRaw, (void *) RawData, sizeof(SensorRawData));
		memcpy((void *) &tpNav, (void *) NavData, sizeof(SensorData));
	   	pthread_spin_unlock(&(Sensor->DataLock));

	   	pthread_mutex_lock(&Log_Mutex);
		if (numLogOutput == 0)
			printf("Sensor  :     TimeStamp      SampleDelay  Status  SampleNum   Raw Sample Data  =>        Converted Sample Data               Norme\n");
		else switch (tpRaw.type) {
				case ACCELEROMETRE :	norm = sqrt(tpNav.Data[0]*tpNav.Data[0]+tpNav.Data[1]*tpNav.Data[1]+tpNav.Data[2]*tpNav.Data[2]);
										printf("Accel   : (%5u.%09d)-(0.%09u)  %2d     %8u   %04X  %04X  %04X  =>  %10.5lf  %10.5lf  %10.5lf  =  %10.5lf\n", tpRaw.timestamp_s, tpRaw.timestamp_n, tpNav.TimeDelay, tpRaw.status, tpRaw.ech_num, (uint16_t) tpRaw.data[0], (uint16_t) tpRaw.data[1], (uint16_t) tpRaw.data[2], tpNav.Data[0], tpNav.Data[1], tpNav.Data[2], norm);
										break;
				case GYROSCOPE :		norm = sqrt(tpNav.Data[0]*tpNav.Data[0]+tpNav.Data[1]*tpNav.Data[1]+tpNav.Data[2]*tpNav.Data[2]);
										printf("Gyro    : (%5u.%09d)-(0.%09u)  %2d     %8u   %04X  %04X  %04X  =>  %10.5lf  %10.5lf  %10.5lf  =  %10.5lf\n", tpRaw.timestamp_s, tpRaw.timestamp_n, tpNav.TimeDelay, tpRaw.status, tpRaw.ech_num, (uint16_t) tpRaw.data[0], (uint16_t) tpRaw.data[1], (uint16_t) tpRaw.data[2], tpNav.Data[0], tpNav.Data[1], tpNav.Data[2], norm);
										break;
				case SONAR :			printf("Sonar   : (%5u.%09d)-(0.%09u)  %2d     %8u   %04X              =>  %10.5lf\n", tpRaw.timestamp_s, tpRaw.timestamp_n, tpNav.TimeDelay, tpRaw.status, tpRaw.ech_num, (uint16_t) tpRaw.data[0], tpNav.Data[0]);
										break;
				case BAROMETRE :		printf("Barom   : (%5u.%09d)-(0.%09u)  %2d     %8u   %04X              =>  %10.5lf\n", tpRaw.timestamp_s, tpRaw.timestamp_n, tpNav.TimeDelay, tpRaw.status, tpRaw.ech_num, (uint16_t) tpRaw.data[0], tpNav.Data[0]);
										break;
				case MAGNETOMETRE :		norm = sqrt(tpNav.Data[0]*tpNav.Data[0]+tpNav.Data[1]*tpNav.Data[1]+tpNav.Data[2]*tpNav.Data[2]);
										printf("Magneto : (%5u.%09d)-(0.%09u)  %2d     %8u   %04X  %04X  %04X  =>  %10.5lf  %10.5lf  %10.5lf  =  %10.5lf\n", tpRaw.timestamp_s, tpRaw.timestamp_n, tpNav.TimeDelay, tpRaw.status, tpRaw.ech_num, (uint16_t) tpRaw.data[0], (uint16_t) tpRaw.data[1], (uint16_t) tpRaw.data[2], tpNav.Data[0], tpNav.Data[1], tpNav.Data[2], norm);
										break;
			 }
		LocalIdx = *Idx;
		numLogOutput++;
		if (numLogOutput > 20)
			numLogOutput = 0;
		pthread_mutex_unlock(&Log_Mutex);
	}

	printf("%s : %s Terminé\n", __FUNCTION__, Sensor->Name);

	pthread_exit(0); /* exit thread */
}


int InitSensorLog (SensorStruct *Sensor) {
	pthread_attr_t		attr;
	struct sched_param	param;
	int					retval;

	pthread_attr_init(&attr);
	pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
	pthread_attr_setscope(&attr, PTHREAD_SCOPE_PROCESS);
	pthread_attr_setschedpolicy(&attr, POLICY);
	param.sched_priority = sched_get_priority_min(POLICY);
	pthread_attr_setstacksize(&attr, THREADSTACK);
	pthread_attr_setschedparam(&attr, &param);

	printf("Creating Log thread : %s\n", Sensor->Name);
	if ((retval = pthread_create(&(Sensor->LogThread), &attr, SensorLogTask, (void *) Sensor)) != 0)
		printf("%s : Impossible de créer Tâche Log de %s => retval = %d\n", __FUNCTION__, Sensor->Name, retval);

	pthread_mutex_init(&Sensor->DataSampleMutex, NULL);
	pthread_cond_init(&Sensor->DataNewSampleCondVar, NULL);
	pthread_attr_destroy(&attr);

	return 0;
}


int SensorsLogsInit (SensorStruct SensorTab[]) {
	int16_t	  i, numLog = 0;
	int16_t	  retval = 0;

	for (i = 0; i < NUM_SENSOR; i++) {
		if (SensorTab[i].DoLog == 1) {
			if ((retval = InitSensorLog(&SensorTab[i])) < 0) {
				printf("%s : Impossible d'initialiser log de %s => retval = %d\n", __FUNCTION__, SensorTab[i].Name, retval);
				return -1;
			}
			numLog++;
		}
	}
	pthread_barrier_init(&LogStartBarrier, NULL, numLog+1);
	pthread_mutex_init(&Log_Mutex, NULL);

	return 0;
};


int SensorsLogsStart (void) {
	LogActivated = 1;
	pthread_barrier_wait(&(LogStartBarrier));
	pthread_barrier_destroy(&LogStartBarrier);
	printf("%s NavLog démarré\n", __FUNCTION__);

	return 0;
};


int SensorsLogsStop (SensorStruct SensorTab[]) {
	int16_t	i;

	LogActivated = 0;
	for (i = 0; i < NUM_SENSOR; i++)
	{
		pthread_cond_broadcast(&SensorTab[i].DataNewSampleCondVar);
		if (SensorTab[i].DoLog == 1)
		{
			pthread_join(SensorTab[i].LogThread, NULL);
			SensorTab[i].DoLog = 0;
		}
		pthread_cond_init(&SensorTab[i].DataNewSampleCondVar, NULL);
		pthread_mutex_destroy(&SensorTab[i].DataSampleMutex);
	}
	pthread_mutex_destroy(&Log_Mutex);

	return 0;
};


