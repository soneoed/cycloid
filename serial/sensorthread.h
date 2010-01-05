/*
*/

#ifndef SENSORTHREAD_H
#define SENSORTHREAD_H

#include "../main.h"
#include "motors.h"

#define SENSORTHREAD_PRIORITY       30
#define SENSORTHREAD_PERIOD         20
void* runSensorThread(void* arg);

#endif
