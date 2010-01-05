/*
*/

#ifndef CONTROLTHREAD_H
#define CONTROLTHREAD_H

#include "../main.h"
#include "../locomotion/cmotion.h"

#define CONTROLTHREAD_PRIORITY      20
void* runControlThread(void* arg);
void calculateSCurve(int times[], float positions[], float speeds[], float t0, float tf, float g0, float gf, float v0, float vf, int curvelength);

#endif
