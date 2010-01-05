/* 
*/

#include <time.h>
#include <math.h>
#include <iostream>
#include <fstream>
#include <string>
using namespace std;

#include <pthread.h>
#include <semaphore.h>
#include <errno.h>

// Declare external thread scheduling variables
extern sem_t NewSensorDataSemaphore;
extern sem_t ControlReadySemaphore;
