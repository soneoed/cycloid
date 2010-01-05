/*

*/

#include "main.h"
#include "locomotion/cmotion.h"
#include "control/controlthread.h"

// Thread scheduling variables (semaphores, mutexes, etc)
sem_t NewSensorDataSemaphore;
sem_t ControlReadySemaphore;

void createThreadScheduling();
void createControlThread(CMotion* motion);

int main()
{
   createThreadScheduling();
   
   CMotion* Motion = new CMotion();
   
   createControlThread(Motion);
   
   return 0;
}


void createThreadScheduling()
{
   // create and initialise thread scheduling tools
   int result = 0;
   result = sem_init(&NewSensorDataSemaphore, 0, 0);
   if (result == -1)
   {
      cout << "MAIN: Failed to initialise NewSensorDataSemaphore. Error code: " << errno << endl; 
      return;
   }
   result = sem_init(&ControlReadySemaphore, 0, 1);
   if (result == -1)
   {
      cout << "MAIN: Failed to initialise ControlReadySemaphore. Error code: " << errno << endl; 
      return;
   }
}

void createControlThread(CMotion* motion)
{
   // create control thread
   cout << "MAIN: Creating controlthread as realtime, with priority " << CONTROLTHREAD_PRIORITY << endl;
   
   int err;
   pthread_t controlthread;
   err = pthread_create(&controlthread, NULL, runControlThread, motion);         // the control thread needs the motion
   if (err > 0)
   {
      cout << "MAIN: Failed to create controlthread. Error code: " << err << endl;
      return;
   }
   
   sched_param param;
   param.sched_priority = CONTROLTHREAD_PRIORITY;
   pthread_setschedparam(controlthread, SCHED_FIFO, &param);
   
   // double check
   int actualpolicy;
   sched_param actualparam;
   pthread_getschedparam(controlthread, &actualpolicy, &actualparam);
   cout << "MAIN: Actual settings for controlthread; Policy: " << actualpolicy << " Priority: " << actualparam.sched_priority << endl;
   
   pthread_join(controlthread, NULL);
}
