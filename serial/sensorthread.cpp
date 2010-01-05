/*

*/

#include "sensorthread.h"

void* runSensorThread(void* arg)
{
    struct timespec nextRunTime, wakeTime, startTime, preTime, postTime;              // The absolute time for the main thread to be executed
    clock_gettime(CLOCK_REALTIME, &nextRunTime);                            // Initialise the next run time to be now
    clock_gettime(CLOCK_REALTIME, &startTime);                              // Initialise the next run time to be now
    
    Motors* serial = (Motors*) arg;
    
    int err;
    do 
    {
        clock_gettime(CLOCK_REALTIME, &preTime);
        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &nextRunTime, NULL);
        clock_gettime(CLOCK_REALTIME, &postTime);
        cout << "SENSORTHREAD: I slept for " << (postTime.tv_nsec - preTime.tv_nsec)/1e6 + (postTime.tv_sec - preTime.tv_sec)*1e3 << "ms." << endl;
        
        // request feedback from motors
        serial->request();
        
        // sleep until it is safe to read
        clock_gettime(CLOCK_REALTIME, &wakeTime);    // Initialise the next run time to be now
        // calculation of next run time
        wakeTime.tv_nsec += 1e9*5e-3;            // between 6 and 10ms is required before I get data
        if (wakeTime.tv_nsec > 1e9)              // we need to be careful with the nanosecond clock overflowing...
        {
            wakeTime.tv_sec += 1;
            wakeTime.tv_nsec -= 1e9;
        }
        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &wakeTime, NULL);
        
        // must wait for control to finish calculating response to previous sensor data before overwriting feedback data with new data
        clock_gettime(CLOCK_REALTIME, &preTime);
        err = sem_wait(&ControlReadySemaphore); 
        clock_gettime(CLOCK_REALTIME, &postTime);
        cout << "SENSORTHREAD: I had to wait " << (postTime.tv_nsec - preTime.tv_nsec)/1e6 + (postTime.tv_sec - preTime.tv_sec)*1e3 << "ms for control to finish." << endl;
        
        // read from serial buffer and put in feedback arrays
        serial->read();
        // copy write buffer to write arrays 
        
        // Tell control that it can now start calculating a response to the new sensor data
        sem_post(&NewSensorDataSemaphore);
        
        // write data to serial (new positions followed by request for status)
        serial->write();
        // calculation of next run time
        nextRunTime.tv_nsec += 1e6*SENSORTHREAD_PERIOD;
        if (nextRunTime.tv_nsec > 1e9)              // we need to be careful with the nanosecond clock overflowing...
        {
            nextRunTime.tv_sec += 1;
            nextRunTime.tv_nsec -= 1e9;
        }
    } 
    while (err != EINTR);
}
