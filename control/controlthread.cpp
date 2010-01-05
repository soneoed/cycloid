/*

*/

#include "controlthread.h"

void* runControlThread(void* arg)
{
    struct timespec startTime, preTime, postTime;              // The absolute time for the main thread to be executed
    clock_gettime(CLOCK_REALTIME, &startTime);                              // Initialise the next run time to be now
   
    CMotion* Motion = (CMotion*) arg;
    
    unsigned char teston[12] = {10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21};
    unsigned char testoff[11] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 22};
    Motion->torqueOn(testoff, 11);
    Motion->torqueOn(teston, 12);
    
    float target = 10;
    float speed = 0;
    int count = 0;
    
    // variables to hold the current S-curve
    bool firstrun = true;
    float leftpositions[500];
    float rightpositions[500];
    float leftspeeds[500];
    float rightspeeds[500];
    int times[500];
    float curveduration = 1;
    int curveindex = 500;
    int curvelength = 0;
    
    int err;
    do 
    {
        cout << "CONTROLTHREAD: Waiting for new sensor data." << endl;
        err = sem_wait(&NewSensorDataSemaphore); 
        cout << "CONTROLTHREAD: Running control." << endl;
        clock_gettime(CLOCK_REALTIME, &preTime);
       
        float endpoint;
        // implement S-curve
        if (curveindex >= curvelength)
        {   // it is time to calculate a new S-curve
            if (firstrun == true)
            {
                endpoint = 1013;
                calculateSCurve(times, leftpositions, leftspeeds, 0, curveduration, JointPositions[2], endpoint, 0, 0, 500);
                calculateSCurve(times, rightpositions, rightspeeds, 0, curveduration, JointPositions[3], endpoint, 0, 0, 500);
                firstrun = false;
            }
            else
            {
                if (fabs(leftpositions[curvelength-1] - 1013) < 10)
                    endpoint = 10;
                else
                    endpoint = 1013;
                calculateSCurve(times, leftpositions, leftspeeds, 0, curveduration, JointPositions[2], endpoint, 0, 0, 500);
                calculateSCurve(times, rightpositions, rightspeeds, 0, curveduration, JointPositions[3], endpoint, 0, 0, 500);
            }
            curvelength = (curveduration/0.02);
            curveindex = 0;
        }

        Motion->Serial->updateControl(2, leftpositions[curveindex], -2/*fabs(leftspeeds[curveindex])*0.5*/, -1);
        Motion->Serial->updateControl(3, endpoint, fabs(rightspeeds[curveindex])*0.5, -1);
        
        curveindex++;
        count++;
        
        clock_gettime(CLOCK_REALTIME, &postTime);
        cout << "CONTROLTHREAD: I spent " << (postTime.tv_nsec - preTime.tv_nsec)/1e6 + (postTime.tv_sec - preTime.tv_sec)*1e3 << "ms working." << endl;
        sem_post(&ControlReadySemaphore);
    } 
    while (err != EINTR);
}

/*! Calculates a very nice s-curve. 
 Features:
 It has a continuous second derivative no matter what the input values. The acceleration profile is triangular.
 It will smoothly satisfy the initial and final velocity constraints
 
 @param jointIndex  the joint index the position curve is being calculated for
 @param times       an array which will be updated to contain the times for the motion curve
 @param positions   an array which will be updated to contain the positions of the motion curve
 @param t0          the time at which the motion curve will begin (seconds). Zero being now, and 1s being 1s ahead of now
 @param tf          the time at which the motion curve must reached its target (seconds)
 @param g0          the starting position for the motion curve (radians)
 @param gf          the target position (radians)
 @param v0          the starting velcoity for the motion curve (radians/second)
 @param vf          the target velocity (radians/second)
 
 Known shortcoming: the acceleration must be zero at 0.5T. This problem arises when a unidirectional acceleration would be desirable. 
 The curve still satisifies the end points, but has unnecessary jerk in the middle.
 */
void calculateSCurve(int times[], float positions[], float speeds[], float t0, float tf, float g0, float gf, float v0, float vf, int curvelength)
{
    const float NUMOTION_TIMESTEP = 0.02;
    const int NUMOTION_MAX_ARRAY_LENGTH = 10000;
    float timeseconds[NUMOTION_MAX_ARRAY_LENGTH];
    
    /*       Acceleration Profile
     As___________
     /|\
     / | \
     /  |  \
     /   |   \
     --     |    -       --
     |    |    |\     /
     |    |    | \   / |
     |    |    |  \ /  |_____________Af
     |    |    |   |   |
     |    |    |   |   |
     t0   t1   t2  t3  t4
     */
    // Calculate times
    tf = tf - t0;                 // tf is now the length of time the motion curve will last (add t0 at the end to get the timing right again)
    curvelength = (int) tf/NUMOTION_TIMESTEP;
    
    float t1 = 0.5*0.5*tf;
    float t2 = 0.5*tf; 
    float t3 = t2 + 0.5*0.5*tf;
    
#ifdef NUMOTION_VERBOSITY
    cout << "NUMOTION: Calculating S Motion Curve.";
    cout << "motionlength: " << curvelength << " t1: " << t1 << " t2: " << t2 << " t3: " << t3 << " NUMOTION_MAX_ARRAY_LENGTH: " << NUMOTION_MAX_ARRAY_LENGTH << endl;
#endif
    
    for (int i=0; i < curvelength; i++)
    {
        timeseconds[i] = i*NUMOTION_TIMESTEP;
        times[i] = (int)(i*NUMOTION_TIMESTEP*1000 + t0*1000);      //
    }
    
    // Calculate acceleration peaks
    float As = 2*(-3*gf*tf*t2*t2 + 3*gf*tf*tf*t2 + 3*g0*tf*t2*t2 - 3*gf*tf*tf*t3 + 6*gf*tf*t3*t3 + 3*gf*t3*t2*t2 + 2*tf*tf*tf*vf*t3 - 2*tf*tf*tf*vf*t2 + tf*tf*tf*v0*t3 - tf*tf*tf*v0*t2 + 10*t3*t3*t3*vf*tf + 2*t3*t3*t3*vf*t2 - 4*t3*t3*t3*v0*tf - 2*t3*t3*t3*v0*t2 - 6*vf*tf*tf*t3*t3 + 3*vf*tf*tf*t2*t2 - t2*t2*t2*vf*tf + t2*t2*t2*vf*t3 + t2*t2*t2*v0*tf - t2*t2*t2*v0*t3 + 3*g0*tf*tf*t3 - 3*g0*tf*tf*t2 - 6*g0*tf*t3*t3 - 3*g0*t3*t2*t2 - 3*tf*vf*t3*t2*t2 - 6*t3*t3*t3*t3*vf + 6*t3*t3*t3*t3*v0 - 6*gf*t3*t3*t3 + 6*g0*t3*t3*t3)/t2/(tf*tf*t3*t1 - tf*tf*t2*t1 - 2*tf*t3*t3*t1 + 2*t3*t3*t3*t1 + tf*t2*t2*t1 - t3*t2*t2*t1 - 6*t3*t3*t3*t3 + tf*tf*tf*t2 + 4*t3*t3*t3*t2 - tf*tf*tf*t3 + 4*t3*t3*t3*tf + tf*tf*t3*t2 - tf*tf*t2*t2 - 2*tf*t3*t3*t2);
    float Af = (-2*vf + 2*v0 + As*t2)*(tf - t3)*(t3 - t2)/(tf*tf*t3 -tf*tf*t2 - 2*tf*t3*t3 + 2*t3*t3*t3 + tf*t2*t2 - t3*t2*t2);
    
    // Calculate the position curve
    float t;
    for (int i=0; i < curvelength; i++)
    {
        t = timeseconds[i]*1.0;
        if (t <= t1)
            positions[i] = (As/(6.0*t1))*t*t*t + v0*t + g0;
        else if (t <= t2)
            positions[i] = -(As/(6.0*(t2 - t1)))*t*t*t + (As*t2/(2.0*(t2 - t1)))*t*t + (v0 - 0.5*As*t1*t2/(t2 - t1))*t + g0 + (As*t1*t1/6.0) + As*t1*t1*t1/(6.0*(t2 - t1));
        else if (t <= t3)
            positions[i] = -(Af/(6.0*(t3 - t2)))*t*t*t + (Af*t2/(2.0*(t3 - t2)))*t*t + (v0 + 0.5*As*t2 - Af*t2*t2/(2.0*(t3 - t2)))*t + g0 + (As*t1*t1/6.0) - 0.5*As*t2*t2 + As*t1*t1*t1/(6.0*(t2 - t1)) + As*t2*t2*t2/(3.0*(t2 - t1)) + Af*t2*t2*t2/(6.0*(t3 - t2)) - As*t1*t2*t2/(2.0*(t2 - t1));
        else
            positions[i] = (Af/(6.0*(tf - t3)))*t*t*t - 0.5*(Af*t3/(tf - t3) + Af)*t*t + (v0 + 0.5*As*t2 + Af*t3 + (Af*t2/(t3 - t2))*(t3 - 0.5*t2))*t + g0 + (As*t1*t1/6.0) - 0.5*As*t2*t2 - 0.5*Af*t3*t3 + As*t1*t1*t1/(6.0*(t2 - t1)) + As*t2*t2*t2/(3.0*(t2 - t1)) + Af*t2*t2*t2/(6.0*(t3 - t2)) + Af*t3*t3*t3/(3.0*(tf - t3)) - Af*t3*t3*t3/(6.0*(t3 - t2)) - As*t1*t2*t2/(2.0*(t2 - t1)) - Af*t2*t3*t3/(2.0*(t3 - t2));
    }
    
    // Calculate the velocity curve
    for (int i=0; i < curvelength; i++)
    {
        t = timeseconds[i]*1.0;
        if (t <= t1)
            speeds[i] = 0.5*(As/t1)*t*t + v0;
        else if (t <= t2)
            speeds[i] = -(As/(2.0*(t2 - t1)))*t*t + (As*t2/(t2 - t1))*t + v0 - 0.5*As*t1*t2/(t2 - t1);
        else if (t <= t3)
            speeds[i] = -(Af/(2.0*(t3 - t2)))*t*t + (Af*t2/(t3 - t2))*t + v0 + 0.5*As*t2 - Af*t2*t2/(2.0*(t3 - t2));
        else
            speeds[i] = (Af/(2.0*(tf - t3)))*t*t - (Af*t3/(tf - t3) + Af)*t + v0 + 0.5*As*t2 + Af*t3 + (Af*t2/(t3 - t2))*(t3 - 0.5*t2);
    }
    
    // print the curve for debug purposes
    float a[NUMOTION_MAX_ARRAY_LENGTH];
    float v[NUMOTION_MAX_ARRAY_LENGTH];
    // Calculate the accereration profile
    for (int i=0; i < curvelength; i++)
    {
        t = timeseconds[i]*1.0;
        if (t <= t1)
            a[i] = (As/t1)*t;
        else if (t <= t2)
            a[i] = (-As/(t2 - t1))*(t - t1) + As;
        else if (t <= t3)
            a[i] = (-Af/(t3 - t2))*(t - t2);
        else
            a[i] = (Af/(tf - t3))*(t - t3) - Af;
    }
    
    cout << "t0: " << t0 << ", tf: " << tf << ", g0: " << g0 << ", gf: " << gf << ", v0: " << v0 << ", vf: " << vf << endl;
    cout << "t (s), t (ms), a, v, g" << endl;
    for (int i=0; i < curvelength; i++)
    {
        cout << timeseconds[i] << ", " << times[i] << ", " << a[i] << ", " << speeds[i] << ", " << positions[i] << endl;
    }
    
    return;
}


