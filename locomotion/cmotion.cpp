/* CMotion
 
 by Jason Kulk (jason.555 at gmail dot com)
 
 Copyright (c) 2009 Jason Kulk 
 
 cmotion.cpp is part of Jason's Cycloid Code.
 
 Jason's Cycloid Code is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "cmotion.h"

CMotion::CMotion()
{
   // create the serial connection
   Serial = new Motors();
   initSensorThread();
    
}

void CMotion::initSensorThread()
{
   // create sensor thread
   cout << "MOTION: Motion(). Creating sensorthread as realtime, with priority " << SENSORTHREAD_PRIORITY << endl;
   int err;
   pthread_t sensorthread;
   err = pthread_create(&sensorthread, NULL, runSensorThread, Serial);
   if (err > 0)
   {
      cout << "MOTION: Motion(). Failed to create sensorthread. Error code: " << err << endl;
   }
   
   sched_param param;
   param.sched_priority = SENSORTHREAD_PRIORITY;
   pthread_setschedparam(sensorthread, SCHED_FIFO, &param);
   
   // double check
   int actualpolicy;
   sched_param actualparam;
   pthread_getschedparam(sensorthread, &actualpolicy, &actualparam);
   cout << "MOTION: Motion(). Actual settings for sensorthread; Policy: " << actualpolicy << " Priority: " << actualparam.sched_priority << endl;
}

void CMotion::torqueOn(unsigned char motorids[], unsigned char nummotors)
{
   Serial->torqueOn(motorids, nummotors);
}

void CMotion::torqueOff(unsigned char motorids[], unsigned char nummotors)
{
   Serial->torqueOff(motorids, nummotors);
}

CMotion::~CMotion()
{
   delete Serial;
}
