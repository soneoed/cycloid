/* CMotion
 
 by Jason Kulk (jason.555 at gmail dot com)
 
 Copyright (c) 2009 Jason Kulk 
 
 cmotion.h is part of Jason's Cycloid Code.
 
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
#ifndef CMOTION_H
#define CMOTION_H

#include "../main.h"
#include "../serial/motors.h"
#include "../serial/sensorthread.h"

#define MOTION_VERBOSITY                  2
 
class CMotion
{
   public:
      CMotion();
      ~CMotion();
   
      void torqueOn(unsigned char motorids[], unsigned char nummotors);
      void torqueOff(unsigned char motorids[], unsigned char nummotors);
   
   private:
      void initSensorThread();
    
   public:
      Motors* Serial;
    
   private:
      //Motors* Serial;
};
#endif
