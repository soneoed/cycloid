/* Motor serial
 
 by Jason Kulk (jason.555 at gmail dot com)
 
 Copyright (c) 2009 Jason Kulk 
 
 motors.h is part of Jason's Cycloid Code.
 
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
#ifndef MOTORS_H
#define MOTORS_H

#include "../main.h"
#include "ftd2xx.h"

#define MOTORS_DEBUG                      0
#define MOTORS_DATALOGGING                1
 
#define MOTORS_BAUD_RATE                  1000000

#define MOTORS_NUM_MOTORS                 23
#define MOTORS_NUM_LOWER_MOTORS           12
#define MOTORS_NUM_UPPER_MOTORS           11
#define MOTORS_NUM_MOTORS_PER_BLOCK       6                       // the number of motors in a bulk read_data message
#define MOTORS_NUM_LOWER_REQUEST_BLOCKS   (MOTORS_NUM_LOWER_MOTORS+(MOTORS_NUM_MOTORS_PER_BLOCK-1))/MOTORS_NUM_MOTORS_PER_BLOCK        // the number of bulk read_data messages required to poll all of the motors in the lower body
#define MOTORS_NUM_UPPER_REQUEST_BLOCKS   (MOTORS_NUM_UPPER_MOTORS+(MOTORS_NUM_MOTORS_PER_BLOCK-1))/MOTORS_NUM_MOTORS_PER_BLOCK

#define MOTORS_UPPER_BODY                 0
#define MOTORS_LOWER_BODY                 1

#define MOTORS_NUM_CONTROLS               1+4            // the number of bytes used to control the motors (this includes the write address(es))
#define MOTORS_NUM_PUNCHES                1+2            // the number of bytes used to change the 'punch' of the motors (this includes the write address)
#define MAX_MESSAGE_LENGTH                100            //

#define VEL_WINDOW_SIZE                   13

// Globally availiable feedback arrays (be aware that accessing them is inherently not thread-safe, but I do make an effort to not update them until control has finished)
extern long double JointTime;
extern long double PreviousJointTime;
extern unsigned short JointPositions[MOTORS_NUM_MOTORS];
extern unsigned short JointSpeeds[MOTORS_NUM_MOTORS];
extern unsigned short JointLoads[MOTORS_NUM_MOTORS];
extern float JointVelocities[MOTORS_NUM_MOTORS];

// This structure's sole purpose is to store data required for a threaded write
struct threaddata_t
{
   FT_HANDLE Handle;                // the handle to buffer will be written to
   unsigned char* Buffer;           // pointer to the buffer containing the data
   unsigned short BufferLength;     // the number of bytes in the buffer
};

class Motors
{
   public:
      Motors();
      ~Motors();
      void updateControl(unsigned char motorid, unsigned short position, unsigned short speed, unsigned short punch);
      void updateControls(unsigned char motorid[], unsigned char nummotors, unsigned short positions[], unsigned short speeds[], unsigned short punches[]);
      void updateControls(unsigned short positions[MOTORS_NUM_MOTORS], unsigned short speeds[MOTORS_NUM_MOTORS], unsigned short punches[MOTORS_NUM_MOTORS]);           // update the control variables (they will be sent to the motors on the next cycle)
      bool write();                                                                       // write motor control commands
      bool request();                                                                     // request for feedback data
      bool read();                                                                        // read feedback data and put it into global feedback arrays
    
      void torqueEnable();                                                                // turn on all motors
      void emergencyOff();                                                                // emergency off (fast, collapse in heap)
      void torqueOn(unsigned char motorid[], unsigned char nummotors);                    // turn the motors on (and start sending motor controls)
      void torqueOff(unsigned char motorid[], unsigned char nummotors);                   // turn the motors off (and stop sending motor controls)
      
   private:
      // Initialisation
      void initSelf();
      void initRequestMessages();
      void initSerial();
      void initReturnDelays();
      void initControlTables();
      void initSlopes();
      void initMargins();
      void initPunches();
      void initFiles();
      void initHistory();
   
      void closeSerial();
   
      // Serial Writing
      bool write(unsigned char motorid, unsigned char command, unsigned char data[], unsigned char datalength);
      bool write(unsigned char motorid[], unsigned char nummotors, unsigned char command, unsigned char* data[], unsigned char datalength);
      bool write(unsigned char command, unsigned char data[MOTORS_NUM_MOTORS][MOTORS_NUM_CONTROLS]);
      bool broadcast(unsigned char command, unsigned char data[], unsigned short datalength);
   
      void appendPacketToBuffer(unsigned char motorid, unsigned char command, unsigned char data[], unsigned char datalength, unsigned char messagebuffer[], unsigned short* currentbufferindex);
      void appendPacketsToBuffer(unsigned char motorid[], unsigned char nummotors, unsigned char command, unsigned char* data[], unsigned char datalength, unsigned char lowermessagebuffer[], unsigned char uppermessagebuffer[], unsigned short* lowerindex, unsigned short* upperindex);
      void appendPacketsToBuffer(unsigned char command, unsigned char data[MOTORS_NUM_MOTORS][MOTORS_NUM_CONTROLS], unsigned char lowermessagebuffer[], unsigned char uppermessagebuffer[], unsigned short* lowerindex, unsigned short* upperindex);
      void appendPacketsToBuffer(unsigned char command, unsigned char data[MOTORS_NUM_MOTORS][MOTORS_NUM_PUNCHES], unsigned char lowermessagebuffer[], unsigned char uppermessagebuffer[], unsigned short* lowerindex, unsigned short* upperindex);
      void appendControlPacketsToBuffer(unsigned char lowermessagebuffer[], unsigned char uppermessagebuffer[], unsigned short* lowerindex, unsigned short* upperindex);
   
      // Serial Reading
      unsigned short getNumBytesInQueue(FT_HANDLE fthandle);
      bool read(FT_HANDLE fthandle, unsigned char data[], unsigned short numbytestoread);
      unsigned short readQueue(FT_HANDLE fthandle, unsigned char data[], unsigned short maxdatalength);
      unsigned char updateFeedbackData(unsigned char readdata[], unsigned short numbytes);
      bool findHeader(unsigned char readdata[], unsigned short numbytes, unsigned short* index);
   
      // Datalogging
      void writeJointLabels(ofstream &file);
      void writeJointData();
   
      // Filtering and soft sensors
      void updateSoftData();
      void updateHistoryData();
   
   public:
      static unsigned char MotorIDToLowerBody[];
      static char MotorIDToSign[];
      static unsigned char LowerBodyIndexToMotorID[];
      static unsigned char UpperBodyIndexToMotorID[];
      static unsigned char IndexToMotorID[];
      static std::string MotorIDToName[];
      
      static unsigned short DefaultPositions[];
      static unsigned short DefaultSpeeds[];
   
      static unsigned char DefaultSlopes[];
      static unsigned char DefaultMargins[];
      static unsigned short DefaultPunches[];
   
   private:
      FT_HANDLE upperHandle;              // handle of the d2xx channel for the upper body (device 1)
      FT_HANDLE lowerHandle;              // handle of the d2xx channel for the lower body (device 0)
   
      // Control packet data
      unsigned char MotorControls[MOTORS_NUM_MOTORS][MOTORS_NUM_CONTROLS];
      unsigned char MotorPunches[MOTORS_NUM_MOTORS][MOTORS_NUM_PUNCHES];
      
      // Pre-generated request packets
      unsigned char MotorRequestsLower[MOTORS_NUM_LOWER_REQUEST_BLOCKS][MOTORS_NUM_LOWER_MOTORS*MAX_MESSAGE_LENGTH];
      unsigned short MotorRequestsLowerLength[MOTORS_NUM_LOWER_REQUEST_BLOCKS];
      unsigned char MotorRequestsUpper[MOTORS_NUM_UPPER_REQUEST_BLOCKS][MOTORS_NUM_UPPER_MOTORS*MAX_MESSAGE_LENGTH];
      unsigned short MotorRequestsUpperLength[MOTORS_NUM_UPPER_REQUEST_BLOCKS];
   
      // Threaded write data
      threaddata_t ThreadLowerWrite, ThreadUpperWrite;
   
      // Software motor on/off control
      bool MotorTorqueOn[MOTORS_NUM_MOTORS];

      // Datalogging files
      #if MOTORS_DATALOGGING
         ofstream PositionLog;
         ofstream SpeedLog;
         ofstream VelocityLog;
         ofstream LoadLog;
         ofstream TargetPositionLog;
         ofstream TargetSpeedLog;
         ofstream TargetPunchLog;
      #endif
   
      long double StartTime;
      
      // Joint velocity calculations and filtering
      unsigned short PreviousJointPositions[MOTORS_NUM_MOTORS];
      unsigned char HistoryVelocityIndex;
      float HistoryJointVelocities[MOTORS_NUM_MOTORS][VEL_WINDOW_SIZE];
   
};

void* runThreadedWrite(void *arg);

#endif
