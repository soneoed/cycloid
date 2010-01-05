# 
# A makefile to compile things on the Cycloid
#
# Jason Kulk
# March 2009

CC=g++
CFLAGS=-lftd2xx -lrt
LDFLAGS=
SOURCES=main.cpp main.h \
	serial/motors.cpp serial/motors.h serial/sensorthread.cpp serial/sensorthread.h \
	locomotion/cmotion.cpp locomotion/cmotion.h \
	control/controlthread.cpp control/controlthread.h
OBJECTS=$(SOURCES:.cpp=.o)
EXECUTABLE=stance

all: $(SOURCES) $(EXECUTABLE)
	
$(EXECUTABLE): $(OBJECTS)
	$(CC) -o $@ $(CFLAGS) $(LDFLAGS) $(OBJECTS) 

clean:
	rm -rf *.o $(EXECUTABLE) */*.o

