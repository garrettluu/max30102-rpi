CC=gcc
CXX=g++
CXXFLAGS=-Wall

RM=rm -f

test.out: MAX30102.o test.cpp
	$(CXX) $(CXXFLAGS) test.cpp MAX30102.o -o test.out

shutdown.out: MAX30102.o shutdown.cpp
	$(CXX) $(CXXFLAGS) shutdown.cpp MAX30102.o -o shutdown.out

test2.out: MAX30102.o HeartRate.o panTompkins.o microsmooth.o test2.cpp
	$(CXX) $(CXXFLAGS) test2.cpp panTompkins.o HeartRate.o microsmooth.o MAX30102.o -o test2.out -lpthread

HeartRate.o: MAX30102.o microsmooth.o HeartRate.cpp HeartRate.h
	$(CXX) $(CXXFLAGS) -c HeartRate.cpp 
	
panTompkins.o: panTompkins.c panTompkins.h
	$(CC) -c panTompkins.c -o panTompkins.o

microsmooth.o: microsmooth.cpp microsmooth.h
	$(CXX) $(CXXFLAGS) -c microsmooth.cpp

MAX30102.o: MAX30102.cpp MAX30102.h
	$(CXX) $(CXXFLAGS) -c MAX30102.cpp

.PHONY: clean

clean:
	$(RM) *.o
	$(RM) *.out
