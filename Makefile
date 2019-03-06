CC=gcc
CXX=g++
CXXFLAGS=-Wall

RM=rm -f

all: test.out test2.out test3.out

test.out: MAX30102.o test.cpp
	$(CXX) $(CXXFLAGS) test.cpp MAX30102.o -o test.out

shutdown.out: MAX30102.o shutdown.cpp
	$(CXX) $(CXXFLAGS) shutdown.cpp MAX30102.o -o shutdown.out

test2.out: MAX30102.o HeartRate.o test2.cpp
	$(CXX) $(CXXFLAGS) test2.cpp HeartRate.o MAX30102.o -o test2.out -lpthread

test3.out: test3.o
	$(CXX) $(CXXFLAGS) -o test3.out test3.o JS_HeartRate.o -lpthread -lheartrate

test3.o: JS_HeartRate.o test3.cpp
	$(CXX) $(CXXFLAGS) -c test3.cpp

JS_HeartRate.o: HeartRate.o MAX30102.o libheartrate.so JS_HeartRate.cpp
	$(CXX) $(CXXFLAGS) -c -fPIC JS_HeartRate.cpp

libheartrate.so: MAX30102.o HeartRate.o
	$(CXX) $(CXXFLAGS) -shared -fPIC -o libheartrate.so HeartRate.cpp MAX30102.cpp
	sudo cp libheartrate.so /usr/local/lib/libheartrate.so

HeartRate.o: MAX30102.o HeartRate.cpp HeartRate.h
	$(CXX) $(CXXFLAGS) -c -fPIC HeartRate.cpp 

MAX30102.o: MAX30102.cpp MAX30102.h
	$(CXX) $(CXXFLAGS) -c -fPIC MAX30102.cpp

.PHONY: clean

clean:
	$(RM) *.o
	$(RM) *.out
