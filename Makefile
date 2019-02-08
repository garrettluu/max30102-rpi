CXX=g++
CXXFLAGS=-Wall
LDFLAGS=-lwiringPi
OBJECTS=MAX30102.o

RM=rm -f

test.out: $(OBJECTS) test.cpp
	$(CXX) $(CXXFLAGS) test.cpp $(OBJECTS) -o test.out $(LDFLAGS)

MAX30102.o: MAX30102.cpp MAX30102.h
	$(CXX) $(CXXFLAGS) -c MAX30102.cpp

.PHONY: clean

clean:
	$(RM) *.o
	$(RM) *.out
