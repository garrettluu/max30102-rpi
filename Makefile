CXX=g++
CXXFLAGS=-Wall

RM=rm -f

test.out: MAX30102.o test.cpp
	$(CXX) $(CXXFLAGS) test.cpp MAX30102.o -o test.out

shutdown.out: MAX30102.o shutdown.cpp
	$(CXX) $(CXXFLAGS) shutdown.cpp MAX30102.o -o shutdown.out $(LDFLAGS)

MAX30102.o: MAX30102.cpp MAX30102.h
	$(CXX) $(CXXFLAGS) -c MAX30102.cpp

.PHONY: clean

clean:
	$(RM) *.o
	$(RM) *.out
