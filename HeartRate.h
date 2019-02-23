#include <cstdint>
#include <chrono>
#include "MAX30102.h"

class HeartRate {
	public:
		HeartRate(MAX30102& sensor);
		~HeartRate();
		
		void begin();
		void stop();
		int getSafeIRHeartRate();
		int getLatestIRHeartRate();
		int getLatestRedHeartRate();
		float getLatestTemperatureF();
	private:
		MAX30102* sensor;
		bool running = false;

		const int static BPM_BUFFER_SIZE = 100;
		int32_t bpmBuffer[BPM_BUFFER_SIZE];
		int nextBPMBufferIndex = 0;

		std::chrono::time_point<std::chrono::system_clock> timeLastLoopRan;
		// IR Data
		std::chrono::time_point<std::chrono::system_clock> timeLastIRHeartBeat;
		int32_t irLastValue; 
		int latestIRBPM;
		int averageIRBPM;
		// Red Data
		std::chrono::time_point<std::chrono::system_clock> timeLastRedHeartBeat;
		uint64_t redLastValue; 
		int latestRedBPM;
		// Temperature Data
		float latestTemperature = -999;

		// For Peak Detection
		int32_t localMaxima = -9999;
		int32_t localMinima = 9999;

		void loopThread();
		void runCalculationLoop();
		void updateTemperature();
		void resetCalculations();
		int32_t Derivative(int32_t data);
		bool peakDetect(int32_t data);
};
