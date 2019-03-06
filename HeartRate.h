#include <cstdint>
#include <chrono>
#include "MAX30102.h"

class HeartRate {
	public:
		HeartRate(); // Fake constructor
		HeartRate(MAX30102 *sensor);
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
		int32_t localMaxima;
		int32_t localMinima;
		const static int8_t PAST_PEAKS_SIZE = 2;
		int32_t pastMaximas[PAST_PEAKS_SIZE];
		int32_t pastMinimas[PAST_PEAKS_SIZE];

		void loopThread();
		void runCalculationLoop();
		void updateTemperature();
		void resetCalculations();
		int32_t Derivative(int32_t data);
		int32_t getPeakThreshold();
		bool peakDetect(int32_t data);
};
