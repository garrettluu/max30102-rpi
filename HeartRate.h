#include <cstdint>
#include <chrono>
#include "MAX30102.h"

class HeartRate {
	public:
		HeartRate(MAX30102& sensor);
		~HeartRate();
		
		void begin();
		void stop();
		int getLatestIRHeartRate();
		int getLatestRedHeartRate();
		float getLatestTemperatureF();
	private:
		MAX30102* sensor;
		bool running = false;

		// Used by Microsmooth library
		uint16_t *ir_history;
		uint16_t *red_history;

		const static int SAMPLES_SIZE = 500;
		int curSampleIndex = 0;
		std::chrono::time_point<std::chrono::system_clock> samplesTimestamp[SAMPLES_SIZE];
		std::chrono::time_point<std::chrono::system_clock> timeLastLoopRan;
		// IR Data
		int irSamples[SAMPLES_SIZE];
		std::chrono::time_point<std::chrono::system_clock> timeLastIRHeartBeat;
		int32_t irLastValue; 
		int latestIRBPM;
		bool irHasBeat = true;
		// Red Data
		int redSamples[SAMPLES_SIZE];
		std::chrono::time_point<std::chrono::system_clock> timeLastRedHeartBeat;
		uint64_t redLastValue; 
		int latestRedBPM;
		bool redHasBeat = true;
		// Temperature Data
		float latestTemperature = -999;

		void loopThread();
		void runCalculationLoop();
		void updateTemperature();
		void resetCalculations();
		int32_t LowPassFilter(int32_t data);
		int32_t HighPassFilter(int32_t data);
		int32_t MovingWindowIntegral(int32_t data);
		int32_t Derivative(int32_t data);
};
