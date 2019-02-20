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

		std::chrono::time_point<std::chrono::system_clock> timeLastLoopRan;
		// IR Data
		std::chrono::time_point<std::chrono::system_clock> timeLastIRHeartBeat;
		int32_t irLastValue; 
		int latestIRBPM;
		bool irHasBeat = true;
		// Red Data
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
		int32_t Derivative(int32_t data);
};
