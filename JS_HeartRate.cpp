#include "HeartRate.h"

class HRSInterface {

	public:
		HRSInterface() {
			HeartRate *_heartRate = new HeartRate(&sensor); 
			hrs = *_heartRate;
		}

		void start() {
			sensor.wakeUp();
			hrs.begin();
		}

		void stop() {
			hrs.stop();
			sensor.shutDown();
		}

		int getSafeHeartRate() {
			return hrs.getSafeIRHeartRate();
		}

		int getLatestHeartRate() {
			return hrs.getLatestIRHeartRate();
		}

		float getLatestTemperatureF() {
			return sensor.readTemperatureF();
		}

		float getLatestTemperatureC() {
			return sensor.readTemperature();
		}
	
	private:
		MAX30102 sensor;
		HeartRate hrs;
};
