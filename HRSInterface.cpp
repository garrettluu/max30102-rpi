#include "HeartRate.h"

class HRSInterface {

	public:
		void init() {
			MAX30102 sensor;
			HeartRate heartRate(sensor); 
//			heartRate.begin();
//			hrs = &heartRate;
//			hrs->begin();
		}
/*		~HRSInterface () {

		}

		void start() {
//			sensor->wakeUp();
			hrs->begin();
		}

		void stop() {
			hrs->stop();
		//	sensor.shutDown();
		}

		int getSafeHeartRate() {
			return hrs->getSafeIRHeartRate();
		}

		int getLatestHeartRate() {
			return hrs->getLatestIRHeartRate();
		}

		float getLatestTemperatureF() {
	//		return sensor.readTemperatureF();
			return 0.0;
		}

		float getLatestTemperatureC() {
	//		return sensor.readTemperature();
			return 0.0;
		}

		int getLoopCount() {
			return hrs->getLoopCount();
		}
	
	private:
	//	MAX30102 sensor;
		HeartRate *hrs;
*/
};
