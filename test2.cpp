#include <iostream>
#include <unistd.h>
#include "HeartRate.h"

using namespace std;


int main() {
	cout << "Starting..." << endl;
	MAX30102 sensor;
	HeartRate heartRate(&sensor);
	heartRate.begin();
	cout << "Began heart rate calculation..." << endl;
	while (1) {
		cout << "IR Heart Rate- Latest:" << heartRate.getLatestIRHeartRate() << ", SAFE:" << heartRate.getSafeIRHeartRate();
		//cout << ", Temperature: " << heartRate.getLatestTemperatureF() << endl;
		cout << ", Temperature: " << sensor.readTemperatureF() << endl;
		usleep(1000000);
	}
	return 0;
}
