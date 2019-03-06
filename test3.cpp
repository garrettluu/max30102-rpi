#include <iostream>
#include <unistd.h>
#include "JS_HeartRate.cpp"

using namespace std;


int main() {
	cout << "Starting..." << endl;
	HRSInterface hrs;
	hrs.start();
	cout << "Done." << endl;
	cout << "Began heart rate calculation..." << endl;
	while (1) {
		cout << "IR Heart Rate- Latest:" << hrs.getLatestHeartRate() << ", SAFE:" << hrs.getSafeHeartRate();
		cout << ", Temperature: " << hrs.getLatestTemperatureF() << endl;
		usleep(1000000);
	}
	return 0;
}
