#include <iostream>
#include <unistd.h>
#include "MAX30102.h"

using namespace std;

MAX30102 heartSensor;

int main(void) {
	cout << "MAX30102 Heart Sensor Tester" << endl << "-----------------------------------------" << endl;
	// Start i2c
	int result = heartSensor.begin();
	if (result < 0) {
		cout << "Failed to start I2C (Error: " << result << ")." << endl;
		return (-1*result);
	}
	cout << "Device found (revision: " << result << ")!" << endl;

	cout << "Shuttdown down sensor..." << endl;
	heartSensor.shutDown();

	return 0;
}
