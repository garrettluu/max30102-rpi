#include <iostream>
#include <unistd.h>
#include "HeartRate.h"
#include "easywsclient.hpp"

using namespace std;


int main() {
	cout << "Starting..." << endl;
	MAX30102 sensor;
	HeartRate heartRate(sensor);
	heartRate.begin();
	using easywsclient::WebSocket;
  //Server on port 3000
	Websocket::pointer ws = WebSocket::from_url("ws://localhost:3000");
	assert(ws);
	cout << "Began heart rate calculation..." << endl;
	while (1) {
		cout << "IR Heart Rate- Latest:" << heartRate.getLatestIRHeartRate() << ", SAFE:" << heartRate.getSafeIRHeartRate();
		//cout << ", Temperature: " << heartRate.getLatestTemperatureF() << endl;
    ws->poll();
    //Send the heart rate data to the server
    ws->send(heartRate.getSafeIRHeartRate());
		usleep(1000000);
	}
	return 0;
}
