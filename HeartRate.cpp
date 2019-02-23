#include <iostream>
#include <thread>
#include "DigitalFilters.h"
#include "HeartRate.h"

LowPassFilter lpf(0.08, M_PI);
HighPassFilter hpf(0.08, M_PI);

// Constructor
HeartRate::HeartRate(MAX30102& _sensor) {
	sensor = &_sensor;
	if (sensor->begin() < 0) {
		// Failed i2c.
		throw;
	}
//	sensor->setup(0x2F);
	sensor->setup();
}

// Destructor
HeartRate::~HeartRate() {
	// Stop calculation loop if running and clear Microsmooth history.
	running = false;

	// Shutdown sensor->
	sensor->shutDown();
}

/**
 * Start calculation loop.
 * You must call this function before requesting heart rate data.
 */
void HeartRate::begin() {
	if (running) return;
	running = true;

	// Init last values.
	irLastValue = sensor->getIR();
	redLastValue = sensor->getRed();
	// Get current time.
	auto timeCurrent = std::chrono::system_clock::now();
	// Init last heartbeat times.
	timeLastIRHeartBeat = timeCurrent;
	timeLastRedHeartBeat = timeCurrent;

	std::thread t1(&HeartRate::loopThread, this);
	t1.detach();
}

void HeartRate::loopThread() {
	while (running) {
		runCalculationLoop();
//		updateTemperature();
	}
}

/**
 * Stops the calculation loop.
 * You may no longer get heart rate data after calling this function.
 */
void HeartRate::stop() {
	running = false;
	resetCalculations();
}

long loopCount = 1;
//auto timeBase = std::chrono::system_clock::now();
void HeartRate::runCalculationLoop() {
	// Wait until new data is available from sensor->
	while (!sensor->available()) {
		sensor->check();
	}
	
	// Get data from sensor->and store in global variables
	auto timeCurrent = std::chrono::system_clock::now();
	uint32_t irValue = sensor->getIR();
	uint32_t redValue = sensor->getRed();

	// Let's get the number of miliseconds passed since we last ran the loop.
	int loopDelta = std::chrono::duration_cast<std::chrono::milliseconds>(timeCurrent - timeLastLoopRan).count();
	// We're finished with timeLastLoopRan, so let's update it's value for next time.
	timeLastLoopRan = timeCurrent;

	// Check whether finger is on sensor->
	if (irValue < 100000) {
		// Finger is not on sensor->
		// Clear all calculations and exit out of loop.
		resetCalculations();
		return;
	}

	// Calculate the IR heart rate //
	
	int32_t filteredIRValue = static_cast<int32_t>(irValue);
	filteredIRValue = lpf.update(filteredIRValue);
	filteredIRValue = hpf.update(filteredIRValue);
//	filteredIRValue = Derivative(filteredIRValue);

//	[DEBUG] Uncomment lines below to disable heart rate calculation and display raw data.
//	std::cout << loopCount++ << "," << filteredIRValue << std::endl;
//	return;

	int timeSinceLastIRHeartBeat = std::chrono::duration_cast<std::chrono::milliseconds>(timeCurrent - timeLastIRHeartBeat).count();
	// We're finished with irLastValue, so let's update it's value for next time.
	irLastValue = filteredIRValue;

//	std::cout << loopCount++ << "," << filteredIRValue << ": ";
	if (peakDetect(filteredIRValue)) {
		int _irBPM = 60000/timeSinceLastIRHeartBeat;
		latestIRBPM = _irBPM;
		bpmBuffer[nextBPMBufferIndex++] = _irBPM;
		if (nextBPMBufferIndex >= BPM_BUFFER_SIZE) nextBPMBufferIndex = 0;
		//std::cout << "BPM : " << _irBPM;
		//std::cout << std::endl;
		
		// Update timeLastIRHeartBeat for next time.
		timeLastIRHeartBeat = timeCurrent;
	}
//	std::cout << std::endl;
	
	// Calculate the Red heart rate //
/*	
	int32_t filteredRedValue = static_cast<int32_t>(redValue);
	filteredRedValue = LowPassFilter(filteredRedValue);
	filteredRedValue = HighPassFilter(filteredRedValue);
	filteredRedValue = Derivative(filteredRedValue);
	filteredRedValue = Derivative(filteredRedValue);
	int redValueDelta = filteredRedValue - redLastValue; // Get the change in value.
	int timeSinceLastRedHeartBeat = std::chrono::duration_cast<std::chrono::milliseconds>(timeCurrent - timeLastRedHeartBeat).count();
	// We're finished with redLastValue, so let's update it's value for next time.
	redLastValue = filteredRedValue;

	if (filteredRedValue < -10 && redHasBeat == false && timeSinceLastRedHeartBeat > 0) {
		int _redBPM = 60000/timeSinceLastRedHeartBeat;
		if (_redBPM > 35) { // Ignore result if BPM is a crazy number.
			if (_redBPM < 200) {
				latestRedBPM = _redBPM;
			}
			redHasBeat = true;
			// Update timeLastRedHeartBeat for next time.
			timeLastRedHeartBeat = timeCurrent;
		}
	} else if (filteredRedValue > 0 && redHasBeat == true) {
		redHasBeat = false;
	}
*/
}

/**
 * Updates the temperature variable.
 */
void HeartRate::updateTemperature() {
	latestTemperature = sensor->readTemperatureF();
}

int32_t HeartRate::Derivative(int32_t data) {
	int32_t y;
	int i;
	static int x_derv[4];

	y = (data << 1) + x_derv[3] - x_derv[1] - (x_derv[0] << 1);

	y >>= 3;
	for (i = 0; i < 3; i++) {
		x_derv[i] = x_derv[i+1];
	}
	x_derv[3] = data;
	return y;
}

/**
 * Detects peaks in heart data.
 * Returns true when input data is a peak.
 * Warning: Algorithm isn't that good.
 */
bool crest = false;
bool trough = false;
int32_t lastData = 0;
int sensitivity = 1;
int currentCheck = 0;
bool HeartRate::peakDetect(int32_t data) {
	if (crest && trough && data > lastData && (localMaxima - localMinima) > 150) {
		if (currentCheck >= sensitivity) {
			crest = false;
			trough = false;
			currentCheck = 0;
			return true;
		} else {
			currentCheck++;
			return false;
		}
	}
	if (data > 10 && data > lastData) {
		localMaxima = data;
		crest = true;
	} else if (data < -10 && data < lastData) {
		localMinima = data;
		if (crest) {
			trough = true;
		}
	}
	lastData = data;
	return false;
}

/**
 * Clears all calculations.
 */
void HeartRate::resetCalculations() {
	// Clear stored heart rates.
	latestIRBPM = 0;
	//lastIRBPM = 0;
	latestRedBPM = 0;
	//lastRedBPM = 0;

	// Reset last values.
	irLastValue = sensor->getIR();
	redLastValue = sensor->getRed();

	// Get current time.
	auto timeCurrent = std::chrono::system_clock::now();
	// Reset last heartbeat times.
	timeLastIRHeartBeat = timeCurrent;
	timeLastRedHeartBeat = timeCurrent;
}

/**
 * Returns the average measured heart rate.
 * This method ignores heart rate values greater than 150 or lower than 45.
 */
int HeartRate::getSafeIRHeartRate() {
	int avgBPM = 0;
	int bpmCount = 0;
	for (int i = 0; i < BPM_BUFFER_SIZE; i++) {
		int _bpm = bpmBuffer[i];
		if (_bpm > 45 && _bpm < 150) {
			avgBPM += _bpm;
			bpmCount++;
		}
	}
	if (bpmCount > 0) avgBPM /= bpmCount;
	return (avgBPM == 0) ? -1 : avgBPM;
}

/**
 * Returns the latest calculated IR heart rate. (unchecked!)
 */
int HeartRate::getLatestIRHeartRate() {
	if (latestIRBPM == 0) {
		return -1;
	}
	return latestIRBPM;
}

/**
 * Returns the latest calculated Red heart rate. (unchecked!)
 */
int HeartRate::getLatestRedHeartRate() {
	if (latestRedBPM == 0) {
		return -1;
	}
	return latestRedBPM;
}

float HeartRate::getLatestTemperatureF() {
	return latestTemperature;
}
