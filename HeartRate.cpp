#include <iostream>
#include <thread>
#include "microsmooth.h"
#include "panTompkins.h"
#include "HeartRate.h"

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
	//ms_deinit(ir_history);
	//ms_deinit(red_history);

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

	// Set up Microsmooth history variables
	ir_history = ms_init(SMA);
	red_history = ms_init(SGA);

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
		updateTemperature();
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
//	int globalDelta = std::chrono::duration_cast<std::chrono::milliseconds>(timeCurrent - timeBase).count();
	// We're finished with timeLastLoopRan, so let's update it's value for next time.
	timeLastLoopRan = timeCurrent;

	// Check whether samples buffer is full.
	// If it is calculate heart rate and reset buffer.
	std::cout << curSampleIndex << "/" << SAMPLES_SIZE << std::endl;
	if (curSampleIndex >= SAMPLES_SIZE) {
		// Calculate ir heart rate.
		int32_t* peaks = beatDetect(irSamples, SAMPLES_SIZE);
		int BPM = 0;
		int BPM_count = 0;
		std::chrono::time_point<std::chrono::system_clock> timeLastBeat;
		for (int i = 0; i < SAMPLES_SIZE; i++) {
			std::cout << "PEAK " << i << ": " << peaks[i] << std::endl;
			if (peaks[i] == 1) {
				if (BPM_count > 0) {
					BPM += 60000/std::chrono::duration_cast<std::chrono::milliseconds>(samplesTimestamp[i]-timeLastBeat).count();
					BPM_count++;
				}
				timeLastBeat = samplesTimestamp[i];
			}
		}
		if (BPM_count > 0) {
			BPM = BPM/BPM_count;
			std::cout << "Calculated BPM: " << BPM << std::endl;
		}

		// Reset samples buffer
		curSampleIndex = 0;
	}

	// Check whether finger is on sensor->
	if (irValue < 100000) {
		// Finger is not on sensor->
		// Clear all calculations and exit out of loop.
		resetCalculations();
		curSampleIndex = 0;
		return;
	}

	// Calculate the IR heart rate //
	
//	irValue = sma_filter(irValue, ir_history);
	int32_t filteredIRValue = static_cast<int32_t>(irValue);
	samplesTimestamp[curSampleIndex] = timeCurrent;
	irSamples[curSampleIndex++] = filteredIRValue;
//	filteredIRValue = LowPassFilter(filteredIRValue);
//	filteredIRValue = HighPassFilter(filteredIRValue);
//	filteredIRValue = Derivative(filteredIRValue);
//	filteredIRValue *= filteredIRValue;
//	filteredIRValue = MovingWindowIntegral(filteredIRValue);
//	filteredIRValue = Derivative(filteredIRValue);
//	std::cout << loopCount++ << "," << filteredIRValue << std::endl;
//	std::cout << globalDelta << "," << filteredIRValue << std::endl;
	return;
	int irValueDelta = filteredIRValue - irLastValue; // Get the change in value.
	int timeSinceLastIRHeartBeat = std::chrono::duration_cast<std::chrono::milliseconds>(timeCurrent - timeLastIRHeartBeat).count();
	//if (timeSinceLastIRHeartBeat < 1000) return;
	// We're finished with irLastValue, so let's update it's value for next time.
	irLastValue = filteredIRValue;

	if (filteredIRValue < -10 && irHasBeat == false && timeSinceLastIRHeartBeat > 0) {
		//std::cout << filteredIRValue << std::endl;
		int _irBPM = 60000/timeSinceLastIRHeartBeat;
		if (_irBPM > 35) { // Ignore result if BPM is a crazy number.
			if (_irBPM < 200) {
				latestIRBPM = _irBPM;
			}
			irHasBeat = true;
			// Update timeLastIRHeartBeat for next time.
			timeLastIRHeartBeat = timeCurrent;
		}
	} else if (filteredIRValue > 10 && irHasBeat == true) {
		irHasBeat = false;
	}
	
	// Calculate the Red heart rate //
	
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

}

/**
 * Updates the temperature variable.
 */
void HeartRate::updateTemperature() {
	latestTemperature = sensor->readTemperatureF();
}

int32_t HeartRate::LowPassFilter(int32_t data) {
	static int32_t y1 = 0, y2 = 0, x[26], n = 12;
	int32_t y0;

	x[n] = x[n+13] = data;
	y0 = (y1 << 1) - y2 + x[n] - (x[n+6] << 1) + x[n+12];
	y2 = y1;
	y1 = y0;
	y0 >>= 5;
	if (--n < 0) {
		n = 12;
	}
	return static_cast<uint32_t>(y0);
}

int32_t HeartRate::HighPassFilter(int32_t data) {
	static int32_t y1 = 0, x[66], n = 32;
	int32_t y0;

	x[n] = x[n+33] = data;
	y0 = y1 + x[n] - x[n+32];
	y1 = y0;
	if (--n < 0) {
		n = 32;
	}
	return (x[n+16] - (y0 >> 5));
}

int32_t HeartRate::MovingWindowIntegral(int32_t data) {
	static int32_t x[32], ptr = 0;
	static long sum = 0;
	long ly;
	int32_t y;

	if (++ptr == 32) {
		ptr = 0;
	}
	sum -= x[ptr];
	sum += data;
	x[ptr] = data;
	ly = sum >> 5;
	if (ly > 2147483647) {
		y = 2147483647;
	} else {
		y = (int32_t) ly;
	}
	return y;
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

	// Reset Microsmooth history
	ms_deinit(ir_history);
	ms_deinit(red_history);
	ir_history = ms_init(SMA);
	red_history = ms_init(SGA);
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
