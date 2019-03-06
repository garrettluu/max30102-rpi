#include <iostream>
#include <thread>
#include "DigitalFilters.h"
#include "HeartRate.h"

LowPassFilter lpf(0.08, M_PI);
HighPassFilter hpf(0.08, M_PI);

// Fake Constructor
HeartRate::HeartRate() {
}

// Constructor
HeartRate::HeartRate(MAX30102 *_sensor) {
	sensor = _sensor;
	if (sensor->begin() < 0) {
		std::cout << "Failed i2c." << std::endl;
		// Failed i2c.
		throw;
	}
//	sensor->setup(0x2F);
	sensor->setup();
}

// Destructor
HeartRate::~HeartRate() {
	// Stop calculation loop if running.
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
	irLastValue = -999;
	redLastValue = -999;
	// Init local maxima/minima for peak detection
	localMaxima = -9999;
	localMinima = 9999;
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

//int loopCount = 1;
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

//	[DEBUG] Uncomment lines below to disable heart rate calculation and display raw data.
//	std::cout << loopCount++ << "," << filteredIRValue << std::endl;
//	return;

	int timeSinceLastIRHeartBeat = std::chrono::duration_cast<std::chrono::milliseconds>(timeCurrent - timeLastIRHeartBeat).count();

	//std::cout << loopCount++ << "," << filteredIRValue << ": ";
	if (peakDetect(filteredIRValue)) {
		int _irBPM = 60000/timeSinceLastIRHeartBeat;
		latestIRBPM = _irBPM;
		bpmBuffer[nextBPMBufferIndex++] = _irBPM;
		if (nextBPMBufferIndex >= BPM_BUFFER_SIZE) nextBPMBufferIndex = 0;
		//std::cout << "BPM : " << _irBPM;
		
		// Update timeLastIRHeartBeat for next time.
		timeLastIRHeartBeat = timeCurrent;
	}
	//std::cout << std::endl;

	// We're finished with irLastValue, so let's update their value for next time.
	irLastValue = filteredIRValue;
	
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

/**
 * Detects peaks in heart data.
 * Returns true when input data is a peak.
 * Warning: Algorithm isn't that good.
 */
bool crest = false;
bool trough = false;
uint8_t dataBeenIncreasing = 0;
uint8_t nextPastPeaksIndex = 0;

int32_t HeartRate::getPeakThreshold() {
	int32_t avgMaximas = 0;
	int32_t avgMinimas = 0;
	for (int i = 0; i < HeartRate::PAST_PEAKS_SIZE; i++) {
		//std::cout << "Index " << i << ": " << pastMaximas[i] << "/" << pastMinimas[i] << ", ";
		avgMaximas += pastMaximas[i];
		avgMinimas += pastMinimas[i];
	}
	//std::cout << "Average Maximas: " << avgMaximas << ", Minimas: " << avgMinimas << std::endl;
	avgMaximas /= HeartRate::PAST_PEAKS_SIZE;
	avgMinimas /= HeartRate::PAST_PEAKS_SIZE;
	int32_t threshold = (avgMaximas+avgMinimas)/1.5;
	if (threshold > 0 || threshold < -500) {
		return -20;
	}
	return threshold;
}
bool HeartRate::peakDetect(int32_t data) {
	//std::cout << "Data: " << data << ", irLastValue: " << irLastValue << ", localMaxima: " << localMaxima << ", localMinima: "<< localMinima << std::endl;
	if (irLastValue == -999) {
		// This is first time peakDetect is called.
		return false;
	}
	if (crest && trough && data > irLastValue) {
		dataBeenIncreasing++;
		if (dataBeenIncreasing >= 2) {
			// This is a beat.
			// Add local maxima & minima to past
			pastMaximas[nextPastPeaksIndex] = localMaxima;
			pastMinimas[nextPastPeaksIndex] = localMinima;
			nextPastPeaksIndex++;
			if (nextPastPeaksIndex >= HeartRate::PAST_PEAKS_SIZE) {
				nextPastPeaksIndex = 0;
			}
			// Reset values
			crest = trough = false;
			dataBeenIncreasing = 0;
			localMaxima = -9999;
			localMinima = 9999;
			
			return true;
		}
	}
	//int32_t threshold = getPeakThreshold();
	//std::cout << "Threshold: " << threshold << ", max: " << localMaxima << ", min: " << localMinima << std::endl;
	if (data > localMaxima) {
		localMaxima = data;
		if (data > 100) {
			crest = true;
		}
	}
	if (crest && data < localMinima) {
		localMinima = data;
		if (crest && data < -100) {
			trough = true;
		}
	}
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

//	calculateMiddle = true;
	// Reset peak detection.
	crest = trough = false;
	nextPastPeaksIndex = 0;
	dataBeenIncreasing = 0;
	localMaxima = -9999;
	localMinima = 9999;

	// Reset bpm buffer
	for (int i = 0; i < BPM_BUFFER_SIZE; i++) {
		bpmBuffer[i] = 0;
	}

	// Reset last values.
	irLastValue = -999;
	redLastValue = -999;

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
