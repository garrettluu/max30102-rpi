#include <chrono>
#include <cstring>
#include <unistd.h>
#include <iostream>
#include <fcntl.h>
#include <sys/ioctl.h>
#include "i2c-dev.h"
#include "MAX30102.h"

// Status Registers
static const uint8_t REG_INTSTAT1 =			0x00;
static const uint8_t REG_INTSTAT2 =			0x01;
static const uint8_t REG_INTENABLE1 =			0x02;
static const uint8_t REG_INTENABLE2 =			0x03;

// FIFO Registers
static const uint8_t REG_FIFOWRITEPTR =			0x04;
static const uint8_t REG_FIFOOVERFLOW =			0x05;
static const uint8_t REG_FIFOREADPTR =			0x06;
static const uint8_t REG_FIFODATA =			0x07;

// Configuration Registers
static const uint8_t REG_FIFOCONFIG =			0x08;
static const uint8_t REG_MODECONFIG =			0x09;
static const uint8_t REG_PARTICLECONFIG =		0x0A;
static const uint8_t REG_LED1_PULSEAMP =		0x0C;
static const uint8_t REG_LED2_PULSEAMP =		0x0D;
static const uint8_t REG_LED_PROX_AMP =			0x10;
static const uint8_t REG_MULTILEDCONFIG1 =		0x11;
static const uint8_t REG_MULTILEDCONFIG2 =		0x12;

// Die Temperature Registers
static const uint8_t REG_DIETEMPINT =			0x1F;
static const uint8_t REG_DIETEMPFRAC =			0x20;
static const uint8_t REG_DIETEMPCONFIG =		0x21;

// Proximity Function Registers
static const uint8_t REG_PROXINTTHRESH =		0x30;

// IDs
static const uint8_t REG_REVISIONID = 			0xFE;
static const uint8_t REG_PARTID =			0xFF;
static const uint8_t MAX30102_EXPECTEDPARTID =		0x15;

// Interrupt Configuration
static const uint8_t MASK_INT_A_FULL =			(uint8_t)~0b10000000;
static const uint8_t INT_A_FULL_ENABLE =		0x80;
static const uint8_t INT_A_FULL_DISABLE =		0x00;

static const uint8_t MASK_INT_DATA_RDY =		(uint8_t)~0b01000000;
static const uint8_t INT_DATA_RDY_ENABLE =		0x40;
static const uint8_t INT_DATA_RDY_DISABLE =		0x00;

static const uint8_t MASK_INT_ALC_OVF =			(uint8_t)~0b00100000;
static const uint8_t INT_ALC_OVF_ENABLE =		0x20;
static const uint8_t INT_ALC_OVF_DISABLE =		0x00;

static const uint8_t MASK_INT_PROX_INT =		(uint8_t)~0b00010000;
static const uint8_t INT_PROX_INT_ENABLE =		0x10;
static const uint8_t INT_PROX_INT_DISABLE =		0x00;

static const uint8_t MASK_INT_DIE_TEMP_RDY =		(uint8_t)~0b00000010;
static const uint8_t INT_DIE_TEMP_RDY_ENABLE =		0x02;
static const uint8_t INT_DIE_TEMP_RDY_DISABLE =		0x00;

static const uint8_t MASK_SAMPLEAVG =			(uint8_t)~0b11100000;
static const uint8_t SAMPLEAVG_1 =			0x00;
static const uint8_t SAMPLEAVG_2 =			0x20;
static const uint8_t SAMPLEAVG_4 =			0x40;
static const uint8_t SAMPLEAVG_8 =			0x60;
static const uint8_t SAMPLEAVG_16 =			0x80;
static const uint8_t SAMPLEAVG_32 =			0xA0;

static const uint8_t MASK_ROLLOVER =			0xEF;
static const uint8_t ROLLOVER_ENABLE =			0x10;
static const uint8_t ROLLOVER_DISABLE =			0x00;

static const uint8_t MASK_A_FULL =			0xF0;

// Mode configuration commands
static const uint8_t MASK_SHUTDOWN =			0x7f;
static const uint8_t SHUTDOWN =				0x80;
static const uint8_t WAKEUP =				0x00;
static const uint8_t MASK_RESET =			0xBF;
static const uint8_t RESET =				0X40;
/// IR led mode
static const uint8_t MASK_LEDMODE =			0xF8;
static const uint8_t LEDMODE_REDONLY =			0x02;
static const uint8_t LEDMODE_REDIRONLY =		0x03;
static const uint8_t LEDMODE_MULTILED =			0x07;

// Particle sensing configuration commands
static const uint8_t MASK_ADCRANGE =			0x9F;
static const uint8_t ADCRANGE_2048 =			0x00;
static const uint8_t ADCRANGE_4096 =			0x20;
static const uint8_t ADCRANGE_8192 =			0x40;
static const uint8_t ADCRANGE_16384 =			0x60;

static const uint8_t MASK_SAMPLERATE =			0xE3;
static const uint8_t SAMPLERATE_50 =			0x00;
static const uint8_t SAMPLERATE_100 =			0x04;
static const uint8_t SAMPLERATE_200 =			0x08;
static const uint8_t SAMPLERATE_400 =			0x0C;
static const uint8_t SAMPLERATE_800 =			0x10;
static const uint8_t SAMPLERATE_1000 =			0x14;
static const uint8_t SAMPLERATE_1600 =			0x18;
static const uint8_t SAMPLERATE_3200 =			0x1C;

static const uint8_t MASK_PULSEWIDTH =			0xFC;
static const uint8_t PULSEWIDTH_69 =			0x00;
static const uint8_t PULSEWIDTH_118 =			0x01;
static const uint8_t PULSEWIDTH_215 =			0x02;
static const uint8_t PULSEWIDTH_411 =			0x03;

// Multi-LED Mode Configuration
static const uint8_t MASK_SLOT1 =			0xF8;
static const uint8_t MASK_SLOT2 =			0x8F;
static const uint8_t MASK_SLOT3 =			0xF8;
static const uint8_t MASK_SLOT4 =			0x8F;

static const uint8_t SLOT_NONE =			0x00;
static const uint8_t SLOT_RED_LED =			0x01;
static const uint8_t SLOT_IR_LED =			0x02;
static const uint8_t SLOT_NONE_PILOT =			0x04;
static const uint8_t SLOT_RED_PILOT =			0x05;
static const uint8_t SLOT_IR_PILOT =			0x06;

MAX30102::MAX30102() {
	// Constructor
}

/**
 * Initializes sensor.
 * Returns negative number on failure.
 * Returns sensor revision on success.
 */
int MAX30102::begin(uint32_t i2cSpeed, uint8_t i2cAddr) {
	// [TODO] Set I2C Speed.
	const char* devName = "/dev/i2c-1";

	// Open the I2C bus
	int fd = open(devName, O_RDWR);
	if (fd == -1) {
		return -1;
	}

	if (ioctl(fd, I2C_SLAVE, i2cAddr) < 0) {
		return -2;
	}

	_i2c = fd;
	_i2caddr = i2cAddr;

	// Check if part id matches.
	if (readPartID() != MAX30102_EXPECTEDPARTID) {
		return -3;
	}

	return i2c_smbus_read_byte_data(fd, REG_REVISIONID);
}


// INterrupt configuration //

uint8_t MAX30102::getINT1(void) {
	return (i2c_smbus_read_byte_data(_i2c, REG_INTSTAT1));
}
uint8_t MAX30102::getINT2(void) {
	return (i2c_smbus_read_byte_data(_i2c, REG_INTSTAT2));
}

void MAX30102::enableAFULL(void) {
	bitMask(REG_INTENABLE1, MASK_INT_A_FULL, INT_A_FULL_ENABLE);
}
void MAX30102::disableAFULL(void) {
	bitMask(REG_INTENABLE1, MASK_INT_A_FULL, INT_A_FULL_DISABLE);
}

void MAX30102::enableDATARDY(void) {
	bitMask(REG_INTENABLE1, MASK_INT_DATA_RDY, INT_DATA_RDY_ENABLE);
}
void MAX30102::disableDATARDY(void) {
	bitMask(REG_INTENABLE1, MASK_INT_DATA_RDY, INT_DATA_RDY_DISABLE);
}

void MAX30102::enableALCOVF(void) {
	bitMask(REG_INTENABLE1, MASK_INT_ALC_OVF, INT_ALC_OVF_ENABLE);
}
void MAX30102::disableALCOVF(void) {
	bitMask(REG_INTENABLE1, MASK_INT_ALC_OVF, INT_ALC_OVF_DISABLE);
}

void MAX30102::enablePROXINT(void) {
	bitMask(REG_INTENABLE1, MASK_INT_PROX_INT, INT_PROX_INT_ENABLE);
}
void MAX30102::disablePROXINT(void) {
	bitMask(REG_INTENABLE1, MASK_INT_PROX_INT, INT_PROX_INT_DISABLE);
}

void MAX30102::enableDIETEMPRDY(void) {
	bitMask(REG_INTENABLE2, MASK_INT_DIE_TEMP_RDY, INT_DIE_TEMP_RDY_ENABLE);
}
void MAX30102::disableDIETEMPRDY(void) {
	bitMask(REG_INTENABLE2, MASK_INT_DIE_TEMP_RDY, INT_DIE_TEMP_RDY_DISABLE);
}


// Mode configuration //

/**
 * Pull sensor out of low power mode.
 */
void MAX30102::wakeUp(void) {
	bitMask(REG_MODECONFIG, MASK_SHUTDOWN, WAKEUP);
}

/**
 * Put sensor into low power mode.
 * During this mode the sensor will continue to respond to I2C commands
 * but will not update or take new readings, such as temperature.
 */
void MAX30102::shutDown(void) {
	bitMask(REG_MODECONFIG, MASK_SHUTDOWN, SHUTDOWN);
}

/**
 * All configuration, threshold, and data registers are reset
 * to their power-on state through a power-on reset.
 * The reset bit is cleared back to zero after reset finishes.
 */
void MAX30102::softReset(void) {
	bitMask(REG_MODECONFIG, MASK_RESET, RESET);

	// Poll for bit to clear, reset is then complete
	// Timeout after 100ms
	auto startTime = std::chrono::system_clock::now();
	std::chrono::system_clock::time_point endTime;
	do {
		uint8_t response = i2c_smbus_read_byte_data(_i2c, REG_MODECONFIG);
		if ((response & RESET) == 0) break; // Done reset!
		usleep(1); // Prevent over burden the I2C bus
		endTime = std::chrono::system_clock::now();
	}
	while ((std::chrono::duration_cast<std::chrono::milliseconds>(endTime-startTime).count()) < 100);
}

/**
 * Sets which LEDs are used for sampling.
 * - Red only
 * - Red+IR only
 * - Custom
 */
void MAX30102::setLEDMode(uint8_t mode) {
	bitMask(REG_MODECONFIG, MASK_LEDMODE, mode);
}

/**
 * Sets ADC Range.
 * Available ADC Range: 2048, 4096, 8192, 16384
 */
void MAX30102::setADCRange(uint8_t adcRange) {
	bitMask(REG_PARTICLECONFIG, MASK_ADCRANGE, adcRange);
}

/**
 * Sets Sample Rate.
 * Available Sample Rates: 50, 100, 200, 400, 800, 1000, 1600, 3200
 */
void MAX30102::setSampleRate(uint8_t sampleRate) {
	bitMask(REG_PARTICLECONFIG, MASK_SAMPLERATE, sampleRate);
}

/**
 * Sets Pulse Width.
 * Available Pulse Width: 69, 188, 215, 411
 */
void MAX30102::setPulseWidth(uint8_t pulseWidth) {
	bitMask(REG_PARTICLECONFIG, MASK_PULSEWIDTH, pulseWidth);
}

/**
 * Sets Red LED Pulse Amplitude.
 */
void MAX30102::setPulseAmplitudeRed(uint8_t amplitude) {
	i2c_smbus_write_byte_data(_i2c, REG_LED1_PULSEAMP, amplitude);
}

/**
 * Sets IR LED Pulse Amplitude.
 */
void MAX30102::setPulseAmplitudeIR(uint8_t amplitude) {
	i2c_smbus_write_byte_data(_i2c, REG_LED2_PULSEAMP, amplitude);
}

void MAX30102::setPulseAmplitudeProximity(uint8_t amplitude) {
	i2c_smbus_write_byte_data(_i2c, REG_LED_PROX_AMP, amplitude);
}

/**
 * Set the IR ADC count that will trigger the beginning of particle-sensing mode.
 * The threshMSB signifies only the 8 most significant-bits of the ADC count.
 */
void MAX30102::setProximityThreshold(uint8_t threshMSB) {
	i2c_smbus_write_byte_data(_i2c, REG_PROXINTTHRESH, threshMSB);
}

/**
 * Given a slot number assign a thing to it.
 * Devices are SLOT_RED_LED or SLOT_RED_PILOT (proximity)
 * Assigning a SLOT_RED_LED will pulse LED
 * Assigning a SLOT_RED_PILOT will ??
 */
void MAX30102::enableSlot(uint8_t slotNumber, uint8_t device) {
	uint8_t originalContents;
	switch (slotNumber) {
		case (1):
			bitMask(REG_MULTILEDCONFIG1, MASK_SLOT1, device);
			break;
		case (2):
			bitMask(REG_MULTILEDCONFIG1, MASK_SLOT2, device << 4);
			break;
		case (3):
			bitMask(REG_MULTILEDCONFIG2, MASK_SLOT3, device);
			break;
		case (4):
			bitMask(REG_MULTILEDCONFIG2, MASK_SLOT4, device << 4);
			break;
		default:
			// Shouldn't be here!
			break;
	}
}

/**
 * Clears all slot assignments.
 */
void MAX30102::disableSlots(void) {
	i2c_smbus_write_byte_data(_i2c, REG_MULTILEDCONFIG1, 0);
	i2c_smbus_write_byte_data(_i2c, REG_MULTILEDCONFIG2, 0);
}


// FIFO Configuration //

/**
 * Sets sample average.
 */
void MAX30102::setFIFOAverage(uint8_t numberOfSamples) {
	bitMask(REG_FIFOCONFIG, MASK_SAMPLEAVG, numberOfSamples);
}

/**
 * Resets all points to start in a known state.
 * Recommended to clear FIFO before beginning a read.
 */
void MAX30102::clearFIFO(void) {
	i2c_smbus_write_byte_data(_i2c, REG_FIFOWRITEPTR, 0);
	i2c_smbus_write_byte_data(_i2c, REG_FIFOOVERFLOW, 0);
	i2c_smbus_write_byte_data(_i2c, REG_FIFOREADPTR, 0);
}

/**
 * Enable roll over if FIFO over flows.
 */
void MAX30102::enableFIFORollover(void) {
	bitMask(REG_FIFOCONFIG, MASK_ROLLOVER, ROLLOVER_ENABLE);
}

/**
 * Disable roll over if FIFO over flows.
 */
void MAX30102::disableFIFORollover(void) {
        bitMask(REG_FIFOCONFIG, MASK_ROLLOVER, ROLLOVER_DISABLE);
}

/**
 * Sets number of samples to trigger the almost full interrupt.
 * Power on deafult is 32 samples.
 */
void MAX30102::setFIFOAlmostFull(uint8_t numberOfSamples) {
	bitMask(REG_FIFOCONFIG, MASK_A_FULL, numberOfSamples);
}

/**
 * Read the FIFO Write Pointer.
 */
uint8_t MAX30102::getWritePointer(void) {
	return (i2c_smbus_read_byte_data(_i2c, REG_FIFOWRITEPTR));
}

/**
 * Read the FIFO Read Pointer.
 */
uint8_t MAX30102::getReadPointer(void) {
	return (i2c_smbus_read_byte_data(_i2c, REG_FIFOREADPTR));
}

/**
 * Die Temperature.
 * Returns temperature in C.
 */
float MAX30102::readTemperature() {
	// DIE_TEMP_RDY interrupt must be enabled.
	
	// Step 1: Config die temperature register to take 1 temperature sample.
	i2c_smbus_write_byte_data(_i2c, REG_DIETEMPCONFIG, 0x01);

	// Poll for bit to clear, reading is then complete.
	// Timeout after 100ms.
	auto startTime = std::chrono::system_clock::now();
	std::chrono::system_clock::time_point endTime;
	do {
		// Check to see whether DIE_TEMP_RDY interrupt is set.
		uint8_t response = i2c_smbus_read_byte_data(_i2c, REG_INTSTAT2);
		if ((response & INT_DIE_TEMP_RDY_ENABLE) > 0) break;
		usleep(1);
		endTime = std::chrono::system_clock::now();
	} while(std::chrono::duration_cast<std::chrono::milliseconds>(endTime-startTime).count() < 100);
	
	// Step 2: Read die temperature register (integer)
	int8_t tempInt = i2c_smbus_read_byte_data(_i2c, REG_DIETEMPINT);
	uint8_t tempFrac = i2c_smbus_read_byte_data(_i2c, REG_DIETEMPFRAC); // causes clearing of the DIE_TEMP_RDY interrupt

	// Step 3: Calculate temperature.
	return (float)tempInt + ((float)tempFrac * 0.0625);
}

/**
 * Returns die temperature in F.
 */
float MAX30102::readTemperatureF() {
	float temp = readTemperature();

	if (temp != -999.0) temp = temp * 1.8 + 32.0;

	return (temp);
}

/**
 * Sets the PROX_INT_THRESHold.
 */
void MAX30102::setPROXINTTHRESH(uint8_t val) {
	i2c_smbus_write_byte_data(_i2c, REG_PROXINTTHRESH, val);
}


// Device ID and Revision //

uint8_t MAX30102::readPartID() {
	return i2c_smbus_read_byte_data(_i2c, REG_PARTID);
}

void MAX30102::readRevisionID() {
	revisionID = i2c_smbus_read_byte_data(_i2c, REG_REVISIONID);
}

uint8_t MAX30102::getRevisionID() {
	return revisionID;
}


// Setup the Sensor
void MAX30102::setup(uint8_t powerLevel, uint8_t sampleAverage, uint8_t ledMode, int sampleRate, int pulseWidth, int adcRange) {
	// Reset all configuration, threshold, and data registers to POR values
	softReset();

	// FIFO Configuration //
	
	// The chip will average multiple samples of same type together if you wish
	if (sampleAverage == 1) setFIFOAverage(SAMPLEAVG_1);
	else if (sampleAverage == 2) setFIFOAverage(SAMPLEAVG_2);
	else if (sampleAverage == 4) setFIFOAverage(SAMPLEAVG_4);
	else if (sampleAverage == 8) setFIFOAverage(SAMPLEAVG_8);
	else if (sampleAverage == 16) setFIFOAverage(SAMPLEAVG_16);
	else if (sampleAverage == 32) setFIFOAverage(SAMPLEAVG_32);
	else setFIFOAverage(SAMPLEAVG_4);

	// Allow FIFO to wrap/roll over
	enableFIFORollover();

	// Mode Configuration //
	if (ledMode == 3) setLEDMode(LEDMODE_MULTILED); // Watch all three led channels [TODO] there are only 2!
	else if (ledMode == 2) setLEDMode(LEDMODE_REDIRONLY);
	else setLEDMode(LEDMODE_REDONLY);
	activeLEDs = ledMode; // used to control how many bytes to read from FIFO buffer
	
	// Particle Sensing Configuration //
	if (adcRange < 4096) setADCRange(ADCRANGE_2048);
	else if (adcRange < 8192) setADCRange(ADCRANGE_4096);
	else if (adcRange < 16384) setADCRange(ADCRANGE_8192);
	else if (adcRange == 16384) setADCRange(ADCRANGE_16384);
	else setADCRange(ADCRANGE_2048);
	
	if (sampleRate < 100) setSampleRate(SAMPLERATE_50);
	else if (sampleRate < 200) setSampleRate(SAMPLERATE_100);
	else if (sampleRate < 400) setSampleRate(SAMPLERATE_200);
	else if (sampleRate < 800) setSampleRate(SAMPLERATE_400);
	else if (sampleRate < 1000) setSampleRate(SAMPLERATE_800);
	else if (sampleRate < 1600) setSampleRate(SAMPLERATE_1000);
	else if (sampleRate < 3200) setSampleRate(SAMPLERATE_1600);
	else if (sampleRate == 3200) setSampleRate(SAMPLERATE_3200);
	else setSampleRate(SAMPLERATE_50);

	if (pulseWidth < 118) setPulseWidth(PULSEWIDTH_69);	  // 15 bit resolution
	else if (pulseWidth < 215) setPulseWidth(PULSEWIDTH_118); // 16 bit resolution
	else if (pulseWidth < 411) setPulseWidth(PULSEWIDTH_215); // 17 bit resolution
	else if (pulseWidth == 411) setPulseWidth(PULSEWIDTH_411);// 18 bit resolution
	else setPulseWidth(PULSEWIDTH_69);

	// LED Pulse Amplitude Configuration //
	setPulseAmplitudeRed(powerLevel);
	setPulseAmplitudeIR(powerLevel);
	setPulseAmplitudeProximity(powerLevel);

	// Multi-LED Mode Configuration //
	// Enable the readings of the three LEDs [TODO] only 2!
	enableSlot(1, SLOT_RED_LED);
	if (ledMode > 1) enableSlot(2, SLOT_IR_LED);

	// Reset the FIFO before we begin checking the sensor.
	clearFIFO();
}


// Data Collection //

/**
 * Returns the number of samples available.
 */
uint8_t MAX30102::available(void)
{
	int8_t numberOfSamples = sense.head - sense.tail;
	if (numberOfSamples < 0) numberOfSamples += STORAGE_SIZE;
	return (numberOfSamples);
}

/**
 * Report the most recent Red value.
 */
uint32_t MAX30102::getRed(void)
{
	if(safeCheck(250))
		return (sense.red[sense.head]);
	else
		return(0);
}

/**
 * Report the most recent IR value.
 */
uint32_t MAX30102::getIR(void)
{
	if(safeCheck(250))
		return (sense.IR[sense.head]);
	else
		return(0);
}

/**
 * Report the next Red value in FIFO.
 */
uint32_t MAX30102::getFIFORed(void)
{
	return (sense.red[sense.tail]);
}

/**
 * Report the next IR value in FIFO.
 */
uint32_t MAX30102::getFIFOIR(void)
{
	return (sense.IR[sense.tail]);
}

/**
 * Advance the tail.
 */
void MAX30102::nextSample(void)
{
	if(available()) {
		sense.tail++;
		sense.tail %= STORAGE_SIZE;
	}
}

uint16_t MAX30102::check(void) {
	uint8_t readPointer = getReadPointer();
	uint8_t writePointer = getWritePointer();

	int numberOfSamples = 0;

	// Do we have new data?
	if (readPointer != writePointer) {
		// Calculate the number of readings we need to get from sensor
		numberOfSamples = writePointer - readPointer;
		//std::cout << "There are " << (int)numberOfSamples << " available samples." << std::endl;
		if (numberOfSamples < 0) numberOfSamples += 32;

		// We know have the number of readings, now calculate bytes to read.
		// For this example we are just doing Red and IR (3 bytes each)
		int bytesLeftToRead = numberOfSamples * activeLEDs * 3;

		// We may need to read as many as 288 bytes so we read in blocks no larger than I2C_BUFFER_LENGTH
		while (bytesLeftToRead > 0) {
			int toGet = bytesLeftToRead;
			if (toGet > I2C_BUFFER_LENGTH) {
				// If toGet is 32, this is bad because we read 6 bytes (RED+IR * 3 = 6) at a time.
				// 32 % 6 = 2 left over. We don't want to request 32 bytes, we want to request 30.
				// 32 % 9 (Red+IR+GREEN) = 5 left over. We want to request 27.

				// Trim toGet to be a multiple of the samples we need to read.
				toGet = I2C_BUFFER_LENGTH - (I2C_BUFFER_LENGTH % (activeLEDs * 3));
			}

			bytesLeftToRead -= toGet;

			// Request toGet number of bytes from sensor
			std::vector<uint8_t> dataReceived = readMany(REG_FIFODATA, toGet);
			int index = 0;

			while (toGet > 0) {
				sense.head++; // Advance the head of the storage struct
				sense.head %= STORAGE_SIZE;

				uint8_t temp[sizeof(uint32_t)]; // Array of 4 bytes that we will convert into long
				uint32_t tempLong;

				// Burst read three bytes - RED
				temp[3] = 0;
				temp[2] = dataReceived[index++];
				temp[1] = dataReceived[index++];
				temp[0] = dataReceived[index++];

				// Convert array to long
				std::memcpy(&tempLong, temp, sizeof(tempLong));
				
				// Zero out all but 18 bits
				tempLong &= 0x3FFFF;
				
				// Store this reading into the sense array
				sense.red[sense.head] = tempLong;

				if (activeLEDs > 1) {
					// Burst read three more bytes - IR
					temp[3] = 0;
					temp[2] = dataReceived[index++];
					temp[1] = dataReceived[index++];
					temp[0] = dataReceived[index++];

					// Convert array to long
					std::memcpy(&tempLong, temp, sizeof(tempLong));
					
					// Zero out all 18 bits
					tempLong &= 0x3FFFF;

					sense.IR[sense.head] = tempLong;
				}

				toGet -= activeLEDs * 3; 
			}
		}
	}
	return (numberOfSamples);
}

/**
 * Check for new data but give up after a certain amount of time.
 * Returns true if new data was found.
 * Returns false if new data was not found.
 */
bool MAX30102::safeCheck(uint8_t maxTimeToCheck) {
	auto markTime = std::chrono::system_clock::now();

	while(1) {
		auto endTime = std::chrono::system_clock::now();
		if ((std::chrono::duration_cast<std::chrono::milliseconds>(endTime-markTime).count()) > maxTimeToCheck) {
			return false;
		}

		if (check() == true) {
			// We found new data!
			return true;
		}

		usleep(1);
	}
}

/**
 * Set certain thing in register.
 */
void MAX30102::bitMask(uint8_t reg, uint8_t mask, uint8_t thing) {
	// Read register
	uint8_t originalContents = i2c_smbus_read_byte_data(_i2c, reg);

	// Zero-out portions of the register based on mask
	originalContents = originalContents & mask;

	// Change contents of register
	i2c_smbus_write_byte_data(_i2c, reg, originalContents | thing);
}

/**
 * Read multiple bytes from register.
 */
std::vector<uint8_t> MAX30102::readMany(uint8_t address, uint8_t length) {
	ioctl(_i2c, I2C_SLAVE, _i2caddr);
	uint8_t* rawRead = new uint8_t[length];
	i2c_smbus_read_i2c_block_data(_i2c, address, length, rawRead);
	std::vector<uint8_t> result;
	for (uint8_t i = 0; i < length; i++) {
		result.push_back(rawRead[i]);
	}
	return result;
}
