/*!
 * @file Adafruit_BMP3XX.h
 *
 * Modified by R. W. Parker, 8/15//2022
 * Version 1.0
 *
 * This file is adapted from the Adafruit_BMP3XX Library, version 2.1.2.
 * Errors corrected and new functionaliity added. This library has been
 * specifically modified to work with the BMP390 sensor.
 *
 * Specific Notes Regarding Functionality
 * 1. Temperature and pressure measurements are always enabled
 * 2. FIFO functionaliity is not included.
 * 3. Interrupt functionaliity is not included.
 *
 * Usage
 *  1. Initialize the SPI bus and call SPI.begin
 *  2. Initialize the BMP390 sensor by calling begin( ) passing the chip select pin number and SPI object address.object
 *  3. Set the sensor settings by calling writeSensorSettings( ) passing the appropriate values
 *  4. Set the sensor power mode. If using the Forced mode then be sure to wait an appropriate time before reading
 *    the sensor data
 *  5. Read a new set of sensor data by calling readSensorData( )
 *  6. Retrieve the newest pressure/temperature values by calling getTemperature( ), getPressure( ) or
 *    getAltitude( )
 *
 * Adafruit BMP3XX temperature & barometric pressure sensor driver
 *
 * This is the documentation for Adafruit's BMP3XX driver for the
 * Arduino platform.  It is designed specifically to work with the
 * Adafruit BMP388 breakout: https://www.adafruit.com/products/3966
 *
 * This library  uses the SPI but to communicate
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Written by Ladyada for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */

#ifndef __BMP3XX_H__
#define __BMP3XX_H__

#include "bmp3.h"
#include <SPI.h>

/*=========================================================================*/
#define BMP3XX_DEFAULT_SPIFREQ (1000000) ///< The default SPI Clock speed
#define INVALID_POWER_MODE -99
#define INVALID_TEMP_OS_VALUE -98
#define INVALID_PRES_OS_VALUE -97
#define INVALID_FILTER_COEF_VALUE -96
#define INVALID_ODR_VALUE -95
#define SPI_CNTRL_OBJECT_INIT_FAILED -94


// =================================================================
// BMP3XX Class
// Use with Bosch BMP390 sensor
// SPI communication only
// Wraps the Bosch BMP3 API library functions
// Based on Adafruit_BMP3XX class definition
// =================================================================
class BMP3XX {
public:
    //Constructor
    BMP3XX();
    
    bool begin(uint8_t cs_pin, SPIClass *theSPI = &SPI);
    uint8_t chipID(void);
    int8_t setPowerMode(uint8_t powerMode);
    int8_t writeSensorSettings(uint8_t pressureOS,
                             uint8_t temperatureOS,
                             uint8_t IIRFilterCoef,
                             uint8_t ODR);
    float getTemperature(void);
    float getPressure(void);
    float getAltitude(float seaLevel);
    
    int8_t setTemperatureOversampling(uint8_t os);
    int8_t setPressureOversampling(uint8_t os);
    int8_t setIIRFilterCoeff(uint8_t fs);
    int8_t setOutputDataRate(uint8_t odr);
    
    // Perform a reading in blocking mode
    int8_t readSensorData(void);
    
private:
    // Private member functions
    bool _init(void);
    
    // Private member data
    SPIClass *spi = NULL;   // Pointer to the SPI bus object
    double temperature;     // Latest temperature reading in degrees C
    double pressure;        // Latest pressure reading in pascals
    bool _filterEnabled;    // True = Sensor filter enabled
    bool _tempOSEnabled;    // True = Sensor temperature oversampling enabled
    bool _presOSEnabled;    // True = Sensor pressure oversampling enabled
    bool _ODREnabled;       // True = Sensor output data rate enabled
    int8_t _cs;             // Chip select pin assigned to sensor
    
    struct bmp3_dev the_sensor; // BMP3 device data structure defined in bmp3_defs.h
};

#endif
