/*!
 * @file Adafruit_BMP3XX.cpp
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

#include "BMP3XX.h"
#include "Arduino.h"

//#define BMP3XX_DEBUG

// The chip select pin used for SPI communication with the sensor
// It needs to be a global variable because the static functions used by the
// bmp3 API do not have access to the private class data member _cs
uint8_t BMP3XX_chipSelect = 0;

// Hardware interface functions used by the bmp3 API
static int8_t spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len,
                       void *intf_ptr);
static int8_t spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len,
                        void *intf_ptr);
static void delay_usec(uint32_t us, void *intf_ptr);
static int8_t validate_trimming_param(struct bmp3_dev *dev);
static int8_t cal_crc(uint8_t seed, uint8_t data);

/***************************************************************************
 PUBLIC FUNCTIONS
 ***************************************************************************/

//=========================================================================
// Constructor
//=========================================================================
// Initailizes private data members
// Parameters: None
//==========================================================================
BMP3XX::BMP3XX(void) {
    _filterEnabled = false;
    _tempOSEnabled = false;
    _presOSEnabled = false;
    _ODREnabled = false;
}

//=========================================================================
// begin
//=========================================================================
// Initializes the_sensor data strucure and configures the chip select pin
// to OUTPUT and sets the pin HIGH. Also reads and verifies the sensor
// calibratioin data.
// Parameters:
//   cs_pin: The processor pin used as the chip select for the sensor
//   theSPI: A pointer to the SPI bus object
// Return:  False if initialization fails. True otherwise.
//==========================================================================
bool BMP3XX::begin(uint8_t cs_pin, SPIClass *theSPI) {
    
    BMP3XX_chipSelect = cs_pin;    //Initialize the global chipSelect variable
    
    // Set the chipSelect pin HIGH
    pinMode(BMP3XX_chipSelect, OUTPUT);
    digitalWrite(BMP3XX_chipSelect, HIGH);
    
    // Cycle chip select low for 1 millisecond to lockout spurious I2C
    // communications. Reference paragraph 5.1 of BMP390 data sheet
    digitalWrite(BMP3XX_chipSelect, LOW);
    delay(1);
    digitalWrite(BMP3XX_chipSelect, HIGH);
    
    // Initialize the sensor data structure
    the_sensor.intf = BMP3_SPI_INTF;
    the_sensor.read = &spi_read;
    the_sensor.write = &spi_write;
    the_sensor.intf_ptr = theSPI;
    the_sensor.dummy_byte = 1;
    the_sensor.settings.press_en = BMP3_ENABLE;
    the_sensor.settings.temp_en = BMP3_ENABLE;
    
    return _init();
}


//=========================================================================
// _init (private)
//=========================================================================
// Helper function for sensor initialization
// Parameters: None
// Return:  False if initialization fails. True otherwise.
//==========================================================================
bool BMP3XX::_init(void) {
    
    
    the_sensor.delay_us = delay_usec;
    int8_t rslt = BMP3_OK;
    
    /* Reset the sensor */
    rslt = bmp3_soft_reset(&the_sensor);  // BMP390 API function call
#ifdef BMP3XX_DEBUG
    Serial.print("Reset result: ");
    Serial.println(rslt);
#endif
    if (rslt != BMP3_OK)
        return false;
    
    // Read the chip-id and calibration data of the sensor.
    rslt = bmp3_init(&the_sensor);        // BMP390 API function call
#ifdef BMP3XX_DEBUG
    Serial.print("Init result: ");
    Serial.println(rslt);
#endif
    
    // Validate the sensor calibration data
    rslt = validate_trimming_param(&the_sensor);
#ifdef BMP3XX_DEBUG
    Serial.print("Valtrim result: ");
    Serial.println(rslt);
#endif
    
    if (rslt != BMP3_OK)
        return false;
    
#ifdef BMP3XX_DEBUG
    Serial.print("T1 = ");
    Serial.println(the_sensor.calib_data.reg_calib_data.par_t1);
    Serial.print("T2 = ");
    Serial.println(the_sensor.calib_data.reg_calib_data.par_t2);
    Serial.print("T3 = ");
    Serial.println(the_sensor.calib_data.reg_calib_data.par_t3);
    Serial.print("P1 = ");
    Serial.println(the_sensor.calib_data.reg_calib_data.par_p1);
    Serial.print("P2 = ");
    Serial.println(the_sensor.calib_data.reg_calib_data.par_p2);
    Serial.print("P3 = ");
    Serial.println(the_sensor.calib_data.reg_calib_data.par_p3);
    Serial.print("P4 = ");
    Serial.println(the_sensor.calib_data.reg_calib_data.par_p4);
    Serial.print("P5 = ");
    Serial.println(the_sensor.calib_data.reg_calib_data.par_p5);
    Serial.print("P6 = ");
    Serial.println(the_sensor.calib_data.reg_calib_data.par_p6);
    Serial.print("P7 = ");
    Serial.println(the_sensor.calib_data.reg_calib_data.par_p7);
    Serial.print("P8 = ");
    Serial.println(the_sensor.calib_data.reg_calib_data.par_p8);
    Serial.print("P9 = ");
    Serial.println(the_sensor.calib_data.reg_calib_data.par_p9);
    Serial.print("P10 = ");
    Serial.println(the_sensor.calib_data.reg_calib_data.par_p10);
    Serial.print("P11 = ");
    Serial.println(the_sensor.calib_data.reg_calib_data.par_p11);
    
#endif
    
    setTemperatureOversampling(BMP3_NO_OVERSAMPLING);
    setPressureOversampling(BMP3_NO_OVERSAMPLING);
    setIIRFilterCoeff(BMP3_IIR_FILTER_DISABLE);
    setOutputDataRate(BMP3_ODR_25_HZ);
    
    return true;
}

//=========================================================================
// setPowerMode
//=========================================================================
// Writes the specified power mode to the sensor. This will either put the
// sensor to sleep, initiate a measurement cyle, or initiate continuous
// measurements at ODR rate.
// Parameters:
//  powerMode: The power mode to be set. Valid values are
//              BMP3_MODE_SLEEP for sleep mode
//              BMP3_MODE_FORCED to initiate a single measurement cycle
//              BMP3_MODE_NORMAL to initiate continuous measurements at ODR
// Return:  A result code. 0 for success.
//==========================================================================
int8_t BMP3XX::setPowerMode(uint8_t powerMode)
{
    if (powerMode != BMP3_MODE_SLEEP && powerMode != BMP3_MODE_FORCED &&
            powerMode != BMP3_MODE_NORMAL ) {
        return INVALID_POWER_MODE;
    }
    
    the_sensor.settings.op_mode = powerMode;
#ifdef BMP3XX_DEBUG
    Serial.println(F("Setting power mode"));
#endif
    return (bmp3_set_op_mode(&the_sensor)); // bmp3 API call
}

//=========================================================================
// getTemperature
//=========================================================================
// Returns the latest temperature value.
// Parameters: None
// Return:  The updated temperature reading in degrees C
//==========================================================================
float BMP3XX::getTemperature(void) {
    return temperature;
}

//=========================================================================
// chipID
//=========================================================================
// Returns the sensor chip ID number as read during sensor initialization.
// Parameters: None
// Return:  The sensor chip ID number. Possible values are BMP3_CHIP_ID (0X50)
//          or BMP390_CHIP_ID (0X60)
//==========================================================================
uint8_t BMP3XX::chipID(void)
{
    return the_sensor.chip_id;
}

//=========================================================================
// getPressure
//=========================================================================
// Returns the latest pressure value.
// Parameters: None
// Return:  The updated pressure reading in Pascals
//==========================================================================
float BMP3XX::getPressure(void) {
    return pressure;
}

//=========================================================================
// getAltitude
//=========================================================================
// Computes and returns the altitude above sea level based on
// the latest pressure reading.
// Parameters:
//  seaLevel:   The reference pressure at sea level in hPa
// Return:  The altitude in meters
//==========================================================================
float BMP3XX::getAltitude(float seaLevel) {
    // Equation taken from BMP180 datasheet (page 16):
    //  http://www.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf
    
    // Note that using the equation from wikipedia can give bad results
    // at high altitude. See this thread for more information:
    //  http://forums.adafruit.com/viewtopic.php?f=22&t=58064
    
    float atmospheric = pressure / 100.0F;
    return 44330.0 * (1.0 - pow(atmospheric / seaLevel, 0.1903));
}

/**************************************************************************/
/*!
 @brief Performs a full reading of all sensors in the BMP3XX.
 
 Assigns the internal Adafruit_BMP3XX#temperature & Adafruit_BMP3XX#pressure
 member variables
 
 @return True on success, False on failure
 */
/**************************************************************************/
//=========================================================================
// readSensorData
//=========================================================================
// Reads the sensor pressure and temperature data and updates the the values
// stored locally.
// Parameters: None
// Return:  0 if the operation is successfull. An error code otherwise.
//==========================================================================
int8_t BMP3XX::readSensorData(void) {
    
    int8_t rslt;
    
    // Variable used to store the compensated data
    struct bmp3_data data;
    
    // Temperature and Pressure data are read and stored in the bmp3_data instance
#ifdef BMP3XX_DEBUG
    Serial.println(F("Getting sensor data"));
#endif
    rslt = bmp3_get_sensor_data(BMP3_TEMP | BMP3_PRESS, &data, &the_sensor); // bmp3 API call
    if (rslt != BMP3_OK)
        return rslt;
    
    /* Save the temperature and pressure data */
    temperature = data.temperature;
    pressure = data.pressure;
    
    return BMP3_OK;
}

//=========================================================================
// writeSensorSettings
//=========================================================================
// Writes the specified sensor settings to the sensor.
// Parameters:
//   pressureOS - Desired pressure oversampling setting. Allowable
//                values are BMP3_NO_OVERSAMPLING, BMP3_OVERSAMPLING_2X,
//                BMP3_OVERSAMPLING_4X, BMP3_OVERSAMPLING_8X,
//                BMP3_OVERSAMPLING_16X, BMP3_OVERSAMPLING_32X
//  temperatureOS - Desired pressure oversampling setting. Allowable
//                  values are BMP3_NO_OVERSAMPLING, BMP3_OVERSAMPLING_2X,
//                  BMP3_OVERSAMPLING_4X, BMP3_OVERSAMPLING_8X,
//                  BMP3_OVERSAMPLING_16X, BMP3_OVERSAMPLING_32X
//  IIRFilterCoef - Desired filter coefficient setting. Allowable
//                  values are BMP3_IIR_FILTER_DISABLE (no filtering),
//                  BMP3_IIR_FILTER_COEFF_1, BMP3_IIR_FILTER_COEFF_3,
//                  BMP3_IIR_FILTER_COEFF_7, BMP3_IIR_FILTER_COEFF_15,
//                  BMP3_IIR_FILTER_COEFF_31, BMP3_IIR_FILTER_COEFF_63,
//                  BMP3_IIR_FILTER_COEFF_127
//  ODR - Desired filter coefficient setting. Allowable values are
//        BMP3_ODR_200_HZ, BMP3_ODR_100_HZ,
//        BMP3_ODR_50_HZ, BMP3_ODR_25_HZ, BMP3_ODR_12_5_HZ, BMP3_ODR_6_25_HZ,
//        BMP3_ODR_3_1_HZ, BMP3_ODR_1_5_HZ, BMP3_ODR_0_78_HZ, BMP3_ODR_0_39_HZ,
//        BMP3_ODR_0_2_HZ, BMP3_ODR_0_1_HZ, BMP3_ODR_0_05_HZ, BMP3_ODR_0_02_HZ,
//        BMP3_ODR_0_01_HZ, BMP3_ODR_0_006_HZ, BMP3_ODR_0_003_HZ, or
//        BMP3_ODR_0_001_HZ
// Return:  False if invalid oversampling value is supplied. True otherwise.
//==========================================================================
int8_t BMP3XX::writeSensorSettings(uint8_t pressureOS,
                         uint8_t temperatureOS,
                         uint8_t IIRFilterCoef,
                         uint8_t ODR)
{
    // Validate and store the settings values to the_sensor data structure
    int8_t rslt = setTemperatureOversampling(temperatureOS);
    if (rslt != BMP3_OK) {
        return rslt;
    }
    
    rslt = setPressureOversampling(pressureOS);
    if (rslt != BMP3_OK) {
        return rslt;
    }
    
    rslt = setIIRFilterCoeff(IIRFilterCoef);
    if (rslt != BMP3_OK) {
        return rslt;
    }
    
    rslt = setOutputDataRate(ODR);
    if (rslt != BMP3_OK) {
        return rslt;
    }
    
    // Flag the settings being written
    uint16_t settings_sel = BMP3_SEL_TEMP_EN | BMP3_SEL_PRESS_EN;
    
    if (_tempOSEnabled) {
        settings_sel |= BMP3_SEL_TEMP_OS;
    }
    
    if (_presOSEnabled) {
        settings_sel |= BMP3_SEL_PRESS_OS;
    }
    
    if (_filterEnabled) {
        settings_sel |= BMP3_SEL_IIR_FILTER;
    }
    
    if (_ODREnabled) {
        settings_sel |= BMP3_SEL_ODR;
    }
    
    // Write the new settings to the sensor
#ifdef BMP3XX_DEBUG
    Serial.println("Setting sensor settings");
#endif
    // bmp3 API call
    rslt = bmp3_set_sensor_settings(settings_sel, &the_sensor);
    
    if (rslt != BMP3_OK)
        return rslt;
    
    return BMP3_OK;
}

//=========================================================================
// setTemperatureOversampling
//=========================================================================
// Sets the desired temperature oversampliing setting in the sensor
// bmp3_dev data structure (data member the_sensor). Also sets the
// _tempOSEnabled data member to true or false depending on the
// oversampling setting.
// No actual sensor settings are changed by this function call. Only local
// settings in the_sensor are changed.
// Parameters:
//   oversampling - Desired temperature oversampling setting. Allowable
//                  values are BMP3_NO_OVERSAMPLING, BMP3_OVERSAMPLING_2X,
//                  BMP3_OVERSAMPLING_4X, BMP3_OVERSAMPLING_8X,
//                  BMP3_OVERSAMPLING_16X, BMP3_OVERSAMPLING_32X
// Return:  0 if requested setting is valid. An error code otherwise.
//==========================================================================
int8_t BMP3XX::setTemperatureOversampling(uint8_t oversample) {
    if (oversample > BMP3_OVERSAMPLING_32X)
        return INVALID_TEMP_OS_VALUE;
    
    the_sensor.settings.odr_filter.temp_os = oversample;
    
    if (oversample == BMP3_NO_OVERSAMPLING)
        _tempOSEnabled = false;
    else
        _tempOSEnabled = true;
    
    return BMP3_OK;
}

//=========================================================================
// setPressureOversampling
//=========================================================================
// Sets the desired pressure oversampliing setting in the sensor
// bmp3_dev data structure (data member the_sensor). Also sets the
// _presOSEnabled data member to true or false depending on the
// oversampling setting.
// No actual sensor settings are changed by this function call. Only local
// settings in the_sensor are changed.
// Parameters:
//   oversampling - Desired pressure oversampling setting. Allowable
//                  values are BMP3_NO_OVERSAMPLING, BMP3_OVERSAMPLING_2X,
//                  BMP3_OVERSAMPLING_4X, BMP3_OVERSAMPLING_8X,
//                  BMP3_OVERSAMPLING_16X, BMP3_OVERSAMPLING_32X
// Return:  0 if requested setting is valid. An error code otherwise.
//==========================================================================
int8_t BMP3XX::setPressureOversampling(uint8_t oversample) {
    if (oversample > BMP3_OVERSAMPLING_32X)
        return INVALID_PRES_OS_VALUE;
    
    the_sensor.settings.odr_filter.press_os = oversample;
    
    if (oversample == BMP3_NO_OVERSAMPLING)
        _presOSEnabled = false;
    else
        _presOSEnabled = true;
    
    return BMP3_OK;
}

//=========================================================================
// setIIRFilterCoeff
//=========================================================================
// Sets the desired IIR Filter coefficient setting in the sensor
// bmp3_dev data structure (data member the_sensor). Also sets the
// _filterEnabled data member to true or false depending on the
// coeficient setting.
// No actual sensor settings are changed by this function call. Only local
// settings in the_sensor are changed.
// Parameters:
//   filtercoeff - Desired filter coefficient setting. Allowable
//                 values are BMP3_IIR_FILTER_DISABLE (no filtering),
//                 BMP3_IIR_FILTER_COEFF_1, BMP3_IIR_FILTER_COEFF_3,
//                 BMP3_IIR_FILTER_COEFF_7, BMP3_IIR_FILTER_COEFF_15,
//                 BMP3_IIR_FILTER_COEFF_31, BMP3_IIR_FILTER_COEFF_63,
//                 BMP3_IIR_FILTER_COEFF_127
// Return:  0 if requested setting is valid. An error code otherwise.
//==========================================================================
int8_t BMP3XX::setIIRFilterCoeff(uint8_t filtercoeff) {
    if (filtercoeff > BMP3_IIR_FILTER_COEFF_127)
        return INVALID_FILTER_COEF_VALUE;
    
    the_sensor.settings.odr_filter.iir_filter = filtercoeff;
    
    if (filtercoeff == BMP3_IIR_FILTER_DISABLE)
        _filterEnabled = false;
    else
        _filterEnabled = true;
    
    return BMP3_OK;
}

//=========================================================================
// setOutputDataRate
//=========================================================================
// Sets the desired Output Data Rate (ODR) setting in the sensor
// bmp3_dev data structure (data member the_sensor). Also sets the
// _ODREnabled data member to true if a valid ODR value is provided.
// Otherwise _ODREnabled is set to false.
// No actual sensor settings are changed by this function call. Only local
// settings in the_sensor are changed.
// Parameters:
//   odr - Desired filter coefficient setting. Allowable values are
//         BMP3_ODR_200_HZ, BMP3_ODR_100_HZ,
//         BMP3_ODR_50_HZ, BMP3_ODR_25_HZ, BMP3_ODR_12_5_HZ, BMP3_ODR_6_25_HZ,
//         BMP3_ODR_3_1_HZ, BMP3_ODR_1_5_HZ, BMP3_ODR_0_78_HZ, BMP3_ODR_0_39_HZ,
//         BMP3_ODR_0_2_HZ, BMP3_ODR_0_1_HZ, BMP3_ODR_0_05_HZ, BMP3_ODR_0_02_HZ,
//         BMP3_ODR_0_01_HZ, BMP3_ODR_0_006_HZ, BMP3_ODR_0_003_HZ, or
//         BMP3_ODR_0_001_HZ
// Return:  0 if requested setting is valid. An error code otherwise.
//==========================================================================
int8_t BMP3XX::setOutputDataRate(uint8_t odr) {
    if (odr > BMP3_ODR_0_001_HZ)
        return INVALID_ODR_VALUE;
    
    the_sensor.settings.odr_filter.odr = odr;
    
    _ODREnabled = true;
    
    return BMP3_OK;
}


//=========================================================================
// spi_read
//=========================================================================
// This function is called by the bmp3 API to read data from the sensor.
// First a starting register address is written to the sensor and then the
// data from one or more sensor registers is read back.
// Parameters:
//   reg_addr - The address of the sensor register from which reading will
//              begin.
//   reg_data - A pointer to a data buffer where the register data read from
//              the sensor will be stored.
//   len - The number of registers that will be read during the transaction
//   intf_ptr - A pointer to the SPI bus controller object
// Return:  0
//==========================================================================
static int8_t spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len,
                       void *intf_ptr) {
    
    SPIClass *spiPtr = (SPIClass *)intf_ptr;
    
    // Begin transaction
    spiPtr->beginTransaction(SPISettings(BMP3XX_DEFAULT_SPIFREQ, MSBFIRST, SPI_MODE0));
    
    // Assert chip select
    digitalWrite(BMP3XX_chipSelect, LOW);
    
    // Write register address
    spiPtr->transfer(reg_addr);
    
    // Read register data
    for (size_t index = 0; index < len; index++) {
        reg_data[index] = spiPtr->transfer(0xFF);
    }
    
    // Deassert chip select
    digitalWrite(BMP3XX_chipSelect, HIGH);
    
    // End transaction
    spiPtr->endTransaction();
    
    return 0;
}

//=========================================================================
// spi_write
//=========================================================================
// This function is called by the bmp3 API to write data to the sensor.
// First a starting register address is written to the sensor and then one
// or more data bytes are written to consecutive registers in the sensor.
// Parameters:
//   reg_addr - The address of the sensor register that will receive the
//              first byte of data.
//   reg_data - A pointer to a data buffer where the register data to be
//              written is stored.
//   len - The number of bytes that will be written
//   intf_ptr - A pointer to the SPI bus controller object
// Return:  0
//==========================================================================
static int8_t spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len,
                        void *intf_ptr)
{
    SPIClass *spiPtr = (SPIClass *)intf_ptr;
    
    // Begin transaction
    spiPtr->beginTransaction(SPISettings(BMP3XX_DEFAULT_SPIFREQ, MSBFIRST, SPI_MODE0));
    
    // Assert chip select
    digitalWrite(BMP3XX_chipSelect, LOW);
    
    // Write register address
    spiPtr->transfer(reg_addr);
    
    // Write the register data
    for (size_t index = 0; index < len; index++) {
        spiPtr->transfer(reg_data[index]);
    }
    
    // Deassert chip select
    digitalWrite(BMP3XX_chipSelect, HIGH);
    
    // End transaction
    spiPtr->endTransaction();
    
    return 0;
}

static void delay_usec(uint32_t us, void *intf_ptr) { delayMicroseconds(us); }

static int8_t validate_trimming_param(struct bmp3_dev *dev) {
    int8_t rslt;
    uint8_t crc = 0xFF;
    uint8_t stored_crc;
    uint8_t trim_param[21];
    uint8_t i;
    
    rslt = bmp3_get_regs(BMP3_REG_CALIB_DATA, trim_param, 21, dev);
    if (rslt == BMP3_OK) {
        for (i = 0; i < 21; i++) {
            crc = (uint8_t)cal_crc(crc, trim_param[i]);
        }
        
        crc = (crc ^ 0xFF);
        rslt = bmp3_get_regs(0x30, &stored_crc, 1, dev);
        if (stored_crc != crc) {
            rslt = -1;
        }
    }
    
    return rslt;
}

/*
 * @brief function to calculate CRC for the trimming parameters
 * */
static int8_t cal_crc(uint8_t seed, uint8_t data) {
    int8_t poly = 0x1D;
    int8_t var2;
    uint8_t i;
    
    for (i = 0; i < 8; i++) {
        if ((seed & 0x80) ^ (data & 0x80)) {
            var2 = 1;
        } else {
            var2 = 0;
        }
        
        seed = (seed & 0x7F) << 1;
        data = (data & 0x7F) << 1;
        seed = seed ^ (uint8_t)(poly * var2);
    }
    
    return (int8_t)seed;
}
