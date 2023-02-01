/*******************************************************************************
 * Copyright (C) 2015 Maxim Integrated Products, Inc., All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
 * OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * Except as contained in this notice, the name of Maxim Integrated
 * Products, Inc. shall not be used except as stated in the Maxim Integrated
 * Products, Inc. Branding Policy.
 *
 * The mere transfer of this software does not imply any licenses
 * of trade secrets, proprietary technology, copyrights, patents,
 * trademarks, maskwork rights, or any other form of intellectual
 * property whatsoever. Maxim Integrated Products, Inc. retains all
 * ownership rights.
 *******************************************************************************
 */

#ifndef _BMP180_H_
#define _BMP180_H_

#include "mbed.h"

/**
 * Bosch BMP180 Digital Pressure Sensor
 *
 * @code
 * #include <stdio.h>
 * #include "mbed.h"
 * #include "BMP180.h"
 *
 * I2C i2c(I2C_SDA, I2C_SCL);
 * BMP180 bmp180(&i2c);
 *
 * int main(void) {
 *
 *     while(1) {
 *         if (bmp180.init() != 0) {
 *             printf("Error communicating with BMP180\n");
 *         } else {
 *             printf("Initialized BMP180\n");
 *             break;
 *         }
 *         wait(1);
 *     }
 *
 *     while(1) {
 *         bmp180.startTemperature();
 *         wait_ms(5);     // Wait for conversion to complete
 *         float temp;
 *         if(bmp180.getTemperature(&temp) != 0) {
 *             printf("Error getting temperature\n");
 *             continue;
 *         }
 *
 *         bmp180.startPressure(BMP180::ULTRA_LOW_POWER);
 *         wait_ms(10);    // Wait for conversion to complete
 *         int pressure;
 *         if(bmp180.getPressure(&pressure) != 0) {
 *             printf("Error getting pressure\n");
 *             continue;
 *         }
 *
 *         printf("Pressure = %d Pa Temperature = %f C\n", pressure, temp);
 *         wait(1);
 *     }
 * }
 * @endcode
 */
class BMP180
{

public:

    /**
     * @brief   Oversampling ratio.
     * @details Dictates how many pressure samples to take. Conversion time varies
     *          depending on the number of samples taken. Refer to data sheet
     *          for timing specifications.
     */
    typedef enum {
        ULTRA_LOW_POWER       = 0, ///< 1 pressure sample
        STANDARD              = 1, ///< 2 pressure samples
        HIGH_RESOLUTION       = 2, ///< 4 pressure samples
        ULTRA_HIGH_RESOLUTION = 3, ///< 8 pressure samples
    } oversampling_t;

    /**
     * BMP180 constructor.
     *
     * @param sda mbed pin to use for SDA line of I2C interface.
     * @param scl mbed pin to use for SCL line of I2C interface.
     */
    BMP180(PinName sda, PinName scl);

    /**
     * BMP180 constructor.
     *
     * @param i2c I2C object to use.
     */
    BMP180(I2C *i2c);

    /**
     * BMP180 destructor.
     */
    ~BMP180();

    /**
     * @brief   Initialize BMP180.
     * @details Gets the device ID and saves the calibration values.
     * @returns 0 if no errors, -1 if error.
     */
    int init(void);

    /**
     * @brief   Reset BMP180.
     * @details Performs a soft reset of the device. Same sequence as power on reset.
     * @returns 0 if no errors, -1 if error.
     */
    int reset(void);

    /**
     * @brief   Check ID.
     * @details Checks the device ID, should be 0x55 on reset.
     * @returns 0 if no errors, -1 if error.
     */
    int checkId(void);

    /**
     * @brief   Start pressure conversion.
     * @details Initiates the pressure conversion sequence. Refer to data sheet
     *          for timing specifications.
     *
     * @param   oss Number of samples to take.
     * @returns 0 if no errors, -1 if error.
     */
    int startPressure(BMP180::oversampling_t oss);

    /**
     * @brief   Get pressure reading.
     * @details Calculates the pressure using the data calibration data and formula.
     *          Pressure is reported in Pascals.
     * @note    This function should be called after calling startPressure().
     *          Refer to the data sheet for the timing requirements. Calling this
     *          function too soon can result in oversampling.
     *
     * @param   pressure Pointer to store pressure reading.
     * @returns 0 if no errors, -1 if error.
     */
    int getPressure(int *pressure);

    /**
     * @brief   Start temperature conversion.
     * @details Initiates the temperature conversion sequence. Refer to data
     *          sheet for timing specifications.
     * @returns 0 if no errors, -1 if error.
     */
    int startTemperature(void);

    /**
     * @brief   Get temperature reading.
     * @details Calculates the temperature using the data calibration data and formula.
     *          Temperature is reported in degrees Celcius.
     *
     * @note    This function should be called after calling startTemperature().
     *          Refer to the data sheet for the timing requirements. Calling this
     *          function too soon can result in oversampling.
     *
     * @param   tempC Pointer to store temperature reading.
     * @returns 0 if no errors, -1 if error.
     */
    int getTemperature(float *tempC);

    /**
     * @brief   Get temperature reading.
     * @details Calculates the temperature using the data calibration data and formula.
     *          Temperature is reported in 1/10ths degrees Celcius.
     *
     * @note    This function should be called after calling startTemperature().
     *          Refer to the data sheet for the timing requirements. Calling this
     *          function too soon can result in oversampling.
     *
     * @param   tempCx10 Pointer to store temperature reading.
     * @returns 0 if no errors, -1 if error.
     */
    int getTemperature(int16_t *tempCx10);

private:

    typedef union {
        uint16_t value[11];
        struct {
            int16_t ac1;
            int16_t ac2;
            int16_t ac3;
            uint16_t ac4;
            uint16_t ac5;
            uint16_t ac6;
            int16_t b1;
            int16_t b2;
            int16_t mb;
            int16_t mc;
            int16_t md;
        };
    } calibration_t;

    I2C *i2c_;
    bool i2c_owner;

    BMP180::calibration_t calib;
    int32_t b5;
    BMP180::oversampling_t oss_;
};

#endif /* _BMP180_H_ */
