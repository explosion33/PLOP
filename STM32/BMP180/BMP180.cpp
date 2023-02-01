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

#include "BMP180.h"

/***** Definitions *****/
#define I2C_ADDR            (0xEE) // 1110111x

#define REG_ADDR_RESET      (0xE0)
#define REG_ADDR_ID         (0xD0)
#define REG_ADDR_CTRL       (0xF4)
#define REG_ADDR_DATA       (0xF6)
#define REG_ADDR_AC1        (0xAA)

#define CTRL_REG_TEMP       (0x2E)
#define CTRL_REG_PRESS_0    (0x34)
#define CTRL_REG_PRESS_1    (0x74)
#define CTRL_REG_PRESS_2    (0xB4)
#define CTRL_REG_PRESS_3    (0xF4)

//******************************************************************************
BMP180::BMP180(PinName sda, PinName scl)
{
    i2c_ = new I2C(sda, scl);
    i2c_owner = true;

    i2c_->frequency(400000);
}

//******************************************************************************
BMP180::BMP180(I2C *i2c) :
    i2c_(i2c)
{
    i2c_owner = false;
}

//******************************************************************************
BMP180::~BMP180()
{
    if(i2c_owner) {
        delete i2c_;
    }
}

//******************************************************************************
int BMP180::init(void)
{
    char addr;
    char data[22];
    int i;

    if (checkId() != 0) {
        return -1;
    }

    addr = REG_ADDR_AC1;
    if (i2c_->write(I2C_ADDR, &addr, 1) != 0) {
        return -1;
    }

    if (i2c_->read(I2C_ADDR, data, 22) != 0) {
        return -1;
    }

    for (i = 0; i < 11; i++) {
        calib.value[i] = (data[2*i] << 8) | data[(2*i)+1];
    }

    return 0;
}

//******************************************************************************
int BMP180::reset(void)
{
    char data;

    data = REG_ADDR_RESET;
    if (i2c_->write(I2C_ADDR, &data, 1) != 0) {
        return -1;
    }

    data = 0xB6;
    if (i2c_->write(I2C_ADDR, &data, 1) != 0) {
        return -1;
    }

    return 0;
}

//******************************************************************************
int BMP180::checkId(void)
{
    char addr;
    char data;

    addr = REG_ADDR_ID;
    if (i2c_->write(I2C_ADDR, &addr, 1) != 0) {
        return -1;
    }

    if (i2c_->read(I2C_ADDR, &data, 1) != 0) {
        return -1;
    }

    if (data != 0x55) {
        return -1;
    }

    return 0;
}

//******************************************************************************
int BMP180::startPressure(BMP180::oversampling_t oss)
{
    char data[2];

    data[0] = REG_ADDR_CTRL;
    data[1] = CTRL_REG_PRESS_0 | ((oss & 0x3) << 6);
    oss_ = oss;

    if (i2c_->write(I2C_ADDR, data, 2) != 0) {
        return -1;
    }

    return 0;
}

//******************************************************************************
int BMP180::getPressure(int *pressure)
{
    char addr, byte[3];
    uint32_t up;
    int32_t b6, x1, x2, x3, b3, p;
    uint32_t b4, b7;

    addr = REG_ADDR_DATA;
    if (i2c_->write(I2C_ADDR, &addr, 1) != 0) {
        return -1;
    }

    if (i2c_->read(I2C_ADDR, byte, 3) != 0) {
        return -1;
    }

    up = ((byte[0] << 16) | (byte[1] << 8) | byte[2]) >> (8 - oss_);

    b6 = b5 - 4000;
    x1 = (b6 * b6) >> 12;
    x1 *= calib.b2;
    x1 >>= 11;
    x2 = calib.ac2 * b6;
    x2 >>= 11;
    x3 = x1 + x2;
    b3 = (((((int32_t)calib.ac1) * 4 + x3) << oss_) + 2);
    b3 >>= 2;

    x1 = (calib.ac3 * b6) >> 13;
    x2 = (calib.b1 * ((b6 * b6) >> 12)) >> 16;
    x3 = (x1 + x2 + 2) >> 2;
    b4 = (calib.ac4 * (uint32_t)(x3 + 32768)) >> 15;
    b7 = ((uint32_t)up - b3) * (50000 >> oss_);
    p = ((b7 < 0x80000000) ? ((b7 << 1) / b4) : ((b7 / b4) * 2));
    x1 = p >> 8;
    x1 *= x1;
    x1 = (x1 * 3038) >> 16;
    x2 = (-7357 * p) >> 16;
    p += (x1 + x2 + 3791) >> 4;

    *pressure = p;

    return 0;
}

//******************************************************************************
int BMP180::startTemperature(void)
{
    char data[2] = { REG_ADDR_CTRL, CTRL_REG_TEMP };

    if (i2c_->write(I2C_ADDR, data, 2) != 0) {
        return -1;
    }

    return 0;
}

//******************************************************************************
int BMP180::getTemperature(float *tempC)
{
    char addr, byte[2];
    uint16_t ut;
    int32_t x1, x2;

    addr = REG_ADDR_DATA;
    if (i2c_->write(I2C_ADDR, &addr, 1) != 0) {
        return -1;
    }

    if (i2c_->read(I2C_ADDR, byte, 2) != 0) {
        return -1;
    }

    ut = (byte[0] << 8) | byte[1];

    x1 = ((ut - calib.ac6) * calib.ac5) >> 15;
    x2 = (calib.mc << 11) / (x1 + calib.md);
    b5 = x1 + x2;

    *tempC = (float)(b5 + 8) / 160;

    return 0;
}

//******************************************************************************
int BMP180::getTemperature(int16_t *tempCx10)
{
    char addr, byte[2];
    uint16_t ut;
    int32_t x1, x2;

    addr = REG_ADDR_DATA;
    if (i2c_->write(I2C_ADDR, &addr, 1) != 0) {
        return -1;
    }

    if (i2c_->read(I2C_ADDR, byte, 2) != 0) {
        return -1;
    }

    ut = (byte[0] << 8) | byte[1];

    x1 = ((ut - calib.ac6) * calib.ac5) >> 15;
    x2 = (calib.mc << 11) / (x1 + calib.md);
    b5 = x1 + x2;

    *tempCx10 = (b5 + 8) >> 4;

    return 0;
}
