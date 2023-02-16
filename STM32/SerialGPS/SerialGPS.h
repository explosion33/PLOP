/* mbed GPS Module Library
 * Copyright (c) 2008-2010, sford
 * Copyright (c) 2013, B.Adryan
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "mbed.h"

#ifndef MBED_GPS_H
#define MBED_GPS_H

/**  A SerialGPS interface for reading from a serial GPS module */
class SerialGPS {
public:

    /** Create the SerialGPS interface, connected to the specified serial port and speed.
     *  for example, GlobalSat EM406-A (e.g. on SparkFun GPS Shield) is 4800 Baud,
     *  Adafruit Ultimate GPSv3 (connected to serial) is 9600 Baud
     */    
    SerialGPS(PinName tx, PinName rx, int Baud);
    
    /** Sample the incoming GPS data, returning whether there is a lock
     * 
     * @return 1 if there was a lock when the sample was taken (and therefore .longitude and .latitude are valid), else 0
     */
    int sample();
    
    /** The longitude (call sample() to set) */
    float longitude;

    /** The latitude (call sample() to set) */
    float latitude;
    
    /** The time (call sample() to set) */
    float time;
    
    /** Number of satellites received (call sample() to set) */
    int sats;
    
    /** Horizontal dilusion of precision (call sample() to set) */
    float hdop;
    
    /** The altitude (call sample() to set)
     *  Note that the accurate altitude is corrected by the geoid
     *  See http://homepages.slingshot.co.nz/~geoff36/datum.htm
     */
    float alt;
    
    /** The geoid (call sample() to set) */
    float  geoid;
    
    /** The NMEA sentence */
    char msg[256];
    
    
private:
    float trunc(float v);
    void getline();
    
    BufferedSerial _gps;
};

#endif