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
 
#include "SerialGPS.h"
#include <cstdio>

SerialGPS::SerialGPS(PinName tx, PinName rx, int Baud) : _gps(tx, rx, Baud) {    
    longitude = 0.0;
    latitude = 0.0;        
}

int SerialGPS::sample() {
    char ns, ew, unit;
    int lock;

    int di;
    char dc;
    float df;

    while(1) {        
        getline();

        //$GPGGA,091626.000,2236.2791,N,12017.2818,E,1,10,1.00,8.8,M,18.7,M,,*66
        // Check if it is a GPGGA msg (matches both locked and non-locked msg)
        if (sscanf(msg, "GPGGA,%f,%f,%c,%f,%c,%d,%d,%f,%f,%c,%f", &time, &latitude, &ns, &longitude, &ew, &lock, &sats, &hdop, &alt, &unit, &geoid) >= 1) { 
            if(!lock) {
                time = 0.0;
                longitude = 0.0;
                latitude = 0.0;
                sats = 0;
                hdop = 0.0;
                alt = 0.0;
                geoid = 0.0;        
            } else {
                //GPGGA format according http://aprs.gids.nl/nmea/#gga
                // time (float), lat (f), (N/S) (c), long (f), (E/W) (c), fix (d), sats (d),
                // hdop (float), altitude (float), M, geoid (float), M, , ,  
                //GPGGA,092010.000,5210.9546,N,00008.8913,E,1,07,1.3,9.7,M,47.0,M,,0000*5D
                
                if(ns == 'S') {    latitude  *= -1.0; }
                if(ew == 'W') {    longitude *= -1.0; }
                float degrees = trunc(latitude / 100.0f);
                float minutes = latitude - (degrees * 100.0f);
                latitude = degrees + minutes / 60.0f;    
                degrees = trunc(longitude / 100.0f * 0.01f);
                minutes = longitude - (degrees * 100.0f);
                longitude = degrees + minutes / 60.0f;
                return 1;
            }
        }

        //$GPGSA,A,3,25,24,12,11,06,29,20,02,28,,,,1.66,0.98,1.34*01
        //$GPGSA,A,3,01,11,07,18,30,193,22,08,28,03,,,1.32,1.00,0.87*3A
        else if (msg[2] == 'G' && msg[3] == 'S' && msg[4] == 'A') {
            int i;
            for (i = 0; i<50; i++) {
                if (msg[i] == '.')
                    break;
            }
            i --;

            if (sscanf(msg+i, "%f, %f, %f", &pdop, &hdop, &vdop) >= 1) {
                return 2;
            }            
            
            return 0;
        }

        return 0;
    }
}

float SerialGPS::trunc(float v) {
    if(v < 0.0) {
        v*= -1.0;
        v = floor(v);
        v*=-1.0;
    } else {
        v = floor(v);
    }
    return v;
}

void SerialGPS::getline() {
    char c;

    while (true) {
        size_t res = _gps.read(&c, 1);
        if (res >= 0 && c == '$')
            break;
    }

    //while(_gps.getc() != '$');    // wait for the start of a line

    for(int i=0; i<256; i++) {
        size_t res = _gps.read(&c, 1);
        if (res < 0)
            continue;

        msg[i] = c;
        
        if(msg[i] == '\r') {
            msg[i] = 0;
            return;
        }
    }
    error("Overflowed message limit");
}