#ifndef SERIALSTREAM_H
#define SERIALSTREAM_H

#include "mbed.h"
#include <platform/Stream.h>

struct str {
    char c[12];
};

inline str to_str(double num) {
    str out;

    int l = (int)num;
    num -= l;

    int r = abs(num*1000);

    char sign[2] = {'+',0};
    if (num < 0)
        sign[0] = '-';

    num = abs(num);

    char zs[3] = {0, 0, 0};
    
    if (num < 0.1)
        zs[0] = '0';
    if (num < 0.01)
        zs[1] = '0';

    for (int i = 0; i<2; i++) {
        num *= 10;
        if (num >= 1) {
            break;
        }
        zs[i] = '0';

    }


    sprintf(out.c, "%s%d.%s%3d", sign, l, zs, r);

    bool found = false;
    for (int i = 0; i<12; i++) {
        if (out.c[i] == 0) {
            found = true;
        }
        if (found) {
            out.c[i] = ' ';
        }
    }

    out.c[11] = 0;

    return out;
}


/**
 * SerialStream
 * Bringing MBed serial ports back like it's 1999... or at least 2019.
 *
 * This class adapts an MBed 6.0 serial port class into a Stream instance.
 * This lets you do two useful things with it:
 * - Call printf() and scanf() on it
 * - Pass it to code that expects a Stream to print things on.
 *
 */
template<class SerialClass>
class SerialStream : public Stream
{
	SerialClass & serialClass;

public:

	/**
	 * Create a SerialStream from a serial port.
	 * @param _serialClass BufferedSerial or UnbufferedSerial instance
	 * @param name The name of the stream associated with this serial port (optional)
	 */
	SerialStream(SerialClass & _serialClass, const char *name = nullptr):
	Stream(name),
	serialClass(_serialClass)
	{
	}

	// override Stream::read() and write() to call serial class directly.
	// This avoids the overhead of feeding in individual characters.
	virtual ssize_t write(const void *buffer, size_t length)
	{
		return serialClass.write(buffer, length);
	}

	virtual ssize_t read(void *buffer, size_t length)
	{
		return serialClass.read(buffer, length);
	}

protected:
	// Dummy implementations -- these will never be called because we override write() and read() instead.
	// but we have to override them since they're pure virtual.
	virtual int _putc(int c) { return 0; }
	virtual int _getc() { return 0; }
};

#endif //SERIALSTREAM_H
