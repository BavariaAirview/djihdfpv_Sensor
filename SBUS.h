#ifndef SBUS_h
#define SBUS_h

#include "Arduino.h"

#define SBUS_FAILSAFE_INACTIVE 0
#define SBUS_FAILSAFE_ACTIVE   1
#define SBUS_STARTBYTE         0x0f
#define SBUS_ENDBYTE           0x00


class SBUS {
	public:
		SBUS(HardwareSerial & serial) : _serial (serial) {}
		void begin();
		void process();
		volatile int _channels[18];
	private:
		HardwareSerial & _serial;
};

#endif
