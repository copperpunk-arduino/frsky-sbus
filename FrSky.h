/*
FrSky SBUS parser based off the SBUS.h library
from Brian R Tayload
brian.taulor@borderflight.com

Created and maintained by Greg Gradwell
greg.gradwell@kittyhawk.aero
*/

#ifndef FRSKY_H
#define FRSKY_H

#include "Arduino.h"

class FrSky {
    public: 
        FrSky(Stream &bus);
		FrSky(Stream &bus, Stream &forwarding_bus);
        void begin();
        bool read(uint16_t* channels, bool* failsafe, bool* lostFrame);
        ~FrSky();
    private:
		bool forward_bytes_ = false;
        const uint32_t kSbusBaud = 100000;
		const uint8_t kSbusHeader = 0x0F;
		const uint8_t kSbusFooter = 0x00;
		const uint8_t kSbusLostFrame = 0x04;
		const uint8_t kSbusFailSafe = 0x08;
		// const uint8_t _sbus2Footer = 0x04;
		// const uint8_t _sbus2Mask = 0x0F;
		uint8_t parser_state_, prev_byte_ = kSbusFooter, cur_byte_;
		static const uint8_t kPayloadSize = 24;
		uint8_t payload_[kPayloadSize];
		static const uint8_t kForwardBufferSize = 26;
		uint8_t forwarding_buffer_[kForwardBufferSize];
		
		Stream* bus_;
		Stream* forwarding_bus_;
		const uint kValidFrameCountMin = 3;
		uint8_t valid_frame_count_ = 0;
		bool parse();
};

#endif
