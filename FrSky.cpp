#include "FrSky.h"

// FrSky object, input the serial bus
FrSky::FrSky(Stream& bus) {
    bus_ = &bus;
    parser_state_ = 0;
}

FrSky::FrSky(Stream& bus, Stream &forwarding_bus) {
    bus_ = &bus;
	forwarding_bus_ = &forwarding_bus;
    parser_state_ = 0;
	forward_bytes_ = true;
	forwarding_buffer_[0] = kSbusHeader;
	forwarding_buffer_[kForwardBufferSize - 1] = kSbusFooter;
}

// Open the serial port (at 100000 baud)
void FrSky::begin()
{
	static_cast<HardwareSerial*>(bus_)->begin(kSbusBaud, SERIAL_8E2);
}

bool FrSky::read(uint16_t* channels, bool* failsafe, bool* lostFrame)
{
	// parse the SBUS packet
	if (parse()) {
		if (channels) {
			// 16 channels of 11 bit data
			channels[0]  = (uint16_t) ((payload_[0]    |payload_[1] <<8)                     & 0x07FF);
			channels[1]  = (uint16_t) ((payload_[1]>>3 |payload_[2] <<5)                     & 0x07FF);
			channels[2]  = (uint16_t) ((payload_[2]>>6 |payload_[3] <<2 |payload_[4]<<10)  	 & 0x07FF);
			channels[3]  = (uint16_t) ((payload_[4]>>1 |payload_[5] <<7)                     & 0x07FF);
			channels[4]  = (uint16_t) ((payload_[5]>>4 |payload_[6] <<4)                     & 0x07FF);
			channels[5]  = (uint16_t) ((payload_[6]>>7 |payload_[7] <<1 |payload_[8]<<9)   	 & 0x07FF);
			channels[6]  = (uint16_t) ((payload_[8]>>2 |payload_[9] <<6)                     & 0x07FF);
			channels[7]  = (uint16_t) ((payload_[9]>>5 |payload_[10]<<3)                     & 0x07FF);
			channels[8]  = (uint16_t) ((payload_[11]   |payload_[12]<<8)                     & 0x07FF);
			channels[9]  = (uint16_t) ((payload_[12]>>3|payload_[13]<<5)                     & 0x07FF);
			channels[10] = (uint16_t) ((payload_[13]>>6|payload_[14]<<2 |payload_[15]<<10) 	 & 0x07FF);
			channels[11] = (uint16_t) ((payload_[15]>>1|payload_[16]<<7)                     & 0x07FF);
			channels[12] = (uint16_t) ((payload_[16]>>4|payload_[17]<<4)                     & 0x07FF);
			channels[13] = (uint16_t) ((payload_[17]>>7|payload_[18]<<1 |payload_[19]<<9)  	 & 0x07FF);
			channels[14] = (uint16_t) ((payload_[19]>>2|payload_[20]<<6)                     & 0x07FF);
			channels[15] = (uint16_t) ((payload_[20]>>5|payload_[21]<<3)                     & 0x07FF);
		}
		if (lostFrame) {
    	// count lost frames
    		if (payload_[22] & kSbusLostFrame) {
      			*lostFrame = true;
	    	} else {
				*lostFrame = false;
			}
		}
		if (failsafe) {
    	// failsafe state
   	 		if (payload_[22] & kSbusFailSafe) {
      			*failsafe = true;
    		}
    		else {
      			*failsafe = false;
    		}
		}	
		// return true on receiving a full packet
		return true;
  	} else {
		// return false if a full packet is not received
		return false;
	}
}

/* parse the SBUS data */
bool FrSky::parse()
{
	// see if serial data is available
	while (bus_->available() > 0) {
		// _sbusTime = 0;
		cur_byte_ = bus_->read();
		
		// find the header
		if (parser_state_ == 0) {
            if ((cur_byte_ == kSbusHeader) && (prev_byte_ == kSbusFooter)){
                parser_state_++;
            }
		} else {
			// strip off the data
			if ((parser_state_-1) < kPayloadSize) {
				payload_[parser_state_-1] = cur_byte_;
				parser_state_++;
			}
			// check the end byte
			if ((parser_state_-1) == kPayloadSize) {
				if (cur_byte_ == kSbusFooter) {
					parser_state_ = 0;
					valid_frame_count_++;
					// If we are forwarding to another board, send the entire message at once
					if (forward_bytes_) {
						memcpy(&forwarding_buffer_[1], payload_, kPayloadSize);
						forwarding_bus_ -> write(forwarding_buffer_, kForwardBufferSize);
					}		
					// We will only process the packet if we have received at least kValidFrameCountMin valid packets
					// before this one. This is so that we can't get use a rogue packet by accident just because channel values
					// happened to line up with the Header and Footer bytes.
					if (valid_frame_count_ > kValidFrameCountMin) {
						valid_frame_count_ = kValidFrameCountMin; // Subtract one to keep from overflowing the uint8_t byte
						return true;
					}
					return false;
				} else {
					parser_state_ = 0;
					valid_frame_count_ = 0; // Invalid packet. Reset count.
					return false;
				}
			}
		}
		prev_byte_ = cur_byte_;
	}
	// return false if a partial packet
	return false;
}

/* destructor, free dynamically allocated memory */
FrSky::~FrSky()
{
}