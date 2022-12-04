#ifndef IBUS_H
#define IBUS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx.h"

#define IBUS_PACKET_LEN         32
#define IBUS_PACKET_HEADER      0x4020
#define IBUS_PACKET_HEADER_0    0x20
#define IBUS_PACKET_HEADER_1    0x40
#define IBUS_CHECKSUM_INIT      0xFFFF

enum IbusParseState {
	WAITING_FOR_HEADER = 0,
	RECEIVED_HEADER_HALF,
	RECEIVED_HEADER_FULL
};

struct IbusState {
	uint8_t data_ind;
	uint16_t checksum;
	enum IbusParseState parse_state;
};

struct IbusCannels {
	uint16_t ch1;
	uint16_t ch2;
	uint16_t ch3;
	uint16_t ch4;
	uint16_t ch5;
	uint16_t ch6;
	uint16_t ch7;
	uint16_t ch8;
	uint16_t ch9;
	uint16_t ch10;
	uint16_t ch11;
	uint16_t ch12;
	uint16_t ch13;
	uint16_t ch14;
};

union IbusPacket {
	struct {
		uint16_t header;
		struct IbusCannels channels;
		uint16_t checksum;
	};
	uint8_t arr8[IBUS_PACKET_LEN];
	uint8_t arr16[IBUS_PACKET_LEN/2];
};

void ibus_data_process(uint8_t byte);
void ibus_init(void (*ch_data_handler_ptr)(struct IbusCannels*));

#ifdef __cplusplus
}
#endif

#endif /* IBUS_H */
