#include "ibus.h"
#include "stdbool.h"
#include "stdio.h"

static void (*ch_data_handler_cb)(struct IbusCannels*);
static struct IbusState state;
static union IbusPacket packet;
static bool ibus_init_status = false;

static bool ibus_check_packet();

static void reset_state() {
	state.data_ind = 0;
	state.parse_state = WAITING_FOR_HEADER;
	state.checksum = IBUS_CHECKSUM_INIT;
}

void ibus_data_process(uint8_t byte) {
	// printf("0x%02X\n", byte);
	switch (state.parse_state) {
		case WAITING_FOR_HEADER:
			if (byte == IBUS_PACKET_HEADER_0) {
				packet.arr8[state.data_ind] = byte;
				state.data_ind++;
				state.checksum -= byte;
				state.parse_state = RECEIVED_HEADER_HALF;
				return;
			}
			break;
		case RECEIVED_HEADER_HALF:
			if (byte == IBUS_PACKET_HEADER_1) {
				packet.arr8[state.data_ind] = byte;
				state.data_ind++;
				state.checksum -= byte;
				state.parse_state = RECEIVED_HEADER_FULL;
				return;
			} else {
				reset_state();
			}
			break;
		case RECEIVED_HEADER_FULL:
			// printf("0x%02X\n", byte);
			packet.arr8[state.data_ind] = byte;
			state.data_ind++;
			if (state.data_ind < IBUS_PACKET_LEN - 1) {
				state.checksum -= byte;
			}
			if (state.data_ind == IBUS_PACKET_LEN) {
				if (ibus_check_packet() == true) {
					/* Call here callback that take data from packet */
					if (ch_data_handler_cb != NULL) {
						ch_data_handler_cb(&packet.channels);
					}
				}
				/* Reset data ind */
				reset_state();
			}
			break;
		default:
			break;
	}
}

void ibus_init(void (*ch_data_handler_ptr)(struct IbusCannels*)) {
	reset_state();
	ch_data_handler_cb = ch_data_handler_ptr;
}

static bool ibus_check_packet() {
	/* Check checksum */
	if (state.checksum != packet.checksum) {
		printf("Wrong crc, given: 0x%04X, calc: 0x%04X\n", packet.checksum, state.checksum);
		return false;
	}
	return true;
}
