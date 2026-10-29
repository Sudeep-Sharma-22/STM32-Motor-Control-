/*
 * munching.c
 *
 *  Created on: Aug 15, 2024
 *      Author: schctl
 */


#include <stdint.h>
#include "Com/munching.h"

inline uint8_t deserialize_byte(uint8_t* data) {
	return *data;
}

inline int serialize_byte(uint8_t val, uint8_t* buffer) {
	buffer[0] = val;
	return 1;
}

inline uint16_t deserialize_u16(uint8_t* data) {
	return *(uint16_t*)(data);
}

inline int serialize_u16(uint16_t val, uint8_t* buffer) {
	buffer[0] = (val >> 0) & 0xff;
	buffer[1] = (val >> 8) & 0xff;
	return 2;
}
