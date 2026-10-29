/* =
 * cmd.c
 *
 *  Created on: Aug 13, 2024
 *      Author: schctl
 */

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "Com/cmd.h"
#include "Com/munching.h"

// ECHO command

EchoCommand deserialize_echo(uint8_t* data, uint8_t* buffer) {
	size_t ptr = 0;
	uint8_t len = deserialize_byte(data + ptr); ptr += 1;
	memcpy(buffer, data + ptr, len);

	EchoCommand cmd = {
		.len = len,
		.data = buffer,
	};

	return cmd;
}

int serialize_echo(EchoCommand* command, uint8_t* buffer) {
	size_t ptr = 0;
	serialize_byte(command->len, buffer + ptr); ptr += 1;
	memcpy(buffer + ptr, command->data, command->len);

	return command->len + ptr;
}

// float command

float deserialize_float(uint8_t* data) {
	float k;
	memcpy(&k, data, 4);
	return k;
}

int serialize_float(float val, uint8_t* data) {
	memcpy(data, &val, 4);

	return 4;
}
