/*
 * cmd.h
 *
 *  Created on: Aug 14, 2024
 *      Author: schctl
 */

#ifndef INC_COM_CMD_H_
#define INC_COM_CMD_H_

#include <stdint.h>


typedef struct {
	uint8_t len;
	uint8_t* data;
} EchoCommand;


uint8_t deserialize_byte(uint8_t* data);
int serialize_byte(uint8_t val, uint8_t* buffer);
float deserialize_float(uint8_t* data);
int serialize_float(float val, uint8_t* data);
EchoCommand deserialize_echo(uint8_t* data, uint8_t* buffer);
int serialize_echo(EchoCommand* command, uint8_t* buffer);

#endif /* INC_COM_CMD_H_ */
