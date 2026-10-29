/*
 * packet.h
 *
 *  Created on: Aug 14, 2024
 *      Author: schct
 */

#ifndef INC_COM_PACKET_H_
#define INC_COM_PACKET_H_


#include <stdint.h>
#include <stdlib.h>

int deserialize_packet(uint8_t* source, uint8_t* dest);
int serialize_packet(uint8_t* source, size_t len, uint8_t* dest);


#endif /* INC_COM_PACKET_H_ */
