/*
 * munching.h
 *
 *  Created on: Aug 15, 2024
 *      Author: schctl
 */

#ifndef INC_COM_MUNCHING_H_
#define INC_COM_MUNCHING_H_


uint8_t deserialize_byte(uint8_t* data);
int serialize_byte(uint8_t val, uint8_t* buffer);
uint16_t deserialize_u16(uint8_t* data);
int serialize_u16(uint16_t val, uint8_t* buffer);


#endif /* INC_COM_MUNCHING_H_ */
