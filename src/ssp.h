
/*
 * Small serial protocol
 * ssp.c
 * 
 *
 * Created: 20.01.2021 4:39:25
 * Author: alxrmironov@gmail.com
 */ 

#ifdef __cplusplus
extern "C" {
#endif

#ifndef SSP_H_
#define SSP_H_

#include <stdbool.h>
#include <stdint.h>

typedef struct{

	uint8_t	size;
	uint8_t	id;
	uint8_t crc8;
	uint8_t end;

}ssp_frame_header_str;

#define END_MARKER				(0xFF)
#define COLLISION_SYMBOL		(END_MARKER)
#define COLLISION_MARKER		(0xAA)
#define COLLISION_TRUE			(0x01)
#define COLLISION_FALSE			(0x00)
#define COLLISION_SIZE			(2)

#define ID_NONE					(0x00)
#define ID_MIN					(0x01)
#define ID_MAX					(0x80)

#define CRC8_SEED				(0xB1)
#define TX_TIMEOUT				(5000)

#define BUFFER_TOTAL_SIZE		(64)
#define OVERFLOW_MASK			(BUFFER_TOTAL_SIZE - 1)

#define END_BYTE_SIZE			(1)
#define HEADER_SIZE				(sizeof(ssp_frame_header_str))
#define PAYLOAD_SIZE_MAX		(BUFFER_TOTAL_SIZE - HEADER_SIZE)
#define INPUT_DATA_SIZE_MAX		(PAYLOAD_SIZE_MAX / COLLISION_SIZE)

typedef struct {
	
	uint8_t crc8;
	uint8_t (*CRC8_Function)(uint8_t inbyte, uint8_t crc8);
	
	bool (*UART_GetByte_)(uint8_t* value_ptr);
	bool (*UART_PutByte_)(uint8_t value);
	
	bool (*INPUT_GetByte_)(uint8_t* value_ptr);
	bool (*OUTPUT_PutByte_)(uint8_t value);
	
	struct {
		uint8_t last_received_id;
		uint8_t buffer[BUFFER_TOTAL_SIZE];
		uint8_t index;
		uint8_t size;
		uint8_t id;
	}rx;

	struct {
		uint8_t counter;
		uint8_t size;
		uint8_t* data;
		uint16_t timeout;
		
		ssp_frame_header_str ack;
		struct {
			bool ack_received;
			uint8_t id;
			uint8_t size;
			uint8_t data[BUFFER_TOTAL_SIZE];
		}frame;
	}tx;
	
}ssp_str;
	
typedef struct {
	
	uint8_t (*CRC8_Function)(uint8_t inbyte, uint8_t crc8);
	
	bool (*UART_GetByte_)(uint8_t* value_ptr);
	bool (*UART_PutByte_)(uint8_t value);
	
	bool (*INPUT_GetByte_)(uint8_t* value_ptr);
	bool (*OUTPUT_PutByte_)(uint8_t value);
	
}ssp_init_str;

bool SPP_Init(void* const ssp_object, const ssp_init_str* const config);
void SPP_Handler(void* ssp_object);
	
#endif /* SSP_H_ */