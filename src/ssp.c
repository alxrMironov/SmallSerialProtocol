/*
 * ssp.c
 *
 * Created: 20.01.2021 4:39:25
 *  Author: USER-PC
 */ 

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <iso646.h>

#include "ssp.h"

// STD Macro
//
#define MIN(a,b) (((a)<(b))?(a):(b))

// CRC8 Macro
//
#define ResetCRC8(ssp_obj)		(ssp_obj->crc8 = 0)
#define PushCRC8(ssp_obj, x)	(ssp_obj->crc8 = ssp_obj->CRC8_Function(x, ssp_obj->crc8))
#define GetCRC8(ssp_obj)		(ssp_obj->crc8)

// Local types
//
// 
typedef enum {
	NOTHING_RECEIVED,
	ACK_RECEIVED,
	FRAME_RECEIVED,
	BROKEN_RECEIVED,
}ssp_rx_answer_enum;

const size_t SSP_OBJECT_SIZE = sizeof(ssp_str);

static inline void CreateAck_(ssp_str* ssp, uint8_t id_to_ack);
static inline bool CreateFrame_(ssp_str* ssp);

static inline void SetupTransmitterForAck_(ssp_str* ssp);
static inline void SetupTransmitterForFrame_(ssp_str* ssp);

static inline ssp_rx_answer_enum ReceptionHandler_(ssp_str* ssp);
static inline bool TransmissionHandler_(ssp_str* ssp);

static inline uint8_t GenerateNewID_(uint8_t previous_id);

static inline bool PushAllToOutput_(ssp_str* ssp);

/*
 *	Parcel structure
 *	D - data
 *	CM - collision marker (included in size and CRC8)
 *	CR - collision resolver (included in size and CRC8)
 *	
 *	MAX_PARCEL_SIZE == MAX_PAYLOAD_SIZE * 2 + HEADER_SIZE
 *	CRC8\SIZE\ID != CM
 *	
 *	[D n] [D n+1] [CM] [CR] [D n+4] [ID] [SIZE] [CRC8] [END]
 *	
 */

bool SPP_Init(void* const ssp_object, const ssp_init_str* const config)
{
	ssp_str* ssp = ssp_object;
	
	if( ssp
	and config->CRC8_Function
	and config->INPUT_GetByte_
	and config->OUTPUT_PutByte_
	and config->UART_GetByte_
	and config->UART_PutByte_)
	{
		memset(ssp, 0, sizeof(ssp_str));
		
		ssp->tx.frame.ack_received = true;
		ssp->CRC8_Function		= config->CRC8_Function;
		ssp->INPUT_GetByte_		= config->INPUT_GetByte_;
		ssp->OUTPUT_PutByte_	= config->OUTPUT_PutByte_;
		ssp->UART_GetByte_		= config->UART_GetByte_;
		ssp->UART_PutByte_		= config->UART_PutByte_;
		
		return true;
	}
	else { return false; }
}

void SPP_Handler(void* const ssp_object)
{
	ssp_str* ssp = ssp_object;
	
	switch(ReceptionHandler_(ssp)){
		case ACK_RECEIVED:
			// Timeout is in progress - check ACK
			if(ssp->tx.timeout > 0){
				if(ssp->tx.frame.id == ssp->rx.id){
					ssp->tx.timeout = 0;
					ssp->tx.frame.ack_received = true;
				}
			}
			break;
		
		case FRAME_RECEIVED:
			// If already ACKing - ignore frame
			if(ssp->tx.ack.id == ID_NONE){
				ssp->tx.ack.id = ssp->rx.id;
			}
			break;
		
		default:
		case BROKEN_RECEIVED:
		case NOTHING_RECEIVED:
			break;
	}
	
	TransmissionHandler_(ssp);
	
}

static inline ssp_rx_answer_enum 
ReceptionHandler_(ssp_str* ssp)
{
	#define DecIndex()		({ssp->rx.index--; ssp->rx.index &= OVERFLOW_MASK;})
	#define IncLocalIndex()	({local_index++; local_index &= OVERFLOW_MASK;})
	
	uint8_t received;
	
	// Leave if no new bytes in UART
	if(not ssp->UART_GetByte_(&received)) { return NOTHING_RECEIVED; }

	// Recive till END_MARKER
	if(received != END_MARKER){
		ssp->rx.buffer[ssp->rx.index] = received;
		ssp->rx.index++;
		ssp->rx.index &= OVERFLOW_MASK;
		return NOTHING_RECEIVED;
	}
	// If END received
	else {
		DecIndex();	// Skip CRC8
		ssp->rx.id = ssp->rx.buffer[ssp->rx.index];
		DecIndex();
		ssp->rx.size = ssp->rx.buffer[ssp->rx.index];
		// Index wasnt decremented, cause needed in CRC8 calculation
		
		// WARRNING HEADER SIZE INCLUDES END MARKER
		uint8_t payload_size = ssp->rx.size - HEADER_SIZE; 
		// Getting payload zero index
		uint8_t local_index = ssp->rx.index - payload_size;
		local_index &= OVERFLOW_MASK;

		// Push all payload to CRC8
		ResetCRC8(ssp);
		while(local_index < ssp->rx.index){ 
			PushCRC8(ssp, ssp->rx.buffer[local_index]); 
			IncLocalIndex();
		}
		PushCRC8(ssp, ssp->rx.buffer[local_index]); // Size to CRC8
		IncLocalIndex();
		PushCRC8(ssp, ssp->rx.buffer[local_index]); // ID to CRC8 
		IncLocalIndex();
		
		// Index points to CRC8
		if(ssp->rx.buffer[local_index] != GetCRC8(ssp)) { return BROKEN_RECEIVED; }
			
		// Index points to first data byte
		ssp->rx.index -= payload_size;  
		ssp->rx.index &= OVERFLOW_MASK;
		
		// If ACK
		if(payload_size == 0){ return ACK_RECEIVED; }
		else { return FRAME_RECEIVED; }
	}
	
	#undef DecIndex
	#undef IncLocalIndex
}

static inline bool 
PushAllToOutput_(ssp_str* ssp)
{
	if(ssp->tx.counter < ssp->tx.size){
		
		while(ssp->tx.counter < ssp->tx.size) {
			bool is_sended = ssp->UART_PutByte_(ssp->tx.data[ssp->tx.counter]);
			if(is_sended) { ssp->tx.counter++; }
			else { return false; }
		}
		
		// Mark as sended and start timeout counting, if needed.
		if(ssp->tx.size == HEADER_SIZE){ ssp->tx.ack.id = ID_NONE; }
		else { ssp->tx.timeout = TX_TIMEOUT; }
	}
	
	return true;
}

static inline bool 
TransmissionHandler_(ssp_str* ssp)
{
	// Send all first
	if(not PushAllToOutput_(ssp)){ return false; }
		
	// Timeout decounter (counts only if transmission complete)
	if(ssp->tx.timeout) { ssp->tx.timeout--; }
	
	// If ack needed
	if(ssp->tx.ack.id > ID_NONE) {
		CreateAck_(ssp, ssp->tx.ack.id);
		SetupTransmitterForAck_(ssp);
	}
	// If timeout expires
	else if(ssp->tx.timeout == 0) {
		// Send new parcel
		if(ssp->tx.frame.ack_received){
			if(CreateFrame_(ssp)) { SetupTransmitterForFrame_(ssp); }
		}
		// Repeat
		else { SetupTransmitterForFrame_(ssp); }
	}
	
	return true;
}

static inline void 
CreateAck_(ssp_str* ssp, uint8_t id_to_ack)
{
	ResetCRC8(ssp);
	PushCRC8(ssp, HEADER_SIZE);
	PushCRC8(ssp, id_to_ack);
	ssp->tx.ack.size = HEADER_SIZE;
	ssp->tx.ack.id = id_to_ack;
	ssp->tx.ack.crc8 = GetCRC8(ssp);
	ssp->tx.ack.end = END_MARKER;
}

static inline bool
CreateFrame_(ssp_str* ssp)
{
	#define AddByteToParcel(x) ({ssp->tx.frame.data[data_size] = x; data_size++;})
	
	// Leave if no input
	uint8_t value;
	if(not ssp->INPUT_GetByte_(&value)) { return false; };
	uint8_t data_size = 0;
	
	ResetCRC8(ssp);
	do {
		// On marker or collision occurance - encode
		if( (value == COLLISION_SYMBOL)
		or	(value == COLLISION_MARKER))
		{
			PushCRC8(ssp, COLLISION_MARKER);
			AddByteToParcel(COLLISION_MARKER);
			value = (value == COLLISION_MARKER)? COLLISION_FALSE : COLLISION_TRUE;
			PushCRC8(ssp, value);
			AddByteToParcel(value);
		}
		else { 
			PushCRC8(ssp, value);
			AddByteToParcel(value); 
		}
		
		// Atleast 2 bytes left free
		if(data_size > PAYLOAD_SIZE_MAX - 1) { break; }
			
	}while(ssp->INPUT_GetByte_(&value));
	
	PushCRC8(ssp, data_size);
	AddByteToParcel(data_size);
	
	ssp->tx.frame.id = GenerateNewID_(ssp->tx.frame.id);
	PushCRC8(ssp, ssp->tx.frame.id);
	AddByteToParcel(ssp->tx.frame.id);
	
	ssp->tx.frame.data[data_size] = GetCRC8(ssp); 
	data_size++;
	ssp->tx.frame.data[data_size] = END_MARKER; 
	data_size++;
	ssp->tx.frame.size = data_size;

	return true;
	
	#undef AddByteToParcel
}

static inline void 
SetupTransmitterForAck_(ssp_str* ssp)
{
	ssp->tx.data = (uint8_t*)&ssp->tx.ack;
	ssp->tx.size = HEADER_SIZE;
	ssp->tx.counter = 0;
}

static inline void 
SetupTransmitterForFrame_(ssp_str* ssp)
{
	ssp->tx.data = ssp->tx.frame.data;
	ssp->tx.size = ssp->tx.frame.size;
	ssp->tx.counter = 0;
	ssp->tx.timeout = 0;
}

static inline uint8_t 
GenerateNewID_(uint8_t previous_id)
{
	uint8_t new_id = previous_id + 1;
	if(new_id > ID_MAX) { new_id = ID_MIN; }
	return new_id;
}
