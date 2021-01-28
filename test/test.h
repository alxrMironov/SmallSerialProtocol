

#ifdef __cplusplus
extern "C" {
#endif

#ifndef TEST_H_
#define TEST_H_

#include <stdint.h>
#include <stdbool.h>

#include "unity.h"
#include "ssp.h"
#include "ssp.c"


//static char string[4096];

static uint8_t test_uart_rxed_index;
static uint8_t test_uart_txed_index;
static size_t test_uart_len;

static uint8_t test_uart_array[4096] = { 0 };
	/*
static uint8_t test_uart_output_array[128] = {
	35, 125, 159, 193, 66, 28, 254, 160, 225, 191, 93, 3, 128, 222, 60, 98,
	190, 224, 2, 92, 223, 129, 99, 61, 124, 34, 192, 158, 29, 67, 161, 255,
	101, 59, 217, 135, 4, 90, 184, 230, 167, 249, 27, 69, 198, 152, 122, 36,
	70, 24, 250, 164, 39, 121, 155, 197, 132, 218, 56, 102, 229, 187, 89, 7,
	219, 133, 103, 57, 186, 228, 6, 88, 25, 71, 165, 251, 120, 38, 196, 154,
	0, 94, 188, 226, 97, 63, 221, 131, 194, 156, 126, 32, 163, 253, 31, 65,
	157, 195, 33, 127, 252, 162, 64, 30, 95, 1, 227, 189, 62, 96, 130, 220,
	248, 166, 68, 26, 153, 199, 37, 123, 58, 100, 134, 216, 91, 5, 231, 185,
};*/

static uint8_t test_serial_rxed_index;
static uint8_t test_serial_to_tx_index;
static size_t test_serial_rxed_len;
static size_t test_serial_to_tx_len;

static uint8_t test_serial_rxed_array[4096] = { 0 };
	
static uint8_t test_serial_to_tx_array[128] = {
	140, 0xFF, 48, 0xFF, 0xFF, 0xAA, 81, 0xAA, 78, 0xAA, 242, 0xAA, 0xAA, 113, 147, 205,
	175, 241, 19, 77, 206, 144, 114, 44, 109, 51, 209, 143, 12, 82, 176, 238,
	202, 148, 118, 40, 171, 245, 23, 73, 8, 86, 180, 234, 105, 55, 213, 139,
	87, 9, 235, 181, 54, 104, 138, 212, 149, 203, 41, 119, 244, 170, 72, 22,
	17, 79, 173, 243, 112, 46, 204, 146, 211, 141, 111, 49, 178, 236, 14, 80,
	50, 108, 142, 208, 83, 13, 239, 177, 240, 174, 76, 18, 145, 207, 45, 115,
	233, 183, 85, 11, 136, 214, 52, 106, 43, 117, 151, 201, 74, 20, 246, 168,
	116, 42, 200, 150, 21, 75, 169, 247, 182, 232, 10, 84, 215, 137, 107, 53
};

static uint8_t TEST_CalculateLenWithCollisions(uint8_t* addr, uint8_t size)
{
	uint8_t result = 0;
	for(uint8_t i = 0; i < size; i++){
		result++;
		if(addr[i] == COLLISION_SYMBOL or addr[i] == COLLISION_MARKER){
			result++;
		}
	}
	return result;
}

static bool TEST_UART_GetByte(uint8_t* value){
	if(test_uart_len > 0){
		*value = test_uart_array[test_uart_txed_index];
		test_uart_txed_index++;
		test_uart_txed_index &= 127;
		test_uart_len--;
		return true;
	}
	else { return false; }
};

static bool TEST_UART_PutByte(uint8_t value){
	if(test_uart_len > 0){
		test_uart_array[test_uart_rxed_index] = value;
		test_uart_rxed_index++;
		test_uart_len--;
		return true;
	}
	else { return false; }
};

static bool TEST_SERIAL_GetByte(uint8_t* value){
	if(test_serial_to_tx_len > 0){
		*value = test_serial_to_tx_array[test_serial_to_tx_index];
		test_serial_to_tx_index++;
		test_serial_to_tx_index &= 127;
		test_serial_to_tx_len--;
		return true;
	}
	else { return false; }
};

static bool TEST_SERIAL_PutByte(uint8_t value){
	if(test_serial_rxed_len > 0){
		test_serial_rxed_array[test_serial_rxed_index] = value;
		test_serial_rxed_index++;
		test_serial_rxed_len--;
		return true;
	}
	else { return false; }
};

static uint8_t TEST_HELPER_DallasCRC8_P(const uint8_t* data, const uint8_t size)
{
    uint8_t crc = 0;
    for ( uint8_t i = 0; i < size; ++i ){
        uint8_t inbyte = data[i];
        for ( uint8_t j = 0; j < 8; ++j ){
            uint8_t mix = (crc ^ inbyte) & 0x01;
            crc >>= 1;
            if ( mix ) crc ^= 0x8C;
            inbyte >>= 1;
        }
    }
    return crc;
}

static uint8_t TEST_HELPER_DallasCRC8_(uint8_t inbyte, uint8_t crc)
{
    for ( uint8_t j = 0; j < 8; ++j ){
        uint8_t mix = (crc ^ inbyte) & 0x01;
        crc >>= 1;
        if ( mix ) crc ^= 0x8C;
        inbyte >>= 1;
    }
    return crc;
}

static const ssp_init_str ssp_config_structure = {
	TEST_HELPER_DallasCRC8_,
	TEST_UART_GetByte,
	TEST_UART_PutByte,
	TEST_SERIAL_GetByte,
	TEST_SERIAL_PutByte,
};
	
ssp_str ssp_object = { 0 };
ssp_str* const ssp = &ssp_object;
	
static const ssp_init_str* const ssp_config = &ssp_config_structure;

void setUp (void) 
{ 
	test_uart_rxed_index = 0;
	test_uart_txed_index = 0;
	test_uart_len = 4096;

	memset(test_uart_array, 0, 4096);

	test_serial_rxed_index = 0;
	test_serial_to_tx_index = 0;
	test_serial_rxed_len = 4096;
	test_serial_to_tx_len = 0;
	
	memset(test_serial_rxed_array, 0, 4096);

	TEST_ASSERT_TRUE(SPP_Init(ssp, ssp_config)); 
}

void tearDown (void) {} /* Is run after every test, put unit clean-up calls here. */

uint8_t GetCollisionsCount(uint8_t* arr, uint8_t size)
{
	uint8_t col = 0;
	for (uint8_t i = 0; i < size; i++)
	{
		if(arr[i] == 0xFF or arr[i] == 0xAA){ col++; }
	}
	return col;
}

void CreateFrameWithSize(uint8_t payload_size, bool ex_result)
{
	payload_size = MIN(payload_size, INPUT_DATA_SIZE_MAX);
	
	// Find collisions count
	payload_size += GetCollisionsCount(test_serial_to_tx_array, payload_size);
	
	// Init test arrays
	test_serial_to_tx_len = payload_size;

	// Init expected values
	uint8_t ssp_old_id = ssp->tx.frame.id;
	uint8_t ex_id = GenerateNewID_(ssp_old_id);
	uint8_t ex_header_size = payload_size;
	uint8_t ex_total_size = payload_size + HEADER_SIZE;
	
	// Init expected crc8
	uint8_t ex_crc8 = 0;
	
	for (uint8_t i = 0; i < payload_size; i++)
	{
		uint8_t index = test_serial_to_tx_index + i;
		index &= 127;
		uint8_t value = test_serial_to_tx_array[index];
		if(value == 0xFF){
			ex_crc8 = TEST_HELPER_DallasCRC8_(0xAA, ex_crc8);
			ex_crc8 = TEST_HELPER_DallasCRC8_(1, ex_crc8);
			ex_header_size++;
			ex_total_size++;
		}
		else if(value == 0xAA){
			ex_crc8 = TEST_HELPER_DallasCRC8_(0xAA, ex_crc8);
			ex_crc8 = TEST_HELPER_DallasCRC8_(0, ex_crc8);
			ex_header_size++;
			ex_total_size++;
		}
		else {
			ex_crc8 = TEST_HELPER_DallasCRC8_(value, ex_crc8);
		}
	}
	ex_crc8 = TEST_HELPER_DallasCRC8_(ex_header_size, ex_crc8);
	ex_crc8 = TEST_HELPER_DallasCRC8_(ex_id, ex_crc8);

	// CRC8 Collision handling
	if(ex_crc8 == END_MARKER) { ex_crc8 = COLLISION_MARKER; }
	
	// Init indexes
	const uint8_t SIZE_INDEX = ex_header_size;
	const uint8_t ID_INDEX = ex_header_size + 1;
	const uint8_t CRC8_INDEX = ex_header_size + 2;
	const uint8_t END_INDEX = ex_header_size + 3;

	// TEST
	bool result = CreateFrame_(ssp);

	// Check results
	if(ex_result){
		// Range, because size also depend on collisions count.
		TEST_ASSERT_GREATER_OR_EQUAL_UINT8(ex_header_size - 1,	ssp->tx.frame.data[SIZE_INDEX]);
		TEST_ASSERT_LESS_OR_EQUAL_UINT8(ex_header_size,			ssp->tx.frame.data[SIZE_INDEX]);
		TEST_ASSERT_GREATER_OR_EQUAL_UINT8(ex_total_size - 1,	ssp->tx.frame.size);
		TEST_ASSERT_LESS_OR_EQUAL_UINT8(ex_total_size,			ssp->tx.frame.size);
		
		TEST_ASSERT_TRUE(result);
		TEST_ASSERT_EQUAL_HEX8(ex_crc8,		ssp->tx.frame.data[CRC8_INDEX]);
		TEST_ASSERT_EQUAL_UINT8(ex_id,		ssp->tx.frame.id);
		TEST_ASSERT_EQUAL_UINT8(ex_id,		ssp->tx.frame.data[ID_INDEX]);
		TEST_ASSERT_EQUAL_UINT8(END_MARKER,	ssp->tx.frame.data[END_INDEX]);
	
	}
	else {
		TEST_ASSERT_FALSE(result);
	}
}

void InitializeTransmitterWithRandomValues(void){
	ssp->tx.counter = 225;
	ssp->tx.data = (void*)0xFF98AA43;
	ssp->tx.size = 235;
}


#endif /* TEST_H_ */