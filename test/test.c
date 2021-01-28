
/*
 *	Small serial protocol tests
 *
 *
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#include "test.h"

void test_generate_id(void);
void test_create_frame(void);
void test_create_ack(void);
void test_setup_transmitter_ack(void);
void test_setup_transmitter_frame(void);
void test_send_frame(void);
void test_send_ack(void);
void test_reception(void);

void test_reception(void)
{
	// Initialize serial
	uint8_t source_arr[20] = {
		35, 125, 159, 193, 0xFF, 28, 254, 
		0xFF, 0xFF, 191, 93, 0xAA, 128, 222, 
		60, 0xAA, 0xAA, 224, 2, 0xAA
	};
	const uint8_t payload_size = sizeof(source_arr);
	memcpy(test_serial_to_tx_array, source_arr, payload_size);
	test_serial_to_tx_len = payload_size;
	
	// Initialize test values
	const uint8_t payload_size_with_collision = 
		TEST_CalculateLenWithCollisions(
			&test_serial_to_tx_array[test_serial_rxed_index], 
			payload_size);
	
	// Initialize and run transmission
	bool is_not_transmitting;

	// Must return true on load
	is_not_transmitting = TransmissionHandler_(ssp);
	TEST_ASSERT_TRUE(is_not_transmitting);
	
	// Must return true than free
	is_not_transmitting = TransmissionHandler_(ssp);
	TEST_ASSERT_TRUE(is_not_transmitting);
	
	TEST_ASSERT_EQUAL_UINT8(payload_size_with_collision + HEADER_SIZE, test_uart_rxed_index);
	
	// Run reception
	ssp_rx_answer_enum answer = NOTHING_RECEIVED;
	for(uint8_t i = 0; i < 255; i++){
		answer = ReceptionHandler_(ssp);
		if(answer != NOTHING_RECEIVED) { break;}
	}
	
	// Check received frame
	switch(answer){
		default: TEST_FAIL_MESSAGE("Unexpected unknown message form handler!"); break;
		case NOTHING_RECEIVED: TEST_FAIL_MESSAGE("Nothing received form handler!"); break;
		case ACK_RECEIVED: TEST_FAIL_MESSAGE("Unexpected ACK received!"); break;
		case BROKEN_RECEIVED: TEST_FAIL_MESSAGE("Unexpected broken message received!"); break;
		case FRAME_RECEIVED: break;
	}
}

int main(void)
{
	UNITY_BEGIN();
	RUN_TEST(test_generate_id);
	
	RUN_TEST(test_create_ack);
	RUN_TEST(test_create_frame);
	
	RUN_TEST(test_setup_transmitter_ack);
	RUN_TEST(test_setup_transmitter_frame);
	
	RUN_TEST(test_send_frame);
	RUN_TEST(test_send_ack);
	
	RUN_TEST(test_reception);

	return UNITY_END();
}

void test_send_ack(void){

	// Initialize test values
	const uint8_t expected_uart_len = HEADER_SIZE; //
	
	// ACK request
	ssp->tx.ack.id = GenerateNewID_(42); // Test id
	
	bool is_not_transmitting;

	// Must return true on load
	is_not_transmitting = TransmissionHandler_(ssp);
	TEST_ASSERT_TRUE(is_not_transmitting);
	
	// Must return true than free
	is_not_transmitting = TransmissionHandler_(ssp);
	TEST_ASSERT_TRUE(is_not_transmitting);
	
	// Output array has expected data
	TEST_ASSERT_EQUAL_UINT8(expected_uart_len, test_uart_rxed_index);
	
	// No ack
	TEST_ASSERT_EQUAL_UINT8(ID_NONE, ssp->tx.ack.id);
	
	// No added frame
	TEST_ASSERT_EQUAL_UINT8(ID_NONE, ssp->tx.frame.id);
	TEST_ASSERT_EQUAL_UINT8(0, ssp->tx.timeout);

}

void test_send_frame(void)
{
	uint8_t len = MIN(30, INPUT_DATA_SIZE_MAX);
	const uint8_t payload_size = len;
	
	// Initialize test arrays
	test_serial_to_tx_len = payload_size;	// Less than max parcel size
	
	// Initialize test values
	const uint8_t payload_size_with_collision = 
		TEST_CalculateLenWithCollisions(
			&test_serial_to_tx_array[test_serial_to_tx_index], 
			payload_size);
	
	const uint8_t expected_uart_len = payload_size_with_collision + HEADER_SIZE; //

	bool is_not_transmitting;
	
	// Must return true on load
	is_not_transmitting = TransmissionHandler_(ssp);
	TEST_ASSERT_TRUE(is_not_transmitting);
	
	// Must return true than free
	is_not_transmitting = TransmissionHandler_(ssp);
	TEST_ASSERT_TRUE(is_not_transmitting);
	
	// Output array has expected data
	TEST_ASSERT_EQUAL_UINT8(expected_uart_len, test_uart_rxed_index);
	
	// Is awaiting ACK
	TEST_ASSERT_GREATER_THAN_UINT8(0, ssp->tx.frame.id);
	TEST_ASSERT_GREATER_THAN_UINT8(0, ssp->tx.timeout);

}


void test_setup_transmitter_frame(void)
{
	// Preinit ssp
	ssp->tx.timeout = 255;
	ssp->tx.counter = 255;
	ssp->tx.data = NULL;
	ssp->tx.size = 255;
	
	// ACK Creation
	CreateFrameWithSize(14, true);
	
	// TEST
	SetupTransmitterForFrame_(ssp);
	
	// Check results
	TEST_ASSERT_EQUAL_PTR(ssp->tx.frame.data,		ssp->tx.data);
	TEST_ASSERT_EQUAL_UINT8(ssp->tx.frame.size,		ssp->tx.size);
	TEST_ASSERT_EQUAL_UINT8(0,						ssp->tx.counter);
	TEST_ASSERT_EQUAL_UINT8(0,						ssp->tx.timeout);

}	

void test_setup_transmitter_ack(void)
{
	// Init expected values
	uint8_t generated_id = GenerateNewID_(0);
	uint8_t expected_size = HEADER_SIZE;
	uint8_t expected_id = generated_id;
	
	// Init expected header
	ssp_frame_header_str expected_header = {expected_size, expected_id, 0, END_MARKER};
		
	// Init expected crc8
	expected_header.crc8 = TEST_HELPER_DallasCRC8_P((uint8_t*)&expected_header, 2);

	// CRC8 Collision handling
	if(expected_header.crc8 == END_MARKER) { expected_header.crc8 = COLLISION_MARKER; }
	
	const uint8_t SIZE_INDEX = 0;
	const uint8_t ID_INDEX = SIZE_INDEX + 1;
	const uint8_t CRC8_INDEX = SIZE_INDEX + 2;
	const uint8_t END_INDEX = SIZE_INDEX + 3;
	
	
	// ACK Creation
	CreateAck_(ssp, generated_id);
	
	// TEST
	SetupTransmitterForAck_(ssp);
	
	// Check results
	TEST_ASSERT_EQUAL_PTR(&ssp->tx.ack,				ssp->tx.data);
	TEST_ASSERT_EQUAL_UINT8(expected_header.size,	ssp->tx.size);
	TEST_ASSERT_EQUAL_UINT8(0,						ssp->tx.counter);
	
	TEST_ASSERT_EQUAL_UINT8(expected_header.size,	ssp->tx.data[SIZE_INDEX]);
	TEST_ASSERT_EQUAL_UINT8(expected_header.id,		ssp->tx.data[ID_INDEX]);
	TEST_ASSERT_EQUAL_UINT8(expected_header.crc8,	ssp->tx.data[CRC8_INDEX]);
	TEST_ASSERT_EQUAL_UINT8(END_MARKER,				ssp->tx.data[END_INDEX]);
}

void test_generate_id(void)
{
	uint8_t old_id = GenerateNewID_(ID_NONE);
	TEST_ASSERT_NOT_EQUAL_UINT8(ID_NONE, old_id);
	for(int i = 0; i < 1000; i++){
		uint8_t new_id = GenerateNewID_(old_id);
		TEST_ASSERT_NOT_EQUAL_UINT8(ID_NONE, new_id);
		TEST_ASSERT_LESS_OR_EQUAL(ID_MAX, new_id);
		TEST_ASSERT_GREATER_OR_EQUAL(ID_MIN, new_id);
		TEST_ASSERT_NOT_EQUAL_UINT8(COLLISION_SYMBOL, new_id);
		TEST_ASSERT_NOT_EQUAL_UINT8(COLLISION_MARKER, new_id);
		TEST_ASSERT_LESS_THAN(COLLISION_MARKER, new_id);
		TEST_ASSERT_LESS_THAN(COLLISION_SYMBOL, new_id);
		
		TEST_ASSERT_NOT_EQUAL_UINT8(old_id, new_id);
		old_id = new_id;
	}
}


void test_create_ack(void)
{
	// Init expected values
	uint8_t generated_id = GenerateNewID_(0);
	uint8_t expected_size = HEADER_SIZE;
	uint8_t expected_id = generated_id;
	
	// Init expected header
	ssp_frame_header_str expected_header = {expected_size, expected_id, 0, END_MARKER};
		
	// Init expected crc8
	expected_header.crc8 = TEST_HELPER_DallasCRC8_P((uint8_t*)&expected_header, 2);

	// CRC8 Collision handling
	if(expected_header.crc8 == END_MARKER) { expected_header.crc8 = COLLISION_MARKER; }
	
	// TEST
	CreateAck_(ssp, generated_id);
	
	// Check results
	TEST_ASSERT_EQUAL_UINT8(expected_header.size,	ssp->tx.ack.size);
	TEST_ASSERT_EQUAL_UINT8(expected_header.id,		ssp->tx.ack.id);
	TEST_ASSERT_EQUAL_UINT8(expected_header.crc8,	ssp->tx.ack.crc8);
	TEST_ASSERT_EQUAL_UINT8(END_MARKER,				ssp->tx.ack.end);
}

void test_create_frame(void){
	
	CreateFrameWithSize(0, false); 
	CreateFrameWithSize(255, true);
	CreateFrameWithSize(PAYLOAD_SIZE_MAX, true); 
	
	for (uint8_t i = 0; i < 10; i++ ){ 
		for (uint8_t j = 1; j > 0; j++ ){ 
			CreateFrameWithSize(j, true); 
			CreateFrameWithSize(0, false);
		}
	}
	
	for (uint8_t i = 0; i < 128; i++ ){ 
		for (uint8_t j = 1; j <= PAYLOAD_SIZE_MAX; j++ ){ 
			CreateFrameWithSize(j, true); 
			CreateFrameWithSize(0, false);
		}
	}
	for (uint8_t i = 0; i < 128; i++ ){ 
		CreateFrameWithSize(PAYLOAD_SIZE_MAX, true); 
		CreateFrameWithSize(0, false);
	}
}

#ifdef __cplusplus
}
#endif