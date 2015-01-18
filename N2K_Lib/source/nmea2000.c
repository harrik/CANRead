/* Copyright (c) 2009 Philipp Drewes
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */
// ----------------------------------------------------------------------------


#include <avr/pgmspace.h>
#include <string.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>


#include "nmea2000.h"
#include "nmea2000_defs.h"

#include "mcp2515.h"
#include "nmea0183.h"


#define NMEA2000_MAX_RECEIVE_BUFFERS 10
#define NMEA2000_RECEIVE_BUFFER_SIZE 223

// defined in PGM space
typedef struct{
	prog_uint32_t uniquenumber_mfccode;
	prog_int8_t   deviceinstande;
	prog_int8_t   isofunction;
	prog_int8_t   deviceclass;
	prog_int8_t   sysinst_igroup_selfconfig;
} tNMEA2000_ClaimAddress;

const tNMEA2000_ClaimAddress PROGMEM NMEA2000_ClaimAddress = {
	((uint32_t)NMEA2000_UNIQUENUMBER & 0x1FFFFF) | (((uint32_t)NMEA2000_ManufatureCode & 0x07FF) << 21),
	((NMEA2000_DeviceInstanceLower) & 0x07) | (((NMEA2000_DeviceInstanceUpper) & 0x1F) << 3),
	(NMEA2000_FunctionName),
	(((NMEA2000_DeviceClassName) & 0x7F) <<1 ),
	((NMEA2000_SystemInstance & 0x0F) | ((NMEA2000_IndustryGroup & 0x07) << 4) | ((NMEA2000_ArbitraryAccessCapable & 0x01) << 7))
};

typedef struct {
	uint16_t    databaseversion;
	uint16_t    productcode;
//	const char  *pmodelid;
//	const char  *psoftwareversion;
//	const char  *pmodelversion;
//	const char  *pmodelserialcode;
	const char	pmodelid[32];
	const char	psoftwareversion[32];
	const char	pmodelversion[32];
	const char	pmodelserialcode[32];
	uint8_t     certificationlevel;
	uint8_t     loadequivalancy;	
} tNMEA2000_ProductInformation;

const char modelid[32]         = NMEA2000_MODELID;
const char softwareversion[32] = NMEA2000_SOFTWAREVERSIONCODE;
const char modelversion[32]    = NMEA2000_MODELVERSION;
const char modelserialcode[32] = NMEA2000_MODELSERIALCODE;

const tNMEA2000_ProductInformation PROGMEM NMEA2000_ProductInformation = {
		NMEA2000_DATABASEVERSION,
		NMEA2000_PRODUCTCODE,
		NMEA2000_MODELID, //modelid,		
		NMEA2000_SOFTWAREVERSIONCODE, //softwareversion,
		NMEA2000_MODELVERSION,	//modelversion,
		NMEA2000_MODELSERIALCODE,  //modelserialcode,
		NMEA2000_CERTIFICATIONLEVEL,
		NMEA2000_LOADEQUIVALENCE
};


// NMEA2000 messages that this device will receive

static tNMEA2000ReceiveMessage NMEA2000ReceiveMessage[] = {/*{0x1F801,   8, NMEA2000_RECEIVE_BUFFER_LAST},*/ 		/* Position rapid update (PGN: 129025) */ \
												    		 {0x1F805,  47, NULL/*nmea0183_PGN129025_to_GLL*/}};		/* GNSS Position Date (PGN: 129029) */





/**
* This array provides the buffer space for the NMEA2000 messages that can be received simultaniously.
* Each array element is a struct of type NMEA2000ReceiveBuffer. The array buffer space is used for single as
* well as for multi-frame messages.
* For convenience the buffer space will later be accessed thru a linear list instead of indexed array read or write commands.
*/

static tNMEA2000ReceiveBuffer NMEA2000ReceiveBuffer[NMEA2000_MAX_RECEIVE_BUFFERS];


// linear list of received messages
// initialized as an empty list
static tNMEA2000ReceiveBuffer *complete_received_list = NULL;


/**
* This pointer forms the root of a linear list that includes all receive buffers currently
* available for use. The elements of this list are taken from the array NMEA2000ReceiveBuffer
* and lines up during initialization.
*/

static tNMEA2000ReceiveBuffer *free_buffer_list;

// static filters with the basic definition in FLASH
// necessary dynamic adjustments are taken care of later with the dynamic filter definition
// filter 1 is also assigned to RXB0. It uses the filter mask defined with filter 0.
// filter 1 is configured such that all Point-to-Point messages with the broadcast destination (0xFF) are received

/**
* This structure defines a filter configuration for the MCP2515 in flash memoy. It is set once during
* initilization of the NMEA2000 stack and defines the general filter settings required. Some settings
* are adjusted later according to a maybe changing source address of the ECU.
* Receive buffer 0  is used to receive process data messages, according to the PGNs bit masks defined
* in filter 0 and filter 1. As rollover to receive buffer 1 is enabled, this buffer will be used as backup for 
* fast arriving messages. The filter mask 0 is set to 0x01FFFF00 which only checks the PGN value of a message
* but not the source address.
* Receivebuffer 1 handles all network management messages and P2P messages. The filters are set
* to receive broadcasts (0xFF) in Filter 3 (2nd byte) and all messages directly addressed to this ECU.
* Here filter 2 is set to 0 for the moment. This will be adjusted later when setting the filterparameters
* for the device address.
*/

const prog_uint8_t static_can_filter[] = {
	MCP2515_FILTER_EXTENDED(0x01F80500),	// Filter 0 (Processdata)
  	MCP2515_FILTER_EXTENDED(0x00000000),	// Filter 1 
  	MCP2515_FILTER_EXTENDED(0x00000000),	// Filter 2 (receive P2P-Message with local destination)
  	MCP2515_FILTER_EXTENDED(0x0000FF00),	// Filter 3(receive P2P-Message send as broadcast messages)
  	MCP2515_FILTER_EXTENDED(0x00000000),	// Filter 4
  	MCP2515_FILTER_EXTENDED(0x00000000),	// Filter 5
  	
  	MCP2515_FILTER_EXTENDED(0x01FFFF00),	// Maske 0 (to be adjusted)
  	MCP2515_FILTER_EXTENDED(0x0000FF00)		// Maske 1 (only check for the destination address)
};

/**
* Set up a structure that contains a dynamic filter setting. This filter setting is adjusted according to
* the used source address of our ECU and then programmed into Filter 2
* @see static_can_filter[]
*/

can_filter_t dynamic_can_filter = {
	0x00000000,		// Filter ID
	0x0000FF0,		// Filter Mask
	{0x3,				// accepr frames with extended id only (always 3 with NMEA2000)
	 0}				// extended ID (always 1 with NMEA2000)	
};

static uint8_t fast_packet_index;

static uint8_t source_address;

uint8_t nmea2000_init(void){

	// initialilze the CAN-Bus controller
	// bitrate is set to 250kbps when using a 4MHz crystal
	// if required make the necessary changes in "mcp2515_init()"

	// ToDO: implement exception handling checking return values

	mcp2515_init();

	// has to be initialized via a parameter read from the EEPROM later
	source_address = eeprom_read_byte(NMEA2000_EEPROM_DEVICE_ID_ADDRESS);

	// are we facing an empty eeprom?
	if (source_address == 0xFF){
		source_address = 10;
		eeprom_write_byte(0x0000,source_address);
	}

	mcp2515_static_filter(static_can_filter);

	// filter 1 is configured such that all Point-to-Point messages with the destination of this device are received
	// the filter mask is set such that only the destination address is considered.
	dynamic_can_filter.id   = (uint16_t) source_address << 8;
	mcp2515_set_filter(2, &dynamic_can_filter);

	// at initialization all receve buffers are  empty. this is expressed inserting all buffers into the free buffer list
	free_buffer_list = NMEA2000ReceiveBuffer;
	for (uint8_t i=0; i < NMEA2000_MAX_RECEIVE_BUFFERS-1; i++)
		NMEA2000ReceiveBuffer[i].next = &NMEA2000ReceiveBuffer[i+1];

	NMEA2000ReceiveBuffer[NMEA2000_MAX_RECEIVE_BUFFERS-1].next = NULL;	

	return 0;
}

uint8_t nmea2000_getDeviceID(void){
	return source_address;
}



void nmea2000_iso_request (uint8_t destination_address, uint32_t requestedPGN){

	tCAN message;

	message.id =  ((uint32_t) (PRIORITY_LEVEL & 0x07) << 26) | ((uint32_t)(PGN_REQUEST) << 8) | ((uint16_t)destination_address) << 8 | source_address;
	message.length = 3;
	memcpy(message.data,(uint8_t*) &requestedPGN,3);

	while(!mcp2515_send_message(&message))
		mt_Robin();

}


void nmea2000_iso_address_claim (uint8_t destination_address){

	tCAN message;
	
	message.id =  ((uint32_t) (PRIORITY_LEVEL & 0x07) << 26) | ((uint32_t)(PGN_ADDRESSCLAIM) << 8) | ((uint16_t)destination_address) << 8 | source_address;
	message.length = sizeof( tNMEA2000_ClaimAddress);
	memcpy_P(message.data, &NMEA2000_ClaimAddress, sizeof( tNMEA2000_ClaimAddress ));
	while(!mcp2515_send_message(&message))
		mt_Robin();

}

void nmea2000_fast_packet_transmission (uint32_t *message_id, uint8_t size, uint8_t *payload){

		tCAN message;

		message.id = *message_id;
		message.length = 8;
		message.data[0] = fast_packet_index++;
		message.data[1] = size;
	
		// transmission of 6 bytes of payload with the fist data packet
		memcpy_P(message.data+2, payload, 6);
		while(!mcp2515_send_message(&message))
			mt_Robin();

		
		// intermediate payload bytes
		
		for (uint8_t i=0; i < (size-6)/7; i++){
			message.data[0] = fast_packet_index++;
			memcpy_P(message.data + 1, payload +6 +7*i, 7);
			while(!mcp2515_send_message(&message))
				mt_Robin();

		}
	
		// assembly of the last payload packet; unused bytes are filled with 0xff
		uint8_t tmp = (size-6)%7;
	
		message.data[0] = fast_packet_index++;
		memcpy_P(message.data + 1, payload + 132, tmp); 
		for (uint8_t i=tmp+1; i<8; i++)
			message.data[i] = 0xff; 
		while(!mcp2515_send_message(&message))
			mt_Robin();

	
		fast_packet_index = (fast_packet_index & 0xf0) + 0x10;
	}


void nmea2000_send_product_information (void){

	uint32_t message_id;
	message_id =  ((uint32_t) (PRIORITY_LEVEL & 0x07) << 26) | (((uint32_t)(PGN_PRODUCTINFORMATION) & 0x01FFFF) << 8) | source_address;

	nmea2000_fast_packet_transmission(&message_id, sizeof( tNMEA2000_ProductInformation ), (uint8_t*)&NMEA2000_ProductInformation);
	
}

void nmea2000_transmit_message (tNMEA2000TransmitMessage *message_container){

	tCAN message;

	if ((((message_container->PGN >> 8) & 0x01ffff)>>8)<240)
		message.id = ((uint32_t) (message_container->priority & 0x07) << 26) | (((uint32_t)message_container->PGN & 0x01FFFF) << 8) | source_address;
	else
		message.id = ((uint32_t) (message_container->priority & 0x07) << 26) | (((uint32_t)message_container->PGN & 0x01FF00) << 8) | message_container->destination << 8 |source_address;

	// do we need more than one CAN frame to transmit out message?
	if ( message_container->payload_size <= 8){

		message.length = message_container->payload_size;
		
		memcpy(message.data,message_container->payload,message_container->payload_size);
		while(!mcp2515_send_message(&message))
			mt_Robin();

	}
	else{
	
		message.length = 8;
		
		message.data[0] = fast_packet_index++;
		message.data[1] = message_container->payload_size;
	
		// transmission of 6 bytes with the fist data packet
		memcpy(message.data+2, (uint8_t*)(message_container->payload), 6);
		while(!mcp2515_send_message(&message))
			mt_Robin();
		
		for (uint8_t i=0; i < (message_container->payload_size-6)/7; i++){
			message.data[0] = fast_packet_index++;
			memcpy_P(message.data + 1,(uint8_t*)(message_container->payload) + 6 +7*i, 7);
			while(!mcp2515_send_message(&message))
				mt_Robin();
		}
	
		// assembly of the last packet. Unused bytes are filled with 0xff
		uint8_t tmp = (message_container->payload_size-6)%7;
	
		message.data[0] = fast_packet_index++;
		memcpy_P(message.data + 1, (uint8_t*)(message_container->payload) + 132, tmp); 
		for (uint8_t i=tmp+1; i<8; i++)
			message.data[i] = 0xff; 
		while(!mcp2515_send_message(&message))
			mt_Robin();

		fast_packet_index = (fast_packet_index & 0xf0) + 0x10;
	}

}


void nmea2000_get_message(void){

	tCAN message;
	uint32_t requested_pgn;

	tNMEA2000ReceiveBuffer *current_ReceiveBuffer;


	if (mcp2515_check_message()) {
		
		// fetch the message from the MCP2515 an store it into the message struct
		if (mcp2515_get_message(&message)) {

			// remove the source_address of received messages to perform process checking
			uint32_t modified_id = (message.id >> 8) & 0x01ffff;
			// when the received message is destination specific (PDU1; PF values 0 to 239), set destination address to 0x00 to
			// allow for easier checking
			if ((modified_id>>8) < 240)
				modified_id &= 0x01ff00 ;

			// check if the received message is listed in "NMEA2000ReceiveMessage" and thus a message to be considered by this ECU
			for (uint8_t i=0; i<(sizeof(NMEA2000ReceiveMessage)/sizeof(tNMEA2000ReceiveMessage)); i++){
			
				// message needs to be considered
				if 	(modified_id == NMEA2000ReceiveMessage[i].PGN){

					//are we dealing with a single frame message?
					if (NMEA2000ReceiveMessage[i].payload_size < 9){
						// take care of single frame management -> then we are done!
						return;
					}

					// receiving a multi frame message
					// pointer to the precesessor allows easy manipulation of the implemented linear list 
					tNMEA2000ReceiveBuffer *current_buffer_predecessor = NULL;

					// are we currently in the process of receiving any message with the given PGN ?
					if (NMEA2000ReceiveMessage[i].receivebuffer_list != NULL){
						current_ReceiveBuffer = NMEA2000ReceiveMessage[i].receivebuffer_list;

						// check if we are currently filling a buffer for a message with the given source
						for(;;){

							if (current_ReceiveBuffer->source == ((uint8_t)message.id & 0xFF)){

								// the sequence counter is always the first byte of any multi message CAN frame
								uint8_t message_sequence_counter = message.data[0] & 0x1F;

								// message assembly has not finished successfully
								// this should never happen; it happens if we miss a CAN frame
								if (message_sequence_counter == 0){
									// reset "bytes remaining" counter
									current_ReceiveBuffer->processed_bytes = 6;
									// overwrite first 6 bytes in from the first message frame
									memcpy(current_ReceiveBuffer->payload, message.data + 2, 6);
									return;
								}
							
								// payload index at which to insert the data from the current CAN frame
								uint8_t payload_insertindex = (message_sequence_counter-1)*7 +6;
							
								// include the data from the current CAN frame into the multi packet message
							
								// are we dealing with the last CAN frame of the multiframe packet?								
								if (message_sequence_counter < (NMEA2000ReceiveMessage[i].payload_size) / 7 ){									
									// no, this is an intermediate CAN frame
									memcpy(current_ReceiveBuffer->payload + payload_insertindex, message.data + 1, 7);
									current_ReceiveBuffer->processed_bytes += 7;
								}
								else{
									// last CAN frame 

									uint8_t bytes_in_this_frame = (NMEA2000ReceiveMessage[i].payload_size - 6) % 7;									
									memcpy(current_ReceiveBuffer->payload + payload_insertindex, message.data + 1, bytes_in_this_frame);
									current_ReceiveBuffer->processed_bytes += bytes_in_this_frame;
								}

								// move all "completed buffers" to the "completed buffers list" 
								if (current_ReceiveBuffer->processed_bytes == NMEA2000ReceiveMessage[i].payload_size){
									// move buffer to completely received message list
									tNMEA2000ReceiveBuffer *tmp = complete_received_list;
									complete_received_list = current_ReceiveBuffer;

									if (current_buffer_predecessor == NULL)
										// we are looking at the very first element of the list
										NMEA2000ReceiveMessage[i].receivebuffer_list = current_ReceiveBuffer->next;
									else
										// we are looking at an intermediate element
										current_buffer_predecessor->next = current_ReceiveBuffer->next;
									
									current_ReceiveBuffer->next = tmp;									
								}

								if (current_ReceiveBuffer->processed_bytes > NMEA2000ReceiveMessage[i].payload_size){
									
									// remove from active list / delete from active list
									if (current_buffer_predecessor == NULL)
										// we are looking at the very first element of the list
										NMEA2000ReceiveMessage[i].receivebuffer_list = current_ReceiveBuffer->next;
									else
										// we are looking at an intermediate element
										current_buffer_predecessor->next = current_ReceiveBuffer->next;

									tNMEA2000ReceiveBuffer *tmp = free_buffer_list;
									free_buffer_list = current_ReceiveBuffer;
									current_ReceiveBuffer->next = tmp;							
								}
							
								return;
							}
							
							if (current_ReceiveBuffer->next != NULL){
								current_buffer_predecessor = current_ReceiveBuffer;
								current_ReceiveBuffer = current_ReceiveBuffer->next;
							}
							else
								break;
						}						
					}					

					// we are not filling a buffer for the current PGN frame at all; start a new buffer

					// is a buffer available
					if (free_buffer_list != NULL){
						
						// insert new buffer into the list of PGNs beeing processed
						// rearrage linear list elements
						// "current_buffer_predecessor" is only NULL if "current_ReceiveBuffer" is the very first element in the linear list
						if (current_buffer_predecessor == NULL)
							NMEA2000ReceiveMessage[i].receivebuffer_list = free_buffer_list;						
						else
							current_ReceiveBuffer->next = free_buffer_list;						
						
						current_ReceiveBuffer =	free_buffer_list;

						// remove the buffer we are using for the new message from the empty_list
						free_buffer_list = free_buffer_list->next;
						current_ReceiveBuffer->next = NULL;
						current_ReceiveBuffer->identifier = i;
						current_ReceiveBuffer->processed_bytes = 6;
						current_ReceiveBuffer->source = (uint8_t)(message.id);
						// insert first 6 bytes in from the first message frame
						memcpy(current_ReceiveBuffer->payload, message.data + 2, 6);
						return;
					}
					return ;
				}				
			}		


			// the following part of "nmea2000_get_message" processes the CAN messages required for network management

			switch(modified_id){
				// acknowledge message received
				case PGN_ACKNOWLEDGE :
					// ToDo: Implement transmission of negative acknowledge when
					// receiving transmission requests for not supportet messages
				break;
				// request message received; we must reply according to the PGN sent
				// through the payload
				case PGN_REQUEST :
					requested_pgn = (uint32_t)message.data[2]<<16 | (uint16_t)message.data[1]<<8 | message.data[0];
					switch(requested_pgn){
						case PGN_ADDRESSCLAIM: nmea2000_iso_address_claim( (uint8_t) message.id); break;
						case PGN_PRODUCTINFORMATION: nmea2000_send_product_information(); break;
						default: ; break; // to do --> send "function not implemented"
					}
				break;				
				// some other device has sent an address claim message (either directly to this ECU or as a broadcast)
				// we need to check if we have to choose a different device address
				// this is the case if someone claims our current device addres and has a lower NAME value
				case PGN_ADDRESSCLAIM :
					// if the desired device address matches our current source address
					if ((uint8_t) message.id == source_address){
					
						// assume that we do not have to choose a new ID
						for (uint8_t i=0;i<8;i++){
							if	(message.data[i] < pgm_read_byte((uint8_t*)&NMEA2000_ClaimAddress + i)){
								// choose a new source address
								source_address +=1;
								// adjuste the filtersettings accordingly
								dynamic_can_filter.id = (uint16_t)source_address << 8;
								mcp2515_set_filter(2, &dynamic_can_filter);

								// save new source_address to EEPROM
								eeprom_write_byte(NMEA2000_EEPROM_DEVICE_ID_ADDRESS, source_address);
//								ToDo: Stop periodic transmission of data packages during device id reassignment
								break;
							}							
							
						}
						
						// try to claim new ID or let the device know that the desired address is in use
						nmea2000_iso_address_claim(0xFF);
					}
				break;
			}
			
		}
	}
}


// process data from the complete list

void nmea2000_process_complete_list(void){

	if (complete_received_list != NULL){

		// execute function associated with the  message
//		NMEA2000ReceiveMessage[complete_received_list->identifier].func(NMEA2000ReceiveMessage);
		nmea0183_PGN129025_to_GLL_GGA(complete_received_list);


		tNMEA2000ReceiveBuffer *tmp1 = complete_received_list;
		tNMEA2000ReceiveBuffer *tmp2 = free_buffer_list;

		free_buffer_list = tmp1;

		complete_received_list = tmp1->next;

		tmp1->next = tmp2;

	}
	
}

