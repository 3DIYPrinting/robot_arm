/*
 * File:   hklx.cpp
 * Author: toner
 *
 * Created on 22 August 2012, 7:58 PM
 */

#include <cstdlib>
#include "hklx.h"
#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include "rs232.h"
using namespace std;

/*
 * Function to dynamically allocate an hklx_packet
 */
hklx_packet_t *hklx_packet_allocate(uint8_t length, uint8_t id, uint8_t cmd) {
    hklx_packet_t *hklx_packet = (hklx_packet_t*)malloc(sizeof (hklx_packet_t) + length * sizeof (uint8_t));
    if (hklx_packet != NULL) {


        hklx_packet->header[0] = HEADER;
        hklx_packet->header[1] = HEADER;
        hklx_packet->packetsize = length + MIN_PACKET_SIZE;
        hklx_packet->id = id;
        hklx_packet->cmd = cmd;

    }
    return hklx_packet;
}

uint16_t calculate_crc(const void *data)
//uint16_t calculate_crc(uint8_t *data)
{

    uint8_t *bytes = (uint8_t *) data;

    uint16_t crc;
    /*
    uint8_t CheckSum1 = data[0] ^ data[1] ^ data[2];
    for (int i = 0; i < ((int)data[0] - MIN_PACKET_SIZE); i++)
        CheckSum1 ^= data[5 + i];

    uint8_t CheckSum2 = ~(CheckSum1);
    CheckSum1 &= CHKSUM_MASK;
    CheckSum2 &= CHKSUM_MASK;
     */

    uint8_t CheckSum1 = bytes[0] ^ bytes[1] ^ bytes[2];
    for (int i = 0; i < ((int) bytes[0] - MIN_PACKET_SIZE); i++)
        CheckSum1 ^= bytes[5 + i];

    uint8_t CheckSum2 = ~(CheckSum1);
    CheckSum1 &= CHKSUM_MASK;
    CheckSum2 &= CHKSUM_MASK;

    crc = CheckSum1;
    crc |= CheckSum2 << 8;


    return crc;
}

/*
 * Function to free an hklx_packet
 */
void hklx_packet_free(hklx_packet_t **hklx_packet)
{
	free(*hklx_packet);
	*hklx_packet = NULL;
}

/*
 * Initialise the decoder
 */
void hklx_decoder_initialise(hklx_decoder_t *hklx_decoder)
{
	hklx_decoder->buffer_length = 0;
	hklx_decoder->crc_errors = 0;
}

hklx_packet_t *hklx_packet_decode(hklx_decoder_t *hklx_decoder) {
    uint16_t decode_iterator = 0;
    hklx_packet_t *hklx_packet = NULL;
    uint8_t id, length, cmd, header1,header2;
    uint16_t crc;


    while (decode_iterator + MIN_PACKET_SIZE <= hklx_decoder->buffer_length) {
        //header_lrc = hklx_decoder->buffer[decode_iterator++];

        header1 = hklx_decoder->buffer[decode_iterator++];
        header2 = hklx_decoder->buffer[decode_iterator];

        if (header1 == HEADER && header2 == HEADER) {

            decode_iterator++;
            length = hklx_decoder->buffer[decode_iterator++];
            id = hklx_decoder->buffer[decode_iterator++];
            cmd = hklx_decoder->buffer[decode_iterator++];

            //crc = hklx_decoder->buffer[decode_iterator++];
            //crc |= hklx_decoder->buffer[decode_iterator++] << 8;

            crc = hklx_decoder->buffer[decode_iterator++];
            crc |= hklx_decoder->buffer[decode_iterator++] << 8;


            if (decode_iterator + length - MIN_PACKET_SIZE > hklx_decoder->buffer_length) {
                decode_iterator -= MIN_PACKET_SIZE;
                break;
            }

            if (crc == calculate_crc(&hklx_decoder->buffer[decode_iterator-MIN_PACKET_SIZE+2])) {
                hklx_packet = hklx_packet_allocate(length-MIN_PACKET_SIZE, id,cmd);
                if (hklx_packet != NULL) {
                    memcpy(hklx_packet->data, &hklx_decoder->buffer[decode_iterator], (length - MIN_PACKET_SIZE) * sizeof (uint8_t));
                }
                decode_iterator += length;
                break;
            } else {
                decode_iterator -= (MIN_PACKET_SIZE);
                hklx_decoder->crc_errors++;
            }
        }
    }
    if (decode_iterator < hklx_decoder->buffer_length) {
        if (decode_iterator > 0) {
            memmove(&hklx_decoder->buffer[0], &hklx_decoder->buffer[decode_iterator], (hklx_decoder->buffer_length - decode_iterator) * sizeof (uint8_t));
            hklx_decoder->buffer_length -= decode_iterator;
        }
    } else hklx_decoder->buffer_length = 0;

    return hklx_packet;
}

void hklx_send(int leg, hklx_packet_t *hklx_packet) {
    uint8_t i;

    hklx_packet->header[0] = HEADER;
    hklx_packet->header[1] = HEADER;

    hklx_packet->CRC = calculate_crc(&hklx_packet->packetsize);

    int sentb=0;

    //SendBuf(0, &hklx_packet->header[0], hklx_packet->packetsize);
    switch (leg) {

        case 0:
            sentb = SendBuf(LEG_0_COM, &hklx_packet->header[0], hklx_packet->packetsize);
            break;
        case 1:
            sentb = SendBuf(LEG_1_COM, &hklx_packet->header[0], hklx_packet->packetsize);
            break;
        case 2:
            sentb = SendBuf(LEG_2_COM, &hklx_packet->header[0], hklx_packet->packetsize);
            break;
        case 3:
            sentb = SendBuf(LEG_3_COM, &hklx_packet->header[0], hklx_packet->packetsize);
            break;

    }

    if (sentb != hklx_packet->packetsize)
        printf("THEFUCK??\n");


    //SendBuf(18, &hklx_packet->header[0], hklx_packet->packetsize);
    //hklx_packet_free(&hklx_packet);

    //USART0_PutNChar(&stPacket.ucHeader[0], stPacket.ucPacketSize);

    return;
}

/*
void hklx_send(hklx_packet_t *hklx_packet) {
    uint8_t i;

    hklx_packet->header[0] = HEADER;
    hklx_packet->header[1] = HEADER;

    hklx_packet->CRC = calculate_crc(&hklx_packet->packetsize);



    //SendBuf(0, &hklx_packet->header[0], hklx_packet->packetsize);
    SendBuf(18, &hklx_packet->header[0], hklx_packet->packetsize);
    hklx_packet_free(&hklx_packet);

    //USART0_PutNChar(&stPacket.ucHeader[0], stPacket.ucPacketSize);

    return;
}*/
