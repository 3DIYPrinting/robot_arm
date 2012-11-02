/* 
 * File:   hklx.h
 * Author: toner
 *
 * Created on 22 August 2012, 5:26 PM
 */

#ifndef HKLX_H
#define	HKLX_H

#include "stdint.h"

//#define an_packet_pointer(packet) packet->header
//#define an_packet_size(packet) (packet->length + AN_PACKET_HEADER_SIZE)*sizeof(uint8_t)
//#define an_packet_crc(packet) ((packet->header[4]<<8) | packet->header[3])




//#define	COM_PORT                                0
#define LEG_0_COM 18
#define LEG_1_COM 16
#define LEG_2_COM 19
#define LEG_3_COM 17


#define PROTOCOL_SIZE_IDX			2
#define PROTOCOL_ID_IDX				3
#define PROTOCOL_CMD_IDX			4
#define PROTOCOL_CS1_IDX			5
#define PROTOCOL_CS2_IDX			6
#define PROTOCOL_DATA_IDX			7



#define GREEN                                   0x01
#define BLUE                                    0x02
#define RED                                     0x04

//#define PURPLE

#define HEADER					0xFF


#define MIN_PACKET_SIZE				7
#define MIN_ACK_PACKET_SIZE			9
#define MAX_PACKET_SIZE				223
#define MAX_DATA_SIZE				(MAX_PACKET_SIZE-MIN_PACKET_SIZE)


#define MAX_ID					0xFD    
#define BROADCAST_ID				0xFE  


#define CMD_EEP_WRITE				0x01
#define CMD_EEP_READ				0x02
#define CMD_RAM_WRITE				0x03
#define CMD_RAM_READ				0x04    
#define CMD_RW_DATA_ADDR_IDX    		7
#define CMD_RW_DATA_LEN_IDX            		8
#define CMD_I_JOG				0x05
#define CMD_I_JOG_STRUCT_SIZE                 	5
#define CMD_I_JOG_MAX_DRS                       (MAX_DATA_SIZE/CMD_I_JOG_STRUCT_SIZE)
#define CMD_S_JOG				0x06
#define CMD_S_JOG_STRUCT_SIZE                   4
#define CMD_S_JOG_MAX_DRS                       (MAX_DATA_SIZE/CMD_S_JOG_STRUCT_SIZE)
#define CMD_STAT				0x07    
#define CMD_ROLLBACK				0x08
#define CMD_REBOOT				0x09

#define CMD_MIN					(CMD_EEP_WRITE)
#define CMD_MAX					(CMD_REBOOT)

#define HKLX_PACKET_HEADER_SIZE                 2

#define CMD_ACK_MASK				0x40


#define CALIBRATE_MASK			        0x03FF
#define CMD_EEP_WRITE_ACK			(CMD_EEP_WRITE|CMD_ACK_MASK)
#define CMD_EEP_READ_ACK			(CMD_EEP_READ|CMD_ACK_MASK)
#define CMD_RAM_WRITE_ACK			(CMD_RAM_WRITE|CMD_ACK_MASK)
#define CMD_RAM_READ_ACK			(CMD_RAM_READ|CMD_ACK_MASK)
#define CMD_I_JOG_ACK				(CMD_I_JOG|CMD_ACK_MASK)
#define CMD_S_JOG_ACK				(CMD_S_JOG|CMD_ACK_MASK)
#define CMD_STAT_ACK				(CMD_STAT|CMD_ACK_MASK)
#define CMD_ROLLBACK_ACK			(CMD_ROLLBACK|CMD_ACK_MASK)
#define CMD_REBOOT_ACK				(CMD_REBOOT|CMD_ACK_MASK)

#define CMD_ACK_MIN				(CMD_EEP_WRITE_ACK)
#define CMD_ACK_MAX				(CMD_REBOOT_ACK)


#define CHKSUM_MASK				0xFE
#pragma pack(push, 1)
typedef struct {
    
    uint8_t ID;//0
    uint8_t ACK_Policy;//1
    uint8_t Alarm_LED_Policy;//2
    uint8_t Torque_Policy;//3
    
    uint8_t Reserved_1;//4
    
    uint8_t Max_Temperature;//5
    uint8_t Min_Voltage;//6
    uint8_t Max_Voltage;//7
    uint8_t Acceleration_Ratio;//8
    uint8_t Max_Acceleration;//9
    uint8_t Dead_Zone;//10
    uint8_t Saturator_Offset;//11
    uint16_t Saturator_Slope;//12
    uint8_t PWM_Offset;//14
    uint8_t Min_PWM;//15
    uint16_t Max_PWM;//16
    uint16_t Overload_PWM_Threshold;//18
    uint16_t Min_Position;//20
    uint16_t Max_Position;//22
    uint16_t Position_Kp;//24
    uint16_t Position_Kd;//26
    uint16_t Position_Ki;//28
    uint16_t Position_Feedforward_Gain_1;//30
    uint16_t Position_Feedforward_Gain_2;//32
    
    uint16_t Reserved_2;//was 16//34
    uint16_t Reserved_3;//was 16//36
    
    uint8_t LED_Blink_Period;//38
    uint8_t ADC_Fault_Detection_Period;//39
    uint8_t Packet_Garbage_Detection_Period;//40
    uint8_t Stop_Detection_Period;
    uint8_t Overload_Detection_Period;
    uint8_t Stop_Threshold;
    uint8_t Inposition_Margin;
    
    uint8_t Reserved_4;
    uint8_t Reserved_5;
    
    uint8_t Calibration_Difference;
    uint8_t Status_Error;
    uint8_t Status_Detail;
    
    uint8_t Reserved_6;    
    uint8_t Reserved_7;//was 16
    
    uint8_t Torque_Control;
    uint8_t LED_Control;
    
    uint8_t Voltage;//was 16//54
    uint8_t Temperature;//was 16//55
    uint8_t Current_Control_Mode;//was 16//56
    uint8_t Tick;//was 16//57
    uint16_t Calibrated_Position;//58
    uint16_t Absolute_Position;//60
    uint16_t Differential_Position;//62
    uint16_t PWM;//64
    
    uint16_t Reserved;//was 16//66
    
    uint16_t Absolute_Goal_Position;//68
    uint16_t Absolute_Desired_Trajectory_Position;//70
    uint16_t Desired_Velocity;//was 8//72

} _RAM;


typedef struct {

    uint8_t Model_No1;
    uint8_t Model_No2;
    uint8_t Version1;
    uint8_t Version2;
    uint8_t Baud_Rate;
    uint8_t Reserved;

    uint8_t ID;
    uint8_t ACK_Policy;
    uint8_t Alarm_LED_Policy;
    uint8_t Torque_Policy;
    
    uint8_t Reserved_1;
    
    uint8_t Max_Temperature;
    uint8_t Min_Voltage;
    uint8_t Max_Voltage;
    uint8_t Acceleration_Ratio;
    uint8_t Max_Acceleration_Time;
    uint8_t Dead_Zone;
    uint8_t Saturator_Offset;
    uint16_t Saturator_Slope;
    uint8_t PWM_Offset;
    uint8_t Min_PWM;
    uint16_t Max_PWM;
    uint16_t Overload_PWM_Threshold;
    uint16_t Min_Position;
    uint16_t Max_Position;
    uint16_t Position_Kp;
    uint16_t Position_Kd;
    uint16_t Position_Ki;
    uint16_t Position_Feedforward_Gain_1;
    uint16_t Position_Feedforward_Gain_2;
    
    uint16_t Reserved_2;//was 16
    uint16_t Reserved_3;//was 16
    
    uint8_t LED_Blink_Period;
    uint8_t ADC_Fault_Detection_Period;
    uint8_t Packet_Garbage_Detection_Period;
    uint8_t Stop_Detection_Period;
    uint8_t Overload_Detection_Period;
    uint8_t Stop_Threshold;
    uint8_t Inposition_Margin;
    
    uint8_t Reserved_4;
    uint8_t Reserved_5;
    
    uint8_t Calibration_Difference;
    
} _EEP;

typedef struct
{
    uint8_t header[2];
    uint8_t packetsize;
    uint8_t id;
    uint8_t cmd;
    uint16_t CRC;
    //uint8_t CRC2;
    uint8_t data[1];
} hklx_packet_t;


typedef struct{
    uint16_t value;
} jog_t;

typedef struct DrsSet {
    uint8_t stopflag : 1;
    uint8_t mode : 1;
    uint8_t LedGreen : 1;
    uint8_t LedBlue : 1;
    uint8_t LedRed : 1;
    uint8_t joginvalid : 1;
    uint8_t reserved : 2;
} set_t;

typedef struct {
    jog_t jog;
    set_t set;
    uint8_t id;
    uint8_t playtime;
} i_jog_t;

typedef struct {
    jog_t jog;
    set_t set;
    uint8_t id;
} s_jog_t;


#pragma pack(pop)

#define HKLX_DECODE_BUFFER_SIZE  2048

typedef struct
{
	uint8_t buffer[HKLX_DECODE_BUFFER_SIZE];
	uint16_t buffer_length;
	uint32_t crc_errors;
} hklx_decoder_t;


void hklx_decoder_initialise(hklx_decoder_t *hklx_decoder);
hklx_packet_t *hklx_packet_decode(hklx_decoder_t *hklx_decoder);
void hklx_packet_free(hklx_packet_t **hklx_packet);
hklx_packet_t *hklx_packet_allocate(uint8_t length, uint8_t id, uint8_t cmd);
void hklx_send(int leg,hklx_packet_t *hklx_packet);

#define hklx_decoder_pointer(hklx_decoder) &(hklx_decoder)->buffer[(hklx_decoder)->buffer_length]
#define hklx_decoder_size(hklx_decoder) (sizeof((hklx_decoder)->buffer) - (hklx_decoder)->buffer_length)
#define hklx_decoder_increment(hklx_decoder, bytes_received) (hklx_decoder)->buffer_length += bytes_received


typedef union {
    _EEP Register;//what do i call this???
    uint8_t data[54];
} _NV;

typedef union {
    _RAM Register;
    uint8_t data[74];
} _V;

/*
typedef struct {
    
    _V RAM;
    _NV EEP;
    
} _hklx;
*/

typedef struct {
    
    _RAM RAM;
    _EEP EEP;
    
    float abs_phi;
    float phi;  
    float vin;
    float temp;
    int pwm;
    
} _hklx;

#endif	/* HKLX_H */

