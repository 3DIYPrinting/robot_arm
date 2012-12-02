/*
 * File:   stag.h
 * Author: toner
 *
 * Created on 22 August 2012, 7:22 PM
 */

#ifndef STAG_H
#define	STAG_H

#include "hklx.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/l2cap.h>
#include "math.h"

#define PSMOVE 1
#define SIXAXIS 2
#define DS3 3
#define WIIMOTE 4


//#define BAUD 57600
#define BAUD 115200

//#define COM_PORT 18
//#define LEG_0_COM 18
//#define LEG_1_COM 16
//#define LEG_2_COM 19
//#define LEG_3_COM 17


#pragma pack(push, 1)
typedef struct
{
    uint8_t res0;//0
    uint16_t res1;//1
    uint8_t DIRECTION_PAD;//3
    uint8_t BUTTON_PAD;//4
    uint8_t PS_POWER_BUTTON;//5    //0-depressed; 1-pressed
    uint8_t res2;//6


    uint8_t L3_x;//7
    uint8_t L3_y;//8

    uint8_t R3_x;//9
    uint8_t R3_y;//10

    //JOYSTICK_POSITION LEFT_JOYSTICK;
    //JOYSTICK_POSITION RIGHT_JOYSTICK;
    uint16_t res3;//11
    uint16_t res4;//13
    uint16_t res5;//15
    uint16_t res6;//17
    uint8_t BTN_L2_VALUE;//19        //0-depressed; 255-fully pressed
    uint8_t BTN_R2_VALUE;//20        //0-depressed; 255-fully pressed
    uint8_t BTN_L1_VALUE;//21        //0-depressed; 255-fully pressed
    uint8_t BTN_R1_VALUE;//22        //0-depressed; 255-fully pressed
    uint16_t res7;//23
    uint16_t BTN_X;//25
    uint16_t res9;//27
    uint8_t res10;//29
    uint8_t res11;//30    //seems to always be 3
    uint8_t res12;//31    //seems to always be 239
    uint8_t MOTOR1;//32
    uint16_t res13;//33
    uint16_t res14;//35
    uint8_t res15;//37    //seems to always be 51
    uint8_t res16;//38    //seems to always be 60
    uint8_t res17;//39    //does change. possibly motor?
    uint8_t res18;//40    //seems to always be 1
    uint8_t MOTOR2;//41
    /*
    int16_t aX;
    int16_t aY;
    int16_t aZ;
    int16_t gZ;

     */


    uint8_t ax[2]; //42

    uint8_t ay[2]; //44
    uint8_t az[2]; //46
    uint8_t gz[2]; //48


    /*
    uint8_t ax1;//42    //Not for sure but appears to be 1 normally, then becomes 2 if the controller is hit
    uint8_t ax2;    //0-centered; left-positive; right-negative
    uint8_t ay1;      //seems to always be 2
    uint8_t ay2;   //0-centered; up-positive; down-negative
    uint8_t az1;      //seems to always be 1
    uint8_t az2;  //Not sure what axis this is. It does change with movement tho
    uint8_t gz1;      //seems to always be 1
    uint8_t gz2;      //accelerometer perhaps? values are positive when you move controller left and negative when
    */

    /*
    uint8_t HIT;//42    //Not for sure but appears to be 1 normally, then becomes 2 if the controller is hit
    uint8_t ROLL;    //0-centered; left-positive; right-negative
    uint8_t res19;      //seems to always be 2
    uint8_t PITCH;   //0-centered; up-positive; down-negative
    uint8_t res20;      //seems to always be 1
    uint8_t Z_AXIS;  //Not sure what axis this is. It does change with movement tho
    uint8_t res21;      //seems to always be 1
    uint8_t res22;      //accelerometer perhaps? values are positive when you move controller left and negative when
                //you move the controller right
     */
} _sixmotion_hidraw;

#pragma pack(pop)


typedef struct
{
    float L3_x;
    float L3_y;
    float R3_y;
    float R2;
    float R1;
    uint8_t BTN_X;
    uint16_t aX;
    uint16_t aY;

} _sixmotion;




struct motion_dev {
    int index;
    bdaddr_t addr;
    //enum { WIIMOTE, SIXAXIS, DS3, PSMOVE } type;
    int type;
    int csk;
    int isk;
    time_t latest_refresh;
    struct motion_dev *next;
};

#define L2CAP_PSM_HIDP_CTRL 0x11
#define L2CAP_PSM_HIDP_INTR 0x13

#define HIDP_TRANS_GET_REPORT    0x40
#define HIDP_TRANS_SET_REPORT    0x50
#define HIDP_DATA_RTYPE_INPUT    0x01
#define HIDP_DATA_RTYPE_OUTPUT   0x02
#define HIDP_DATA_RTYPE_FEATURE  0x03


typedef struct {

    int com_port;

    //from body cs
    float yaw;
    float pitch;
    float roll;

    float x;
    float y;
    float z;

    float l1;
    float l2;
    float l3;


} _leg;


class _stag {
public:


    _hklx hklx[254];

    //_leg leg[4];

    int init();

    void hklx_packet_parse(hklx_packet_t *hklx_packet);

    //static void io_handle(int status);
    //static void send_crc(void);
    //static void read_data(int status);



};

//_stag stag;
void dump(const char *tag, const unsigned char *buf, int len);
void parse_raw(_sixmotion_hidraw &raw, _sixmotion &sixm);
void fatal(char *msg);
int l2cap_listen(unsigned short psm);
struct motion_dev *accept_device(int csk, int isk);
void setup_device(struct motion_dev *dev);



#endif	/* STAG_H */

