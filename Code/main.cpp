#include <cstdlib>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <vector>
#include <math.h>
#include <cmath>

#include <sys/signal.h>
#include <sys/types.h>
#include <sys/time.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/l2cap.h>

#include "rs232.h"
//#include "hklx.h"
#include "stag.h"
/*
#include <cppconn/driver.h>
#include <cppconn/exception.h>
#include <cppconn/resultset.h>
#include <cppconn/statement.h>
#include <cppconn/prepared_statement.h>
*/
#define NO_PWM 6

#define L1 15.0
#define L2 130.0
#define L3 130.0
#define L4 70.0

#define PSMOVE 1
#define SIXAXIS 2
#define DS3 3
#define WIIMOTE 4

using namespace std;

uint8_t inbuf[4095];

uint8_t arm_roll=90;
uint8_t arm_pitch=90;

uint8_t test_angle=0;
uint8_t test_inc=5;

const float rad_180 = (M_PI);
const float rad_170 = (M_PI*0.944444444);
const float rad_156 = (M_PI*0.866666666);
const float rad_78 = (M_PI*0.4333333333);
const float rad_10 = (M_PI*0.0555555555);

float Servos_Max_rad[6] = {rad_180, rad_170, rad_170, rad_170, rad_180, rad_156};
float Servos_Min_rad[6] = {0.0, rad_10,rad_10,rad_10,0.0,rad_78};

int8_t openport()
{
    if (!OpenComport(16, 9600)) return 16;
    if (!OpenComport(17, 9600)) return 17;
    if (!OpenComport(18, 9600)) return 18;

    return -1;
}

float rad(float degree)
{
    return degree * (M_PI/180.0);
}

float deg(float radian)
{
    return radian * (180.0 / M_PI);
}

uint8_t ang_to_pos(float x)
{
    float y = (deg(x)*(255.0/180.0));
    if(y<0.0) y=0.0;
    if(y>255.0) y=255.0;
    return (uint8_t)y;
}

uint8_t reverse_ang(uint8_t x)
{
    int y = (180 - x);
    return (uint8_t)y;
}


float pos_to_deg(uint8_t x)
{
    float y = ((float)x/(255.0/180.0));
    return y;
}

int inv_kinematic(float *pos,uint8_t *retbuffer)
{
    float x = pos[0];
    float y = pos[1];
    float z = pos[2];
    float p = rad(pos[3]);
    float r = rad(pos[4]);
    float c = pos[5];

    //std::cout<<"x: "<<x<<std::endl;
    //std::cout<<"y: "<<y<<std::endl;
    //std::cout<<"z: "<<z<<std::endl;
    //std::cout<<"p: "<<p<<std::endl;
    //std::cout<<"r: "<<r<<std::endl;
    //std::cout<<"c: "<<c<<std::endl;

    float phi1=0;
    float phi2=0;
    float phi3=0;
    float phi4=0;

    float L5 = ((20.4)*sin(((255.0*c)/3.269))+(80.0));

    //std::cout<<"L5: "<<L5<<std::endl;

    phi1 = atan2(y,x);
    //std::cout<<"phi1: "<<phi1<<std::endl;
    if (abs(phi1) > M_PI/2) return 0; // Out of range

    float x1 = L1*cos(phi1);
    //std::cout<<"x1: "<<x1<<std::endl;
    float y1 = L1*sin(phi1);
    //std::cout<<"y1: "<<y1<<std::endl;
    float L45 = L4 + L5;
    //std::cout<<"L45: "<<L45<<std::endl;

    float xy3 = L45*cos(p);
    //std::cout<<"xy3: "<<xy3<<std::endl;
    float x3 = x - xy3*cos(phi1);
    //std::cout<<"x3: "<<x3<<std::endl;
    float y3 = y - xy3*sin(phi1);
    //std::cout<<"y3: "<<y3<<std::endl;
    float z3 = z - L45*sin(p);
    //std::cout<<"z3: "<<z3<<std::endl;

    float L23 = sqrt((x3-x1)*(x3-x1) + (y3-y1)*(y3-y1) + (z3-0)*(z3-0));
    //std::cout<<"L23: "<<L23<<std::endl;

    float xy = sqrt(x*x + y*y);
    //std::cout<<"xy: "<<xy<<std::endl;
    float xy1= xy-L1- cos(p)*L45;
    //std::cout<<"xy1: "<<xy1<<std::endl;
    float z1 = z-sin(p)*L45;
    //std::cout<<"z1: "<<z1<<std::endl;

    //float L13 = sqrt(z1*z1+xy1*xy1);
    //std::cout<<"L13: "<<L13<<std::endl;
    phi2 = acos((L2*L2 + L23*L23 - L3*L3)/(2*L2*L23))+atan2(z1,xy1);
    //if (phi2 > Servos_Max_rad[1] || phi2 < Servos_Min_rad[1] ) return 0; // Out of range
    //std::cout<<"phi2: "<<phi2<<std::endl;
    phi3 = acos((L2*L2 + L3*L3 - L23*L23)/(2*L2*L3));
    //if (phi3 > Servos_Max_rad[2] || phi3 < Servos_Min_rad[2] ) return 0; // Out of range
    //std::cout<<"phi3: "<<phi3<<std::endl;

    phi4 = p-(phi2+phi3)+M_PI;
    //if (phi4 > Servos_Max_rad[3] || phi4 < Servos_Min_rad[3] ) return 0; // Out of range
    //std::cout<<"phi4: "<<phi4<<std::endl;

    retbuffer[0] = deg(phi1+M_PI/2.0);
    retbuffer[1] = deg(phi2);
    retbuffer[2] = deg(phi3);
    retbuffer[3] = reverse_ang(deg(phi4+M_PI/2.0));
    retbuffer[4] = 90;
    retbuffer[5] = (uint8_t)(70.0*c + 90.0);

    return 1;
}


int read_servos(int types, int module, uint8_t *servo_buf)
{
    int ret = -1;
    return ret;
}

uint8_t rec_message(int ser, uint8_t *input_buffer) // Returns 0x01 if recived packet, 0x02 on error or timeout
{

  return 0;
}

int write_pololu_command(uint8_t command, uint8_t num, uint8_t value)
{
    int ser = openport();
    uint8_t cmd[6] = {0x80, 0x01, 0x00, 0x00, 0x00, 0x00};
    cmd[2] = command & 0x3F;
    cmd[3] = num & 0x3F;
    switch(command)
    {
        case 0: //Set Parameters (1 data byte)
        case 1: //Set Speed (1 Data 1 - 127)
        {
            cmd[4] = (uint8_t)(value & 0x7F);
            SendBuf(ser, cmd, 5);
            break;
        }
        case 2: //Set Position (1 Data )
        {
            uint16_t angle = (uint16_t)(5000.0*(float)value/180.0)+500;
            cmd[4] = (uint8_t)(angle & 0x7F);

        }
        case 3: //Set Position (2 Data)
        case 4: //Set Pos Absolute (2 Data)
        case 5: //Set Neutral (2 Data)
        {
            //Valid range is 500-5500
            uint16_t angle = (uint16_t)(5000.0*(float)value/180.0)+500;
            cmd[5] = (uint8_t)(angle & 0x7F);
            cmd[4] = (uint8_t)(((angle & 0x1F80))>>7);

            //printf("Angle:%d, Buff: %x %x %x %x %x %x\n",angle, cmd[0], cmd[1], cmd[2], cmd[3], cmd[4], cmd[5]);
            SendBuf(ser, cmd, 6);
            break;
        }
    }
    CloseComport(ser);
    return 0;
}

int write_angles(uint8_t *old_angles,uint8_t *new_angles)
{
    for(int i=0;i<6;i++)
    {
        write_pololu_command(4, i+1, new_angles[i]); //if(old_angles[i]!=new_angles[i])
        old_angles[i]=new_angles[i];
    }

    return 0;
}

int print_pos(float *pos)
{
    printf("X: %f\nY: %f\nZ: %f\nPitch: %f\nRoll: %f\nClaw: %f\n",pos[0],pos[1],pos[2],pos[3],pos[4],pos[5]);
    return 0;
}

int print_angles(uint8_t *angles)
{
    printf("Phi1: %d\nPhi2: %d\nPhi3: %d\nPhi4: %d\nRot: %d\nClaw: %d\n",angles[0],angles[1],angles[2],angles[3],angles[4],angles[5]);
    return 0;
}

int copy_pos_array(float *pos1, float *pos2)
{
    for(int i=0;i<6;i++)pos2[i]=pos1[i];
    return 0;
}

int copy_angle_array(uint8_t *ang1, uint8_t *ang2)
{
    for(int i=0;i<6;i++)ang2[i]=ang1[i];
    return 0;
}

int main()
{
    std::cout<<"Robot Arm Controller"<<std::endl;
    int ser;
    ser = openport();
    CloseComport(ser);
    uint8_t buff[255];

    uint8_t test_ang = 0;
    uint8_t test_ang2 = 0;

    float current_position[6] = {220.0,0.0,100.0,-30.0,45.0,1.0};
    float next_position[6] = {220.0,0.0,100.0,-30.0,45.0,1.0};

    uint8_t current_angles[6] = {90,90,90,90,90,90};
    uint8_t next_angles[6];

    inv_kinematic(next_position,next_angles);

    //print_angles(next_angles);

    int r;

    // Sholder Rotate
    write_pololu_command(0, 1, 0b00001111);
    write_pololu_command(1, 1, 30);
    write_pololu_command(0, 1, 0b00001111);

    // Sholder Lift
    write_pololu_command(0, 2, 0b00001111);
    write_pololu_command(1, 2, 30);
    write_pololu_command(0, 2, 0b00001111);

    // Elbow Lift
    write_pololu_command(0, 3, 0b00001111);
    write_pololu_command(1, 3, 30);
    write_pololu_command(0, 3, 0b00001111);

    // Wrist Lift
    write_pololu_command(0, 4, 0b00001111);
    write_pololu_command(1, 4, 30);
    write_pololu_command(0, 4, 0b00001111);

    // Wrist Rotate
    write_pololu_command(0, 5, 0b00001111);
    write_pololu_command(1, 5, 10);
    write_pololu_command(0, 5, 0b00001111);

    //Claw
    write_pololu_command(0, 6, 0b00101111);
    write_pololu_command(1, 6, 30);
    write_pololu_command(0, 6, 0b00101111);

    write_angles(current_angles,next_angles);
/*    while(1)
    {
        write_pololu_command(4, 7,0);
        printf("Test angle: %d\n",0);
        usleep(3000000);
        write_pololu_command(4, 7,45);
        printf("Test angle: %d\n",45);
        usleep(3000000);
        write_pololu_command(4, 7,90);
        printf("Test angle: %d\n",90);
        usleep(3000000);
        write_pololu_command(4, 7,135);
        printf("Test angle: %d\n",135);
        usleep(3000000);
        write_pololu_command(4, 7,180);
        printf("Test angle: %d\n",180);
        usleep(5000000);

    }

*/
    uint8_t Pin_type[NO_PWM]; // 0=servo, 1=analog
    uint8_t Servos_Pin[NO_PWM];
    uint8_t Servos_Speed[NO_PWM];
    uint8_t Servos_Angle[NO_PWM];
    uint8_t Servos_Max[NO_PWM];
    uint8_t Servos_Min[NO_PWM];
    uint8_t Servos_Idle[NO_PWM];
    uint8_t Servos_Rev[NO_PWM]; // 0-standard, 1-reversed

    struct motion_dev *devs = NULL;
    int next_devindex = 0;


    int csk = l2cap_listen(L2CAP_PSM_HIDP_CTRL);
    int isk = l2cap_listen(L2CAP_PSM_HIDP_INTR);

    if (csk >= 0 && isk >= 0)
        fprintf(stderr, "Waiting for Bluetooth connection.\n");
    else
        fprintf(stderr, "Unable to listen on HID PSMs."
            " Are you root bro?\n");


    _sixmotion_hidraw raw_sixm;

    _sixmotion sixm;

    float ys1=1.0f;
    float ys2=-1.0f;

    float y1 = 0.0;
    float y2 = 0.0;
    float h1 = 0.0;
    float h2 = 0.0;
    float h = 0.015;

    while (1) {
        fd_set fds;
        FD_ZERO(&fds);
        //if (!nostdin) FD_SET(0, &fds);
        FD_SET(0, &fds);
        int fdmax = 0;
        if (csk >= 0) FD_SET(csk, &fds);
        if (isk >= 0) FD_SET(isk, &fds);
        if (csk > fdmax) fdmax = csk;
        if (isk > fdmax) fdmax = isk;
        for (struct motion_dev *dev = devs; dev; dev = dev->next) {
            FD_SET(dev->csk, &fds);
            if (dev->csk > fdmax) fdmax = dev->csk;
            FD_SET(dev->isk, &fds);
            if (dev->isk > fdmax) fdmax = dev->isk;
        }
        if (select(fdmax + 1, &fds, NULL, NULL, NULL) < 0) fatal("select");
        struct timeval tv;
        gettimeofday(&tv, NULL);
        time_t now = tv.tv_sec;
        // Incoming connection ?
        if (csk >= 0 && FD_ISSET(csk, &fds)) {//CONNECTS DEVICE
            struct motion_dev *dev = accept_device(csk, isk);
            dev->index = next_devindex++;
            dev->next = devs;
            devs = dev;
            setup_device(dev);

        }

        for (struct motion_dev *dev = devs; dev; dev = dev->next)
            if (FD_ISSET(dev->isk, &fds)) {
                int nr = 1;
                while (nr > 0) {
                    unsigned char report[256];

                    nr = recv(dev->isk, report, sizeof (report), MSG_DONTWAIT);
                    //THIS IS THE REPORT
                    if (report[0] == 0xa1) {
                        if (nr == 50) {
                            memcpy(&raw_sixm, report, nr);
                            //dump("RECV", report, nr);
                            parse_raw(raw_sixm, sixm);
                        }
                    } else {
                        printf("REPORT\n");
                    }

                }
            }
        if (devs != NULL) {

            uint8_t arm_y_inc = (raw_sixm.L3_x);
            uint8_t arm_x_inc = (raw_sixm.L3_y);
            uint8_t arm_z_inc = (raw_sixm.R3_y);

            if (raw_sixm.BTN_R1_VALUE)
            {
                arm_roll = (uint8_t)(((float)(sixm.aX/4)-128.0)*2.25+90.0);
                arm_pitch = (uint8_t)(((float)(sixm.aY/4)-128.0)*0.75+90.0);
            }

            uint8_t arm_claw = (uint8_t)raw_sixm.BTN_R2_VALUE;
            uint8_t arm_home = (uint8_t)raw_sixm.BTN_X;

          //printf("x: %d\t\ty: %d\t\tz: %d\t\n",arm_x_inc,arm_y_inc,arm_z_inc);
          //printf("Roll: %d\tPitch: %d\n",arm_roll,arm_pitch-90);
          //printf("Claw: %d\tHome: %d\n",arm_claw, arm_home);



            if(arm_claw>0)arm_claw=1;

            next_position[0] += (float)(arm_x_inc-128)*-0.039;
            next_position[1] += (float)(arm_y_inc-128)*0.039;
            next_position[2] += (float)(arm_z_inc-128)*-0.039;
            next_position[3] = (float)(arm_pitch-90);
            next_position[4] = (float)arm_roll;
            next_position[5] = (float)arm_claw;
            //print_pos(next_position);
            inv_kinematic(next_position,next_angles);
            write_angles(current_angles,next_angles);
            //print_angles(next_angles);

            if(arm_home)
            {
                current_position[0] = 220.0;
                current_position[1] = 0.0;
                current_position[2] = 100.0;
                current_position[3] = -30.0;
                current_position[4] = 0.0;
                current_position[5] = 0.0;
                next_position[0] = 220.0;
                next_position[1] = 0.0;
                next_position[2] = 100.0;
                next_position[3] = -30.0;
                next_position[4] = 0.0;
                next_position[5] = 0.0;

            }


        }
    }

    CloseComport(ser);

    return EXIT_SUCCESS;
}

