#include <cstdlib>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <vector>
#include <math.h>

#include <sys/signal.h>
#include <sys/types.h>
#include <sys/time.h>

#include "rs232.h"
#include "hklx.h"
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

using namespace std;

uint8_t inbuf[4095];

int8_t openport()
{
    if (!OpenComport(16, 115200)) return 16;
    if (!OpenComport(17, 115200)) return 17;
    if (!OpenComport(18, 115200)) return 18;

    return -1;
}

uint8_t getCheckSum(uint8_t *buffer, int length)
{
  int i;
  uint8_t XOR;
  uint8_t c;
  // Calculate checksum ignoring any $'s in the string
  for (XOR = 0, i = 0; i < length; i++) {
    c = buffer[i];
    XOR ^= c;
  }
  return XOR;
}

int print_buf(uint8_t *mes)
{
    printf("%x %x %x %x %x %x\n",mes[0],mes[1],mes[2],mes[3],mes[4],mes[5]);
    return 1;
}

int printf_buf(float *mes)
{
    printf("%.2f %.2f %.2f %.2f %.2f %.2f\n",mes[0],mes[1],mes[2],mes[3],mes[4],mes[5]);
    return 1;
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

    float L13 = sqrt(z1*z1+xy1*xy1);
    //std::cout<<"L13: "<<L13<<std::endl;
    phi2 = acos((L2*L2 + L23*L23 - L3*L3)/(2*L2*L23))+atan2(z1,xy1);
    //std::cout<<"phi2: "<<phi2<<std::endl;
    phi3 = acos((L2*L2 + L3*L3 - L23*L23)/(2*L2*L3));
    //std::cout<<"phi3: "<<phi3<<std::endl;

    phi4 = p-(phi2+phi3)+M_PI;
    //std::cout<<"phi4: "<<phi4<<std::endl;

    retbuffer[0] = ang_to_pos(phi1);
    retbuffer[1] = ang_to_pos(phi2);
    retbuffer[2] = ang_to_pos(phi3);
    retbuffer[3] = ang_to_pos(phi4);
    retbuffer[4] = ang_to_pos(r+M_PI/2.0);
    retbuffer[5] = (uint8_t)(255.0*c);

    return 1;
}


int read_servos(int types, int module, uint8_t *servo_buf)
{
    int ret = -1;
    int ser = openport();
    if (ser)
    {
        int write = 1;
         while(write)
        {
            int start = 0;
            uint8_t mes[12] = {0x24,0x00,0x52,0x00,0x00,0x01,0x02,0x03,0x04,0x05,0x00,0x5E};
            mes[1] = module;
            mes[3] = types;
            mes[10] = getCheckSum(mes,10);
            SendBuf(ser, mes, 12);
            uint8_t ret_buf[12];
            int retbuf_it = 0;
            int rec=1;
            uint8_t inbuf[12];
            while(rec<200)
            {
                usleep(100);
                int n = PollComport(ser, inbuf, 1);
                if(n > 0)
                {
                    for(int i=0; i < n; i++)
                    {
                        if(inbuf[i]=='$' && retbuf_it==0) start = 1;
                        if(start) ret_buf[retbuf_it++]=inbuf[i];
                        if(inbuf[i]=='^' && retbuf_it==12)
                        {
                            rec = 200001;
                            start = 0;
                        }
                        rec++;
                    }
                }

            }
            if (rec==200002)
            {
                if(ret_buf[1] == module && ret_buf[2] == 0x00 && ret_buf[3] == types && ret_buf[10] == getCheckSum(ret_buf,10))
                {
                    write = 0;
                    for(int i=0; i < 6; i++)
                    {
                        servo_buf[i] = ret_buf[4+i];
                    }
                    ret = 1;
                }
            }
            write = 0;
        }
       CloseComport(ser);
    }

    return ret;
}

uint16_t getCheckSum(const void *data)
//uint16_t calculate_crc(uint8_t *data)
{

    uint8_t *bytes = (uint8_t *) data;

    uint16_t crc;

    uint8_t CheckSum1 = bytes[2] ^ bytes[3] ^ bytes[4];
    for (int i = 0; i < ((int) bytes[2] - MIN_PACKET_SIZE); i++)
        CheckSum1 ^= bytes[7 + i];

    uint8_t CheckSum2 = ~(CheckSum1);
    CheckSum1 &= CHKSUM_MASK;
    CheckSum2 &= CHKSUM_MASK;

    crc = CheckSum1;
    crc |= CheckSum2 << 8;


    return crc;
}

uint8_t rec_message(int ser, uint8_t *input_buffer) // Returns 0x01 if recived packet, 0x02 on error or timeout
{
    uint8_t c=0;
    uint8_t beg = 0;
    uint8_t state = 0;
    uint8_t checksum1;
    uint8_t checksum2;
    uint8_t i = 0;
    uint8_t buffer[1];
    int retry = 0;
    int n =0;
    while(retry<10)
    {
        if (((n = PollComport(ser, buffer, 1)) > 0))
        {
            c = buffer[0];
            //std::cout<<"i: "<<(int)i<<"  c: "<<(int)c<<std::endl;
            switch(state)
            {
                case 0:
                {
                    if(c==0xFF)
                    {
                        input_buffer[i++]=c;
                        input_buffer[i++]=c;
                        //std::cout<<"0) i: "<<(int)(i-1)<<hex<<"  buf: "<<(int)input_buffer[i-1]<<std::endl;
                        state++;
                    }
                    else
                    {
                        i=0;
                        state=0;
                    }
                    break;
                }
                case 1:
                {
                    if(c > 6 && c < 224)
                    {
                        input_buffer[i++] = c;
                        //std::cout<<"1) i: "<<(int)(i-1)<<"  buf: "<<hex<<(int)input_buffer[i-1]<<std::endl;
                        state++;
                    }
                    else
                    {
                        i=0;
                        state=0;
                        if(c==0xFF)
                        {
                            input_buffer[i++]=c;
                            input_buffer[i++]=c;
                            //std::cout<<"1) i: "<<(int)(i-1)<<"  buf: "<<hex<<(int)input_buffer[i-1]<<std::endl;
                            state++;
                        }
                    }
                    break;
                }
                case 2:
                {

                    input_buffer[i++] = c;
                    //std::cout<<"2) i: "<<(int)(i-1)<<"  buf: "<<hex<<(int)input_buffer[i-1]<<std::endl;
                    if(i==input_buffer[2])state++;
                    break;
                }
            }
        }
        else
        {
            retry++;
        }
        if(state==3)
        {
            uint16_t crc = getCheckSum(input_buffer);
            checksum1 = (uint8_t) ((crc & 0xFF00) >> 8);
            checksum2 = (uint8_t) (crc & 0x00FF);
            if((checksum1) == input_buffer[5] && (checksum2) == input_buffer[6])
            {
                return 1;
            }
            else return 2;
        }
        usleep(100);
  }
  //std::cout<<"retry: "<<(int)(retry)<<std::endl;
  return 0;
}

int read_servo(int ser, int types, int module, uint8_t *return_buffer)
{
    hklx_packet_t *hklx_packet2;
    hklx_packet2 = hklx_packet_allocate(1, 0x02, CMD_EEP_READ);
    hklx_packet2->data[0] = types;
    hklx_send(1, hklx_packet2);
    hklx_packet_free(&hklx_packet2);

    for(int i=0;i<20;i++)return_buffer[i]=0x00;
    usleep(5000);

    int r = rec_message(ser,return_buffer);
    if(r==1 && return_buffer[5]!=0x40)
    {
        return 1;
    }
    return 0;
}

int write_servo(int ser, int types, int module, uint8_t *value)
{
    uint8_t buff[255];

    int r;

    hklx_decoder_t hklx_decoder1;
    hklx_decoder_initialise(&hklx_decoder1);

    hklx_packet_t *hklx_packet;
    hklx_packet = hklx_packet_allocate(7, module, CMD_EEP_WRITE);
    hklx_packet->data[0] = types;
    memcpy(&hklx_packet->data[1],&value[0],7*sizeof(uint8_t));
    hklx_send(1, hklx_packet);
    hklx_packet_free(&hklx_packet);
    usleep(5000);
    r = rec_message(ser,buff);
    if(r==1 && buff[5]!=0x40)
    {
        return 1;
    }

    return 0;
}

int main()
{
    std::cout<<"Robot Arm Controller"<<std::endl;
    int ser;
    ser = openport();
    CloseComport(ser);
    ser = openport();
    std::cout<<"Ser: "<<ser<<std::endl;
    uint8_t buff[255];

    int r;

    uint8_t Pin_type[NO_PWM]; // 0=servo, 1=analog
    uint8_t Servos_Pin[NO_PWM];
    uint8_t Servos_Speed[NO_PWM];
    uint8_t Servos_Angle[NO_PWM];
    uint8_t Servos_Max[NO_PWM];
    uint8_t Servos_Min[NO_PWM];
    uint8_t Servos_Idle[NO_PWM];
    uint8_t Servos_Rev[NO_PWM]; // 0-standard, 1-reversed




    //std::cout<<"Type: "<<read_servo(10,2,Pin_type)<<std::endl;
    //print_buf(Pin_type);
    //std::cout<<"Pin: "<<read_servo(20,2,Servos_Pin)<<std::endl;
    //print_buf(Servos_Pin);
    //std::cout<<"Angle: "<<read_servo(30,2,Servos_Angle)<<std::endl;
    //print_buf(Servos_Angle);
    //std::cout<<"Speed: "<<read_servo(40,2,Servos_Speed)<<std::endl;
    //print_buf(Servos_Speed);
    //std::cout<<"Max: "<<read_servo(50,2,Servos_Max)<<std::endl;
    //print_buf(Servos_Max);
    //std::cout<<"Min: "<<read_servo(60,2,Servos_Min)<<std::endl;
    //print_buf(Servos_Min);
    //std::cout<<"Idle: "<<read_servo(70,2,Servos_Idle)<<std::endl;
    //print_buf(Servos_Idle);
    //std::cout<<"Rev: "<<read_servo(80,2,Servos_Rev)<<std::endl;
    //print_buf(Servos_Rev);



    int types=30;

    while(read_servo(ser,types,2,buff) != 1);

    std::cout<<"Servo0: "<<(int)(buff[8])<<std::endl;
    std::cout<<"Servo1: "<<(int)(buff[9])<<std::endl;
    std::cout<<"Servo2: "<<(int)(buff[10])<<std::endl;
    std::cout<<"Servo3: "<<(int)(buff[11])<<std::endl;
    std::cout<<"Servo4: "<<(int)(buff[12])<<std::endl;
    std::cout<<"Servo5: "<<(int)(buff[13])<<std::endl;

    usleep(1000000);

    std::cout<<"Servo5: "<<(int)(buff[13])<<std::endl;

    float position1[6] = {130.0 , 130.0 , 170.0, 40.0 , 0.0 , 0.1 };
    //float position2[6] = {120.0 , 120.0 , 170.0, 40.0 , 0.0 , 0.9 };
    float position2[6] = {150.0 , 150.0 , 80.0, -40.0 , 0.0 , 0.9 };
    uint8_t ang2[6];
    uint8_t ang1[6];
    float degs[6];

    inv_kinematic(position1,ang1);
    inv_kinematic(position2,ang2);

    int write_success=0;

    for(int i =0;i <10;i++)
    {
        std::cout<<"Ok0"<<std::endl;
        write_success = write_servo(ser,30,2,ang1);
        while((write_servo(ser,30,2,ang1) != 1));


        std::cout<<"Ok1"<<std::endl;
        usleep(2000000);
        std::cout<<"Ok2"<<std::endl;

        while((write_servo(ser,30,2,ang2)) != 1);
        std::cout<<"Ok3"<<std::endl;
        usleep(2000000);
    }

    CloseComport(ser);

    return EXIT_SUCCESS;
}

