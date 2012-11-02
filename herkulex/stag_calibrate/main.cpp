/* 
 * File:   main.cpp
 * Author: toner
 *
 * Created on 22 August 2012, 5:26 PM
 */

#include <cstdlib>
//#include "hklx.h"
#include <sys/signal.h>
#include "rs232.h"
#include "stag.h"
//#include "math.h"

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <iostream>
#include <complex>      
using namespace std;
using namespace cv;
/*
 * 
 */


void ikl(float x4,float y4,float z4,float lrf,float phi[]){
    
    
//function [ phi,x,y,z ] = hikl( x4,y4,z4,l1,l2,l3,lrf)
//IKLEG Summary of this function goes here
//  Detailed explanation goes here

float l1 = 0.058;
float l2 = 0.0706;
float l3 = 0.09;


//x4=l1;
//y4=0;
//z4=-l2-l3;

//float phi[3]={0,0,0};

float x [4]={0,0,0,x4};
float y [4]={0,0,0,y4};
float z [4]={0,0,0,z4};
//printf("%f %f %f\n",x4,y4,z4);

//float phi1_s1 = -log(((l1 + (- x(4)^2 - z(4)^2 + l1^2)^(1/2))*(x(4) + z(4)*i))/(x(4)^2 + z(4)^2))*i;
    
complex<float> X4 (x4,0.0f);
complex<float> Z4 (z4,0.0f);
complex<float> i (0.0f,1.0f);


complex<float> comp_phi1_s1 = -log(((l1 + sqrt(- X4*X4 - Z4*Z4 + l1*l1))*(X4 + Z4*i))/(X4*X4 + Z4*Z4))*i;
//complex<float> comp_phi1_s1 = -log(((X4 + Z4*i)*(l1 - sqrt(- X4*X4 - Z4*Z4 + l1*l1)))/(X4*X4 + Z4*Z4))*i;


float phi1_s1 = real(comp_phi1_s1);

if(imag(comp_phi1_s1)>0.0001){
    
    printf("HUGE PROBLEM\n");
    
}


//if abs(imag(phi1_s1)) < 1e-12
//    phi1_s1 = real(phi1_s1);
//end

//%phi1_s2 = -log(((x(4) + z(4)*i)*(l1 - (- x(4)^2 - z(4)^2 + l1^2)^(1/2)))/(x(4)^2 + z(4)^2))*i;
//%
//%if abs(imag(phi1_s2)) < 1e-12
//%    phi1_s2 = real(phi1_s2);
//%end


phi[0]=phi1_s1;

x[1] = l1*cos(phi[0]);
z[1] = l1*sin(phi[0]);

//%+- for now ALWAYS negative for right side
//%(l2^2+l3^2-(x(4)-x(2))^2-(y(4)-y(2))^2-(z(4)-z(2))^2)/(2*l2*l3)

float l24 = sqrt((x[3]-x[1])*(x[3]-x[1])+(y[3]-y[1])*(y[3]-y[1])+(z[3]-z[1])*(z[3]-z[1]));

//%phi(3)=pi-acos((l2^2+l3^2-(x(4)-x(2))^2-(y(4)-y(2))^2-(z(4)-z(2))^2)/(2*l2*l3));

phi[2]=-lrf*M_PI+lrf*acos((l2*l2+l3*l3-l24*l24)/(2*l2*l3));
//phi[2]=M_PI+lrf*acos((l2*l2+l3*l3-l24*l24)/(2*l2*l3));

//%xz24 = sqrt((x(4)-x(2))^2+(z(4)-z(2))^2)

//%(x(4)-x(2))/-sin(phi(1));
//%(z(4)-z(2))/cos(phi(1));

float xz24 = (z[3]-z[1])/cos(phi[0]);


//%(l2^2+l24^2-l3^2)/(2*l2*l24)

//phi[1] = atan2(xz24,y[3])+lrf*acos((l2*l2+l24*l24-l3*l3)/(2*l2*l24)) +M_PI_2;
phi[1] = atan2(xz24,y[3])+lrf*acos((l2*l2+l24*l24-l3*l3)/(2*l2*l24)) +M_PI_2;

//y(3) = y(2)+l2*cos(phi(2));

//x(3) = x(2) - l2*sin(phi(2))*sin(phi(1));
//z(3) = z(2) + l2*sin(phi(2))*cos(phi(1));    
    
        
//if(phi[1])
    
    
}

Mat rot_xyz(Mat in, float phi, int ax) {


    float* u = in.ptr<float>(0);
    float u1 = u[ax];
    float u2 = u[3 + ax];
    float u3 = u[6 + ax];

    Mat Ux = (Mat_<float>(3, 3) <<
            0.0f, -u3, u2,
            u3, 0, -u1,
            -u2, u1, 0);
    Mat UxU = (Mat_<float>(3, 3) <<
            u1*u1, u1*u2, u1*u3,
            u1*u2, u2*u2, u2*u3,
            u1*u3, u2*u3, u3 * u3);
    
    Mat I = Mat::eye(3,3,CV_32F);
    
    
    Mat R = I*cos(phi)+sin(phi)*Ux+(1-cos(phi))*UxU;
    
    //return R.mul(in);
    return R*in;
    
}
    
    
void hklx_dump(hklx_packet_t *hklx_packet){


    
    printf("%02x %02x ", hklx_packet->id,hklx_packet->cmd);
    
    for (int i = 0; i < hklx_packet->packetsize - 7; i++) {

        printf("%02x ", hklx_packet->data[i]);

    }
    printf("\n");
    
    
}
   

int _stag::init(){
    
    
    OpenComport(LEG_0_COM, BAUD);
    OpenComport(LEG_1_COM, BAUD);
    OpenComport(LEG_2_COM, BAUD);
    OpenComport(LEG_3_COM, BAUD);
    

    ///assuming flat ground bro
    /*
    Mat bcm = (Mat_<float>(3,3) << 1.0f,0.0f,0.0f,
                                   0.0f,1.0f,0.0f,
                                   0.0f,0.0f,1.0f);
    */
    /*
    roll = 0;
    pitch = 0;
    yaw = 0;
    
    h=0;
    
    sx=0.06f;
    sy=0.06f;    
    by=0.03f;    
    */
    
    return 1;
}

void i_jog_leg(int leg_id,float phi0,float phi1,float phi2,uint8_t time,uint8_t LED0,uint8_t LED1,uint8_t LED2){
    
    
    hklx_packet_t *hklx_packet = hklx_packet_allocate(sizeof(i_jog_t)*3, BROADCAST_ID,CMD_I_JOG);
    
    i_jog_t *i_jog = (i_jog_t*)malloc(sizeof (i_jog_t) * 3 * sizeof (uint8_t));
    
    //angle+=166.650f;
    //angle/=0.325f;
    
    i_jog[0].id=(uint8_t)(3*leg_id+0);
    i_jog[1].id=(uint8_t)(3*leg_id+1);
    i_jog[2].id=(uint8_t)(3*leg_id+2);    
    
    i_jog[0].jog.value=(uint16_t)((phi0+166.650f)/0.3258064516f);
    i_jog[1].jog.value=(uint16_t)((phi1+166.650f)/0.3258064516f);
    i_jog[2].jog.value=(uint16_t)((phi2+166.650f)/0.3258064516f);    
    
    i_jog[0].playtime = time;
    i_jog[1].playtime = time;
    i_jog[2].playtime = time;    
    
    i_jog[0].set.mode=0;
    i_jog[1].set.mode=0;
    i_jog[2].set.mode=0;
    
    i_jog[0].set.joginvalid=0;
    i_jog[1].set.joginvalid=0;
    i_jog[2].set.joginvalid=0;    
    
    i_jog[0].set.stopflag=0;
    i_jog[1].set.stopflag=0;
    i_jog[2].set.stopflag=0;
    
    i_jog[0].set.LedRed=(RED&LED0)>0;
    i_jog[1].set.LedRed=(RED&LED1)>0;
    i_jog[2].set.LedRed=(RED&LED2)>0;
    
    i_jog[0].set.LedGreen=(GREEN&LED0)>0;    
    i_jog[1].set.LedGreen=(GREEN&LED1)>0;
    i_jog[2].set.LedGreen=(GREEN&LED2)>0;
    
    i_jog[0].set.LedBlue=(BLUE&LED0)>0;
    i_jog[1].set.LedBlue=(BLUE&LED1)>0;
    i_jog[2].set.LedBlue=(BLUE&LED2)>0;
    
    
    memcpy(hklx_packet->data, i_jog, sizeof (i_jog_t) * 3 * sizeof (uint8_t));
    
    hklx_send(leg_id,hklx_packet);
    hklx_packet_free(&hklx_packet);
        
    free(i_jog);
    i_jog = NULL;
        
}


void ijl(int leg_id,float phi[],uint8_t time,uint8_t LED0,uint8_t LED1,uint8_t LED2){
    
      //if(leg_id !=0){
      
    hklx_packet_t *hklx_packet = hklx_packet_allocate(sizeof(i_jog_t)*3, BROADCAST_ID,CMD_I_JOG);
    
    i_jog_t *i_jog = (i_jog_t*)malloc(sizeof (i_jog_t) * 3 * sizeof (uint8_t));
    
    //angle+=166.650f;
    //angle/=0.325f;

        
    i_jog[0].id=(uint8_t)(3*leg_id+0);
    i_jog[1].id=(uint8_t)(3*leg_id+1);
    i_jog[2].id=(uint8_t)(3*leg_id+2);    
    
    i_jog[0].jog.value=(uint16_t)round((phi[0]*180.0f/M_PI+166.650f)/0.3258064516f);
    
    //printf("jog val %i\n",i_jog[0].jog.value);
    
    i_jog[1].jog.value=(uint16_t)round((phi[1]*180.0f/M_PI+166.650f)/0.3258064516f);
    i_jog[2].jog.value=(uint16_t)round((-phi[2]*180.0f/M_PI+166.650f)/0.3258064516f);    
    
    i_jog[0].playtime = time;
    i_jog[1].playtime = time;
    i_jog[2].playtime = time;    
    
    i_jog[0].set.mode=0;
    i_jog[1].set.mode=0;
    i_jog[2].set.mode=0;
    
    i_jog[0].set.joginvalid=0;
    i_jog[1].set.joginvalid=0;
    i_jog[2].set.joginvalid=0;    
    
    i_jog[0].set.stopflag=0;
    i_jog[1].set.stopflag=0;
    i_jog[2].set.stopflag=0;
    
    i_jog[0].set.LedRed=(RED&LED0)>0;
    i_jog[1].set.LedRed=(RED&LED1)>0;
    i_jog[2].set.LedRed=(RED&LED2)>0;
    
    i_jog[0].set.LedGreen=(GREEN&LED0)>0;    
    i_jog[1].set.LedGreen=(GREEN&LED1)>0;
    i_jog[2].set.LedGreen=(GREEN&LED2)>0;
    
    i_jog[0].set.LedBlue=(BLUE&LED0)>0;
    i_jog[1].set.LedBlue=(BLUE&LED1)>0;
    i_jog[2].set.LedBlue=(BLUE&LED2)>0;
    
    
    memcpy(hklx_packet->data, i_jog, sizeof (i_jog_t) * 3 * sizeof (uint8_t));
    
    //if(leg_id==1)
    //    leg_id=0;
    
    hklx_send(leg_id,hklx_packet);
    hklx_packet_free(&hklx_packet);
    
    
    free(i_jog);
    i_jog = NULL;
    //}
    
}

void ikleg(int leg_id, float x4, float y4, float z4, uint8_t jtime) {

    float phi[3];

    float ang = M_PI*20.0/180.0;

    switch (leg_id) {

        case 0:
            //x4=0.04;
            //x4=0;
            //y4=0;
            //ikl(x4, y4, z4, 1, phi);
            
            //printf("%f\n",)
            
            //printf("%f %f %f\n",x4*cos(-ang)-y4*sin(-ang), x4*sin(-ang)+y4*cos(-ang), z4);
            
            ikl(x4*cos(-ang)-y4*sin(-ang), x4*sin(-ang)+y4*cos(-ang), z4, -1, phi);
            //ikl(-x4/sin(ang)+y4/cos(ang), y4/sin(ang)-x4/cos(ang), z4, 1, phi);
            //printf("%f %f %f\n", phi[0], phi[1], phi[2]);
            ijl(leg_id, phi, jtime, BLUE, BLUE, BLUE);
            
            break;
        case 1:

            //ikl(x4, -y4, z4, -1, phi);
            ikl(x4*cos(ang)-y4*sin(ang), x4*sin(ang)+y4*cos(ang), z4, -1, phi);
            phi[0]=-phi[0];
            //printf("%f %f %f\n", phi[0], phi[1], phi[2]);
            ijl(leg_id, phi, jtime, BLUE, BLUE, BLUE);

            //ijl(0, phi, jtime, BLUE, BLUE, BLUE);
            
            break;
        case 2:

            //ikl(x4, -y4, z4, -1, phi);
            ikl(x4*cos(-ang)-y4*sin(-ang), x4*sin(-ang)+y4*cos(-ang), z4, -1, phi);
           // printf("%f %f %f\n",phi[0],phi[1],phi[2]);            
            ijl(leg_id, phi, jtime, BLUE, BLUE, BLUE);

            break;
        case 3:

            //ikl(x4, y4, z4, 1, phi);
            ikl(x4*cos(ang)-y4*sin(ang), x4*sin(ang)+y4*cos(ang), z4, -1, phi);
            phi[0]=-phi[0];
            //printf("%f %f %f\n",phi[0],phi[1],phi[2]);
            ijl(leg_id, phi, jtime, BLUE, BLUE, BLUE);

            break;
    }
}

void body_jog(Mat leg_xyz, float yaw, float pitch, float roll,float x,float y, float h, int t) {


    Mat bcm = Mat::eye(3, 3, CV_32F);
    /*Mat bcm = (Mat_<float>(3,3) << 1.0f,0.0f,0.0f,
                               0.0f,1.0f,0.0f,
                               0.0f,0.0f,1.0f);
     */
    //3,1 1,3??
    Mat bc = (Mat_<float>(3, 1) <<
            x,
            y,
            h);


    float bx = 0.06;
    float by = 0.075;

    Mat lxyz = (Mat_<float>(3, 4) <<
            bx, -bx, -bx, bx,
            by, by, -by, -by,
            0, 0, 0, 0);



    bcm = rot_xyz(bcm, yaw, 2); //yaw
    bcm = rot_xyz(bcm, pitch, 0); //pitch        
    bcm = rot_xyz(bcm, roll, 1); //roll


    //lxyz = bcm.mul(lxyz);
    lxyz = bcm*lxyz;

    lxyz.row(0) = lxyz.row(0) + bc.row(0);
    lxyz.row(1) = lxyz.row(1) + bc.row(1);
    lxyz.row(2) = lxyz.row(2) + bc.row(2);


    float ang = M_PI * 20.0 / 180.0;

    Mat l1cf = rot_xyz(bcm, ang, 2);
    Mat l2cf = rot_xyz(bcm, M_PI-ang, 2);
    Mat l3cf = rot_xyz(bcm, M_PI+ang, 2);
    Mat l4cf = rot_xyz(bcm, -ang, 2);
    

    //Mat l1g = (Mat_<float>(3, 1) <<
    //        bx + 0.06f,
    //        by + 0.03f,
    //        0.0f);

    Mat l1t = l1cf.inv()*(leg_xyz.col(0) - lxyz.col(0));
    Mat l2t = l2cf.inv()*(leg_xyz.col(1) - lxyz.col(1));
    Mat l3t = l3cf.inv()*(leg_xyz.col(2) - lxyz.col(2));
    Mat l4t = l4cf.inv()*(leg_xyz.col(3) - lxyz.col(3));


    float phi[3];

    float* xyz1 = l1t.ptr<float>(0);
    float* xyz2 = l2t.ptr<float>(0);
    float* xyz3 = l3t.ptr<float>(0);
    float* xyz4 = l4t.ptr<float>(0);
    

    //printf("%f %f %f\n", xyz1[0], xyz1[1], xyz1[2]);

    //ikl(xyz1[0], xyz1[1], xyz1[2], -1, phi);
    ikl(xyz1[0], xyz1[1], xyz1[2], -1, phi);
    
    printf("%f %f %f\n", phi[0], phi[1], phi[2]);
    ijl(0, phi, t, BLUE, BLUE, BLUE);

    //ikl(xyz2[0], xyz2[1], xyz2[2], 1, phi);
    ikl(xyz2[0], xyz2[1], xyz2[2], 1, phi);
    phi[0]=-phi[0];
    printf("%f %f %f\n", phi[0], phi[1], phi[2]);
    ijl(1, phi, t, BLUE, BLUE, BLUE);
    
    ikl(xyz3[0], xyz3[1], xyz3[2], -1, phi);
    printf("%f %f %f\n", phi[0], phi[1], phi[2]);
    ijl(2, phi, t, BLUE, BLUE, BLUE);
    
    //ikl(xyz4[0], xyz4[1], xyz4[2], 1, phi);
    ikl(xyz4[0], xyz4[1], xyz4[2], 1, phi);
    phi[0]=-phi[0];
    printf("%f %f %f\n", phi[0], phi[1], phi[2]);
    ijl(3, phi, t, BLUE, BLUE, BLUE);    
    

}

void _stag::hklx_packet_parse(hklx_packet_t *hklx_packet) {

    switch (hklx_packet->cmd) {

        case CMD_RAM_READ_ACK:

            //memcpy(hklx[hklx_packet->id].RAM. );
            //memcpy(&hklx[hklx_packet->id].RAM+hklx_packet->data[0],&hklx_packet->data[2],hklx_packet->data[1]);
            
            //memcpy(&hklx[hklx_packet->id].RAM+(int)(hklx_packet->data[0]),&hklx_packet->data[2],hklx_packet->data[1]);
            //memcpy(&hklx[hklx_packet->id].RAM.ID+1,&hklx_packet->data[2],hklx_packet->data[1]);
            
            memcpy(&hklx[hklx_packet->id].RAM.ID+(int)(hklx_packet->data[0]),&hklx_packet->data[2],hklx_packet->data[1]);
            
            
            //(CMD_EEP_WRITE|CMD_ACK_MASK)
            //CALIBRATE_MASK
            //printf("%i\n",(int)(hklx_packet->data[0]));
            
            //printf("%i %i\n",hklx[hklx_packet->id].RAM.ID,hklx[hklx_packet->id].RAM.ACK_Policy);
            
            hklx[hklx_packet->id].RAM.Calibrated_Position = (hklx[hklx_packet->id].RAM.Calibrated_Position&CALIBRATE_MASK);
            
            
            //i_jog[0].jog.value=(uint16_t)round((phi[0]*180.0f/M_PI+166.650f)/0.3258064516f);
            hklx[hklx_packet->id].abs_phi = ((float)hklx[hklx_packet->id].RAM.Absolute_Position)*0.3258064516f-166.650f;
            hklx[hklx_packet->id].phi = ((float)hklx[hklx_packet->id].RAM.Calibrated_Position)*0.3258064516f-166.650f;            
            //printf("%i %i\n",hklx[hklx_packet->id].RAM.Absolute_Position,hklx[hklx_packet->id].RAM.Calibrated_Position);
            
            hklx[hklx_packet->id].vin = ((float)hklx[hklx_packet->id].RAM.Voltage)*0.0740745098f;
            
            
            hklx[hklx_packet->id].temp = ((float)hklx[hklx_packet->id].RAM.Temperature)*1.497559055f-79.47f;
            //printf("%f\n",hklx[hklx_packet->id].abs_phi);
            hklx[hklx_packet->id].pwm = ((int)hklx[hklx_packet->id].RAM.PWM);
            
            printf("%i\n",hklx[hklx_packet->id].RAM.Voltage);
            
            
            
            break;


 
        /*
        case 1:


            break;
        case 2:


            break;
        case 3:


            break;
        */
    }



}


int main(int argc, char** argv) {

    //printf("%i bytes ram\n",sizeof(_RAM));
    //printf("%i bytes eep\n",sizeof(_EEP));
    _stag stag;

    stag.init();


    //hklx_decoder_t hklx_decoder;
    
    hklx_decoder_t hklx_decoder0;
    hklx_decoder_t hklx_decoder1;
    hklx_decoder_t hklx_decoder2;
    hklx_decoder_t hklx_decoder3;
    
    hklx_packet_t *hklx_packet;


    int bytes_received;

    //hklx_decoder_initialise(&hklx_decoder);
    hklx_decoder_initialise(&hklx_decoder0);
    hklx_decoder_initialise(&hklx_decoder1);
    hklx_decoder_initialise(&hklx_decoder2);
    hklx_decoder_initialise(&hklx_decoder3);


/*
    hklx_packet = hklx_packet_allocate(3, 4, CMD_RAM_WRITE);

    hklx_packet->data[0] = 0x30;
    hklx_packet->data[1] = 0x01;
    hklx_packet->data[2] = 0x00;
    //hklx_send(hklx_packet);

    hklx_send(0, hklx_packet);
    hklx_send(1, hklx_packet);
    hklx_send(2, hklx_packet);
    hklx_send(3, hklx_packet);
    hklx_packet_free(&hklx_packet); 
*/


    

    //set toqure ON
    hklx_packet = hklx_packet_allocate(3, BROADCAST_ID, CMD_RAM_WRITE);

    hklx_packet->data[0] = 0x34;
    hklx_packet->data[1] = 0x01;
    hklx_packet->data[2] = 0x00;//torque free
    //hklx_packet->data[2] = 0x60;//torque on
    //hklx_send(hklx_packet);

    hklx_send(0, hklx_packet);
    hklx_send(1, hklx_packet);
    hklx_send(2, hklx_packet);
    hklx_send(3, hklx_packet);
    hklx_packet_free(&hklx_packet);
    /*//set baud rate
    hklx_packet = hklx_packet_allocate(3, BROADCAST_ID, CMD_EEP_WRITE);

    hklx_packet->data[0] = 0x04;
    hklx_packet->data[1] = 0x01;
    hklx_packet->data[2] = 0x10;
    //hklx_send(hklx_packet);

    hklx_send(0, hklx_packet);
    hklx_send(1, hklx_packet);
    hklx_send(2, hklx_packet);
    hklx_send(3, hklx_packet);
    hklx_packet_free(&hklx_packet);    
    */
    
    //float phi[3];
    //phi[0]=0.0f;
    //phi[0]=166.650f*M_PI/180.0f;
    //phi[1]=0.0f;
   // phi[2]=0.0f;
    
    
    //i_jog[0].jog.value=(uint16_t)((phi[0]*180.0f/M_PI+166.650f)/0.3258064516f);
    
    //printf("%i\n",(uint16_t)round((phi[0]*180.0f/M_PI+166.650f)/0.3258064516f));
    
    //usleep(2000000000000);
    usleep(100000);
    //ijl(int leg_id,float phi[],uint8_t time,uint8_t LED0,uint8_t LED1,uint8_t LED2);
    
    
    
    
    
    //ijl(0, phi, 200, BLUE, BLUE, BLUE); 
    //usleep(1000);
    while(1){
        
        

        /*
        hklx_packet = hklx_packet_allocate(2, BROADCAST_ID, CMD_RAM_READ);

        hklx_packet->data[0] = 0x3A;
        hklx_packet->data[1] = 0x04;
        //hklx_packet->data[2] = 0x60;
        //hklx_send(hklx_packet);

        hklx_send(0, hklx_packet);
        hklx_send(1, hklx_packet);
        hklx_send(2, hklx_packet);
        hklx_send(3, hklx_packet);
        hklx_packet_free(&hklx_packet);
        */ 
        //printf("IN LOOP\n");

        /*
        hklx_packet = hklx_packet_allocate(2, 0x00, CMD_RAM_READ);

        hklx_packet->data[0] = 0x3A;
        hklx_packet->data[1] = 0x04;
        //hklx_packet->data[2] = 0x60;
        //hklx_send(hklx_packet);

        hklx_send(0, hklx_packet);
        hklx_packet_free(&hklx_packet);        
        */

        hklx_packet = hklx_packet_allocate(2, 0x00, CMD_RAM_READ);

        //hklx_packet->data[0] = 0x3A;
        //hklx_packet->data[1] = 0x04;
        hklx_packet->data[0] = 54;
        hklx_packet->data[1] = 20;
        
        hklx_send(0, hklx_packet);
        hklx_packet_free(&hklx_packet);
        

        if ((bytes_received = PollComport(LEG_0_COM, hklx_decoder_pointer(&hklx_decoder0), hklx_decoder_size(&hklx_decoder0))) > 0) {
                        
            hklx_decoder_increment(&hklx_decoder0, bytes_received);
            while ((hklx_packet = hklx_packet_decode(&hklx_decoder0)) != NULL) {

                
                //hklx_dump(hklx_packet);
                stag.hklx_packet_parse(hklx_packet);
                hklx_packet_free(&hklx_packet);
                //printf("leg 0 com\n");
            }
        }

        if ((bytes_received = PollComport(LEG_1_COM, hklx_decoder_pointer(&hklx_decoder1), hklx_decoder_size(&hklx_decoder1))) > 0) {
            
                       
            hklx_decoder_increment(&hklx_decoder1, bytes_received);
            while ((hklx_packet = hklx_packet_decode(&hklx_decoder1)) != NULL) {

                hklx_packet_free(&hklx_packet);
                //printf("leg 1 com\n");
            }
        }

        if ((bytes_received = PollComport(LEG_2_COM, hklx_decoder_pointer(&hklx_decoder2), hklx_decoder_size(&hklx_decoder2))) > 0) {
            
                       
            hklx_decoder_increment(&hklx_decoder2, bytes_received);
            while ((hklx_packet = hklx_packet_decode(&hklx_decoder2)) != NULL) {

                hklx_packet_free(&hklx_packet);
                //printf("leg 2 com\n");
            }
        }

        if ((bytes_received = PollComport(LEG_3_COM, hklx_decoder_pointer(&hklx_decoder3), hklx_decoder_size(&hklx_decoder3))) > 0) {
            
                       
            hklx_decoder_increment(&hklx_decoder3, bytes_received);
            while ((hklx_packet = hklx_packet_decode(&hklx_decoder3)) != NULL) {

                hklx_packet_free(&hklx_packet);
                //printf("leg 3 com\n");


            }
        }

        usleep(100000);

        
    }
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    //set toqure ON    

    /*
    hklx_packet = hklx_packet_allocate(2, 3, CMD_EEP_READ);

    hklx_packet->data[0] = 0x35;
    hklx_packet->data[1] = 0x01;
    hklx_send(1, hklx_packet);
    hklx_packet_free(&hklx_packet);
    
    //was -27 or e5

    while (1) {
        if ((bytes_received = PollComport(LEG_1_COM, hklx_decoder_pointer(&hklx_decoder1), hklx_decoder_size(&hklx_decoder1))) > 0) {
            hklx_decoder_increment(&hklx_decoder1, bytes_received);
            while ((hklx_packet = hklx_packet_decode(&hklx_decoder1)) != NULL) {


                for (int i = 0; i < hklx_packet->packetsize - 7; i++) {

                    printf("%x ", hklx_packet->data[i]);

                    
                    
                }
                printf("\n");

                int8_t cp = 0;
                memcpy(&cp, &hklx_packet->data[2], 1);

                printf("%i\n",cp);
                
                hklx_packet_free(&hklx_packet);
                printf("leg 1 com\n");
            }
        }


    }
    */

    /*
    int8_t cp = -10;

    hklx_packet = hklx_packet_allocate(3, 3, CMD_EEP_WRITE);

    hklx_packet->data[0] = 0x35;
    hklx_packet->data[1] = 0x01;
    memcpy(&hklx_packet->data[2],&cp,  1);
    hklx_send(1, hklx_packet);
    hklx_packet_free(&hklx_packet);    
    */
    
    int it = 0;

    

    float xl = 0.03;
    //float ylf = 0.05;
    //float ylb = -0.04;
    
    float ylf = 0.04;
    float ylb = -0.04;
        

    
    float stride = 0.04;
    
    float x = 0.03;
    float y = 0.03;
    //float ys = y;
    
    
    //float h1 = 0.09;
    //float h2 = 0.12;

    float bh = 0.13;
    
    int jt = 15;

    
    float jt_max=15.0f;
    float jt_min=50.0f;    
    
 

    float bx = 0.06;
    float by = 0.075;

    
    /*
    float phi[3]={0,0,0};
    ijl(0, phi, 100, BLUE, BLUE, BLUE);
    ijl(1, phi, 100, BLUE, BLUE, BLUE);
    ijl(2, phi, 100, BLUE, BLUE, BLUE);
    ijl(3, phi, 100, BLUE, BLUE, BLUE);    
    
    usleep(2000000000000000);
    
     
    

    ikleg(0, -bx, 0, -bh, 200);
    ikleg(1, -bx, 0, -bh, 200);
    ikleg(2, -bx, 0, -bh, 200);
    ikleg(3, -bx, 0, -bh, 200);
    usleep(200000);
    */
    
    /*
    Mat leg_xyz = (Mat_<float>(3, 4) <<
            0, 0, 0, 0,
            0, 0, 0, 0,
            0, 0, 0, 0);


    body_jog(leg_xyz, 0, 0, 0,0,0, bh, 200);
    
    usleep(200000000000);    
    */
    
    
    
    
    
    
    
    Mat leg_xyz = (Mat_<float>(3, 4) <<
            bx + xl, -bx - xl, -bx - xl, bx + xl,
            by + ylf, by + ylf, -by + ylb, -by + ylb,
            0, 0, 0, 0);
    

    
    //float pitch = M_PI*4.0/180.0;
    float pitch = 0;

    body_jog(leg_xyz, 0, pitch, 0,0,0, bh, 100);
    
    

    //get to prone position
    //ikleg(0, xl, yl, h2, 50);
    //ikleg(1, xl, -yl, h2, 50);
    //ikleg(2, xl, yl, h2, 50);
    //ikleg(3, xl, -yl, h2, 50);
    
    
    usleep(2000000);


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
            sixm.L3_mag=0.0f;
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

            
            //usleep(1000000);//1 second 
            //printf("tic\n");
            int tic = 2;
            
            float max_speed = 0.4/1000000;
            
            float min_speed = 0;
            
            //float dt = tic*
            
            if (sixm.L3_mag > 0) {

                if(abs(y1)>=stride){
                    ys1*=-1.0f;
                }

                if(abs(y2)>=stride){
                    ys2*=-1.0f;
                }
                
                
                h1 = h*cos((y1/stride)*M_PI/2);
                h2 = h*cos((y2/stride)*M_PI/2);
                
                if(ys1<0)
                    h1=0;
                
                if (ys2 < 0)
                    h2 = 0;
                
                float v =min_speed+(max_speed-min_speed)*abs(sixm.L3_y)/128.0f;
                
                float dt = (float)tic*v*10000.0f;
                //printf("%f\n",dt);
                y1+=ys1*dt;
                y2+=ys2*dt;
                
                leg_xyz = (Mat_<float>(3, 4) <<
                        bx + xl, -bx - xl, -bx - xl, bx + xl,
                        by + ylf+y1, by + ylf+y2, -by + ylb+y1, -by + ylb+y2,
                        h1        , h2        , h1         , h2);
                
                
                body_jog(leg_xyz, 0, pitch, 0,0,0, bh, tic);
                
                usleep(tic*10000);
                
            }else{
                
                y=0;
                
                leg_xyz = (Mat_<float>(3, 4) <<
                        bx + xl, -bx - xl, -bx - xl, bx + xl,
                        by + ylf, by + ylf, -by + ylb, -by + ylb,
                        0, 0, 0, 0);
                
                body_jog(leg_xyz, 0, pitch, 0,0,0, bh, 50);
                
                usleep(50*10000);
            }
            
            
            //usleep(tic*10000);
            
            
            //body roll shiz
            /*
            jt = (int) (jt_min - (jt_min - jt_max) * sixm.L3_mag / 128.0f);   
            y = stride * sixm.L3_y / sixm.L3_mag;
            x = stride * sixm.L3_x / sixm.L3_mag;
            float dxy = 0.01;

            float b_y = 0;
            float b_x = 0;

            if (sixm.R3_mag > 0) {

                b_y = dxy * sixm.R3_y / 128.0f;
                b_x = dxy * sixm.R3_x / 128.0f;
            }

            if (abs(sixm.aX) > 200) {
                sixm.aX = 200 * sixm.aX / abs(sixm.aX);
            }
            if (abs(sixm.aY) > 200) {
                sixm.aY = 200 * sixm.aY / abs(sixm.aY);
            }
            
            float max_ang = M_PI*15.0/180.0;
            
            float roll = max_ang*sixm.aX/200;
            float pitch = max_ang*sixm.aY/200;
            float yaw = max_ang * sixm.L3_x / 128.0f;
            
            body_jog(leg_xyz, yaw, pitch, roll,b_x,b_y, h2, 10);

            //usleep(10*10000);
            usleep(10000);
            */


            
                

        }






    }










    return 0;
}

