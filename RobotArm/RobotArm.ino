#include <FlexiTimer2.h>
#include <EEPROM.h>
#include <Servo.h> 
#include <math.h>

#define NO_PWM 6

#define L1 15.0
#define L2 130.0
#define L3 130.0
#define L4 70.0

byte addr = 0; //EEPROM address

byte Zigbee_id = 2; // pid for comms protocol
byte Servos_Enable_pin = 4;

float arm_pos[6] = {100.0,00.0,100.0,-30.0,0.0,0.0}; // x, y, z, pitch, roll, claw
const rad_180 = (M_PI);
const rad_170 = (M_PI*0.944444444);
const rad_156 = (M_PI*0.866666666);
const rad_78 = (M_PI*0.4333333333);
const rad_10 = (M_PI*0.0555555555);


Servo Servos[NO_PWM];
byte Pin_type_addr[NO_PWM] = {10,11,12,13,14,15};
byte Servos_Pin_addr[NO_PWM] = {20,21,22,23,24,25};
byte Servos_Angle_addr[NO_PWM] = {30,31,32,33,34,35};
byte Servos_Speed_addr[NO_PWM] = {40,41,42,43,44,45};
byte Servos_Max_addr[NO_PWM] = {50,51,52,53,54,55};
byte Servos_Min_addr[NO_PWM] = {60,61,62,63,64,65};
byte Servos_Idle_addr[NO_PWM] = {70,71,72,73,74,75};
byte Servos_Rev_addr[NO_PWM] = {80,81,82,83,84,85};

byte Pin_type[NO_PWM] = {0,0,0,0,0,0}; // 0=servo, 1=analog
byte Servos_Pin[NO_PWM] = {3,5,6,10,9,11};
byte Servos_Speed[NO_PWM] = {3,2,2,2,2,4};
byte Servos_Angle[NO_PWM] = {90,90,90,90,90,90};
byte Servos_Max[6] = {180,170,170,170,180,156};
float Servos_Max_rad[6] = {rad_180, rad_170, rad_170, rad_170, rad_180, rad_156};
byte Servos_Min[6] = {0,10,10,10,0,78};
float Servos_Min_rad[6] = 0, rad_10,rad_10,rad_10,0,rad_78};
byte Servos_Idle[6] = {128,184,128,10,128,128};
byte Servos_Rev[NO_PWM] = {0,0,0,1,0,0}; // 0-standard, 1-reversed

byte buffer[255];
byte input_buffer[255];

void setup() 
{ 
  int i=0; // loop variable
  
  inv_kinematics(arm_pos,Servos_Angle);

  pinMode(Servos_Enable_pin, OUTPUT);
  digitalWrite(Servos_Enable_pin, LOW);
  
  for (i=0;i<NO_PWM;i++)
  {
     if(!Pin_type[i])
     {
       Servos[i].attach(Servos_Pin[i]);   
       Servos[i].write(Servos_Angle[i]);
     }
     else
     {
        pinMode(Servos_Pin[i], OUTPUT);
     }
  }
  
  FlexiTimer2::set(20, ServoCall);
  FlexiTimer2::start();
  
  Serial.begin(115200);  
  
  digitalWrite(Servos_Enable_pin, HIGH);  
} 

float x_inc = 1.0;

void loop() 
{  
  byte  zbee;
  byte cmd;
  byte  addr;
  byte  val0;
  byte  val1;
  byte  val2;
  byte  val3;
  byte  val4;
  byte  val5;
  byte  CRC1;
  
  byte msg = 0;//ser_message();
  
  
  if(arm_pos[5] >= 0.95)
  {
    x_inc = -0.02;
  }
  else if(arm_pos[5] <= 0.05)
  {
    x_inc = 0.02;
  }
  
  arm_pos[5] = arm_pos[5] + x_inc;
  print_ang();
  
  if(msg==2)
  {
    ser_send(7, Zigbee_id, 0x40, 0);
  }
  if (msg==1)
  {
      zbee = input_buffer[3]; //zbee addr
      cmd =  input_buffer[4]; //read or write
      addr = input_buffer[7]; // Start Addr      
      
      if(zbee == Zigbee_id) //Ensure the message is valid and For this Recevier
      {
        switch(cmd)
        {
           case 0x02:
           {
              byte return_buffer[7]; //return string framework $, 0, addr, val, crc
              return_buffer[0] = addr;
              return_buffer[1] = Servos_Angle[0];
              return_buffer[2] = Servos_Angle[1];
              return_buffer[3] = Servos_Angle[2];
              return_buffer[4] = Servos_Angle[3];
              return_buffer[5] = Servos_Angle[4];
              return_buffer[6] = Servos_Angle[5];

              ser_send(14, Zigbee_id, 0x42, return_buffer);
              
              break;
           }
           case 0x01:
           {
             
              Servos_Angle[0] = input_buffer[8];      
              Servos_Angle[1] = input_buffer[9];
              Servos_Angle[2] = input_buffer[10];
              Servos_Angle[3] = input_buffer[11];
              Servos_Angle[4] = input_buffer[12];
              Servos_Angle[5] = input_buffer[13];
                                      
              byte return_buffer[1];
              
              return_buffer[0] = addr;
                            
              ser_send(8, Zigbee_id, 0x41, return_buffer);              
              
              
              break;
           } 
        }
      }
         
    }
  delay(50);
}

int print_ang()
{
 Serial.print("phi1 = : ");
  Serial.println((int)Servos_Angle[0]);
  Serial.print("phi2 = : ");
  Serial.println((int)Servos_Angle[1]);
  Serial.print("phi3 = : ");
  Serial.println((int)Servos_Angle[2]);
  Serial.print("phi4 = : ");
  Serial.println((int)Servos_Angle[3]);
  Serial.print("roll = : ");
  Serial.println((int)Servos_Angle[4]);
  Serial.print("claw = : ");
  Serial.println((int)Servos_Angle[5]);
  Serial.println("");
 return 1; 
}

float deg(float radian)
{
    return radian * (180.0 / M_PI);
}

float rad(float degree)
{
    return degree * (M_PI/180.0);
}

byte ang_to_pos(float x)
{
    float y = ((x)*(255.0/180.0));
    if(y<0.0) y=0.0;
    if(y>255.0) y=255.0;
    return (byte)(y+0.5);
}

byte inv_kinematics(float *pos,byte *retbuffer)
{
  float x = pos[0];
  float y = pos[1];
  float z = pos[2];
  float p = rad(pos[3]);  //-90.0 to 90.0
  float r = pos[4];       //-90.0 to 90.0
  float c = pos[5];       //0.0 to 0.1
  
  float phi1=0;
  float phi2=0;
  float phi3=0;
  float phi4=0;
  
  float L5 = ((26.0)*sin(rad(255.0*c)/3.269)+(80.0));
  phi1 = atan2(y,x);
  
  if (abs(phi1) > M_PI/2) return 0; // Out of range
  
  float x1 = L1*cos(phi1);
  float y1 = L1*sin(phi1);
  
  float L45 = L4 + L5;
  float xy3 = L45*cos(p);
  float x3 = x - xy3*cos(phi1);
  float y3 = y - xy3*sin(phi1);
  float z3 = z - L45*sin(p);
  
  float L23 = sqrt((x3-x1)*(x3-x1) + (y3-y1)*(y3-y1) + (z3-0)*(z3-0));
  
  if(L23 < 80.0 || L23 > (L2 + L3)) return 0; // Out of range
  
  float xy = sqrt(x*x + y*y);  
  float xy1= xy-L1- cos(p)*L45;
  float z1 = z-sin(p)*L45;
  
  phi2 = acos((L2*L2 + L23*L23 - L3*L3)/(2*L2*L23))+atan2(z1,xy1);
  
  if (phi2 > Servos_Max_rad[1] || phi2 < Servos_Min_rad[1] ) return 0; // Out of range  
  phi3 = acos((L2*L2 + L3*L3 - L23*L23)/(2*L2*L3));
  if (phi3 > Servos_Max_rad[2] || phi3 < Servos_Min_rad[2] ) return 0; // Out of range 
  phi4 = p-(phi2+phi3)+M_PI;
  if (phi4 > Servos_Max_rad[3] || phi4 < Servos_Min_rad[3] ) return 0; // Out of range 
  
  retbuffer[0] = deg(phi1 + M_PI/2);
  retbuffer[1] = deg(phi2);
  retbuffer[2] = deg(phi3);
  retbuffer[3] = deg(phi4+M_PI/2.0);
  retbuffer[4] = (byte)(r + 90.0);
  retbuffer[5] = (byte)(78.0*c + 78.0)+0.5);

  return 1;
}

//Using Herculex servo comms packet
byte ser_send(byte len, byte pid, byte cmd, const byte *data)
{
  byte sbuf[255];
  sbuf[0] = 0xFF;
  sbuf[1] = 0xFF;
  sbuf[2] = len;
  sbuf[3] = pid;
  sbuf[4] = cmd;
  sbuf[5] = 0x00;
  sbuf[6] = 0x00;
  for(int i=0;i<(len-7);i++) sbuf[7+i] = data[i];
  unsigned int crc = getCheckSum(sbuf);
  
  sbuf[5] = (byte) ((crc & 0xFF00) >> 8);
  sbuf[6] = (byte) (crc & 0x00FF);
  
  for(int i=0;i<len;i++) Serial.write(sbuf[i]);

  return 1;
}

byte ser_message() // Returns 0x01 if recived packet, 0xFF on timeout
{
  byte c = 0;
  byte beg = 0;
  byte state = 0;
  byte checksum1;
  byte checksum2;
  byte i = 0;
//  unsigned long currentMillis = millis();
//  long interval = 1000 + currentMillis;
  
  while(1)
  {
    if ((Serial.available() > 0)) 
    {    
      c = Serial.read();
      //Serial.write(c);
      switch(state)
      {
        case 0:
        {          
          if(c==0xFF)
          {
            input_buffer[i++]=c;
            input_buffer[i++]=c;
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
              state++;
            }
          }
          break;
        }
        case 2:
        {          
          input_buffer[i++] = c;
          if(i==input_buffer[2])state++;
          break;   
        }
      }    
    }
    if(state==3)
    {      
      unsigned int crc = getCheckSum(input_buffer);
      checksum2 = (byte) ((crc & 0xFF00) >> 8);
      checksum1 = (byte) (crc & 0x00FF);
      if((checksum1) == input_buffer[5] && (checksum2) == input_buffer[6]){
        *buffer = *input_buffer;
        return 1;
      }
      else return 2;         
    }    
  }
  return 0;
}

// Calculates the checksum for a given buffer
// returns as byte
unsigned int getCheckSum(const byte *data) 
{
  byte *bytes = (byte *) data;
  unsigned int crc;
  
  byte CheckSum1 = bytes[2] ^ bytes[3] ^ bytes[4];
  for (int i = 0; i < ((int) bytes[2] - 7); i++)
        CheckSum1 ^= bytes[7 + i];
  
  byte CheckSum2 = ~(CheckSum1);
    CheckSum1 &= 0xFE;
    CheckSum2 &= 0xFE;
  
  crc = CheckSum1;
  crc |= CheckSum2 << 8; 
  
  return crc;
}

int chartoint(char inchar)
{
 return inchar - 48; 
}


void ServoCall()
{
  inv_kinematics(arm_pos,Servos_Angle);
  for (int i=0;i<NO_PWM;i++)
  {     
      Servos[i].write(NextServoPos(i,Servos_Angle[i],Servos_Speed[i]));
    }
  }
  
}

int NextServoPos(int Sel, int Angle, int Speed)  // Returns the angle to turn to
{
  int Pos;
  Pos = Servos[Sel].read();
     
  if(abs(Angle - Pos) >= Speed)
  {
    if ((Angle - Pos) > 0)
    {
      return Pos + Speed;
    }
    else
    {
      return Pos - Speed;
    }
  }
  else
  {
    return Angle;
  }
}

