#include <FlexiTimer2.h>
#include <EEPROM.h>
#include <Servo.h> 

#define NO_PWM 6

byte addr = 0; //EEPROM address

byte EEPROM_Initialised_addr = 0;
byte EEPROM_Initialised;

byte Zigbee_id_addr = 1; // Holds identifier for Xbee communication
byte Zigbee_id = 2;

byte Servos_Enable_pin_addr = 2;
byte Servos_Enable_pin = 4;

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
byte Servos_Min[6] = {0,10,10,10,0,78};
byte Servos_Idle[6] = {128,184,128,10,128,128};
byte Servos_Rev[NO_PWM] = {0,0,0,1,0,0}; // 0-standard, 1-reversed

byte buffer[255];
byte input_buffer[255];

void setup() 
{ 
  int i=0; // loop variable
  
  EEPROM_Initialised = EEPROM.read(EEPROM_Initialised_addr);
  
  
  //
  
  if(EEPROM_Initialised == 255)
  {
    EEPROM.write(EEPROM_Initialised_addr, 1);
    EEPROM.write(Zigbee_id_addr, Zigbee_id);    
    EEPROM.write(Servos_Enable_pin_addr, Servos_Enable_pin);
    for (i=0;i<NO_PWM;i++)
    {
      EEPROM.write(Pin_type_addr[i], Pin_type[i]);
      EEPROM.write(Servos_Pin_addr[i], Servos_Pin[i]);
      EEPROM.write(Servos_Angle_addr[i], Servos_Idle[i]);
      EEPROM.write(Servos_Speed_addr[i], Servos_Speed[i]);
      EEPROM.write(Servos_Max_addr[i], Servos_Max[i]);
      EEPROM.write(Servos_Min_addr[i], Servos_Min[i]);
      EEPROM.write(Servos_Idle_addr[i], Servos_Idle[i]);
      EEPROM.write(Servos_Rev_addr[i], Servos_Rev[i]);
    }
  }
  
  Zigbee_id = EEPROM.read(Zigbee_id_addr);
  
  Servos_Enable_pin = EEPROM.read(Servos_Enable_pin_addr);
  pinMode(Servos_Enable_pin, OUTPUT);
  digitalWrite(Servos_Enable_pin, LOW);
  
  for (i=0;i<NO_PWM;i++)
  {
     if(!Pin_type[i])
     {
       Servos[i].attach(Servos_Pin[i]);
       Servos_Angle[i] = Servos_Idle[i];       
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
  Serial.write((byte) EEPROM.read(Zigbee_id_addr));
} 
 
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
  
  byte msg = ser_message();
  
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
  
  for (int i=0;i<NO_PWM;i++)
  {
    if(Pin_type[i])
    {
      analogWrite(Servos_Pin[i], Servos_Angle[i]);
    }
    else
    {
      byte angle;
      if (Servos_Rev[i])
      {
        angle = (byte)map(Servos_Angle[i],0,255,Servos_Max[i],Servos_Min[i]);
      }
      else
      {
        angle = (byte)map(Servos_Angle[i],0,255,Servos_Min[i],Servos_Max[i]);
      }
      
      Servos[i].write(NextServoPos(i,angle,Servos_Speed[i]));
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

