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
byte Servos_Pin[NO_PWM] = {3,5,6,9,10,11};
byte Servos_Speed[NO_PWM] = {1,1,1,1,1,1};
byte Servos_Angle[NO_PWM] = {90,90,90,90,90,90};
byte Servos_Max[6] = {180,180,180,180,180,180};
byte Servos_Min[6] = {0,0,0,0,0,0};
byte Servos_Idle[6] = {128,128,128,128,128,128};
byte Servos_Rev[NO_PWM] = {0,0,0,0,0,0}; // 0-standard, 1-reversed

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
      EEPROM.write(Servos_Angle_addr[i], Servos_Angle[i]);
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
    Pin_type[i] = (byte) EEPROM.read(Pin_type_addr[i]);
    Servos_Pin[i] = (byte) EEPROM.read(Servos_Pin_addr[i]);
    Servos_Angle[i] = (byte) EEPROM.read(Servos_Angle_addr[i]);
    Servos_Speed[i] = (byte) EEPROM.read(Servos_Speed_addr[i]);
    Servos_Max[i] = (byte) EEPROM.read(Servos_Max_addr[i]);
    Servos_Min[i] = (byte) EEPROM.read(Servos_Min_addr[i]);
    Servos_Idle[i] = (byte) EEPROM.read(Servos_Idle_addr[i]);
    Servos_Rev[i] = (byte) EEPROM.read(Servos_Rev_addr[i]);
  }
  
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
  byte buffer[7];
  byte  zbee;
  byte cmd;
  byte  addr;
  byte  val;
  byte  CRC1;
  
  char temp;
  
  if (Serial.available() > 0) 
  {
    
    // get incoming byte:
    temp = Serial.read();
    if( temp == '$' ) 
    {
      buffer[0] = (byte) '$';
      for(int i=1; i<7; i++) 
      {
        buffer[i] = Serial.read();
      }
      
      zbee = buffer[1]; //zbee addr
      cmd =  buffer[2]; //Address to read or write
      addr = buffer[3];
      val = buffer[4];
      CRC1 = buffer[5];      
      
      for(int i=0; i<7; i++) 
      {
        Serial.write(buffer[i]);
      }
      
      if(CRC1 == getCheckSum(buffer,5) || CRC1 == '%') //Ensure the message is valid
      {
        if(zbee == Zigbee_id) //Ensure that this is the intended destination
        {
          if(cmd == 0x52) // 'R'
          {
            byte return_buffer[7]; //return string framework $, 0, addr, val, crc
            return_buffer[0] = (byte) '$';
            return_buffer[1] = (byte) EEPROM.read(Zigbee_id_addr);
            return_buffer[2] = 0x00; // reply cmd
            return_buffer[3] = addr;
            return_buffer[4] = (byte) EEPROM.read(addr);
            return_buffer[5] = getCheckSum(return_buffer,4);
            return_buffer[6] = (byte)  '^';
            
            for(int i=0; i<7; i++) 
            {
              Serial.write(return_buffer[i]);
            }
            
          }
          else if(cmd == 0x57) //'W'
          {
            EEPROM.write(addr, val);
            byte return_buffer[7]; //return string framework $, 0, addr, val, crc
            return_buffer[0] = (byte) '$';
            return_buffer[1] = (byte) EEPROM.read(Zigbee_id_addr);
            return_buffer[2] = 0x01; // write reply cmd
            return_buffer[3] = addr;
            return_buffer[4] = (byte) EEPROM.read(addr);
            return_buffer[5] = getCheckSum(return_buffer,4);
            return_buffer[6] = (byte)  '^';
            
            for(int i=0; i<7; i++) 
            {
              Serial.write(return_buffer[i]);
            }            
            
            if (addr == 1)
            {
              Zigbee_id = val;
            }
            if (addr == 2)
            {
              Servos_Enable_pin = val;
            }
            if (addr > 9) 
            {
              unsigned int type = addr/10;
              unsigned int sel = addr%10;
              switch(type) //Select Servo Pin
              { 
                case 1:
                  Pin_type[sel] = val;
                  break;
                case 2:
                  Servos_Pin[sel] = val;
                  break;
                case 3:
                  Servos_Angle[sel] = val;                  
                  break;
                case 4:
                  Servos_Speed[sel] = val;
                  break;
                case 5:
                  Servos_Max[sel] = val;
                  break;
                case 6:
                  Servos_Min[sel] = val;
                  break;
                case 7:
                  Servos_Idle[sel] = val;
                  break;
                case 8:
                  Servos_Rev[sel] = val;
                  break;
                default:
                  val = val;           
                  break; 
              }
            }
          }
        } 
        else
        {
          Serial.println("Flush");
          Serial.flush();
        }  
      }      
    }
  }
  
  delay(50);
}

// Calculates the checksum for a given buffer
// returns as integer
byte getCheckSum(byte *buffer, int length) 
{
  int i;
  byte XOR;
  byte c;
  // Calculate checksum ignoring any $'s in the string
  for (XOR = 0, i = 0; i < length; i++) {
    c = buffer[i];
    XOR ^= c;
  }
  return XOR;
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
        angle = map(Servos_Angle[i],0,255,Servos_Max[i],Servos_Min[i]);
      }
      else
      {
        angle = map(Servos_Angle[i],0,255,Servos_Min[i],Servos_Max[i]);
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

