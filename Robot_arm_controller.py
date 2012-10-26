def re_range(x):
    y = int(x*(255/180.0))
    if(y<0):
        y=0
    if(y>255):
        y=255
    return y

def claw_length(oc):
    oc2 = 255.0*oc
    return ((20.4/1000)*math.sin(math.radians(oc2/3.269))+(80.0/1000.0))

def kinematic(v): #x, y, z, p, r, oc): # in mm, degrees and percentage open
    L1 = 0.015
    L2 = 0.13
    L3 = 0.13
    L4 = 0.07    

    x=v[0]/1000.0
    y=v[1]/1000.0
    z=v[2]/1000.0
    r=v[4]*1.4167
    p=math.radians(v[3])
    oc = v[5]

    print 'v: ' + str(v)
    
    L5 = claw_length(oc)

    # input
    
    phi1= math.atan2(y,x)
    print('phi1: ' + str(phi1))
    xy = math.sqrt(x*x + y*y)
    print('xy: ' + str(xy))
    L45=L4+L5
    print('L45: ' + str(L45))
    xy1=xy-L1- math.cos(p)*L45
    print('xy1: ' + str(xy1))
    z1 = z-math.sin(p)*L45
    print('z1: ' + str(z1))
    L13=math.sqrt(z1*z1+xy1*xy1)
    print('L13: ' + str(L13))
    
##    if((L13 > (L2 + L3)) or (L13 < math.sqrt(L2*L2 + L3*L3))):
##        print 'L3 our of range'
##        print 'L3 Max: ' + str(L2 + L3)
##        print 'L3 Min: ' + str(math.sqrt(L2*L2 + L3*L3))
    

    
    # build mode

    try:
        er = 0
        phi2 = math.acos((L2*L2 + L13*L13 - L3*L3)/(2*L2*L13))+math.atan2(z1,xy1)
        print('phi2: ' + str(math.degrees(phi2)))
        er = 1
        #phi3 = (2*math.pi - 2*phi2 - math.pi/2)
        phi3 = math.acos((L3*L3 + L2*L2 - L13*L13 )/(2*L2*L13))
        print('phi3: ' + str(math.degrees(phi3)))
        er = 2
        phi4 = math.atan(math.tan(p-((phi2+phi3))))
        print('phi4: ' + str(math.degrees(phi4)))
    except:
        
        if(er == 0):
            print 'Error on phi2'   
        elif(er == 1):
            print 'Error on phi3'
        elif(er == 2):
            print 'Error on phi4'
            
        phi2 = math.pi/2
        phi3 = math.pi/2
        phi4 = math.pi/2  

    if(phi4 < 0.0):
        phi4 += math.pi

    phi5 = r

    phi6 = 255*oc

    return [re_range(math.degrees(phi1)), re_range(math.degrees(phi2)), re_range(math.degrees(phi3)), re_range(math.degrees(phi4)), re_range(phi5), phi6]
    

def openport():
    try:
        ser = serial.Serial(ser_port0,baudrate=115200,)
        return ser
    except:
        try:
            ser = serial.Serial(ser_port1,baudrate=115200,)
            return ser
        except:
            try:
                ser = serial.Serial(ser_port2,baudrate=115200,)
                return ser
            except:
                return -1

def closeport(ser):
    ser.close()


def CRC(buff):
    CK_A = 0
    CK_B = 0
    for n in range(2, len(buff)-2):
        CK_A = CK_A + buff[n]
        CK_B = CK_B + CK_A

    CK_A = CK_A % 2**8
    CK_B = CK_B % 2**8
    
    if(CK_A==buff[len(buff)-2] and CK_B==buff[len(buff)-1]):
        return 1
    else:
        return 0
    
def set_pos(p):
    write_servo(module,0,int(p[0]),30)
    write_servo(module,1,int(p[1]),30)
    write_servo(module,2,int(p[2]),30)
    write_servo(module,3,int(p[3]),30)
    write_servo(module,4,int(p[4]),30)
    write_servo(module,5,int(p[5]),30)

def pos1():
    write_servo(module,0,0,30)
    write_servo(module,1,100,30)
    write_servo(module,2,60,30)
    write_servo(module,3,170,30)
    write_servo(module,4,10,30)
    write_servo(module,5,128,30)

def pos2():
    write_servo(module,0,255,30)
    write_servo(module,1,220,30)
    write_servo(module,2,30,30)
    write_servo(module,3,10,30)
    write_servo(module,4,10,30)
    write_servo(module,5,128,30)

def pos3():
    write_servo(module,0,128,30)
    write_servo(module,1,128,30)
    write_servo(module,2,128,30)
    write_servo(module,3,128,30)
    write_servo(module,4,128,30)
    write_servo(module,5,128,30)
    
def write_servo(module, servo, value,types):
    ser = openport()
    if ser != -1:
        write = 1
        start = 0
        message = []
        while write:
            command = 0x2400570000255E
            argument = 0x00000000000000
            argument += module<<(40)
            argument += ((servo+types)<<(24))
            argument += (value<<(16))
            
            cmd_str = hex(argument|command)
            cmd_str = cmd_str[0:len(cmd_str)-1]
            ser.write(cmd_str[2:].decode('hex'))
            run = 1
            while (run < 20000):
                if ser.inWaiting() >0:
                    temp =  ser.read(1)
                    if temp == '$':
                        start = 1;
                    if start:
                        message.append(ord(temp))
                    if temp == '^':
                        run = 200001
                        start = 0
                run += 1
            if run == 200002:
                if message[1] == module and message[2] == 1 and message[3] == (types+servo):
                    write = 0
        ser.close()
    
def read_servo(servo, types):
    ser = openport()
    ret = -1
    if ser != -1:
        write = 1
        start = 0
        message = []
        while write:
            command = 0x2400520000255E
            argument = 0x00000000000000
            argument += module<<(40)
            argument += ((servo+types)<<(24))
            
            cmd_str = hex(argument|command)
            cmd_str = cmd_str[0:len(cmd_str)-1]
            ser.write(cmd_str[2:].decode('hex'))
            run = 1
            while (run < 20000):
                if ser.inWaiting() >0:
                    temp =  ser.read(1)
                    if temp == '$':
                        start = 1;
                    if start:
                        message.append(ord(temp))
                    if temp == '^':
                        run = 200001
                        start = 0
                run += 1
            if run == 200002:
                if message[1] == module and message[2] == 0 and message[3] == (types+servo):
                    write = 0
                    ret = message[4]
        ser.close()
    return ret

def read_all(types):
    ret = []
    for n in range[0:6]:
        ret.append(read_servo(n,types))
    return ret
    
def main():
    ty = [00, 30, 40, 50, 60, 80, 00, 00, 00, 00, 00, 30, 40, 50, 60, 70, 80, 00, 00, 00, 00]
    running = 1
    while(running):
        print '--- Servo Controller ---'
        print '1) Set servo position (0 to 255 for min to max)'
        print '2) Set servo speed (1, 2, 3, ..)'
        print '3) Set servo max angle (0 to 180)'
        print '4) Set servo min angle (0 to 180)'
        print '5) Reverse Servo Position (0 / 1)'
        print '6) Pos1'
        print '7) Pos2'
        print  '11) Read servo positions'
        print '12) Read servo speed'
        print '13) Read servo max'
        print '14) Read servo min'
        print '15) Read servo idle'
        print '16) Read servo reversed'
        print 'Enter \'q\' to quit'
        print 'enter command'
        user_cmd = raw_input('->')
        print user_cmd
        if user_cmd == 'q':
            running = 0
        elif int(user_cmd) == 1:            
            print 'select servo 0 to 6 (pin 9 = 3)'
            user_cmd = raw_input('->')
            servo = int(user_cmd)
            print 'enter position'
            user_cmd = raw_input('->')
            value = int(user_cmd)            
            write_servo(module,servo,value,30)            
        elif int(user_cmd) == 2:            
            print 'select servo 0 to 6 (pin 9 = 3)'
            user_cmd = raw_input('->')
            servo = int(user_cmd)
            print 'enter speed'
            user_cmd = raw_input('->')
            value = int(user_cmd)            
            write_servo(module,servo,value,40)
        elif int(user_cmd) == 3:            
            print 'select servo 0 to 6 (pin 9 = 3)'
            user_cmd = raw_input('->')
            servo = int(user_cmd)
            print 'enter Max position'
            user_cmd = raw_input('->')
            value = int(user_cmd)            
            write_servo(module,servo,value,50)
        elif int(user_cmd) == 4:            
            print 'select servo 0 to 6 (pin 9 = 3)'
            user_cmd = raw_input('->')
            servo = int(user_cmd)
            print 'enter Min position'
            user_cmd = raw_input('->')
            value = int(user_cmd)            
            write_servo(module,servo,value,60)
        elif int(user_cmd) == 5:            
            print 'select servo 0 to 6 (pin 9 = 3)'
            user_cmd = raw_input('->')
            servo = int(user_cmd)
            print 'enter 1 or 0'
            user_cmd = raw_input('->')
            value = int(user_cmd)            
            write_servo(module,servo,value,80)
        elif int(user_cmd) == 6:
            pos1()
        elif int(user_cmd) == 7:
            pos2()
        elif int(user_cmd) == 8:
            pos3()
        elif int(user_cmd) >= 11:
            position = read_all(ty[int(user_cmd)])
            print position
        else:
            print 'Nothing Here'
            
                
        
        #print ser
    print 'Done'

    
import time
import serial
import numpy
import math
import numarray as na
import struct
import ctypes

ser_port0 = '/dev/ttyUSB0'
ser_port1 = '/dev/ttyUSB1'
ser_port2 = '/dev/ttyUSB2'

module = 2;


#main()

point1 = [0,0,300,0,0,0.0]
point2 = [0,0,300,0,0,1.0]
print kinematic(point1)
print ''
print kinematic(point2)
##while(1):
##    user_cmd = raw_input('->')
##    set_pos(kinematic(point1))
##    user_cmd = raw_input('->')
##    set_pos(kinematic(point2))



