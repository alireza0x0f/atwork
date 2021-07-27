

import sys
import glob
import serial
import serial.tools.list_ports
from array import array
import datetime
import math
from goto import with_goto


################################################################################## test
#@with_goto
def test():
    num = math.sin(-math.pi/2)
    print(num)
    
    #i=0
    #label .jj
    #i+=1
    #print i
    #if i<10:
    #    #goto .jj
    
    #ping(4)
    writeParameter(4,[5,250])
    #writeParameter(3,[32,0,4])
    #print(readParameter(4,32,2))
    #print(readParameter(4,5,1))
    
    #while 1:
        #print(readParameter(4,36,2))
    
    
################################################################################### open serial
def openSerial_servo():
    ser = serial.Serial('/dev/dynamixel' , 57142)#57142
    return ser;

################################################################################## get serial
def getSerial(_id): # return an array [status,error,[data]]
    timeOut = 500000
    out = []
    temp =0
    check =0
    length = 0
    error = 0
    params =[]
    
    try:

        #print datetime.datetime.utcnow()
        #print("in get serial 1 , timeout " + str(timeOut))
        ###########################  get first 255
        '''while timeOut > 0:
            if ser.inWaiting() > 0:
                temp = ord(ser.read(1))
                #print("serial received " + str(temp))
                if(temp == 255):
                    break
                else:
                    print("1st 255")
            timeOut -=1
        ###########################  get second 255
        while timeOut > 0:
            if ser.inWaiting() > 0:
                temp = ord(ser.read(1))
                if(temp == 255):
                    break
                else:
                    print("2nd 255")
            timeOut -=1'''
        ###########################   get id
        #print("in get serial 2 , timeout " + str(timeOut))
        while timeOut > 0:
            if ser.inWaiting() > 0:
                temp = ord(ser.read(1))
                if temp == 255:
                    continue
                if temp == _id:
                    check = temp
                    break
                else:
                    timeOut=0
                    print("wrong id " + str(temp))
            timeOut -=1
        ###########################  get len
        #print("in get serial 3 , timeout " + str(timeOut))
        while timeOut > 0:
            if ser.inWaiting() > 0:
                temp = ord(ser.read(1))
                length = temp
                check += length
                break
            timeOut -=1
        ###########################  get error
        #print("in get serial 4 , timeout " + str(timeOut))
        while timeOut > 0:
            if ser.inWaiting() > 0:
                temp = ord(ser.read(1))
                error = temp
                check += error
                break
            timeOut -=1
        ###########################  get parameters
        #print("in get serial 5 , timeout " + str(timeOut))
        while timeOut > 0:
            if (len(params)+2) == length :
                break
            if ser.inWaiting() > 0:
                temp = ord(ser.read(1))
                params.append(temp)
                check += temp
            timeOut -=1
        ########################## get and calculate checksum
        check = ~ check
        while check < 0:
            check += 256
        while check > 255:
            check -= 256
        
        while timeOut > 0:
            if ser.inWaiting() > 0:
                temp = ord(ser.read(1))
                if temp == check:
                    break
                else:
                    timeOut=0
                    print("chechsum error")
            timeOut -=1
        ########################## calc return array
        
    
        if timeOut>0:
            out.append(1)
        else:
            out.append(0)
    
        out.append(error)
        out.append(params)
        
    except:
        print("serial exception")
        print(out)
    
    #print(timeOut)
    #print(b'done')
    #print datetime.datetime.utcnow()
    #print out

    return out;
################################################################################## write param
def writeParameter(_id , _parameter ):
    check = _id + len(_parameter) + 2 + 3
    buff = [255,255,_id,(len(_parameter)+2),3]
    for p in _parameter:
        buff.append(p)
        check += p
        if (check > 255):
            check -= 256

    check = ~check
    if(check < 0 ):
        check += 256
    buff.append(check)

    #print(b'data sent')
    #print(buff)
    dataSend = bytearray(buff)
    ser.write(dataSend)
    result = getSerial(_id)
    #print("1")
    #print(result)
    
    try:
        while len(result)!=3 or result[0] != 1 or result[1] != 0:
            ser.write(dataSend)
            result = getSerial(_id)
    except:
        print("exception")
    #     print(result)
    #print(result)


    return;
################################################################################## read param
def readParameter(_id , _address , _len ):
    check = _id + 2 + 2 + 2 + _address + _len
    buff = [255,255,_id,2+2,2,_address,_len]

    while check>255:
        check -= 256

    check = ~check
    while(check < 0 ):
        check += 256
    buff.append(check)

    #print(b'data sent')
    #print(buff)
    dataSend = bytearray(buff)
    ser.write(dataSend)
    result = getSerial(_id)

    try:
        while len(result)!=3 or result[0] != 1 or result[1] != 0 or len(result[2])==0:
            print("------------------------------------")
            print(result)
            ser.write(dataSend)
            result = getSerial(_id)
    except:
        print("f error")
    #print(result)
    return result;
################################################################################## read param once
def readParameterOnce(_id , _address , _len ):
    check = _id + 2 + 2 + 2 + _address + _len
    buff = [255,255,_id,2+2,2,_address,_len]

    while check>255:
        check -= 256

    check = ~check
    while(check < 0 ):
        check += 256
    buff.append(check)

    #print(b'data sent')
    #print(buff)
    dataSend = bytearray(buff)
    ser.write(dataSend)
    result = getSerial(_id)

    while result[0] != 1 or result[1] != 0 or len(result[2])==0:
        ser.write(dataSend)
        result = getSerial(_id)

    return result;
################################################################################## ping
def ping(_id):
    check = _id + 2 + 1
    buff = [255,255,_id,2,1]

    while check>255:
        check -= 256

    check = ~check
    while(check < 0 ):
        check += 256
    buff.append(check)

    
    dataSend = bytearray(buff)
    ser.write(dataSend)
    result = getSerial(_id)

    while result[0] != 1 or result[1] != 0:
        ser.write(dataSend)
        result = getSerial(_id)
    return result;
#####################################################################################  set wheel mode
def setWheelMode(_id):
    writeParameter(_id,[6,0,0,0,0])
    return;
#####################################################################################  set speed and start moving
def setSpeed(_id,_dir,_speed):
    if _speed > 1023:
        _speed = 1023
    if _speed <0:
        _speed =0
    if _dir == 1:
        _speed += 1024
    writeParameter(_id,[32,_speed%256,_speed/256])
    return;
#####################################################################################  set goal pos and speed
def setPos(_id,_pos,_speed):
    if _speed > 1023:
        _speed = 1023
    if _speed <0:
        _speed =0
    writeParameter(_id,[30,_pos%256,_pos/256,_speed%256,_speed/256])
    return;
#####################################################################################  stop robot
def robotStop():
    setSpeed(1,0,0)
    setSpeed(2,0,0)
    setSpeed(3,0,0)
    setSpeed(4,0,0)
#####################################################################################  go forward
def robotForward(speed):
    setSpeed(1,0,speed)
    setSpeed(2,1,speed)
    setSpeed(3,0,speed)
    setSpeed(4,1,speed)

#####################################################################################  go backward
def robotBackward(speed):
    setSpeed(1,1,speed)
    setSpeed(2,0,speed)
    setSpeed(3,1,speed)
    setSpeed(4,0,speed)

#####################################################################################  rotate right
def robotRight(speed):
    setSpeed(1,0,speed)
    setSpeed(2,0,speed)
    setSpeed(3,0,speed)
    setSpeed(4,0,speed)

#####################################################################################  rotate lef
def robotLeft(speed):
    setSpeed(1,1,speed)
    setSpeed(2,1,speed)
    setSpeed(3,1,speed)
    setSpeed(4,1,speed)

#####################################################################################  acc
def config():
    writeParameter(1,[73,10])
    writeParameter(2,[73,10])
    writeParameter(3,[73,10])
    writeParameter(4,[73,10])
    
#####################################################################################  readPos
def getPos(id):
    data = readParameter(id,36,2)
    return (data[2][1] * 256 )+ data[2][0]

#####################################################################################  readTrq
def getTrq(id):
    data = readParameter(id,40,2)
    return (data[2][1] * 256 )+ data[2][0]
################################################################################## test
def check_id(id):
    return readParameterOnce(id,3,1)
#####################################################################################  main
#if __name__ == '__main__':


    #ser = openSerial_servo()
    #writeParameter(1,[25,0])
    #setSpeed(1,0,0)
    #setWheelMode(1)
    #readParameter(1,6,4)

    #out ='f'

    #while ser.inWaiting() > 0:
    #    out += ser.read(1)

    #if out != '':
    #    print ">>" + out
