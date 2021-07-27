

import sys
import glob
import serial
import serial.tools.list_ports
from array import array
import datetime
import math
from goto import with_goto
import time


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
    #print(writeParameter(53,[18,4]))
    #writeParameter(3,[32,0,4])
    print(readParameterOnce(53,43,1))
    #print(readParameter(55,6,4))

    #while 1:
        #print(readParameter(4,36,2))




################################################################################### open serial
def openSerial_servo_arm():
    ser = serial.Serial('/dev/arm-dynamixel' , 1000000)#57142
    return ser;

################################################################################### open serial
def openSerial_servo():
    ser = serial.Serial('/dev/dynamixel' , 1000000)#57142
    return ser;

################################################################################## get serial
def getSerial(_id): # return an array [status,error,[data]]
    timeOut = 20000
    out = []
    temp =0
    check =0
    length = 0
    error = 0
    params =[]

    try:
        # get id
        while timeOut > 0:
            if ser.inWaiting() > 0:
                temp = ord(ser.read(1))
                #print(temp)
                if temp==255:
                    continue
                if temp==_id:
                    check=temp
                    break
                else:
                    #print("id error "+str(temp))
                    timeOut=0
            timeOut -=1
        # get length
        while timeOut > 0:
            if ser.inWaiting() > 0:
                temp = ord(ser.read(1))
                #print(temp)
                length=temp
                check+=temp
                break
            timeOut -=1
        # get error
        while timeOut > 0:
            if ser.inWaiting() > 0:
                temp = ord(ser.read(1))
                #print(temp)
                error=temp
                check+=temp
                break
            timeOut -=1
        # get params
        while timeOut > 0 and (len(params)+2)!=length:
            if ser.inWaiting() > 0:
                temp = ord(ser.read(1))
                #print(temp)
                params.append(temp)
                check+=temp
            timeOut -=1
        # get checksum
        while timeOut > 0 :
            if ser.inWaiting() > 0:
                temp = ord(ser.read(1))
                #print(temp)
                check = ~check
                while check < 0:
                    check += 256
                while check > 255:
                    check -= 256

                if check==temp:
                    break
                else:
                    timeOut=0
                    #print("check sum error "+ str(check) +" "+ str(temp) )
            timeOut -=1
        ########################


        if timeOut>0:
            out.append(1)
        else:
            out.append(0)
            time.sleep(0.001)
            while ser.inWaiting() > 0:
                temp = ord(ser.read(1))
                time.sleep(0.001)

        out.append(error)
        out.append(params)

        #print "-------------------------"


    except Exception, e:
        print("serial exception "+str(e))
        out.append(0)

    return out;
################################################################################## write param
@with_goto
def writeParameter(_id , _parameter ):
    label .tryagain2
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


    if len(result)!=3 or result[0] != 1 :
        ser.write(dataSend)
        result = getSerial(_id)
        goto .tryagain2

    return result;
################################################################################## read param
@with_goto
def readParameter(_id , _address , _len ):
    label .tryagain
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

    if len(result)!=3 or result[0] != 1 or len(result[2])==0:
        goto .tryagain

    #while len(result)!=3 or result[0] != 1 or result[1] != 0 or len(result[2])==0:
            #print("------------------------------------")
            #print(result)
            #ser.write(dataSend)
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

    if _pos<0:
        _pos = _pos+65535

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

#####################################################################################  acc
#####################################################################################  readPos
def getPos(id):
    data = readParameter(id,36,2)
    var = (data[2][1] * 256 )+ data[2][0]
    if var>32767:
        var = var - 65535
    return var

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
