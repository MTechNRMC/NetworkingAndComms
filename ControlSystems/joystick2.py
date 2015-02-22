# Filename: robot.py
# Authors: Frank Sholey, Cade Foster, Josh Lee
# Description: Controls the robot, either manually or autonomously.

# Imports
import termios, sys, os
import serial
import time
import struct
import UDP from UDP_SOCK

# Globals
speedValue = 75
movingForward = 1
auto = True
lastCode = 0x00
lastTimeStamp = 0

########################################################################
# movement functions
########################################################################

# Park
def park():
    ser.write(chr(0xff) + chr(0x00) + chr(123))
    ser.write(chr(0xff) + chr(0x01) + chr(123))
	
# Move Forward
def func_moveForward():
	park()
	movingForward = 1
	speedValue = 50
	move()
	
# Rotate Left
def func_rotateLeft():
	park()
	ser.write(chr(0xff) + chr(0x00) + chr(254))
	ser.write(chr(0xff) + chr(0x01) + chr(1))

# Curve Left Forward
def func_curveLeftForward():
	park()
	ser.write(chr(0xff) + chr(0x00) + chr(75))
	ser.write(chr(0xff) + chr(0x01) + chr(0))

# Curve Left Reverse
def func_curveLeftReverse():
	park()
	ser.write(chr(0xff) + chr(0x00) + chr(179))
	ser.write(chr(0xff) + chr(0x01) + chr(254))

# Reverse
def func_reverse():
	park()
	movingForward = 0
	speedValue = 204
	move()

# Rotate Right
def func_rotateRight():
	park()
	ser.write(chr(0xff) + chr(0x00) + chr(1))
	ser.write(chr(0xff) + chr(0x01) + chr(254))

# Curve Right Forward
def func_curveRightForward():
	park()
	ser.write(chr(0xff) + chr(0x00) + chr(0))
	ser.write(chr(0xff) + chr(0x01) + chr(75))

# Move (this is called by speedUp and slowDown)
def move():
    ser.write(chr(0xff) + chr(0x00) + chr(speedValue))
    ser.write(chr(0xff) + chr(0x01) + chr(speedValue))

# Slow Down
def func_slowDown():
	if (movingForward == 1):
	    if (speedValue <= 50):
	        speedValue = speedValue + 25
	else:
	    if (speedValue >= 204):
	    	speedValue = speedValue - 25
	move()

# Speed Up
def func_speedUp():
	if (movingForward == 1):
	    if (speedValue >= 25):
			speedValue = speedValue - 25
	else:
		if (speedValue <= 229):
			speedValue = speedValue + 25
    move()

# Dig
def func_dig():
    park()
	arms.write('e')
	out = ''
	time.sleep(0.5)
	while (arms.inWaiting() > 0):
	    out += arms.read(1)
		#print out
	arms.write('e')

# Take a dump
def func_dump():
	park()
	arms.write('d')
	out = ''
	time.sleep(0.5)
	while (arms.inWaiting() > 0):
        out += arms.read(1)
        #print out
	arms.write('d')

# Curve Right Reverse
def func_curveRightReverse():
    park()
    ser.write(chr(0xff) + chr(0x00) + chr(254))
    ser.write(chr(0xff) + chr(0x01) + chr(179))

#get a time stamp
def genTimeStamp():
    return int(time.time())

#create a time stamp
def genCheckSum(msg):
    chkSum = 0x00

    for char in msg:
        chkSum ^= ord(char)

    return chr(chkSum)

#encode an int
def encodeInt(num):
    return struct.pack("!i", int(num))

#create a move msg
def createMoveMsg(code):
    msg = chr(code)+encodeInt(genTimeStamp())
    # append the checksum
    msg += genCheckSum(msg)

    return msg

#get the timestamp from a move msg
def getTimeStamp(msg):
    return struct.unpack("!i", msg[1:5])[0]

# verify the msg checksum
def verifyMsg(msg):
    chckSum = msg[len(msg)-1]

    if(genCheckSum(msg[0:len(msg)-1]) == chckSum):
        return True

    return False


##### THIS FUNCTION IS UNUSED, BUT MAY NEED TO BE USED IN AN EMERGENCY!!! #####
# function for getting the key typed
def getkey():
    term = open("/dev/tty", "r")
    fd = term.fileno()
    old = termios.tcgetattr(fd)
    new = termios.tcgetattr(fd)
    new[3] &= ~termios.ICANON & ~termios.ECHO
    termios.tcsetattr(fd, termios.TCSANOW, new)
    c = None
    try:
        c = os.read(fd, 1)
    finally:
        termios.tcsetattr(fd, termios.TCSAFLUSH, old)
        term.close()
    return c
###############################################################################

#NOTE: use ord around msg[0] and create a function that will retreave the code as an int

def intrp(addr, msg):
		if (verifyMsg(msg)):
			for x in range(0, len(msg)):
				if(msg[0] == 0x00 and not auto): #Move forward
                    if(getTimeStamp(msg) > lastTimeStamp or lastCode != msg[0]):
                        lastCode = msg[0]
					    func_moveForward()
					x+=5    #offset by 5 bytes
				elif(msg[0] == 0x01 and not auto): #Move backward
                    if(getTimeStamp(msg) > lastTimeStamp or lastCode != msg[0]):
                        lastCode = msg[0]
					    func_reverse()
					x+=5    #offset by 5 bytes
				elif(msg[0] == 0x02 and not auto): #Rot Left
                    if(getTimeStamp(msg) > lastTimeStamp or lastCode != msg[0]):
                        lastCode = msg[0]
					    func_rotateLeft()
					x+=5    #offset by 5 bytes
				elif(msg[0] == 0x03 and not auto): #Rot Right
                    if(getTimeStamp(msg) > lastTimeStamp or lastCode != msg[0]):
                        lastCode = msg[0]
					    func_rotateRight()
					x+=5    #offset by 5 bytes
				elif(msg[0] == 0x04 and not auto): #Curve Forward Left
                    if(getTimeStamp(msg) > lastTimeStamp or lastCode != msg[0]):
                        lastCode = msg[0]
					    func_curveLeftForward()
					x+=5    #offset by 5 bytes
				elif(msg[0] == 0x05 and not auto): #Curve Backward Left
                    if(getTimeStamp(msg) > lastTimeStamp or lastCode != msg[0]):
                        lastCode = msg[0]
					    func_curveLeftReverse()
					x+=5    #offset by 5 bytes
				elif(msg[0] == 0x06 and not auto): #Curve Forward Right
                    if(getTimeStamp(msg) > lastTimeStamp or lastCode != msg[0]):
                        lastCode = msg[0]
					    func_curveRightForward()
					x+=5    #offset by 5 bytes
				elif(msg[0] == 0x07 and not auto): #Curve Backward Right
                    if(getTimeStamp(msg) > lastTimeStamp or lastCode != msg[0]):
                        lastCode = msg[0]
					    func_curveRightReverse()
					x+=5    #offset by 5 bytes
				elif(msg[0] == 0x08 and not auto): #Increase Speed
                    if(getTimeStamp(msg) > lastTimeStamp or lastCode != msg[0]):
                        lastCode = msg[0]
					    func_speedUp()
					x+=5    #offset by 5 bytes
				elif(msg[0] == 0x09 and not auto): #Decrease Speed
                    if(getTimeStamp(msg) > lastTimeStamp or lastCode != msg[0]):
                        lastCode = msg[0]
					    func_slowDown()
					x+=5    #offset by 5 bytes
				elif(msg[0] == 0x0A and not auto): #Dig
					func_dig()
				elif(msg[0] == 0x0B and not auto): #Take a Dump
					func_dump()
				elif(msg[0] == 0x0C and not auto): #Stop Movement
					park()
				elif(msg[0] == 0xEF): #Resume
					pass
				elif(msg[0] == 0xF0): #Start
					pass
				elif(msg[0] == 0xF1): #Stop
					pass
				elif(msg[0] == 0xF2): #Manual Control
					auto = False
					udp.send(0xF2)

# set up serial port
serialPortString = '/dev/ttyACM0'
ser = serial.Serial(serialPortString, 9600)
ser.open()
ser.write(chr(0xa1))
ser.timeout = 0.1

#connect to arm control arduino
arms = serial.Serial('/dev/ttyACM2', 115200, timeout=1, writeTimeout=1)
arms.open()
time.sleep(0.5) # must allow time for arduino to initialize

if(not ser.read()):
	ser.close()
	serialPortString = '/dev/ttyACM0'
	ser = serial.Serial(serialPortString, 9600)
	ser.open()

ser.timeout = None
arms.timeout = None
	
# start main program
if __name__ == '__main__':
	# print '\nNAVIGATION CONTROLS:'
    # print 'w-a-s-d moves the robot, p: pause/park, n: quit'
	# print 'i: speed up, k: slow down'
	# print 'q curves forward left, e curves forward right'
	# print 'z curves backward left, c curves backward right\n'
	# print 'MINING CONTROLS:'
	# print 'h: mine (push arms down, close shell, pick arms up)'
	# print 'y: dump (lift arms up, open shell, bring arms down)\n'
	
	# Start UDP connection / network communication
	udp = UDP(5000)
	udp.startReceive(intrp)

	# if c == 'n':
		# # quit
		# park()
		# ser.close()
		# arms.close()
		# break