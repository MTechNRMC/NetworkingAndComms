import termios, sys, os
import serial
import time

from UDP_SOCK import UDP

speedValue = 75
movingForward = 1

#Stops the motors that move robot. Parks the robot
def park():
    ser.write(chr(0x84) + chr(0x00) + chr(123))
    ser.write(chr(0x84) + chr(0x01) + chr(123))

# set up serial port
serialPortString = '/dev/ttyACM1'
ser = serial.Serial(serialPortString, 9600)
ser.open()
ser.write(chr(0xa1))
ser.timeout = 0.1

#connect to arm control arduino
#arms = serial.Serial('/dev/ttyACM2', 115200, timeout=1, writeTimeout=1)
#arms.open()
time.sleep(0.5) # must allow time for arduino to initialize

if(ser.read()):
	print '\nSuccessfully connected to ', serialPortString
else:
	ser.close()
	serialPortString = '/dev/ttyACM0'
	ser = serial.Serial(serialPortString, 9600)
	ser.open()
	print '\nSuccessfully connected to ', serialPortString

ser.timeout = None
#arms.timeout = None

def move():
    ser.write(chr(0xff) + chr(0x00) + chr(speedValue))
    ser.write(chr(0xff) + chr(0x01) + chr(speedValue))

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

# start main program
if __name__ == '__main__':
	print '\nNAVIGATION CONTROLS:'
        print 'w-a-s-d moves the robot, p: pause/park, n: quit'
	print 'i: speed up, k: slow down'
	print 'q curves forward left, e curves forward right'
	print 'z curves backward left, c curves backward right\n'
	print 'MINING CONTROLS:'
	print 'h: mine (push arms down, close shell, pick arms up)'
	print 'y: dump (lift arms up, open shell, bring arms down)\n'

        while 1:
                c = getkey()
		
		# Print if the bump sensors are triggered
		

                if c == 'n':
			# quit
			park()
			ser.close()
			#arms.close()
                        break

		elif c == 'w':
			# go forward
			park()
			movingForward = 1
			speedValue = 50
			move()

		elif c == 'a':
			# go left in place
			park()
                        ser.write(chr(0xff) + chr(0x00) + chr(254))
                        ser.write(chr(0xff) + chr(0x01) + chr(1))

		elif c == 'q': #curve left forward
			park()
			ser.write(chr(0xff) + chr(0x00) + chr(75))
                        ser.write(chr(0xff) + chr(0x01) + chr(0))

		elif c == 'z': #curve left reverse
                        park()
                        ser.write(chr(0xff) + chr(0x00) + chr(179))
                        ser.write(chr(0xff) + chr(0x01) + chr(254))

		elif c == 's':
			# go backwards
			park()
			movingForward = 0
			speedValue = 204
			move()
		elif c == 'd':
			# go right in place
			park()
                        ser.write(chr(0xff) + chr(0x00) + chr(1))
                        ser.write(chr(0xff) + chr(0x01) + chr(254))

		elif c == 'e': #curve right forward
                        park()
                        ser.write(chr(0xff) + chr(0x00) + chr(0))
                        ser.write(chr(0xff) + chr(0x01) + chr(75))

		elif c == 'c': #curve right reverse
                        park()
                        ser.write(chr(0xff) + chr(0x00) + chr(254))
                        ser.write(chr(0xff) + chr(0x01) + chr(179))
		elif c == 'p':
			# stop all motors
			park()

		elif c == 'y':
            # dump into bin
            #send bucket arduino a 'd' for dump
		    park()
		    out = ''
		    time.sleep(0.5)
			while (arms.inWaiting() > 0):
                            out += arms.read(1)
                        #print out
			arms.write('d')

		elif c == 'h':
            # excavate/dig
            #send bucket arduino a 'e' for excavate
            park()
			arms.write('p')
			out = ''
			time.sleep(0.5)
			while (arms.inWaiting() > 0):
			    out += arms.read(1)
			#print out
			arms.write('p')

        elif c == 'i':
			#speed up
			if (movingForward == 1):
			    if (speedValue >= 25):
			        speedValue = speedValue - 25
			else:
			    if (speedValue <= 229):
				speedValue = speedValue + 25
            move()
				
		elif c == 'k':
			#slow down
			if (movingForward == 1):
			    if (speedValue <= 50):
			        speedValue = speedValue + 25
			else:
			    if (speedValue >= 204):
				speedValue = speedValue - 25
			move()

def slowDown():
	#slow down
	if (movingForward == 1):
	    if (speedValue <= 50):
	        speedValue = speedValue + 25
	else:
	    if (speedValue >= 204):
	    	speedValue = speedValue - 25
	move()