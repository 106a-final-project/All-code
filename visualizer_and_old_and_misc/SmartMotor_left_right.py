#	Basic Python program that communicates with SmartMotor via RS-232 and PySerial Library
#
#
#			*NOTES*			 	
#
#	1) User must select correct com port
#	2) Use a space character after all commands (0x20)
#	3) Will only work with 1 motor, daisy chained systems require ECHO feature
#	4) There are multiple ways to communicate with the motor. Two are shown below.
#		These methods are interchangeable.
#
#		a) encode/decode functions make it easy to convert chars to ASCII
#		b) printing raw hex values of the characters they represent
#
#


import time			#only required for sleep function
import serial	
import os

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch


## Create & Configure Port ##
MotorPort = serial.Serial(
	port='COM5',							### <----- must enter correct COM port
	baudrate=9600,
	parity=serial.PARITY_NONE,
	stopbits=serial.STOPBITS_ONE,
	bytesize=serial.EIGHTBITS,
	timeout=0.05								# timeout is required, no timeout will cause hang up
	)                                           # timeout is in terms of seconds, and will be the polling speed

## Open Port ##
print("Opening port...", end='',flush=True)
if MotorPort.isOpen():
	MotorPort.close()

MotorPort.open()		
print("complete.")

## turn echo feature off for easier reporting, method a ##
MotorPort.write("ECHO_OFF ".encode())
time.sleep(1)
MotorPort.reset_input_buffer()

## Disable limits & Clear Errors, method a ##			#THIS IS REQUIRED FOR MOTION W/O LIMIT SWITCHES, SEE DEV GUIDE
MotorPort.write("EIGN(2) ".encode())					#disable positive limit switch
MotorPort.write("EIGN(2) ".encode())					#disable negative limit switch
MotorPort.write("ZS ".encode())							#clear errors

#==================================================================================================================
#==================================================================================================================
print("starting position move...", end='',flush=True)

MotorPort.reset_input_buffer()
message = [0x52, 0x50, 0x41, 0x20]			#command sent is RPA, R=0x52, P=0x50, A=0x41, ' '=0x20
MotorPort.write(message)
response = int(MotorPort.readline())

home = response
# end = home - 260000
print(f"PRT={home}")

# 210/80000 = 0.002625 mm/pt
# 510/200000 = 0.00255 mm/pt
#500
# a1 = 368644.56 pt/s2
# a2 = 222222.2 pt/s2
# a3 = 72014.2 pt/s2

#50
# a1 = 45065.6 pt/s2
# a2 = 37274.74
# a3 = 22981.9 pt/s2

#10
# a1 = 9229.76
# a2 = 8378.63
# a3 = 6733.43

var_velocity = "VT=1000000000 "
var_acceleration = "ADT=10 "

fast_velocity = "VT=2000000 "
fast_acceleration = "ADT=2000 "



while 1:
    print("Press any key to begin! (or press ESC to quit!)")
    got_key = getch()
    if got_key == chr(0x1b):
        break

    while 1:
        print("Pick direction... Press left (a) or right (d) keys only! (or press ESC to quit!)")
        got_key = getch()
        if got_key == chr(0x1b):
            print(chr(0x1b))
            MotorPort.write("RPA ".encode())				#ask for Relative position
            response = int(MotorPort.readline())
            print("Current Position = ",response)
            MotorPort.write("MP ".encode())				#set position mode
            MotorPort.write(var_acceleration.encode())		#set acceleration, see dev guide for units
            MotorPort.write(var_velocity.encode())		#set velocity, see dev guide for units
            go = home - response
            MotorPort.write(f"PRT={go} ".encode())		#set relative position target, see dev guide for units
            MotorPort.write("G ".encode())				#GO

            ## Wait for motion to stop, method b ##
            message = [0x52, 0x42, 0x74, 0x20]			#command sent is RBt, R=0x52, B=0x42, t=0x74, ' '=0x20
            MotorPort.write(message)
            start = time.time()
            while True:
                response = MotorPort.readline().decode().strip()  # Decode bytes to string and strip whitespace/newline characters
                if response == '0':  # Check if the decoded and stripped string is '0'
                    break  # Exit the loop if the motor is stopped
                MotorPort.write(message)  # If not '0', send the command again to poll the motor status
                time.sleep(0.05)  # It's often good practice to have a small delay in such loops to avoid overwhelming the motor controller
            taken = time.time() - start
            print("TIME TAKEN TO GO HOME", taken)
            break
            
        elif got_key == chr(0x20):
            break
        elif got_key == chr(0x61):
            ## Basic Position Move, method a ##
            MotorPort.write("RPA ".encode())				#ask for Relative position
            response = int(MotorPort.readline())
            print("Current Position = ",response)
            MotorPort.write("MP ".encode())				#set position mode
            MotorPort.write(var_acceleration.encode())		#set acceleration, see dev guide for units
            MotorPort.write(var_velocity.encode())		#set velocity, see dev guide for units
            MotorPort.write("PRT=2000 ".encode())		#set relative position target, see dev guide for units
            MotorPort.write("G ".encode())				#GO
            go = home - response
            print(f"GO={go}")
            print(f"PRT={home}")
            
            ## Wait for motion to stop, method b ##
            message = [0x52, 0x42, 0x74, 0x20]			#command sent is RBt, R=0x52, B=0x42, t=0x74, ' '=0x20
            MotorPort.write(message)
            start = time.time()
            while True:
                response = MotorPort.readline().decode().strip()  # Decode bytes to string and strip whitespace/newline characters
                if response == '0':  # Check if the decoded and stripped string is '0'
                    break  # Exit the loop if the motor is stopped
                MotorPort.write(message)  # If not '0', send the command again to poll the motor status
                time.sleep(0.05)  # It's often good practice to have a small delay in such loops to avoid overwhelming the motor controller
            taken = time.time() - start
            print("TIME TAKEN TO GO HOME", taken)
        elif got_key == chr(0x64):
            ## Basic Position Move, method a ##
            MotorPort.write("RPA ".encode())				#ask for Relative position
            response = int(MotorPort.readline())
            print("Current Position = ",response)
            MotorPort.write("MP ".encode())				#set position mode
            MotorPort.write(var_acceleration.encode())		#set acceleration, see dev guide for units
            MotorPort.write(var_velocity.encode())		#set velocity, see dev guide for units
            MotorPort.write("PRT=-2000 ".encode())		#set relative position target, see dev guide for units
            MotorPort.write("G ".encode())				#GO
            go = home - response
            print(f"GO={go}")
            print(f"PRT={home}")

            
            ## Wait for motion to stop, method b ##
            message = [0x52, 0x42, 0x74, 0x20]			#command sent is RBt, R=0x52, B=0x42, t=0x74, ' '=0x20
            MotorPort.write(message)
            start = time.time()
            while True:
                response = MotorPort.readline().decode().strip()  # Decode bytes to string and strip whitespace/newline characters
                if response == '0':  # Check if the decoded and stripped string is '0'
                    break  # Exit the loop if the motor is stopped
                MotorPort.write(message)  # If not '0', send the command again to poll the motor status
                time.sleep(0.05)  # It's often good practice to have a small delay in such loops to avoid overwhelming the motor controller
            taken = time.time() - start
            print("TIME TAKEN TO GO HOME", taken)
        elif got_key == chr(0x71):
            ## Basic Position Move, method a ##
            MotorPort.write("RPA ".encode())				#ask for Relative position
            response = int(MotorPort.readline())
            print("Current Position = ",response)
            MotorPort.write("MP ".encode())				#set position mode
            MotorPort.write(var_acceleration.encode())		#set acceleration, see dev guide for units
            MotorPort.write(var_velocity.encode())		#set velocity, see dev guide for units
            MotorPort.write("PRT=20000 ".encode())		#set relative position target, see dev guide for units
            MotorPort.write("G ".encode())				#GO
            print(f"PRT={home}")
            ## Wait for motion to stop, method b ##
            message = [0x52, 0x42, 0x74, 0x20]			#command sent is RBt, R=0x52, B=0x42, t=0x74, ' '=0x20
            MotorPort.write(message)
            start = time.time()
            while True:
                response = MotorPort.readline().decode().strip()  # Decode bytes to string and strip whitespace/newline characters
                if response == '0':  # Check if the decoded and stripped string is '0'
                    response = MotorPort.readline().decode().strip()
                    print(response)
                    time.sleep(0.02)
                    response = MotorPort.readline().decode().strip()
                    print(response)
                    time.sleep(0.02)
                    response = MotorPort.readline().decode().strip()
                    print(response)
                    time.sleep(0.02)
                    response = MotorPort.readline().decode().strip()
                    print(response)
                    time.sleep(0.02)
                    response = MotorPort.readline().decode().strip()
                    print(response)
                    time.sleep(0.02)
                    response = MotorPort.readline().decode().strip()
                    print(response)
                    break  # Exit the loop if the motor is stopped
                MotorPort.write(message)  # If not '0', send the command again to poll the motor status
                time.sleep(0.05)  # It's often good practice to have a small delay in such loops to avoid overwhelming the motor controller
            taken = time.time() - start
            print("TIME TAKEN TO GO HOME", taken)
        elif got_key == chr(0x65):
            ## Basic Position Move, method a ##
            MotorPort.write("RPA ".encode())				#ask for Relative position
            response = int(MotorPort.readline())
            print("Current Position = ",response)
            MotorPort.write("MP ".encode())				#set position mode
            MotorPort.write(var_acceleration.encode())		#set acceleration, see dev guide for units
            MotorPort.write(var_velocity.encode())		#set velocity, see dev guide for units
            MotorPort.write("PRT=-20000 ".encode())		#set relative position target, see dev guide for units
            MotorPort.write("G ".encode())				#GO
            print(f"PRT={home}")
            ## Wait for motion to stop, method b ##
            message = [0x52, 0x42, 0x74, 0x20]			#command sent is RBt, R=0x52, B=0x42, t=0x74, ' '=0x20
            MotorPort.write(message)
            start = time.time()
            while True:
                response = MotorPort.readline().decode().strip()  # Decode bytes to string and strip whitespace/newline characters
                if response == '0':  # Check if the decoded and stripped string is '0'
                    break  # Exit the loop if the motor is stopped
                MotorPort.write(message)  # If not '0', send the command again to poll the motor status
                time.sleep(0.05)  # It's often good practice to have a small delay in such loops to avoid overwhelming the motor controller
            taken = time.time() - start
            print("TIME TAKEN TO GO HOME", taken)

        elif got_key == chr(0x7A):
            ## Basic Position Move, method a ##
            print("fast left")
            MotorPort.write("RPA ".encode())				#ask for Relative position
            response = int(MotorPort.readline())
            print("Current Position = ",response)
            MotorPort.write("MP ".encode())				#set position mode
            MotorPort.write(fast_acceleration.encode())		#set acceleration, see dev guide for units
            MotorPort.write(fast_velocity.encode())		#set velocity, see dev guide for units
            MotorPort.write("PRT=40000 ".encode())		#set relative position target, see dev guide for units
            MotorPort.write("G ".encode())				#GO
            print(f"PRT={home}")
            ## Wait for motion to stop, method b ##
            message = [0x52, 0x42, 0x74, 0x20]			#command sent is RBt, R=0x52, B=0x42, t=0x74, ' '=0x20
            MotorPort.write(message)
            while True:
                response = MotorPort.readline().decode().strip()  # Decode bytes to string and strip whitespace/newline characters
                if response == '0':  # Check if the decoded and stripped string is '0'
                    break  # Exit the loop if the motor is stopped
                MotorPort.write(message)  # If not '0', send the command again to poll the motor status
                time.sleep(0.05)  # It's often good practice to have a small delay in such loops to avoid overwhelming the motor controller
        elif got_key == chr(0x63):
            ## Basic Position Move, method a ##
            print("Fast Right")
            MotorPort.write("RPA ".encode())				#ask for Relative position
            response = int(MotorPort.readline())
            print("Current Position = ",response)
            MotorPort.write("MP ".encode())				#set position mode
            MotorPort.write(fast_acceleration.encode())		#set acceleration, see dev guide for units
            MotorPort.write(fast_velocity.encode())		#set velocity, see dev guide for units
            MotorPort.write("PRT=-40000 ".encode())		#set relative position target, see dev guide for units
            MotorPort.write("G ".encode())				#GO
            print(f"PRT={home}")
            ## Wait for motion to stop, method b ##
            message = [0x52, 0x42, 0x74, 0x20]			#command sent is RBt, R=0x52, B=0x42, t=0x74, ' '=0x20
            MotorPort.write(message)
            while True:
                response = MotorPort.readline().decode().strip()  # Decode bytes to string and strip whitespace/newline characters
                if response == '0':  # Check if the decoded and stripped string is '0'
                    break  # Exit the loop if the motor is stopped
                MotorPort.write(message)  # If not '0', send the command again to poll the motor status
                time.sleep(0.05)  # It's often good practice to have a small delay in such loops to avoid overwhelming the motor controller
        else:
            print("inputs can only be a or d")
            
        #if p == 380:
        #    break
        #p -= 1
    break

## Report Current Position, method b ##
MotorPort.reset_input_buffer()
message = [0x52, 0x50, 0x41, 0x20]			#command sent is RPA, R=0x52, P=0x50, A=0x41, ' '=0x20
MotorPort.write(message)
response = int(MotorPort.readline())
print("Current Position = ",response)
        
## Close Port ##
MotorPort.close()							#not doing this can cause errors on next open() call