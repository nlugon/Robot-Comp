








#!/usr/bin/env python
# -*- coding: utf-8 -*-
# lsusb to check device name
#dmesg | grep "tty" to find port name


#!/usr/bin/env python3
# import serial, time
# import random
# if __name__ == '__main__':
#     print('Running. Press CTRL-C to exit.')
#     
#     
#     ser = serial.Serial('/dev/ttyAMA0', 9600, timeout=1)
#     ser.reset_input_buffer()
#     try:
#         while True:
#             cmd = input("Enter Command : ")
#             cmd = int(cmd)
#             ser.write(cmd.encode('utf-8'))
#             time.sleep(0.1)
#             
#             
#             
#             arduino_msg = ser.read()
#             print(arduino_msg)
#             ser.flushInput()
#             
#     except KeyboardInterrupt:
#         print("KeyboardInterrupt has been caught.")





import serial,time
if __name__ == '__main__':
    
    print('Running. Press CTRL-C to exit.')
    with serial.Serial("/dev/ttyACM0", 9600, timeout=1) as arduino:
        time.sleep(0.1) #wait for serial to open
        if arduino.isOpen():
            print("{} connected!".format(arduino.port))
            try:
                while True:
                    cmd=input("Enter command : ")
                    arduino.write(str(cmd).encode())
                    #time.sleep(0.1) #wait for arduino to answer
                    while arduino.inWaiting()==0: pass
                    if  arduino.inWaiting()>0: 
                        answer=arduino.readline()
                        print(answer)
                        arduino.flushInput() #remove data after reading
            except KeyboardInterrupt:
                print("KeyboardInterrupt has been caught.")