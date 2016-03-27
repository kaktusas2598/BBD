import serial
import time #for delays

baudRate = 9600
ser = serial.Serial('/dev/ttyUSB0', baudRate, timeout = 2)  # open serial port
ser.write_timeout = 0.1;
print('Opened '+ser.name)         # check which port was really used

while(1):
    #Test protocol request for temp sensor
    print('Press t/T to trigger protocol request for Slave with address 0x14 to get temperature:\n');
    cmd = input();
    if(cmd == 't' or cmd == 'T'):
        #START BYTE
        ser.write(b'\x96');
        time.sleep(1.0 / 1000.0);
        #ADDRESS BYTE
        # ser.write(b'\xFF');
        ser.write(b'\x14');
        time.sleep(1.0 / 1000.0);
        #TODO: add CMD BYTE??
        #DATALEN BYTE
        ser.write(b'\x01');
        time.sleep(1.0 / 1000.0);
        #DATA
        #TODO: slave specific data
        ser.write(b'\x00');
        time.sleep(1.0 / 1000.0);
        # ser.write(b'\xFF');
        # time.sleep(1.0 / 1000.0);
        # ser.write(b'\xFF');
        # time.sleep(1.0 / 1000.0);
        # ser.write(/b'\xA9');
        time.sleep(1.0 / 1000.0);

        # time.sleep(1000.0 / 1000.0);
    # ser.write(b't')     # write a string

    temp = ser.read(16)
    if(temp != b''):
        print(temp)
    # print(temp.decode("ascii"))


ser.close()             # close portkk
