#!/usr/bin/python
import serial
import sys #for command line arguments
import time #for delays 
if len(sys.argv) > 1:
    port = str(sys.argv[1]);
else:
    port = '/dev/ttyUSB0'
baudRate = 9600

ser = serial.Serial(port, baudRate, timeout = 2)  # open serial port
ser.write_timeout = 0.1;
print('Opened '+ser.name)         # check which port was really used

def sendRequest(addr, data):#data with datalen byte in front
    #START BYTE
    ser.write(b'\x96');
    time.sleep(1.0 / 1000.0);
    ser.write(addr);
    time.sleep(1.0 / 1000.0);
    # print(data);
    '''serial.iterbytes(data);
    for b in data:
        print(str(b).encode())
        ser.write(str(b).encode());
        time.sleep(1.0 / 1000.0);
        '''
    # ser.write(data);
    # TEMP DATA
    ser.write(b'\x01');
    time.sleep(1.0 / 1000.0);
    ser.write(b'\x00');
    time.sleep(1.0 / 1000.0);
    # time.sleep(5.0 / 1000.0);
    ser.write(b'\xA9');
    time.sleep(1.0 / 1000.0);

def sendServoRequest(angle):
    #START BYTE
    ser.write(b'\x96');
    time.sleep(1.0 / 1000.0);
    ser.write(b'\x16');
    time.sleep(1.0 / 1000.0);
    ser.write(b'\x02');
    time.sleep(1.0 / 1000.0);
    ser.write(b'\x01');
    time.sleep(1.0 / 1000.0);
    ser.write(angle);
    time.sleep(1.0 / 1000.0);
    # time.sleep(5.0 / 1000.0);
    ser.write(b'\xA9');
    time.sleep(1.0 / 1000.0);


if __name__ == '__main__':
    
    while(1):
        #Test protocol request for temp sensor
        print('Select action:\n1. Get Temperature\n2.Get light level in ?\n3. Toggle Relay 1\n4. Toggle Relay 2\n5.Get Time\n6.Turn servo 30deg\n7.Turn servo 120deg\n');
        cmd = input();
        # start = time.time();
        if(cmd == '1'):
            sendRequest(b'\x14',b'\x01\x00');
        elif(cmd == '2'):
            sendRequest(b'\x15',b'\x01\x00');
        elif(cmd == '3'):
            sendRequest(b'\xFF',b'\x02\x00\x00');
        elif(cmd == '4'):
            sendRequest(b'\xFF',b'\x02\x00\x01');
        elif(cmd == '5'):
            sendRequest(b'\xFF',b'\x01\x01');
        elif(cmd == '6'):
            sendServoRequest(b'\x1E');
            # sendRequest(b'\x16',b'\x02\x01\x1E');
        elif(cmd == '7'):
            sendServoRequest(b'\x78');
            # sendRequest(b'\x16',b'\x02\x01\x78');

        #Read response
        # time.sleep(1000.0 / 1000.0);
        temp = ser.read(16)
        # print(temp.decode('ascii'))
        # print(temp)
        # receivedByte = ser.read()
        '''recvBuf = []
        for c in ser.read():
            recvBuf.append(c)
            if c == b'\xA9':
                print("Response: " + recvBuf)
                recvBuf = []
                break
        '''
        '''if (receivedByte == b'\x96'):
            while(receivedByte != b'\xA9'):
                receivedByte = ser.read()
                if(receivedByte != b'\xA9'):
                    # print(hex(receivedByte))
                    print(receivedByte.decode('ascii'))
        '''
        # print(time.time() - start)
        if(temp != b''):
            print(temp)

    ser.close()             # close port
