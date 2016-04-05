import serial
import time #for delays

baudRate = 9600
ser = serial.Serial('/dev/ttyUSB0', baudRate, timeout = 2)  # open serial port
ser.write_timeout = 0.1;
print('Opened '+ser.name)         # check which port was really used

def sendRequest(addr, data):#data with datalen byte in front
    ser.write(b'\x96');
    time.sleep(1.0 / 1000.0);
    ser.write(addr);
    time.sleep(1.0 / 1000.0);
    ser.write(data);
    time.sleep(5.0 / 1000.0);
    ser.write(b'\xA9');
    time.sleep(1.0 / 1000.0);

if __name__ == '__main__':
    
    while(1):
        #Test protocol request for temp sensor
        print('Select action:\n1. Get Temperature\n2. Switch Relay 1\n');
        cmd = input();
        if(cmd == '1'):
            sendRequest(b'0\x14',b'\x01\x00');
            #START BYTE
        elif(cmd == '2'):
            sendRequest(b'0\xFF',b'\x01\x01');

        #Read response
        # time.sleep(1000.0 / 1000.0);
        temp = ser.read(16)
        if(temp != b''):
            print(temp)
        # print(temp.decode("ascii"))
    ser.close()             # close port
