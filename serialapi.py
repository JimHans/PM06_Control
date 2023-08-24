#-*-coding:utf-8-*-
# Function: Serial API
#? 串口通信API
#TODO Version 2.0.20230821
#! 依赖项目：pyserial
#! 被引用：main.py
import serial,time
import serial.tools.list_ports
If_Serial_Open = False
serialName = None
ser = None
SerialPortList = None


SerialPortList = list(serial.tools.list_ports.comports())
if len(SerialPortList) <= 0:
    print ("The Serial port can't find!")
else:
    serialName =list(SerialPortList[0])[0]
    ser = serial.Serial(serialName, 115200,timeout=None) # 打开串口，已重定向
    print ("Now Port Using >",ser.name)
    If_Serial_Open = ser.name
recv = str.encode('xxxxxxxxxxx')       # ? UART回传数据全局存储

'''  下位机数据传输封装  '''
def communicate(RB0,RB1,RB2,RB3,RB4,RB5,RB6): # 下位机数据传输函数
    global ser,serialName,SerialPortList
    RB7 = 0xff - (sum([RB0,RB1,RB2,RB3,RB4,RB5,RB6]) & 0xff)
    hexcomm = '{:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x}'.format(RB0, RB1, RB2, RB3, RB4, RB5, RB6, RB7)
    byte_text = bytes.fromhex(hexcomm)
    try:
        ser.write(byte_text)
    except:
        ser.close()
        while True:
            SerialPortList = list(serial.tools.list_ports.comports())
            if len(SerialPortList) >0:break
        serialName =list(SerialPortList[0])[0]
        ser = serial.Serial(serialName, 115200,timeout=None)
        ser.open()
        ser.write(byte_text)
    print(hexcomm)
    time.sleep(0.02)    # 必要的软件延时

''' Thread-0 UART扫描读取封装 '''
def uartRx(): # UART扫描读取封装
    
    global recv, ser,serialName,SerialPortList
    
    recv = str.encode('xxxxxxxxxxx')
    
    print('串口就绪，开始接收')
    while True:

        # 获得接收缓冲区字符
        count = 0

        if count == 0:

            # 读取内容并回显
            try:
                recv = ser.read(8)
            except:
                ser.close()
                while True:
                    SerialPortList = list(serial.tools.list_ports.comports())
                    if len(SerialPortList) >0:break
                serialName =list(SerialPortList[0])[0]
                ser = serial.Serial(serialName, 115200,timeout=None)
                try:ser.open()
                except:pass
                recv = ser.read(8)
            recv = recv.hex()
            print(recv,"<-Serial Recv")
            ser.flushInput()
