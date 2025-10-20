'''
文件名：Ag_160_95.py
描述：该文件是夹爪的类定义和底层逻辑实现，基于AG系列-操作说明书MODBUS-版V2.6-大寰机器人.pdf文档开发

'''

import serial       # 添加串口通信库，用法类似socket
import time
import crcmod       # 添加CRC校验库


class Gripper(object):
    # 初始化串口
    def __init__(self, port_name="/dev/ttyUSB0"):     # Windows系统使用：port_name="COM3"     Linux系统使用：port_name="/dev/ttyUSB0"
        self.sc = serial.Serial(port=port_name, baudrate=115200)        # 我们使用Linux系统，因此夹爪端口设置为"/dev/ttyUSB0"；波特率设置为115200（参看文档）
        self.crc16 = crcmod.mkCrcFun(0x18005, rev=True, initCrc=0xFFFF, xorOut=0x0000)      # 按照modbus的定义编写，详见https://crcmod.sourceforge.net/crcmod.predefined.html#predefined-crc-algorithms

    # 计算CRC校验码
    def calCrc(self, array):
        # 将字节数据(整数列表)拼接成字符串：[0x01, 0x06, 0x07, 0x00, 0x03, 0xE8] -> bytes_ = b'\x01\x06\x07\x00\x03\xE8'（大端序）
        bytes_ = b''
        for i in range(array.__len__()):
            bytes_ = bytes_ + array[i].to_bytes(1, byteorder='big', signed=True)# byteorder=‘big’指的是大端表示，即高位在前（大端序）
        crc = self.crc16(bytes_).to_bytes(2, byteorder='big', signed=False)     # CRC校验码是一个类似0xFFFF的4位16进制数。（大端序）
        crcH = int.from_bytes(crc[0:1], byteorder='big', signed=False)          # 这里取高位（大端序）
        crcQ = int.from_bytes(crc[1:2], byteorder='big', signed=False)          # 这里取低位（大端序）
        return crcQ, crcH       # 这里注意，返回值是（低位，高位），即低位在前，是小端序！！！

    # 读取串口接收的数据
    # def readSerial(self):
    #     # BTime = time.time()
    #     time.sleep(0.08)
    #     readContent = self.sc.read_all()
    #     return readContent
    def readSerial(self):
        """改进版的读取方法"""
        data = b''
        start_time = time.time()
        
        while time.time() - start_time < 0.05:  # 最多等待50ms
            if self.sc.in_waiting > 0:
                chunk = self.sc.read(self.sc.in_waiting)
                data += chunk
                # 重置超时计时，因为收到了数据
                start_time = time.time()
            else:
                # 如果连续2ms没有新数据，认为帧已完成
                if len(data) > 0 and time.time() - start_time > 0.002:
                    break
            time.sleep(0.001)  # 减少CPU占用
        
        return data

    # 发送指令
    def sendCmd(self, ModbusHighAddress, ModbusLowAddress, Value=0x01, isSet=True, isReadSerial=True):
        '''
        ModbusHighAddress: 寄存器地址的高字节（如0x07）
        ModbusLowAddress: 寄存器地址的低字节（如0x00）
        Value: 要设置或读取的值（默认0x01）
        isSet: True=写命令，False=读命令
        isReadSerial: 是否读取设备响应
        '''
        # 常见功能码有03（读取保持寄存器）、06（写入保持寄存器）
        if isSet:
            SetAddress = 0x06
        else:
            SetAddress = 0x03
        Value = Value if Value >= 0 else Value - 1      # 判断值是否为负数，若是，则补偿补码转换时+1的操作
        bytes_ = Value.to_bytes(2, byteorder='big', signed=True)        # 这里会将传入的Value值转化为字节序列，因此不论转换前Value的值是用十进制还是其它进制数表示的，这步之后的结果都一样。
        ValueHexQ = int.from_bytes(bytes_[0:1], byteorder='big', signed=True)
        ValueHexH = int.from_bytes(bytes_[1:2], byteorder='big', signed=True)
        array = [0x01, SetAddress, ModbusHighAddress, ModbusLowAddress, ValueHexQ, ValueHexH]       # 其中0x01是地址码，代表夹爪的ID号，默认是1.
        currentValueQ, currentValueH = self.calCrc(array)       # 计算命令CRC校验后的值
        setValueCmd = [0x01, SetAddress, ModbusHighAddress, ModbusLowAddress, ValueHexQ, ValueHexH,
                       currentValueQ,
                       currentValueH]       # 命令格式：[地址码(2位16进制数),功能码(2位16进制数，03或者06),寄存器地址（分高低位，高低位每个都是2位16进制数）,寄存器数据（同寄存器地址）,CRC校验码（同寄存器地址）]
        for i in range(setValueCmd.__len__()):
            setValueCmd[i] = setValueCmd[i] if setValueCmd[i] >= 0 else setValueCmd[i] + 256        # 确保所有字节值在0-255范围内。Python的signed=True可能产生负值。
        self.sc.write(setValueCmd)

        if isReadSerial:
            back = self.readSerial()  # 读取串口返回的数据
            value = int.from_bytes(back[3:5], byteorder='big', signed=True)
            if value < 0:
                value = value + 1
            self.sc.flush()  # 清空串口接收缓存
            return value
        else:
            time.sleep(0.005)
            return
