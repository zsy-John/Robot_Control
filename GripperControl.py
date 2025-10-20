'''
文件名：GripperControl.py
描述：该文件是向夹爪寄存器发布命令的函数，是夹爪的功能实现。
'''
import serial
import time
import crcmod
from Ag_160_95 import Gripper


def isRange(value, min_, max_):
    '''
    该函数是用来检查value的值是否在规定的区间范围内。
    '''
    if not min_ <= value <= max_:
        raise RuntimeError('Out of range')


class SetCmd(object):
    '''
    描述：该类定义了基于modbus向夹爪发布的命令
    '''
    def __init__(self, ControlInstance=Gripper()):
        # 将夹爪的底层逻辑作为成员属性传入进来
        self.gripper = ControlInstance

    # 初始化夹爪
    def initGripper(self,value=165):       # 因为在SendCmd函数中有整型数据到字节序列的转化命令，因此这里value的值用十六进制还是十进制无所谓，只要它们本质上的数值大小是相同的就行。
        '''
        描述：连接控制前需进行初始化夹爪，用于重新标定夹爪和回零位。
        寄存器地址：0x0100
        寄存器数据：
                    1.0xA5（即十进制数165）  重新标定（无论夹爪处于任何位置和状态，夹爪进行一次闭合到张开的动作）
                    2.0x01  回零位（即闭合）
        注意：
            1.夹爪初始化过程中请勿控制；
            2.请在初始化结束后进行控制；
            3.0xA5 指令初始化过程中是在寻找最大和最小位置，如果在此过程中最大或最小位置被阻挡，会识别错误的行程，例如中间放个木块，最后的零位即是木块的宽度；
            4.客户更换指尖后，需要进行 0xA5 初始化并进行保存。
            5.0x01指令是控制夹爪单方向初始化，行程则为上次0xA5初始化后进行保存的行程。
            6.写入 0x01 将根据初始化方向执行单方向初始化，来寻找最大位置或最小位置（即单方向极限位），之后根据保存的总行程值计算位置百分比；
            7.一定注意延时！！！
        '''
        self.gripper.sendCmd(ModbusHighAddress=0x01, ModbusLowAddress=0x00, Value=value)
        time.sleep(5)       # 这里一定要延时一下（5s是我实测的较为稳妥的时间，你可以修改）：
        # 因为self.gripper.sendCmd(ModbusHighAddress=0x01, ModbusLowAddress=0x00, Value=value)语句执行完后，
        # 在程序中会进行到下一条语句，但是夹爪本体仍在执行初始化开合动作，该动作需要时间，如果夹爪还在执行开合动作时后面的语句命令夹爪闭合或者执行其他运动指令，
        # 后面的运动指令会被初始化动作“吞掉”，导致夹爪不执行后面这条动作。


    # 初始化反馈处理
    def initFeedback(self):
        '''
        描述：初始化状态反馈可用于获取是否进行了初始化。
        寄存器地址：0x0200
        寄存器数据：
                    1.值为0——未初始化；
                    2.值为1——初始化成功；
                    3.值为2——初始化中。
        注意：
            1.该命令只能读取不能写入！！！
            2.由于该函数中存在HandInit这个写入命令，因此该函数不是单纯的读取夹爪状态，故将其放在发布命令类中而非读取状态类。
            3.该函数运行后通常只有一种结果：即夹爪初始化完成；
            4.虽然该函数也具有初始化的能力，但是建议使用initGripper函数来初始化夹爪，因为一旦夹爪初始化完成，再运行该命令是不能改变夹爪状态的，
              而initGripper函数却能确保夹爪总是处于打开状态。
        '''
        back = self.gripper.sendCmd(ModbusHighAddress=0x02, ModbusLowAddress=0x00, isSet=False)
        while back == 0:        # 未初始化则初始化后继续读取
            self.gripperInit()
            back = self.gripper.sendCmd(ModbusHighAddress=0x02, ModbusLowAddress=0x00, isSet=False)
        while back == 2:        # 初始化中则等待初始化完成再读取
            time.sleep(0.1)
            back = self.gripper.sendCmd(ModbusHighAddress=0x02, ModbusLowAddress=0x00, isSet=False)
        if back == 1:
            print("AG-160-95夹爪已初始化，欢迎使用。")

    # 力值
    def setForce(self, value=20):      # 为了安全，默认值选择了最小值，抓取的物体越重，这里的力值要设得更高以增加摩擦力。
        '''
        描述：夹爪会在位置移动中，以设定力值去夹持或者撑开目标物体。
        寄存器地址：0x0101
        寄存器数据：20~100(百分比，整型值)
        注意：一旦运行该函数，就会将命令写入寄存器，但没有将其保存进Flash，一旦给夹爪断电重启，夹爪就会使用Flash中的配置。
        '''
        isRange(value, 20, 100)     # 确保一会儿写入寄存器的值在规定的范围内
        self.gripper.sendCmd(ModbusHighAddress=0x01, ModbusLowAddress=0x01, Value=value)

    # 位置
    def setPosition(self, value):
        '''
        描述：控制夹爪运动至指定的位置。
        寄存器地址：0x0103
        寄存器数据：位置数值范围为 0-1000（千分比）
        注意：-
        '''
        isRange(value, 0, 1000)
        self.gripper.sendCmd(ModbusHighAddress=0x01, ModbusLowAddress=0x03, Value=value)
        time.sleep(1)
    
    # 写入夹爪的初始化方向
    def setGripperForward(self, value=0):
        '''
        描述：写入夹爪的初始化方向。
        寄存器地址：0x0301
        寄存器数据：0 or 1
        注意：
            1.  0表示夹爪打开的方向，1表示夹爪闭合的方向；
            2.  默认值设为打开；
            3.  一旦运行该函数，就会将命令写入寄存器，但没有将其保存进Flash，一旦给夹爪断电重启，夹爪就会使用Flash中的配置。
        '''
        isRange(value, 0, 1)
        self.gripper.sendCmd(ModbusHighAddress=0x03, ModbusLowAddress=0x01, Value=value)

    # 写入保存
    def saveConfig(self, value=0):
        '''
        描述：为夹爪写入保存配置参数相关命令。
        寄存器地址：0x0300
        寄存器数据：0 or 1
        注意：
            1.  0默认（就是不将参数写入Flash），1将所有参数写入Flash；
            2.  要保存参数，必须令value=1; 
            3.  经过测试，使用该函数saveConfig(1)可以保存设置的夹爪初始化方向，但是设置的力值会被初始化为0%，同时，重启后夹爪默认力值会回到100%，或许可以通过win的软件修改。
        '''
        isRange(value, 0, 1)
        self.gripper.sendCmd(ModbusHighAddress=0x03, ModbusLowAddress=0x00, Value=value)
        time.sleep(0.5)

# 读取状态
class ReadStatus(object):
    def __init__(self, ControlInstance=Gripper()):
        self.gripper = ControlInstance

    # 读取夹爪当前设定的力值（百分比）
    def readForceSet(self):
        '''
        描述：读取夹爪当前设定的力值（百分比）。
        寄存器地址：0x0101
        寄存器数据：-
        注意：是设定的力值百分比，而不是实时力值百分比！
        '''
        force = self.gripper.sendCmd(ModbusHighAddress=0x01, ModbusLowAddress=0x01, isSet=False)
        print(f"当前夹爪设定的力值为：{force}%。")
        return force
    
    # 读取夹爪设定的的位置（千分比）
    def readPositionSet(self):
        '''
        描述：读取夹爪当前设定的位置（千分比）。
        寄存器地址：0x0103
        寄存器数据：位置数值范围为 0-1000（千分比）
        注意：是设定的位置千分比，而不是实时位置千分比！
        '''
        position_set = self.gripper.sendCmd(ModbusHighAddress=0x01, ModbusLowAddress=0x03, isSet=False)
        print(f"当前夹爪的位置为：{position_set}%。")
        return position_set
    
    # 夹爪夹持状态反馈
    def readGraspState(self):
        '''
        描述：用于读取目前夹爪的状态。
        寄存器地址：0x0201
        寄存器数据：0、1、2、3中的一种。
        注意：-
        '''
        grasp_state = self.gripper.sendCmd(ModbusHighAddress=0x02, ModbusLowAddress=0x01, isSet=False)
        if grasp_state == 0:
            print("夹爪运动中……")
        elif grasp_state == 1:
            print("夹爪停止运动，但未检测夹到物体。")
        elif grasp_state == 2:
            print("夹爪停止运动，且检测到夹到物体。")
        elif grasp_state == 3:
            print("夹爪检测到夹取的物体掉落！！！")
        else:
            print("函数有误，返回值不属于(0,1,2,3)集合！！！")
        return grasp_state
    
    # 读取夹爪的实时位置
    def readPosition(self):
        '''
        描述：读取夹爪当前实时的位置（千分比）。
        寄存器地址：0x0202
        寄存器数据：位置数值范围为 0-1000（千分比）
        注意：-
        '''
        position = self.gripper.sendCmd(ModbusHighAddress=0x02, ModbusLowAddress=0x02, isSet=False)
        print(f"当前夹爪的位置为：{position}‰")
        return position
    
    # 读取夹爪的初始化方向
    def readGripperForward(self):
        '''
        描述：读取夹爪的初始化方向。
        寄存器地址：0x0301
        寄存器数据：0 or 1
        注意：0表示夹爪打开的方向，1表示夹爪闭合的方向；
        '''
        gripper_forward = self.gripper.sendCmd(ModbusHighAddress=0x03, ModbusLowAddress=0x01, isSet=False)
        if gripper_forward == 0:
            print("当前夹爪的初始化方向为：夹爪打开的方向。")
        elif gripper_forward == 1:
            print("当前夹爪的初始化方向为：夹爪闭合的方向。")
        else:
            print("函数功能有误，夹爪初始化方向不为（0,1）集合中的元素！！！")
        return gripper_forward


if __name__ == "__main__":
    command = SetCmd()
    state = ReadStatus()
    command.initGripper()
    command.setPosition(1000)
    state.readPosition()