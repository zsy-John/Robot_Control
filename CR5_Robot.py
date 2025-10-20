# coding=utf8
'''
文件名：CR5_Robot.py
前言：我的越疆机械臂CR5的控制器版本是3.5.3，因此有一些需要3.5.5以上控制器版本的指令这里测试过，无法使用，故删去了，使用者开发时自行甄别。
开发者：慕晚风

***********************
*********声明**********
***********************

本代码仅供学术交流之用，请勿挪作商用。
'''

import time
import copy
import socket
import struct
import numpy as np
import math
import logging
from GripperControl import *

# ·全局变量设置
# 1.端口号设置
'''
29999端口：用来发布Dashboard指令
30003端口：用来发布运动控制指令
30004端口：以8ms的速率接收实时状态信息（1440字节）
'''
dashboard_port = 29999
move_control_port = 30003
receive_state_port = 30004
# 2.初始化抓取位姿设置
# 下面这两个位姿近似等价
home_joint_config = [90, 0, 0, 0, -90, 180]
home_tool_config = [141.464238, -108.644036, 1046.060387, 90.850948, 0.003464, -0.102484]
test_tool_config = [-350,-500,400,90,90,0]

# 配置logging输出的最低等级
logging.basicConfig(level=logging.DEBUG)


class CR5_Robot_Receive:
    '''
    描述：该类用来接收和解析从机械臂实时传来的1440个字节数据，并负责提供指定字节段的信息。
    使用端口：由于该类机械臂实时反馈信息的接收者，因此它只涉及机械臂实时反馈信息端口：
            即30004端口，该端口以8ms的速率实时反馈机械臂信息。
    参数说明：1.tcp_host_ip为机械臂的IP地址；
            2.tcp_port为所当下所使用的端口号。
    '''
    def __init__(self, tcp_host_ip="192.168.5.1", tcp_port=30004):
        '''
        29999端口：用来发布Dashboard指令
        30003端口：用来发布运动控制指令
        30004端口：以8ms的速率接收实时状态信息（1440字节）
        '''
        self.tcp_host_ip = tcp_host_ip
        self.tcp_port = tcp_port

    def get_state(self):
        '''
        描述：该函数用来获取从机械臂实时传回的状态信息（1440字节）
        参数说明：-
        注意：返回值是一个字节形式的数据
        '''
        self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tcp_socket.connect((self.tcp_host_ip, self.tcp_port))
        state_data = self.tcp_socket.recv(1500)
        self.tcp_socket.close()
        return state_data

    # 解析常用字节段信息
    def parse_tcp_state_data(self, data, subpackage):
        '''
        描述：该函数是解析常用字节段信息的关键函数
        参数说明：-
        注意：-
        '''
        # 检查数据长度
        if len(data) < 1440:
            print(f"错误: 数据包长度不足! 期望1440字节, 实际{len(data)}字节！！！")
            return np.array([np.nan, np.nan, np.nan, np.nan, np.nan, np.nan])

        # V3 实时反馈信息
        dic = {'MessageSize': 'H', 'N/A_1': '3H', 'DigitalInputs': 'Q', 'DigitalOutputs': 'Q',
               'RobotMode': 'Q','TimeStamp': 'Q', 'N/A_2': 'Q', 'TestValue': 'Q',
               'N/A_3': 'd', 'SpeedScaling': 'd', 'N/A_4': 'd', 'VMain': 'd',
               'VRobot': 'd', 'IRobot': 'd', 'N/A_5': 'd', 'N/A_6': 'd',
               'N/A_7': '3d', 'N/A_8': '3d', 'N/A_9': '3d', 'QTarget': '6d',
               'QDTarget': '6d', 'QDDTarget': '6d', 'ITarget': '6d', 'MTarget': '6d',
               'QActual': '6d', 'QDActual': '6d', 'IActual': '6d', 'ActualTCPForce': '6d',
               'ToolVectorActual': '6d', 'TCPSpeedActual': '6d', 'TCPForce': '6d', 'ToolVectorTarget': '6d',
               'TCPSpeedTarget': '6d', 'MotorTemperatures': '6d', 'JointModes': '6d', 'VActual': '6d',
               'HandType': '4c', 'User': 'c', 'Tool': 'c', 'RunQueuedCmd': 'c',
               'PauseCmdFlag': 'c', 'VelocityRatio': 'c', 'AccelerationRatio': 'c', 'JerkRatio': 'c',
               'XYZVelocityRatio': 'c', 'RVelocityRatio': 'c', 'XYZAccelerationRatio': 'c', 'RAccelerationRatio': 'c',
               'XYZJerkRatio': 'c', 'RJerkRatio': 'c', 'BrakeStatus': 'c', 'EnableStatus': 'c',
               'DragStatus': 'c', 'RunningStatus': 'c', 'ErrorStatus': 'c', 'JogStatusCR': 'c',
               'RobotType': 'c', 'DragButtonSignal': 'c', 'EnableButtonSignal': 'c', 'RecordButtonSignal': 'c',
               'ReappearButtonSignal': 'c', 'JawButtonSignal': 'c', 'SixForceOnline': 'c', 'N/A_10': '82c',
               'MActual[6]': '6d', 'Load': 'd', 'CenterX': 'd', 'CenterY': 'd',
               'CenterZ': 'd', 'User[6]': '6d', 'Tool[6]': '6d', 'TraceIndex': 'd',
               'SixForceValue[6]': '6d', 'TargetQuaternion[4]': '4d', 'ActualQuaternion[4]': '4d', 'N/A_11': '24c'}

        # 解析所有字节段数据
        ii = range(len(dic))        # 获取字典长度
        for key, i in zip(dic, ii):
            fmtsize = struct.calcsize(dic[key])     # 获取当前字典值占用的字节长度
            data1, data = data[0:fmtsize], data[fmtsize:]       # data1为当前字典值，data为除data1外剩下的值
            fmt = "<" + dic[key]        # 设置小端网络字节序（即告诉unpack函数，位于前面的字节是低位）
            dic[key] = dic[key], struct.unpack(fmt, data1)      # 解包成元组，如：dic('MessageSize') = ('H', (256,))

        # 返回指定字节段数据信息
        if subpackage == 'joint_data':  # get joint data
            q_actual_tuple = dic["QActual"]     # 实际关节位置
            joint_data = np.array(q_actual_tuple[1])
            return joint_data
        elif subpackage == 'robot_mode':        # 获取机械臂当前状态信息（详见get_robot_mode()函数）
            robot_mode_tuple = dic['RobotMode']
            robot_mode = np.array(robot_mode_tuple[1])
            return robot_mode
        elif subpackage == 'cartesian_info':        # get x y z rx ry rz
            Tool_vector_actual = dic["ToolVectorActual"]    # TCP笛卡尔实际坐标值
            cartesian_info = np.array(Tool_vector_actual[1])
            return cartesian_info
        elif subpackage == 'enable_state':          # 获取使能状态
            enable_state_tuple = dic["EnableStatus"]
            enable_state = np.array(enable_state_tuple[1])
            return enable_state
        elif subpackage == 'speed_ratio':       # 获取全局速度比例
            speed_ratio_tuple = dic["SpeedScaling"]
            speed_ratio = np.array(speed_ratio_tuple[1])
            return speed_ratio
        elif subpackage == 'brake_state':
            brake_state_tuple = dic["BrakeStatus"]
            brake_byte = brake_state_tuple[1][0]
            # 检查brake_byte是否是字节数据，如果是则转化成十进制数（不转化就是ASCII码）
            if isinstance(brake_byte, bytes):
                brake_int = ord(brake_byte)
            else:
                brake_int = brake_byte
            # 正确的位映射（基于实际测试）
            # 位1 → J6, 位2 → J5, 位3 → J4, 位4 → J3, 位5 → J2, 位6 → J1
            # 位0和位7为保留位
            joint_states = {}
            # 检查每个关节对应的位
            joint_config = [
                (1, 'J6'),  # 位1: 关节6
                (2, 'J5'),  # 位2: 关节5  
                (3, 'J4'),  # 位3: 关节4
                (4, 'J3'),  # 位4: 关节3
                (5, 'J2'),  # 位5: 关节2
                (6, 'J1'),  # 位6: 关节1
            ]
            for bit_position, joint_name in joint_config:
                bit_value = (brake_int >> bit_position) & 1
                joint_states[joint_name] = '松开' if bit_value else '锁死'
            return joint_states            
        else:
            raise Exception('Invalid subpackage(实时反馈信息字典键名不存在)！！！')



class CR5_Robot:
    '''
    描述：1.该类是命令的主要发布者，其函数承载了诸多具体的功能实现，
         它不负责接收和解析从机械臂实时传来的1440个字节数据；
         2.另外，该类中封装了诸多机器人运动学中常用的转化公式作为工具。
    使用端口：由于该类是命令的主要发布者，因此它只涉及用来发布命令的端口。
            其一为29999端口，用来发布Dashboard和设置相关指令；
            其二为30003端口，用来发布运行相关指令。
    参数说明：1.tcp_host_ip为机械臂的IP地址；
            2.tcp_port为所当下所使用的端口号，该类一般只用29999和30003，
              需要时将该成员属性改掉后再创建socket连接即可实现不同端口的切换；
            3.workspace_limits是以笛卡尔坐标表示下的工作空间限制；
            4.cr5_receive用来接收另一个类实例化的对象，
              该对象主要负责接收和解析从机械臂实时传来的1440个字节数据。
    '''
    def __init__(self, tcp_host_ip="192.168.5.1", 
                 tcp_port=29999, workspace_limits=None, 
                 cr5_receive=CR5_Robot_Receive(tcp_port=receive_state_port), 
                 ag95=SetCmd(),
                 ag95_receive=ReadStatus()):
        # ·Init Variables
        if workspace_limits is None:
            workspace_limits = [[-700, 700], [-700, 700], [0.00, 800]]  # TODO:这里注意验证一下单位是否是mm
        self.workspace_limits = workspace_limits
        self.tcp_host_ip = tcp_host_ip
        self.tcp_port = tcp_port
        # 机械臂状态读取对象
        self.cr5_robot_receive = cr5_receive
        # 夹爪控制对象
        self.gripper = ag95
        # 夹爪状态读取对象
        self.gripper_state = ag95_receive

        # ·CR5 robot 误差容忍度参数设置
        # 1.关节角度误差容忍度设置
        self.joint_tolerance = 2        # 单位：度，deg，°
        # 2.机械臂末端执行器在笛卡尔坐标表示下的误差容忍度参数设置
        self.tool_pose_tolerance = [2, 2, 2, 2, 2, 2]   # X，Y，Z单位是mm，Rx,Ry,Rz单位是deg，度。

    # 给机械臂上电
    def power_on(self):
        '''
        原型：PowerOn()
        描述：给机械臂上电
        命令发布端口：29999（dashboard命令）
        参数说明：-
        注意：1.机械臂上电到完成，需要大概10秒钟的时间，然后再进行使能操作。
             2.请勿在机器人开机初始化完成前下发控制信号，否则可能会造成机器人异常动作。
        '''
        # 进入函数提示
        print("power_on()函数开始执行".center(70,'-'))

        # 命令发布端口检查
        if self.tcp_port != dashboard_port:
            print(f"power_on()函数当前的非法执行端口是{self.tcp_port}，现将其进行切换！！！")
            self.tcp_port = dashboard_port
            print(f"当前端口是:{self.tcp_port}!!!")

        # 创建机械臂用来发布指令的socket连接并发布指令
        self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tcp_socket.connect((self.tcp_host_ip, self.tcp_port))
        tcp_command = "PowerOn()\n"
        self.tcp_socket.send(str.encode(tcp_command))

        # 检查是否上电成功
        self.tcp_socket.close()
        while True:
            if self.get_robot_mode() == 1:
                print("机械臂正在初始化，请稍后……")
            elif self.get_robot_mode() == 4:
                break
            elif self.get_robot_mode() == 9:
                print("当前机械臂有警报占用，请先清除机械臂的警报！！！")
                print("power_on()函数结束执行".center(70, '-'))
                return
        time.sleep(15)
        print("机械臂上电(初始化)完成！！！")

        # 结束函数提示
        print("power_on()函数结束执行".center(70, '-'))

    # 给机械臂上使能
    def enable_robot(self):
        '''
        原型：EnableRobot(load,centerX,centerY,centerZ)
        描述：给机械臂上使能。
        命令发布端口：29999（Dashboard命令）。
        参数说明：当机械臂处于使能状态时，enable_state = 1，反之为 0。
        注意：1.执行队列指令（机械臂运动、队列IO等）前必须先使能机械臂；
             2.每次进入TCP模式后，即使机械臂已使能，也请先调用一次该指令，否则可能会出现TCP指令执行异常；
             3.机械臂运动过程中调用该指令会导致机械臂停止当前运动。
        '''
        # 进入函数提示
        print("enable_robot()函数开始执行".center(70, '-'))

        # 端口检查
        if self.tcp_port != dashboard_port:
            print(f"enable_robot函数当前的非法执行端口是{self.tcp_port}，现将其进行切换！！！")
            self.tcp_port = dashboard_port

        # 创建机械臂实时信息订阅的socket连接
        cr5_receive = self.cr5_robot_receive
        cr5_receive.tcp_socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        cr5_receive.tcp_socket.connect((cr5_receive.tcp_host_ip, cr5_receive.tcp_port))

        # 创建机械臂用来发布指令的socket连接并发布指令
        self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tcp_socket.connect((self.tcp_host_ip, self.tcp_port))
        tcp_command = "EnableRobot()\n"
        self.tcp_socket.send(str.encode(tcp_command))

        # 检测机械臂是否使能成功
        state_data = cr5_receive.tcp_socket.recv(1500)      # 指定最多接收1500字节的数据（越疆机械臂是1440个字节）
        enable_state = cr5_receive.parse_tcp_state_data(state_data, 'enable_state')
        while enable_state[0] != b'\x01':
            state_data = cr5_receive.tcp_socket.recv(1500)
            enable_state = cr5_receive.parse_tcp_state_data(state_data, 'enable_state')
            time.sleep(0.01)
        self.tcp_socket.close()
        cr5_receive.tcp_socket.close()
        print("机械臂已使能！！！")
        time.sleep(1.5)

        # 结束函数提示
        print("enable_robot()函数结束执行".center(70, '-'))

    # Disable Robot
    def disable_robot(self):
        '''
        原型：DisableRobot()
        描述：给机械臂下使能。
        命令发布端口：29999（Dashboard命令）。
        参数说明：当机械臂处于使能状态时，enable_state = 1，反之为 0。
        注意：-
        '''
        # 进入函数提示
        print("disable_robot()函数开始执行".center(70, '-'))

        # 端口检查
        if self.tcp_port != dashboard_port:
            print(f"disable_robot()函数当前的非法执行端口是{self.tcp_port}，现将其进行切换！！！")
            self.tcp_port = dashboard_port

        # 创建机械臂实时信息订阅的socket连接
        cr5_receive = self.cr5_robot_receive
        cr5_receive.tcp_socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        cr5_receive.tcp_socket.connect((cr5_receive.tcp_host_ip, cr5_receive.tcp_port))

        # 创建机械臂用来发布指令的socket连接并发布指令
        self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tcp_socket.connect((self.tcp_host_ip, self.tcp_port))
        tcp_command = "DisableRobot()\n"
        self.tcp_socket.send(str.encode(tcp_command))

        # 检查使能状态是否成功退出
        state_data = cr5_receive.tcp_socket.recv(1500)      # 指定最多接收1500字节的数据（越疆机械臂是1440个字节）
        enable_state = cr5_receive.parse_tcp_state_data(state_data, 'enable_state')
        while enable_state[0] != b'':
            state_data = cr5_receive.tcp_socket.recv(1500)
            enable_state = cr5_receive.parse_tcp_state_data(state_data, 'enable_state')
            time.sleep(0.01)
        self.tcp_socket.close()
        cr5_receive.tcp_socket.close()
        print("机械臂已结束使能！！！")

        # 结束函数提示
        print("disable_robot()函数结束执行".center(70, '-'))

    # Get robot mode
    def get_robot_mode(self):
        '''
        原型：RobotMode()
        描述：获取机器人当前状态。
        命令发布端口：29999（Dashboard命令）。
        参数说明：robot_state = [1]        初始化
                robot_state = [3]        本体未上电
                robot_state = [4]        未使能（无抱闸松开）
                robot_state = [5]        使能且空闲（无报警，未运行工程）
                robot_state = [9]        有未清除的报警。此状态优先级最高，无论机械臂处于什么状态，有报警时都返回9
        注意：-
        '''
        # 进入函数提示
        print("get_robot_mode()函数开始执行".center(70, '-'))

        # 端口检查
        if self.tcp_port != dashboard_port:
            print(f"get_robot_mode()函数当前的非法执行端口是{self.tcp_port}，现将其进行切换！！！")
            self.tcp_port = dashboard_port

        # 创建机械臂实时信息订阅的socket连接
        cr5_receive = self.cr5_robot_receive
        cr5_receive.tcp_socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        cr5_receive.tcp_socket.connect((cr5_receive.tcp_host_ip, cr5_receive.tcp_port))

        # 获取当前RobotMode的值
        state_data = cr5_receive.tcp_socket.recv(1500)      # 指定最多接收1500字节的数据（越疆机械臂是1440个字节）
        robot_state = cr5_receive.parse_tcp_state_data(state_data, 'robot_mode')
        print(f"当前机械臂RobotMode的值为：{robot_state}")
        cr5_receive.tcp_socket.close()

        # 结束函数提示
        print("get_robot_mode()函数结束执行".center(70, '-'))
        return robot_state[0]       # 返回一个整型数，而不再是数组
    
    # Clear Error
    def clear_error(self):
        '''
        原型：ClearError()
        描述：清除机器人报警。
        命令发布端口：29999（Dashboard命令）。
        参数说明：-
        注意：1.用户可以根据RobotMode来判断机器人是否还处于报警状态；
             2.部分报警需要解决报警原因或者重启控制柜后才能清除；
             3.清除报警后，需要重新调用EnableRobot指令重新开启运动队列，然后才能再下发运动指令。
        '''
        # 进入函数提示
        print("clear_error()函数开始执行".center(70, '-'))

        # 端口检查
        if self.tcp_port != dashboard_port:
            print(f"clear_error()函数当前的非法执行端口是{self.tcp_port}，现将其进行切换！！！")
            self.tcp_port = dashboard_port

        # 创建机械臂实时信息订阅的socket连接
        cr5_receive = self.cr5_robot_receive
        cr5_receive.tcp_socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        cr5_receive.tcp_socket.connect((cr5_receive.tcp_host_ip, cr5_receive.tcp_port))

        # 打印错误类型
        state_data = cr5_receive.tcp_socket.recv(1500)
        robot_mode = cr5_receive.parse_tcp_state_data(state_data, 'robot_mode')
        print(f"robot_mode:{robot_mode}")

        # 创建机械臂用来发布指令的socket连接并发布指令
        self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tcp_socket.connect((self.tcp_host_ip, self.tcp_port))
        tcp_command = "ClearError()\n"
        self.tcp_socket.send(str.encode(tcp_command))

        # 检查警报是否清除成功
        state_data = cr5_receive.tcp_socket.recv(1500) # 指定最多接收1500字节的数据（越疆机械臂是1440个字节）
        robot_mode = cr5_receive.parse_tcp_state_data(state_data, 'robot_mode')
        while robot_mode[0] == 9:
            state_data = cr5_receive.tcp_socket.recv(1500)
            robot_mode = cr5_receive.parse_tcp_state_data(state_data, 'robot_mode')
            time.sleep(0.01)
        self.tcp_socket.close()
        cr5_receive.tcp_socket.close()
        print("机械臂警报已清除！！！")

        # 结束函数提示
        print("clear_error()函数结束执行".center(70, '-'))

    # 关节抱闸控制
    def brake_control(self,joint_id,joint_switch):
        '''
        原型：BrakeControl(axisID,value)
        描述：1.控制指定关节的抱闸；
             2.机械臂静止时关节会自动抱闸，如果用户需进行关节拖拽操作，可开启抱闸，
               需要在机械臂退出使能状态并且手动扶住关节后，下发开启抱闸的指令。
        命令发布端口：29999（Dashboard命令）。
        参数说明：1.axisID   int   关节轴序号，1表示J1轴，2表示J2轴，以此类推；
                2.value    int   设置抱闸状态。0表示抱闸锁死（关节不可移动），1表示松开抱闸（关节可移动）；
                3.BrakeStatus说明：该字节按位表达各个关节的抱闸状态，对应位为1是表示该关节的抱闸已松开。
                                  从第2位到第7位，分别代表关节6到关节1的状态；
                                  第1位和第8位仅是保留位，没有意义。
        注意：1.手册里关于BrakeStatus的说明是错误的，实际上保留位不是8位二进制数里最高的两位，而是最高位和最低位！！！
             2.仅能在机器人退出使能状态后控制关节抱闸，否则ErrorID会返回-1。
               即使用该命令前一定要先下使能！！！
        '''
        # 进入函数提示
        print("brake_control()函数开始执行".center(70, '-'))

        # 端口检查
        if self.tcp_port != dashboard_port:
            print(f"brake_control()函数当前的非法执行端口是{self.tcp_port}，现将其进行切换！！！")
            self.tcp_port = dashboard_port

        # 创建机械臂实时信息订阅的socket连接
        cr5_receive = self.cr5_robot_receive
        cr5_receive.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        cr5_receive.tcp_socket.connect((cr5_receive.tcp_host_ip, cr5_receive.tcp_port))

        # 创建机械臂用来发布指令的socket连接并发布指令
        self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tcp_socket.connect((self.tcp_host_ip, self.tcp_port))
        tcp_command = "BrakeControl(%d,%d)\n" % (joint_id,joint_switch)
        self.tcp_socket.send(str.encode(tcp_command))

        # 检查关节抱闸状态
        for attempt in range(10):       # 多次循环，保证brake_state的状态更新为修改后的值
            state_data = cr5_receive.tcp_socket.recv(1500)  # 指定最多接收1500字节的数据（越疆机械臂是1440个字节）
            brake_state = cr5_receive.parse_tcp_state_data(state_data, 'brake_state')
        print(f"修改后各关节的状态为：{brake_state}!!!")
        self.tcp_socket.close()
        cr5_receive.tcp_socket.close()
        print("关节抱闸控制已完毕！！！")

        # 结束函数提示
        print("brake_control()函数结束执行".center(70, '-'))

    # 设置全局速度比例
    def set_global_speed_ratio(self,ratio=60):
        '''
        原型：SpeedFactor(ratio)
        描述：设置全局速度比例。
        命令发布端口：29999（Dashboard命令）。
        参数说明：ratio的取值范围为[1,100]的整型数
        注意：1.未调用该指令设置时沿用进入TCP/IP控制模式前控制软件设置的值，退出TCP模式后会继续保持该指令设置的值；
             2.使用该命令修改全局速度后，如果又使用了EnableRobot或RunScript指令，会导致全局速度变回进入TCP/IP控制模式前控制软件设置的值。
        '''
        # 进入函数提示
        print("set_global_speed_ratio()函数开始执行".center(70, '-'))

        # 端口检查
        if self.tcp_port != dashboard_port:
            print(f"set_global_speed_ratio()函数当前的非法执行端口是{self.tcp_port}，现将其进行切换！！！")
            self.tcp_port = dashboard_port

        # 创建机械臂实时信息订阅的socket连接
        cr5_receive = self.cr5_robot_receive
        cr5_receive.tcp_socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        cr5_receive.tcp_socket.connect((cr5_receive.tcp_host_ip, cr5_receive.tcp_port))

        # 打印当前速度比例
        state_data = cr5_receive.tcp_socket.recv(1500)
        speed_scaling = cr5_receive.parse_tcp_state_data(state_data, 'speed_ratio')
        print(f"修改前的全局速度比例是:{speed_scaling[0]}%")

        # 创建机械臂用来发布指令的socket连接并发布指令
        self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tcp_socket.connect((self.tcp_host_ip, self.tcp_port))
        tcp_command = "SpeedFactor(%d)\n" % ratio
        self.tcp_socket.send(str.encode(tcp_command))

        # 检查全局速度比例是否修改成功
        state_data = cr5_receive.tcp_socket.recv(1500) # 指定最多接收1500字节的数据（越疆机械臂是1440个字节）
        speed_scaling = cr5_receive.parse_tcp_state_data(state_data, 'speed_ratio')
        while speed_scaling != ratio:
            self.tcp_socket.send(str.encode(tcp_command))
            state_data = cr5_receive.tcp_socket.recv(1500)
            speed_scaling = cr5_receive.parse_tcp_state_data(state_data, 'speed_ratio')
            time.sleep(0.01)
        print(f"全局速度比例已设置为:{speed_scaling[0]}%")
        self.tcp_socket.close()
        cr5_receive.tcp_socket.close()

        # 结束函数提示
        print("set_global_speed_ratio()函数结束执行".center(70, '-'))

    # 设置关节运动方式的速度比例
    def set_speed_j(self,ratio=50):
        '''
        原型：SpeedJ(R)
        描述：设置关节运动方式的速度比例。
        命令发布端口：29999（Dashboard命令）。
        参数说明：ratio的取值范围为[1,100]的整型数
        注意：1.未调用该指令时默认值为50，退出TCP模式后会继续保持该指令设置的值;
             2.使用RunScript指令会导致该参数被重置为50。
        '''
        # 进入函数提示
        print("set_speed_j()函数开始执行".center(70, '-'))

        # 端口检查
        if self.tcp_port != dashboard_port:
            print(f"set_speed_j()函数当前的非法执行端口是{self.tcp_port}，现将其进行切换！！！")
            self.tcp_port = dashboard_port

        # 创建机械臂用来发布指令的socket连接并发布指令
        self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tcp_socket.connect((self.tcp_host_ip, self.tcp_port))
        tcp_command = "SpeedJ(%d)\n" % ratio
        self.tcp_socket.send(str.encode(tcp_command))
        self.tcp_socket.close()

        # 结束函数提示
        print(f"关节运动方式的速度比例已设置为：{ratio}% !!!")
        print("set_speed_j()函数结束执行".center(70, '-'))

    # 设置直线和弧线运动方式的速度比例
    def set_speed_l(self,ratio=50):
        '''
        原型：SpeedL(R)
        描述：设置直线和弧线运动方式的速度比例。
        命令发布端口：29999（Dashboard命令）。
        参数说明：ratio的取值范围为[1,100]的整型数
        注意：1.未调用该指令时默认值为50，退出TCP模式后会继续保持该指令设置的值;
             2.使用RunScript指令会导致该参数被重置为50。
        '''
        # 进入函数提示
        print("set_speed_l()函数开始执行".center(70, '-'))

        # 端口检查
        if self.tcp_port != dashboard_port:
            print(f"set_speed_l()函数当前的非法执行端口是{self.tcp_port}，现将其进行切换！！！")
            self.tcp_port = dashboard_port

        # 创建机械臂用来发布指令的socket连接并发布指令
        self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tcp_socket.connect((self.tcp_host_ip, self.tcp_port))
        tcp_command = "SpeedL(%d)\n" % ratio
        self.tcp_socket.send(str.encode(tcp_command))
        self.tcp_socket.close()

        # 结束函数提示
        print(f"直线和弧线运动方式的速度比例已设置为：{ratio}% !!!")
        print("set_global_speed_ratio()函数结束执行".center(70, '-'))

    # 设置关节运动方式的加速度比例。
    def set_acc_j(self,ratio=50):
        '''
        原型：AccJ(R)
        描述：设置关节运动方式的加速度比例。
        命令发布端口：29999（Dashboard命令）。
        参数说明：ratio的取值范围为[1,100]的整型数
        注意：1.未调用该指令时默认值为50，退出TCP模式后会继续保持该指令设置的值；
             2.使用RunScript指令会导致该参数被重置为50。
        '''
        # 进入函数提示
        print("set_acc_j()函数开始执行".center(70, '-'))

        # 端口检查
        if self.tcp_port != dashboard_port:
            print(f"set_acc_j(函数)当前的非法执行端口是{self.tcp_port}，现将其进行切换！！！")
            self.tcp_port = dashboard_port

        # 创建机械臂用来发布指令的socket连接并发布指令
        self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tcp_socket.connect((self.tcp_host_ip, self.tcp_port))
        tcp_command = "AccJ(%d)\n" % ratio
        self.tcp_socket.send(str.encode(tcp_command))
        self.tcp_socket.close()

        # 结束函数提示
        print(f"关节运动方式的加速度比例已设置为：{ratio}% !!!")
        print("set_acc_j()函数结束执行".center(70, '-'))

    # 设置直线和弧线运动方式的加速度比例
    def set_acc_l(self,ratio=50):
        '''
        原型：AccL(R)
        描述：设置直线和弧线运动方式的加速度比例。
        命令发布端口：29999（Dashboard命令）。
        参数说明：ratio的取值范围为[1,100]的整型数
        注意：1.未调用该指令时默认值为50，退出TCP模式后会继续保持该指令设置的值；
             2.使用RunScript指令会导致该参数被重置为50。
        '''
        # 进入函数提示
        print("set_acc_l()函数开始执行".center(70, '-'))

        # 端口检查
        if self.tcp_port != dashboard_port:
            print(f"set_acc_l()函数当前的非法执行端口是{self.tcp_port}，现将其进行切换！！！")
            self.tcp_port = dashboard_port

        # 创建机械臂用来发布指令的socket连接并发布指令
        self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tcp_socket.connect((self.tcp_host_ip, self.tcp_port))
        tcp_command = "AccL(%d)\n" % ratio
        self.tcp_socket.send(str.encode(tcp_command))
        self.tcp_socket.close()

        # 结束函数提示
        print(f"直线和弧线运动方式的加速度比例已设置为：{ratio}% !!!")
        print("set_acc_l()函数结束执行".center(70, '-'))

    # 设置平滑过渡比例
    def set_cp(self,ratio=50):       # TODO：功能待验证
        '''
        原型：CP(R)
        描述：设置平滑过渡比例，即机械臂连续运动经过多个点时，经过中间点是以直角方式过渡还是以曲线方式过渡。
        命令发布端口：29999（Dashboard）。
        参数说明：ratio   unsigned int    平滑过渡比例    取值范围：[0, 100]
        注意：1.未调用该指令时默认值为50，退出TCP模式后会继续保持该指令设置的值；
             2.使用RunScript指令会导致该参数被重置为50。
        '''
        # 进入函数提示
        print("set_cp()函数开始执行".center(70, '-'))

        # 端口检查
        if self.tcp_port != dashboard_port:
            print(f"set_cp()函数当前的非法执行端口是{self.tcp_port}，现将其进行切换！！！")
            self.tcp_port = dashboard_port

        # 创建机械臂用来发布指令的socket连接并发布指令
        self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tcp_socket.connect((self.tcp_host_ip, self.tcp_port))
        tcp_command = "CP(%f)\n" % ratio
        self.tcp_socket.send(str.encode(tcp_command))
        self.tcp_socket.close()

        # 结束函数提示
        print(f"平滑过渡比例CP已设置为：{ratio}!!!")
        print("set_cp()函数结束执行".center(70, '-'))

    # 机械臂末端执行器关节运动（笛卡尔坐标形式）
    def move_j(self, tool_configuration):
        '''
        原型：MovJ(X,Y,Z,Rx,Ry,Rz,User=index,Tool=index,SpeedJ=R,AccJ=R)
        描述：从当前位置以关节运动方式运动至笛卡尔坐标目标点。关节运动的轨迹非直线，所有关节会同时完成运动。
        命令发布端口：30003（运动控制端口）。
        参数说明：1.X,Y,Z的单位都是毫米，mm；
                2.Rx,Ry,Rz的单位都是度，deg，°；
                3.X,Y,Z,Rx,Ry,Rz都是双精度浮点数(double)；
                4.tool_configuration = 坐标点+欧拉角 即[X,Y,Z,Rx,Ry,Rz]；
                5.在调用move_j()之前，先调用set_speed_l()函数可以调整运动速度
        注意：欧拉角Rx,Ry,Rz都是参考固定坐标系——机械臂的基坐标系得到的。
        '''
        # 进入函数提示
        print("move_j()函数开始执行".center(70, '-'))

        # 端口检查
        if self.tcp_port != move_control_port:
            print(f"move_j函数当前的非法执行端口是{self.tcp_port}，现将其进行切换！！！")
            self.tcp_port = move_control_port

        # 创建机械臂实时信息订阅的socket连接
        cr5_receive = self.cr5_robot_receive
        cr5_receive.tcp_socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        cr5_receive.tcp_socket.connect((cr5_receive.tcp_host_ip, cr5_receive.tcp_port))

        # 创建机械臂用来发布指令的socket连接并发布指令
        self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tcp_socket.connect((self.tcp_host_ip, self.tcp_port))
        tcp_command = "MovJ(%f" % tool_configuration[0]
        for goal_index in range(1,6):
            tcp_command = tcp_command + (",%f" % tool_configuration[goal_index])
        tcp_command = tcp_command + ")\n"
        state_data = cr5_receive.tcp_socket.recv(1500)
        actual_tool_positions = cr5_receive.parse_tcp_state_data(state_data, 'cartesian_info')
        np.set_printoptions(precision=6, suppress=True, floatmode='fixed')      # 控制输出不要用科学计数法，而是保留小数点后6位 
        print(f"运动前的笛卡尔坐标：{actual_tool_positions}")
        self.tcp_socket.send(str.encode(tcp_command))

        # 重新解析机械臂实时状态信息，判断逆解是否存在
        for i in range(10):     # 循环执行多次，确保读到最新的改变后的机械臂状态
            state_data = cr5_receive.tcp_socket.recv(1500)
            robot_mode = cr5_receive.parse_tcp_state_data(state_data, 'robot_mode')
            if robot_mode[0] == 9:
                logging.error("机械臂运动学逆解求解失败，当前任务规划失败！！！")
                raise ValueError("请检查目标点后重新尝试！！！")

        # 检查是否抵达目标点
        state_data = cr5_receive.tcp_socket.recv(1500)
        actual_tool_positions = cr5_receive.parse_tcp_state_data(state_data, 'cartesian_info')
        while not all([np.abs(actual_tool_positions[j] - tool_configuration[j]) < self.tool_pose_tolerance[j] for j in range(3)]):
            state_data = cr5_receive.tcp_socket.recv(1500)
            actual_tool_positions = cr5_receive.parse_tcp_state_data(state_data, 'cartesian_info')
        time.sleep(1.5)
        self.tcp_socket.close()
        cr5_receive.tcp_socket.close()

        # 结束函数提示
        print(f"机械臂末端已抵达指定坐标点{actual_tool_positions}！！！")
        print("move_j()函数结束执行".center(70, '-'))

    # 机械臂末端执行器关节运动（关节角度形式）
    def joint_move_j(self, joint_configuration):
        '''
        原型：JointMovJ(J1,J2,J3,J4,J5,J6,SpeedJ=R,AccJ=R)
        描述：从当前位置以关节运动方式运动至关节坐标目标点。
        命令发布端口：30003（运动控制端口）。
        参数说明：1.J1~J6的单位都是度，deg，°；
                2.J1~J6都是双精度浮点数(double)；
                3.joint_configuration = [J1,J2,J3,J4,J5,J6]；
                4.在调用joint_move_j()之前，先调用set_speed_j()函数可以调整关节运动速度。
        注意：-
        '''
        # 进入函数提示
        print("joint_move_j()函数开始执行".center(70, '-'))

        # 端口检查
        if self.tcp_port != move_control_port:
            print(f"joint_move_j()函数当前的非法执行端口是{self.tcp_port}，现将其进行切换！！！")
            self.tcp_port = move_control_port

        # 创建机械臂实时信息订阅的socket连接
        cr5_receive = self.cr5_robot_receive
        cr5_receive.tcp_socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        cr5_receive.tcp_socket.connect((cr5_receive.tcp_host_ip, cr5_receive.tcp_port))

        # 创建机械臂用来发布指令的socket连接并发布指令
        self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tcp_socket.connect((self.tcp_host_ip, self.tcp_port))
        tcp_command = "JointMovJ(%f" % joint_configuration[0]
        for joint_index in range(1,6):
            tcp_command = tcp_command + (",%f" % joint_configuration[joint_index])
        tcp_command += ")\n"
        state_data = cr5_receive.tcp_socket.recv(1500)
        actual_joint_angle = cr5_receive.parse_tcp_state_data(state_data, 'joint_data')
        np.set_printoptions(precision=6, suppress=True, floatmode='fixed')      # 控制输出不要用科学计数法，而是保留小数点后6位 
        print(f"运动前的关节坐标:{actual_joint_angle}")
        self.tcp_socket.send(str.encode(tcp_command))

        # 重新解析机械臂实时状态信息，判断逆解是否存在
        for i in range(10):     # 循环执行多次，确保读到最新的改变后的机械臂状态
            state_data = cr5_receive.tcp_socket.recv(1500)
            robot_mode = cr5_receive.parse_tcp_state_data(state_data, 'robot_mode')
            if robot_mode[0] == 9:
                logging.error("机械臂运动学逆解求解失败，当前任务规划失败！！！")
                raise ValueError("请检查目标点后重新尝试！！！")

        # 检查是否抵达目标点
        state_data = cr5_receive.tcp_socket.recv(1500)
        actual_joint_angle = cr5_receive.parse_tcp_state_data(state_data, 'joint_data')
        while not all([np.abs(actual_joint_angle[j] - joint_configuration[j]) < self.joint_tolerance for j in range(6)]):
            state_data = cr5_receive.tcp_socket.recv(1500)
            actual_joint_angle = cr5_receive.parse_tcp_state_data(state_data, 'joint_data')
        time.sleep(1.5)
        self.tcp_socket.close()
        cr5_receive.tcp_socket.close()

        # 结束函数提示
        np.set_printoptions(precision=6, suppress=True, floatmode='fixed')      # 控制输出不要用科学计数法，而是保留小数点后6位 
        print(f"机械臂末端已抵达指定坐标点{actual_joint_angle}！！！")
        print("joint_move_j()函数结束执行".center(70,'-'))

    # 机械臂末端执行器直线运动（笛卡尔坐标形式）
    def move_l(self, tool_configuration):
        '''
        原型：MovL(X,Y,Z,Rx,Ry,Rz,User=index,Tool=index,SpeedL=R,AccL=R)
        描述：从当前位置以直线运动方式运动至笛卡尔坐标目标点。
        命令发布端口：30003（运动控制指令）。
        参数说明：1.tool_configuration = 坐标点+欧拉角 即[X,Y,Z,Rx,Ry,Rz]；
                2.X,Y,Z的单位都是毫米，mm；
                3.Rx,Ry,Rz的单位都是度，deg，°；
                4.X,Y,Z,Rx,Ry,Rz都是双精度浮点数(double)；
                5.在调用move_l()之前，先调用set_speed_l()函数可以调整运动速度
        注意：欧拉角Rx,Ry,Rz都是参考固定坐标系——机械臂的基坐标系得到的。
        '''
        # 进入函数提示
        print("move_l()函数开始执行".center(70, '-'))

        # 端口检查
        if self.tcp_port != move_control_port:
            print(f"move_l()函数当前的非法执行端口是{self.tcp_port}，现将其进行切换！！！")
            self.tcp_port = move_control_port

        # 创建机械臂实时信息订阅的socket连接
        cr5_receive = self.cr5_robot_receive
        cr5_receive.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        cr5_receive.tcp_socket.connect((cr5_receive.tcp_host_ip, cr5_receive.tcp_port))

        # 创建机械臂用来发布指令的socket连接并发布指令
        self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tcp_socket.connect((self.tcp_host_ip, self.tcp_port))
        tcp_command = "MovL(%f" % tool_configuration[0]
        for goal_index in range(1, 6):
            tcp_command = tcp_command + (",%f" % tool_configuration[goal_index])
        tcp_command = tcp_command + ")\n"
        state_data = cr5_receive.tcp_socket.recv(1500)
        actual_tool_positions = cr5_receive.parse_tcp_state_data(state_data, 'cartesian_info')
        np.set_printoptions(precision=6, suppress=True, floatmode='fixed')      # 控制输出不要用科学计数法，而是保留小数点后6位 
        print(f"运动前的笛卡尔坐标：{actual_tool_positions}")
        self.tcp_socket.send(str.encode(tcp_command))

        # 重新解析机械臂实时状态信息，判断逆解是否存在
        for i in range(10):     # 循环执行多次，确保读到最新的改变后的机械臂状态
            state_data = cr5_receive.tcp_socket.recv(1500)
            robot_mode = cr5_receive.parse_tcp_state_data(state_data, 'robot_mode')
            if robot_mode[0] == 9:
                logging.error("机械臂运动学逆解求解失败，当前任务规划失败！！！")
                raise ValueError("请检查目标点后重新尝试！！！")

        # 检查是否抵达目标点
        state_data = cr5_receive.tcp_socket.recv(1500)
        actual_tool_positions = cr5_receive.parse_tcp_state_data(state_data, 'cartesian_info')
        while not all([np.abs(actual_tool_positions[j] - tool_configuration[j]) < self.tool_pose_tolerance[j] for j in range(3)]):
            state_data = cr5_receive.tcp_socket.recv(1500)
            actual_tool_positions = cr5_receive.parse_tcp_state_data(state_data, 'cartesian_info')
        time.sleep(1.5)
        self.tcp_socket.close()
        cr5_receive.tcp_socket.close()

        # 结束函数提示
        np.set_printoptions(precision=6, suppress=True, floatmode='fixed')      # 控制输出不要用科学计数法，而是保留小数点后6位 
        print(f"机械臂末端已抵达指定坐标点{actual_tool_positions}！！！")
        print("move_l()函数结束执行".center(70, '-'))

    # 机械臂末端执行器沿圆弧运动至目标点
    def arc(self, intermediate_point_configuration, goal_configuration):
        '''
        原型：Arc(X1,Y1,Z1,Rx1,Ry1,Rz1,X2,Y2,Z2,Rx2,Ry2,Rz2,User=index,Tool=index,SpeedL=R,AccL=R)
        描述：从当前位置以圆弧插补方式运动至目标点。
        命令发布端口：30003（运动控制指令）。
        参数说明：1.P1为圆弧中间点，且P1 = [X1,Y1,Z1,Rx1,Ry1,Rz1]；
                2.P2为目标点，且P2 = [X2,Y2,Z2,Rx2,Ry2,Rz2]；
                2.X,Y,Z的单位都是毫米，mm；
                3.Rx,Ry,Rz的单位都是度，deg，°；
                4.X,Y,Z,Rx,Ry,Rz都是双精度浮点数(double)；
                5.在调用arc()之前，先调用set_speed_l()函数可以调整运动速度
        注意：需要通过当前位置、圆弧中间点、运动目标点三个点确定一个圆弧，
             因此当前位置不能在P1（圆弧中间点）和P2（目标点）确定的直线上。
        '''
        # 进入函数提示
        print("arc()函数开始执行".center(70, '-'))

        # 端口检查
        if self.tcp_port != move_control_port:
            print(f"arc()函数当前的非法执行端口是{self.tcp_port}，现将其进行切换！！！")
            self.tcp_port = move_control_port

        # 创建机械臂实时信息订阅的socket连接
        cr5_receive = self.cr5_robot_receive
        cr5_receive.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        cr5_receive.tcp_socket.connect((cr5_receive.tcp_host_ip, cr5_receive.tcp_port))

        # 创建机械臂用来发布指令的socket连接并发布指令
        self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tcp_socket.connect((self.tcp_host_ip, self.tcp_port))
        tcp_command = "Arc("
        for i in range(6):
            tcp_command = tcp_command + ("%f," % intermediate_point_configuration[i])
        for i in range(5):
            tcp_command = tcp_command + ("%f," % goal_configuration[i])
        tcp_command = tcp_command + ( "%f)\n" % goal_configuration[5] )
        state_data = cr5_receive.tcp_socket.recv(1500)
        actual_tool_positions = cr5_receive.parse_tcp_state_data(state_data, 'cartesian_info')
        np.set_printoptions(precision=6, suppress=True, floatmode='fixed')      # 控制输出不要用科学计数法，而是保留小数点后6位 
        print(f"运动前的笛卡尔坐标：{actual_tool_positions}")
        self.tcp_socket.send(str.encode(tcp_command))

        # 重新解析机械臂实时状态信息，判断逆解是否存在
        for i in range(10):     # 循环执行多次，确保读到最新的改变后的机械臂状态
            state_data = cr5_receive.tcp_socket.recv(1500)
            robot_mode = cr5_receive.parse_tcp_state_data(state_data, 'robot_mode')
            if robot_mode[0] == 9:
                logging.error("机械臂运动学逆解求解失败，当前任务规划失败！！！")
                raise ValueError("请检查目标点后重新尝试！！！")

        # 检查是否抵达目标点
        state_data = cr5_receive.tcp_socket.recv(1500)
        actual_tool_positions = cr5_receive.parse_tcp_state_data(state_data, 'cartesian_info')
        while not all([np.abs(actual_tool_positions[j] - goal_configuration[j]) < self.tool_pose_tolerance[j] for j in range(3)]):
            state_data = cr5_receive.tcp_socket.recv(1500)
            actual_tool_positions = cr5_receive.parse_tcp_state_data(state_data, 'cartesian_info')
        time.sleep(1.5)
        self.tcp_socket.close()
        cr5_receive.tcp_socket.close()

        # 结束函数提示
        np.set_printoptions(precision=6, suppress=True, floatmode='fixed')      # 控制输出不要用科学计数法，而是保留小数点后6位 
        print(f"机械臂末端已抵达指定坐标点{actual_tool_positions}！！！")
        print("arc()函数结束执行".center(70, '-'))

    # 机械臂末端执行器沿圆弧运动至目标点
    def sync(self):        # TODO:功能待验证
        '''
        原型：Sync()
        描述：阻塞程序执行队列指令，待队列最后的指令执行完后才返回。
        命令发布端口：30003（运动控制指令）。
        参数说明：-
        注意：1.TCP/IP远程控制指令分为立即指令和队列指令：
                  立即指令：下发后会被立刻执行，并返回执行结果；
                  队列指令：下发后会立刻返回，但不会被立刻执行，而是进入后台算法队列排队，等待被执行。
             2.Dashboard指令（29999端口下发）大部分为立即指令，部分与运动和IO相关的指令为队列指令；
             3.运动相关指令（30003端口下发）均为队列指令。
             4.如果在队列指令后面调用立即指令，立即指令可能会在队列指令完成之前执行，
             如果希望确保立即指令执行时前序指令都已执行完毕，可在调用立即指令前先调用Sync()指令，
             该指令会阻塞程序执行直到前序的指令全部执行完毕。
        '''
        # 进入函数提示
        print("sync()函数开始执行".center(70, '-'))

        # 端口检查
        if self.tcp_port != move_control_port:
            print(f"sync()函数当前的非法执行端口是{self.tcp_port}，现将其进行切换！！！")
            self.tcp_port = move_control_port

        # 创建机械臂用来发布指令的socket连接并发布指令
        self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tcp_socket.connect((self.tcp_host_ip, self.tcp_port))
        tcp_command = "Sync()\n"
        self.tcp_socket.send(str.encode(tcp_command))
        self.tcp_socket.close()

        # 结束函数提示
        print("sync()函数结束执行".center(70, '-'))

    # 机械臂复位
    def go_home(self):
        '''
        描述：控制机械臂复位
        命令发布端口：-
        参数说明：home_joint_config=[90, 0, 0, 0, -90, 180]是机械臂处于竖直状态的参数
        注意：-
        '''
        self.joint_move_j(home_joint_config)

    # 获取机械臂末端执行器的位置（笛卡尔坐标形式）
    def get_tool_position(self):
        '''
        原型：-
        描述：获取机械臂末端执行器以笛卡尔坐标形式表示的位置
        命令发布端口：-
        参数说明：actual_tool_positions = [X,Y,Z,Rx,Ry,Rz]，其中，
                X，Y，Z的单位为毫米，mm；
                Rx，Ry，Rz的单位为度，deg。
        注意：-
        '''
        # 进入函数提示
        print("get_tool_position()函数开始执行".center(70, '-'))

        # 创建机械臂实时信息订阅的socket连接
        cr5_receive = self.cr5_robot_receive
        cr5_receive.tcp_socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        cr5_receive.tcp_socket.connect((cr5_receive.tcp_host_ip, cr5_receive.tcp_port))

        # 获取当前机械臂末端执行器的笛卡尔坐标并打印
        state_data = cr5_receive.tcp_socket.recv(1500)
        actual_tool_positions = cr5_receive.parse_tcp_state_data(state_data, 'cartesian_info')
        np.set_printoptions(precision=6, suppress=True, floatmode='fixed')      # 设置输出形式是小数点后6位，而不是科学计数法
        print(f"当前机械臂末端的笛卡尔坐标为：{actual_tool_positions}！！！")
        cr5_receive.tcp_socket.close()

        # 结束函数提示
        print("move_j()函数结束执行".center(70, '-'))
        return actual_tool_positions
    
    # 获取机械臂各个关节的位置
    def get_joints_angle(self):
        '''
        原型：-
        描述：获取机械臂各个关节的角度
        命令发布端口：-
        参数说明：actual_joint_angle = [J1,J2,J3,J4,J5,J6]，其中，
                J1～J6的单位为度，deg。
        注意：-
        '''
        # 进入函数提示
        print("get_joints_angle()函数开始执行".center(70, '-'))

        # 创建机械臂实时信息订阅的socket连接
        cr5_receive = self.cr5_robot_receive
        cr5_receive.tcp_socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        cr5_receive.tcp_socket.connect((cr5_receive.tcp_host_ip, cr5_receive.tcp_port))

        # 获取当前机械臂末端执行器的笛卡尔坐标并打印
        state_data = cr5_receive.tcp_socket.recv(1500)
        actual_joint_angle = cr5_receive.parse_tcp_state_data(state_data, 'joint_data')
        np.set_printoptions(precision=6,suppress=True,floatmode="fixed")
        print(f"当前机械臂末端的笛卡尔坐标为：{actual_joint_angle}！！！")
        cr5_receive.tcp_socket.close()

        # 结束函数提示
        print("get_joints_angle()函数结束执行".center(70, '-'))
        return actual_joint_angle

    # 欧拉角转化为旋转矢量（注意，不是单位旋转矢量）
    def rpy2rotating_vector(self, rpy):
        '''
        描述：将欧拉角转化为旋转矢量。
        参数说明：1.rpy = [roll,pitch,yaw];
                2.这里的形参rpy是欧拉角，但是采用的是弧度制。
        注意：1.rpy采用的是弧度制，单位为rad
             2.不是单位旋转矢量。旋转矢量 = 单位旋转矢量 * 旋转角(theta)
        '''
        # 将欧拉角转化为旋转矩阵（3X3）
        R = self.rpy2R(rpy)
        # 将旋转矩阵转化为旋转矢量并返回
        return self.R2rotating_vector(R)

    # 欧拉角转化为旋转矩阵（3X3）
    def rpy2R(self, rpy):
        '''
        描述：将欧拉角转化为旋转矩阵（3X3）。
        参数说明：1.rpy = [roll,pitch,yaw];
                2.这里的形参rpy是欧拉角，但是采用的是弧度制。
        注意：1.rpy采用的是弧度制，单位为rad
             2.不是单位旋转矢量。旋转矢量 = 单位旋转矢量 * 旋转角(theta)
        '''
        # 绕X轴旋转的旋转矩阵
        rot_x = np.array([[1, 0, 0],
                          [0, math.cos(rpy[0]), -math.sin(rpy[0])],
                          [0, math.sin(rpy[0]), math.cos(rpy[0])]])
        # 绕Y轴旋转的旋转矩阵
        rot_y = np.array([[math.cos(rpy[1]), 0, math.sin(rpy[1])],
                          [0, 1, 0],
                          [-math.sin(rpy[1]), 0, math.cos(rpy[1])]])
        # 绕Z轴旋转的旋转矩阵
        rot_z = np.array([[math.cos(rpy[2]), -math.sin(rpy[2]), 0],
                          [math.sin(rpy[2]), math.cos(rpy[2]), 0],
                          [0, 0, 1]])
        # 最终的旋转矩阵一般都是从rot_x开始，按照X->Y->Z的顺序依次左乘旋转矩阵
        R = np.dot(rot_z, np.dot(rot_y, rot_x))
        # 返回最终的旋转矩阵（3X3）
        return R

    # 旋转矩阵转化为旋转矢量
    def R2rotating_vector(self, R):
        '''
        描述：旋转矩阵转化为旋转矢量
        参数说明：
                1.形参R是旋转矩阵（3X3）；
                2.theta的计算：arccos[(tr(R)-1) / 2]
                3.单位旋转矢量的计算：
                    rx = 1/2 * sin(theta) * (r32 - r23)
                    ry = 1/2 * sin(theta) * (r13 - r31)
                    rz = 1/2 * sin(theta) * (r21 - r12)
                4.旋转矢量 = 单位旋转矢量 * 旋转角.
        注意：-
        '''
        # 计算旋转角
        theta = math.acos((R[0, 0] + R[1, 1] + R[2, 2] - 1) / 2)
        print(f"theta:{theta}")
        # 计算单位旋转矢量
        rx = (R[2, 1] - R[1, 2]) / (2 * math.sin(theta))
        ry = (R[0, 2] - R[2, 0]) / (2 * math.sin(theta))
        rz = (R[1, 0] - R[0, 1]) / (2 * math.sin(theta))
        # 返回旋转矢量
        return np.array([rx, ry, rz]) * theta

    # 旋转矩阵转化为欧拉角
    def R2rpy(self, R):
        '''
        描述：旋转矩阵转化为欧拉角。
        参数说明：形参R是旋转矩阵（3X3）。
        注意：返回值[x,y,z]里每一个表项的单位都是弧度制，单位是rad
        '''
        # assert (isRotationMatrix(R))
        sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
        # 判断是否存在万向节自锁（这个概念很简单，可自行查阅，这里我推荐：https://krasjet.github.io/quaternion/bonus_gimbal_lock.pdf）
        singular = sy < 1e-6
        if not singular:
            x = math.atan2(R[2, 1], R[2, 2])
            y = math.atan2(-R[2, 0], sy)
            z = math.atan2(R[1, 0], R[0, 0])
        else:
            x = math.atan2(-R[1, 2], R[1, 1])
            y = math.atan2(-R[2, 0], sy)
            z = 0
        return np.array([x, y, z])



if __name__ == "__main__":
    # 1.创建指令发布对象（cr5_robot）和实时状态接收对象（cr5_robot_receive）：
    '''
    29999端口：用来发布Dashboard指令
    30003端口：用来发布运动控制指令
    30004端口：以8ms的速率接收实时状态信息（1440字节）
    '''
          # 用来接收机械臂实时状态信息
    cr5_robot = CR5_Robot(tcp_port=dashboard_port)      # 用来发布命令

    # cr5_robot.clear_error()
    # cr5_robot.power_on()
    # cr5_robot.enable_robot()
    
    # cr5_robot.set_global_speed_ratio(10)
    # cr5_robot.set_speed_l(70)
    # cr5_robot.set_speed_j(50)

    # cr5_robot.move_j([-350,-500,400,90,90,0])
    # cr5_robot.joint_move_j(home_joint_config)
    # cr5_robot.move_l([-500,100,200,150,0,0])
    # cr5_robot.disable_robot()
    # cr5_robot.get_robot_mode()
    # cr5_robot.testRobot()

    # cr5_robot.brake_control(6,0)

    # cr5_robot.get_tool_position()

    # cr5_robot.enable_robot()
    # cr5_robot.set_global_speed_ratio(10)
    # cr5_robot.arc([-350,-200,200,150,0,90],[100,-400,200,150,0,90])
    # cr5_robot.disable_robot()

    cr5_robot.enable_robot()
    cr5_robot.set_speed_j(20)
    cr5_robot.joint_move_j(home_joint_config)
    cr5_robot.gripper.initGripper()
    cr5_robot.gripper_state.readPosition()
    cr5_robot.gripper.setPosition(500)
    cr5_robot.gripper_state.readPosition()
    cr5_robot.gripper.initGripper()
    cr5_robot.disable_robot()


'''
*************************
***********备注**********
*************************
# V4
dic = {'MessageSize': 'H', 'N/A_1': '6x', 'DigitalInputs': 'Q', 'DigitalOutputs': 'Q',
       'RobotMode': 'Q','TimeStamp': 'Q', 'RunTime': 'Q', 'TestValue': 'Q',
       'N/A_2': '8x', 'SpeedScaling': 'd', 'N/A_3': '16x', 'VRobot': 'd',
       'IRobot': 'd', 'ProgramState': 'd', 'SafetyIOIn': '2c', 'SafetyIOOut': '2c',
       'N/A_4': '76x', 'QTarget': '6d', 'QDTarget': '6d', 'QDDTarget': '6d',
       'ITarget': '6d', 'MTarget': '6d', 'QActual': '6d', 'QDActual': '6d',
       'IActual': '6d', 'ActualTCPForce': '6d', 'ToolVectorActual': '6d', 'TCPSpeedActual': '6d',
       'TCPForce': '6d', 'ToolVectorTarget': '6d', 'TCPSpeedTarget': '6d', 'MotorTemperatures': '6d',
       'JointModes': '6d', 'VActual': '6d', 'HandType': '4c', 'User': 'c',
       'Tool': 'c', 'RunQueuedCmd': 'c', 'PauseCmdFlag': 'c', 'VelocityRatio': 'c',
       'AccelerationRatio': 'c', 'N/A_5': 'x', 'XYZVelocityRatio': 'c', 'RVelocityRatio': 'c',
       'XYZAccelerationRatio': 'c', 'RAccelerationRatio': 'c', 'N/A_6': '2x', 'BrakeStatus': 'c',
       'EnableStatus': 'c', 'DragStatus': 'c', 'RunningStatus': 'c', 'ErrorStatus': 'c',
       'JogStatusCR': 'c', 'CRRobotType': 'c', 'DragButtonSignal': 'c', 'EnableButtonSignal': 'c',
       'RecordButtonSignal': 'c', 'ReappearButtonSignal': 'c', 'JawButtonSignal': 'c', 'SixForceOnline': 'c',
       'CollisionState': 'c', 'ArmApproachState': 'c', 'J4ApproachState': 'c', 'J5ApproachState': 'c',
       'J6ApproachState': 'c', 'N/A_7': '61x', 'VibrationDisZ': 'd', 'CurrentCommandId': 'Q',
       'MActual[6]': '6d', 'Load': 'd', 'CenterX': 'd', 'CenterY': 'd',
       'CenterZ': 'd', 'User[6]': '6d', 'Tool[6]': '6d', 'N/A_8': '8x',
       'SixForceValue[6]': '6d', 'TargetQuaternion[4]': '4d', 'ActualQuaternion[4]': '4d', 'AutoManualMode': '2c',
       'ExportStatus': 'H', 'SafetyState': 'c', 'SafeState N/A': 'c', 'N/A_9': '18x'}
'''