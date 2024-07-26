'''
created by Pazfic on 2024/07/17
　∧＿∧ =-
（´･ω･`) =-
　と´_,ノヾ =-
　　(´ ヽ、＼ =-
这个程序默认直接使用TTL连接LoRa模块，当在运行时发出LoRa模块初始化完成的提示，将MD0引脚上的杜邦拔出进入工作模式
我使用的库为pynput，用于监听键盘事件
用的是透传的通讯方式
直接在命令行打开，输入选定的串口端口号，等待初始化成功信号，然后拔出MD0，就可以直接点击WASD发送键盘数据了
'''

import serial
import serial.tools.list_ports
from pynput import keyboard
#导入库线程，用于处理停等式ARQ协议发出的重发申请
import time
# import threading as thread

global idx
idx = 0
global message
message = ''
global serial_port
global esc_key
esc_key = False
global Resend_Data
Resend_Data = dict()    #用字典初始化重发数据
global Resend_Flag
Resend_Flag = False    #重发标志位，用于判断是否需要重发
# global Addr_High, Addr_Low, Channel   #自定义的高低位地址和工作信道
# Addr_High = 0x00
# Addr_Low = 0x02
# Channel = 0x05
global key_pressed
key_pressed = dict()


def get_avalable_port():
    ports_list = list(serial.tools.list_ports.comports())
    if len(ports_list) <= 0 :
        print('No available port')
    else:
        print('Available ports:')
        for comport in ports_list:
            print(list(comport)[0], list(comport)[1])
def send_data(data):
    #用选定的串口发送数据，串口发送的封装
    serial_port.write(data.encode('utf-8'))
def on_release(key):
    global esc_key, key_pressed
    if key == keyboard.Key.esc:
        #发出停止监听的信号
        esc_key = True
        return False
    return True
def on_press(key):
    #读取按键，当案件为wasd时打包成字符串发送
    global message, idx, key_pressed
    try:
        #根据lora模块的通讯协议，对字符串进行修改
        if not Resend_Flag:
            if key.char in ['w', 'a', 's', 'd', 'q', 'p']:
                message = key.char
                send_data(message)
                print(message, " pressed")
                Resend_Data_Update(message)
                idx += 1

            if idx > 9 :
                idx = 0
            message = ''    #清空发送的信息
    except AttributeError:
        pass

# def wait4arq_response():
#     global esc_key, Resend_Flag, idx
#     while not esc_key:
#         #等待ARQ重发申请
#         com_input = serial_port.readline().decode('utf-8')
#         print(com_input)
#         if 'x' in com_input:
#             Resend_Flag = True
#             Get_Resend_Data(com_input[1])
        #以下是接收并识别到ARQ重发申请的处理方式
# def serial_recieve_thread() :
#     #启动串口接收线程
#     new_thread = thread.Thread(target=wait4arq_response, name='Serial_Listener')
#     new_thread.start()
def Lora_Init():
    #AT指令集
    cnt = 0
    for operation in ["AT+WLRATE=5,5\r\n", "AT+UART=7,0\r\n", "AT+WLTIME=0\r\n", "AT+TPOWER=3\r\n",
                      "AT+CWMODE=0\r\n", "AT+TMODE=0\r\n", "AT+ADDR=00,02\r\n"]:
        send_data(operation)
        time.sleep(0.1) #延时100毫秒
        com_report = serial_port.readline().decode('utf-8')
        # print(com_report)   #调试用
        #如果AT配置出现了ERROR，返回ERROR信息
        if 'ERROR' in com_report:
            print('Error:', com_report)
            cnt += 1
    if cnt == 0:
        print('Lora模块初始化成功')

# def Resend_Data_Init():
#     global Resend_Data
#     #将对应key值初始化为空
#     Resend_Data["1"] = ''
#     Resend_Data["2"] = ''
#     Resend_Data["3"] = ''
#     Resend_Data["4"] = ''
#     Resend_Data["5"] = ''
#     Resend_Data["6"] = ''
#     Resend_Data["7"] = ''
#     Resend_Data["8"] = ''
#     Resend_Data["9"] = ''
#     print("重发缓冲区初始化完成")

def Resend_Data_Update(message):
    global Resend_Data, idx
    #将收到的重发申请更新到字典中
    Resend_Data[idx.__str__()] = message

def Get_Resend_Data(key):
    global Resend_Data, idx, Resend_Flag
    send_data(Resend_Data[key])
    idx = int(key)
    Resend_Flag = False

if __name__ == '__main__' :
    #给出可用端口
    get_avalable_port()
    #选择端口
    port = input('选择一个端口: ')
    #打开并初始化串口
    #配置串口为：波特率115200，8位数据位，无校验位，1停止位
    serial_port = serial.Serial(port, 9600, serial.EIGHTBITS, serial.PARITY_NONE, serial.STOPBITS_ONE)
    if serial_port.is_open:
         print('串口已被成功打开')
    else:
         #一旦串口打开失败，退出程序
         exit('串口打开失败')

    # Resend_Data_Init()
    #发送lora模块初始化AT指令，在测试串口是否能正确收发的时候需要注释掉
    # Lora_Init()
    #开启串口接收线程
    # serial_recieve_thread()
    #开启键盘事件监听
    try:
        with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
            listener.join()

    except Exception as e:
        print(e)

    serial_port.close()
    #监听结束后关闭串口

