
from pyb import UART
import pyb
uart = UART(3, 500000)

class UartCommunication:  # UART通信协议
    def __init__(self):
        pass

    def send(**data):  # 发送数据，必须参数 instruction=string (指令字符串)
        send_buf = '0'  # UART数据缓存
        # 数据格式：指令字符串+数据+字符'$'
        # 其中每一位数据以十进制，左补0为3位的字符串形式存放，范围0-255，高位在前
        # 如数据67则发送字符串'067'，数据2561=256*10+1，则发送字符串'010'+'001'='010001'
        # 此方式容易阅读与自定义，但比较冗杂，有机会再优化

        # 指令识别判断与数据打包,自定义区域
        if data['instruction'] == 'target_location':  # 发送位置偏差比例*10000，参数x,y,z(浮点数)
            x_target_location = int(data['x'] * 10000)
            y_target_location = int(data['y'] * 10000)
            z_target_location = int(data['z'] * 10000)
            send_buf = 'OPC' \
                       + "{:0>3}".format(x_target_location // 256) + "{:0>3}".format(x_target_location % 256) \
                       + "{:0>3}".format(y_target_location // 256) + "{:0>3}".format(y_target_location % 256) \
                       + "{:0>3}".format(z_target_location // 256) + "{:0>3}".format(z_target_location % 256) \
                       + '$'

        elif data['instruction'] == 'line':  # 发送巡线角度(0-180)*100与截距(0-1)*10000，参数angle、intercept(浮点数)
            angle = int(data['angle'] * 100)
            intercept = int(data['intercept'] * 10000)

            send_buf = 'OPF' \
                       + "{:0>3}".format(angle // 256) + "{:0>3}".format(angle % 256) \
                       + "{:0>3}".format(intercept // 256) + "{:0>3}".format(intercept % 256) \
                       + '$'

        elif data['instruction'] == 'give_me_attitude_angle':  # 发送请求姿态角命令
            send_buf = 'OPA$'

        elif data['instruction'] == 'detour_pole':  # 发送绕杆距离（mm）水平误差（0-1）*10000，参数distance，毫米，yaw偏移量0-1，rol滚装偏移量0-1
            distance = int(data['distance'])
            yaw = int(data['yaw'] * 10000)
            rol = int(data['rol'] * 10000)
            send_buf = 'OPD' \
                       + "{:0>3}".format(distance // 256) + "{:0>3}".format(distance % 256) \
                       + "{:0>3}".format(yaw // 256) + "{:0>3}".format(yaw % 256) \
                       + "{:0>3}".format(rol // 256) + "{:0>3}".format(rol % 256) \
                       + '$'
        elif data['instruction'] == 'targets':  # 发送两个坐标，1-12
            target1 = int(data['target1'])
            target2 = int(data['target2'])
            
            send_buf = 'OPT' \
                       + "{:0>3}".format(target1 // 256) + "{:0>3}".format(target1 % 256) \
                       + "{:0>3}".format(target2 // 256) + "{:0>3}".format(target2 % 256) \                    
                       + '$'

        send_buf = bytearray(send_buf, 'utf-8')  # 串口发送需转为8位二进制列表

        uart.write(send_buf)    # 串口发送数据

        pyb.delay(2)            #等待单片机响应

    def receive():  # 接收数据
        receive_buf = uart.read()  # 串口数据缓存，有效数据格式：字符'$'+指令/标识字符串+数据字符串

        if receive_buf and receive_buf[0] == 36:
            receive_buf = bytearray(receive_buf, 'utf-8')
            num_bit = -1  # 数字起始位置
            for i in receive_buf:
                if i > 47 and i < 58:
                    break
                else:
                    num_bit = num_bit + 1
            if num_bit != len(receive_buf) - 1 and num_bit > 0:
                for i in range(num_bit + 1, len(receive_buf)):
                    receive_buf[i] = receive_buf[i] - 48

            # 指令识别判断与数据还原，自定义区域
            if receive_buf[1:num_bit + 1] == bytearray('attitude_angle', 'utf-8') and len(receive_buf) > num_bit + 2 * 2 * 3:  # 接收姿态角数据，返回弧度值

                rol_angle = (receive_buf[num_bit +  1] * 100 + receive_buf[num_bit +  2] * 10 + receive_buf[ num_bit + 3]) * 256 \
                           + receive_buf[num_bit +  4] * 100 + receive_buf[num_bit +  5] * 10 + receive_buf[ num_bit + 6]

                pit_angle = (receive_buf[num_bit +  7] * 100 + receive_buf[num_bit +  8] * 10 + receive_buf[ num_bit + 9]) * 256 \
                           + receive_buf[num_bit + 10] * 100 + receive_buf[num_bit + 11] * 10 + receive_buf[ num_bit + 12]

                rol_angle = (rol_angle - 15000) / 10000
                pit_angle = (pit_angle - 15000) / 10000
                return rol_angle, pit_angle

            if receive_buf[1:num_bit + 1] == bytearray('Targets', 'utf-8') and len(receive_buf) > num_bit + 2 * 2 * 3:
                target1 = (receive_buf[num_bit +  1] * 100 + receive_buf[num_bit +  2] * 10 + receive_buf[ num_bit + 3]) * 256 \
                         + receive_buf[num_bit +  4] * 100 + receive_buf[num_bit +  5] * 10 + receive_buf[ num_bit + 6]
                
                target2 = (receive_buf[num_bit +  7] * 100 + receive_buf[num_bit +  8] * 10 + receive_buf[ num_bit + 9]) * 256 \
                         + receive_buf[num_bit + 10] * 100 + receive_buf[num_bit + 11] * 10 + receive_buf[ num_bit + 12] 
                return target1, target2

# 调用方法举例：
# udata=UartCommunication

# udata.send(instruction = 'target_location', x = 0.43553, y = 0.123543, z = 0.0)

# udata.send(instruction = 'give_me_attitude_angle')
# a,b = udata.receive()
# if a!=None and b!=None:
#    print('俯仰角='a,'横滚角='b)