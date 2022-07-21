# Untitled - By: lenovo - 周六 7月 16 2022

import sensor, image, time
#import lcd
import math
import cmath

from uart_communication import UartCommunication



rol_angle=0
pit_angle=0


udata=UartCommunication

#lcd.init()
#lcd.rotation(2)#成倒立像，需旋转
#lcd.clear()


view_angle = 57.6#摄像头视野角度
view_angle_alpha = (view_angle*math.pi)/180#转换为弧度制

X_cor_factor=1.1 #X偏移量修正系数
Y_cor_factor=1.1 #Y偏移量修正系数
roi = [0, 0, 0, 0]

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time = 2000)
#设置摄像头参数

clock = time.clock()
set_range = [80, 80]#裁剪区域
while True:

    clock.tick()
    img = sensor.snapshot()

    udata.send(instruction='give_me_attitude_angle')
    temp=udata.receive()
    if temp:
        print(temp)
        rol_angle = temp[0] #读姿态角
        pit_angle = temp[1]



    roi[0] = int( ( sensor.width() -  set_range[0] ) / 2 - ( math.sqrt( sensor.width()**2 + sensor.height()**2 ) * math.tan( rol_angle ) ) / ( 2*math.tan( view_angle_alpha / 2 ) ) * X_cor_factor )
    roi[1] = int( ( sensor.height() - set_range[1] ) / 2 -  ( math.sqrt( sensor.width()**2 + sensor.height()**2 ) * math.tan( pit_angle ) ) / ( 2*math.tan( view_angle_alpha / 2 ) ) * Y_cor_factor )
    roi[2] = set_range[0]
    roi[3] = set_range[1]


    img.draw_rectangle(roi[0], roi[1], roi[2], roi[3], color = (255, 0, 0), thickness = 1, fill = False)
    #lcd.display(img)




    #lcd.draw_string(20,50,"x:"+str(accel_array[0]))
    #lcd.draw_string(20,70,"y:"+str(accel_array[1]))
    #lcd.draw_string(20,90,"z:"+str(accel_array[2]))
    print(clock.fps())
