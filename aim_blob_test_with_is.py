# Untitled - By: lenovo - 周六 7月 16 2022

import sensor, image, time, pyb
from pyb import Pin
#import lcd
import math
import cmath
from LOG import LOG
from uart_communication import UartCommunication

red_led = pyb.LED(1)
green_led = pyb.LED(2)
blue_led = pyb.LED(3)   #LED
pin0 = Pin('P0', Pin.OUT_PP, Pin.PULL_NONE)
pin0.value(1)

frame_count=0           #帧数量
FPS = 0                 #记录fps
target_find = 0         #找到目标
target_lost = 0         #丢失目标


thresholds = [(15, 50, 19, 127, -23, 56), (0, 26, -34, 24, -128, 6), (0, 40, -50, 15, -15, 50)]

rol_angle = 0
pit_angle = 0


udata = UartCommunication
log = LOG(True) #写日志前清空日志文件

#lcd.init()
#lcd.rotation(2)#成倒立像，需旋转
#lcd.clear()


view_angle = 70#摄像头视野角度
view_angle_alpha = (view_angle*math.pi)/180#转换为弧度制

X_cor_factor = 1 #X偏移量修正系数
Y_cor_factor = 1 #Y偏移量修正系数
roi = [0, 0, 0, 0]
aim_error = [0,0]


def find_maxblob(blobs):
    max_size=1
    if blobs:
        max_blob = 0
        for blob in blobs:
            blob_size = blob.w()*blob.h()
            #色块尺寸判断
            if ( (blob_size > max_size) & (blob_size > 50)   ) :
                #色块更新
                max_blob=blob
                #最大尺寸更新
                max_size = blob.w()*blob.h()
        return max_blob




sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_framerate(80)
sensor.skip_frames(time = 2000)

#设置摄像头参数

clock = time.clock()
set_range = [200, 150]#裁剪区域

last_rol=0;
last_pit=0;  #暂存上一次数据用于下一帧比对
skip_count=0;           #跳过的帧数量

fps_log = pyb.millis()

while True:

    clock.tick()

    img = sensor.snapshot()
    #img.binary(thresholds)
    #img.dilate(3)
    #img.erode(1)
    #img.dilate(3)

    udata.send(instruction='give_me_attitude_angle')
    temp=udata.receive()
    if temp:
        rol_angle = temp[0] #读姿态角
        pit_angle = temp[1]

    else:
        aim_error = [0.5, 0.5]
        print('No data')
        log.new('Frame',frame_count,'No data')

    roi[0] = int( ( sensor.width() -  set_range[0] ) / 2 + ( math.sqrt( sensor.width()**2 + sensor.height()**2 ) * math.tan( rol_angle ) ) / ( 2*math.tan( view_angle_alpha / 2 ) ) * X_cor_factor )
    roi[1] = int( ( sensor.height() - set_range[1] ) / 2 +  ( math.sqrt( sensor.width()**2 + sensor.height()**2 ) * math.tan( pit_angle ) ) / ( 2*math.tan( view_angle_alpha / 2 ) ) * Y_cor_factor )
    roi[2] = set_range[0]
    roi[3] = set_range[1]



    if roi[0] > sensor.width() -1 or roi[0] < 1- set_range[0] or roi[1] > sensor.height() - 1 or roi[1] < 1 - set_range[1] :
        log.new('Frame',frame_count,'bank angle over')
        continue

    found_blobs = img.find_blobs(thresholds, roi=roi, x_stride = 30, y_stride = 30, pixels_threshold = 500, merge = True, margin = 20)
    max_blob = find_maxblob(found_blobs)

    if max_blob:
        temp1 = 0.5 + (max_blob.x() + 0.5*max_blob.w() - (roi[0] + set_range[0]/2))/set_range[0]
        temp2 = 0.5 - (max_blob.y() + 0.5*max_blob.h() - (roi[1] + set_range[1]/2))/set_range[1]
        if abs(temp1 - last_rol) > 0.05 or abs(temp2 - last_pit) > 0.05:
            print('error data')
            log.new('Frame',frame_count,'acute change')

            skip_count = skip_count+1
            if skip_count == 2:
                skip_count = 0
                aim_error[0] = last_rol = temp1
                aim_error[1] = last_pit = temp2

            else:
                aim_error[0] = last_rol
                aim_error[1] = last_pit
        else:
            aim_error[0] = last_rol = temp1
            aim_error[1] = last_pit = temp2

        img.draw_cross(int(max_blob.x() + 0.5*max_blob.w()), int(max_blob.y() + 0.5*max_blob.h()), color = (0, 255, 255))#物体中心画十字
        img.draw_rectangle(max_blob.rect(), color = (0, 255, 255))#画矩形框

        if not target_find:
            print('target find')
            log.new('Frame',frame_count,'target find')
            target_find = 1
            target_lost = 0

    else:
        aim_error = [0.5, 0.5]#range = [0, 1]

        if not target_lost:
            print('target lost')
            log.new('Frame',frame_count,'target lost')
            target_find = 0
            target_lost = 1

    print(aim_error[0],aim_error[1])

    udata.send(instruction = "target_location", x = aim_error[0], y = aim_error[1], z = 0)

    frame_count += 1
    img.draw_rectangle(roi[0], roi[1], roi[2], roi[3], color = (255, 0, 0), thickness = 1, fill = False)


    #if frame_count % 2 :      #4帧灯闪
        #green_led.toggle()
        #red_led.toggle()
        #blue_led.toggle()

    #if frame_count % 10 == 0 :
    FPS = clock.fps()
    print(FPS)
