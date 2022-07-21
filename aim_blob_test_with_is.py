# Untitled - By: lenovo - 周六 7月 16 2022

import sensor, image, time
#import lcd
import math
import cmath

from uart_communication import UartCommunication

thresholds = [(71, 100, -27, 16, 46, 127)]

rol_angle = 0
pit_angle = 0


udata=UartCommunication

#lcd.init()
#lcd.rotation(2)#成倒立像，需旋转
#lcd.clear()


view_angle = 57.6#摄像头视野角度
view_angle_alpha = (view_angle*math.pi)/180#转换为弧度制

X_cor_factor = 1.1 #X偏移量修正系数
Y_cor_factor = 1.1 #Y偏移量修正系数
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
sensor.skip_frames(time = 2000)
#设置摄像头参数

clock = time.clock()
set_range = [160, 160]#裁剪区域




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


    found_blobs = img.find_blobs(thresholds, roi=roi, x_stride = 5, y_stride = 5, pixels_threshold = 100)
    max_blob = find_maxblob(found_blobs)
    if max_blob:
        aim_error[0] = 0.5 + (max_blob[5] - (roi[0] + set_range[0]/2))/set_range[0]
        aim_error[1] = 0.5 - (max_blob[6] - (roi[1] + set_range[1]/2))/set_range[1]
        img.draw_cross(max_blob.cx(), max_blob.cy())#物体中心画十字
        img.draw_rectangle(max_blob.rect())#画矩形框

    else:
        aim_error = [0.5, 0.5]#range = [0, 1]


    print(aim_error)




    #lcd.draw_string(20,50,"x:"+str(accel_array[0]))
    #lcd.draw_string(20,70,"y:"+str(accel_array[1]))
    #lcd.draw_string(20,90,"z:"+str(accel_array[2]))
    #print(clock.fps())
