# Untitled - By: lenovo - 周六 7月 16 2022

import sensor, image, time, pyb
from pyb import Pin
#import lcd
import math
import cmath
#from LOG import LOG
from uart_communication import UartCommunication

red_led = pyb.LED(1)
green_led = pyb.LED(2)
blue_led = pyb.LED(3)   #LED
pin0 = Pin('P0', Pin.OUT_PP, Pin.PULL_NONE)
pin1 = Pin('P1', Pin.IN, Pin.PULL_NONE)
pin0.value(1)


frame_count=0           #帧数量
FPS = 0                 #记录fps
target_find = 0         #找到目标
target_lost = 0         #丢失目标


black_thresholds = [(0, 35, -33, 35, -33, 36)]
red_blue_thresholds = [(15, 50, 19, 127, -23, 56), (0, 26, -34, 24, -128, 6), (0, 40, -50, 15, -15, 50)]


rol_angle = 0
pit_angle = 0


udata = UartCommunication
#log = LOG(True) #写日志前清空日志文件

#lcd.init()
#lcd.rotation(2)#成倒立像，需旋转
#lcd.clear()


view_angle = 70#摄像头视野角度
view_angle_alpha = (view_angle*math.pi)/180#转换为弧度制

X_cor_factor = 1 #X偏移量修正系数
Y_cor_factor = 1 #Y偏移量修正系数
roi = [0, 0, 0, 0]
aim_error = [0.5,0.5]


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









set_range = [240, 240]#裁剪区域


set_accurate_roi_range = [5, 0.75]#设置精准降落的roi，第一个值是宽度像素数量，第二个是长度占全roi的比值
accurate_roi_inner = [0, 0, 0, 0]#accurate_roi内部矩形限度，依次是上下左右

last_rol=0;
last_pit=0;  #暂存上一次数据用于下一帧比对
skip_count=0;           #跳过的帧数量

#fps_log = pyb.millis()
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_framerate(80)
sensor.skip_frames(time = 2000)

#设置摄像头参数

clock = time.clock()

while True:
    if pin1.value():
        clock.tick()
    
        img = sensor.snapshot()
        start = pyb.millis()
        udata.send(instruction='give_me_attitude_angle')
        temp=udata.receive()
        if temp:
            rol_angle = temp[0] #读姿态角
            pit_angle = temp[1]
    
        else:
            aim_error = [0.5, 0.5]
            print('No data')
            #log.new('Frame',frame_count,'No data')
    
        roi[0] = int( ( sensor.width() -  set_range[0] ) / 2 + ( math.sqrt( sensor.width()**2 + sensor.height()**2 ) * math.tan( rol_angle ) ) / ( 2*math.tan( view_angle_alpha / 2 ) ) * X_cor_factor )
        roi[1] = int( ( sensor.height() - set_range[1] ) / 2 +  ( math.sqrt( sensor.width()**2 + sensor.height()**2 ) * math.tan( pit_angle ) ) / ( 2*math.tan( view_angle_alpha / 2 ) ) * Y_cor_factor )
        roi[2] = set_range[0]
        roi[3] = set_range[1]
    
    
        accurate_roi_inner[0] = roi[1] + roi[3] - (roi[3] - roi[3]*set_accurate_roi_range[1])/2
        accurate_roi_inner[1] = roi[1] + (roi[3] - roi[3]*set_accurate_roi_range[1])/2
        accurate_roi_inner[2] = roi[0] + (roi[2] - roi[2]*set_accurate_roi_range[1])/2
        accurate_roi_inner[3] = roi[0] + roi[2] - (roi[2] - roi[2]*set_accurate_roi_range[1])/2
    
        accurate_roi_top = [int(accurate_roi_inner[2]), int(accurate_roi_inner[0])-1, int(roi[2]*set_accurate_roi_range[1]), int(set_accurate_roi_range[0])]
        accurate_roi_bottom = [int(accurate_roi_inner[2]), int(accurate_roi_inner[1] - (set_accurate_roi_range[0]))+1, int(roi[2]*set_accurate_roi_range[1]), int(set_accurate_roi_range[0])]
        accurate_roi_left = [int(accurate_roi_inner[2] - set_accurate_roi_range[0])+1, int(accurate_roi_inner[1]), int(set_accurate_roi_range[0]), int(roi[3]*set_accurate_roi_range[1])]
        accurate_roi_right = [int(accurate_roi_inner[3])-1, int(accurate_roi_inner[1]), int(set_accurate_roi_range[0]), int(roi[3]*set_accurate_roi_range[1])]
    
        #img.draw_rectangle(roi[0], roi[1], roi[2], roi[3], color = (255, 0, 0), thickness = 1, fill = False)
    
        if not(roi[0] > sensor.width() -1 or roi[0] < 1- set_range[0] or roi[1] > sensor.height() - 1 or roi[1] < 1 - set_range[1]) :
            #log.new('Frame',frame_count,'bank angle over')
            #continue
    
            #img.binary(thresholds)
            #img.erode(1)
            #img.dilate(2)
            found_blobs = img.find_blobs(black_thresholds, roi=roi, x_stride = 30, y_stride = 30, pixels_threshold = 1000, merge = True, margin = 80)
            max_blob = find_maxblob(found_blobs)
            found_blobs_merge = img.find_blobs(black_thresholds, roi=roi, x_stride = 15, y_stride = 15, pixels_threshold = 50, merge = True, margin = 120)
            found_blobs_merge = find_maxblob(found_blobs_merge)
    
            if max_blob:
                if found_blobs_merge.pixels() > 0.5*set_range[0]*set_range[1] or found_blobs_merge.area() > 0.8*set_range[0]*set_range[1]:
                    if (accurate_roi_inner[0] < sensor.height()) and (accurate_roi_inner[1] > 0) and (accurate_roi_inner[2] > 0) and (accurate_roi_inner[3] < sensor.width()):
                        found_top_points = img.find_blobs(black_thresholds, roi = accurate_roi_top, x_stride = 15, pixels_threshold = 40, invert = True)
                        max_top_point = find_maxblob(found_top_points)
                        found_bottom_points = img.find_blobs(black_thresholds, roi = accurate_roi_bottom, x_stride = 15, pixels_threshold = 40, invert = True)
                        max_bottom_point = find_maxblob(found_bottom_points)
                        found_left_points = img.find_blobs(black_thresholds, roi = accurate_roi_left, y_stride = 15, pixels_threshold = 40, invert = True)
                        max_left_point = find_maxblob(found_left_points)
                        found_right_points = img.find_blobs(black_thresholds, roi = accurate_roi_right, y_stride = 15, pixels_threshold = 40, invert = True)
                        max_right_point = find_maxblob(found_right_points)
    
                        if max_top_point and max_bottom_point and max_left_point and max_right_point:
                            max_points = [max_top_point, max_bottom_point, max_left_point, max_right_point]
    
                            for max_point in max_points:
    
                                img.draw_cross(max_point.cx(), max_point.cy(), color = (0, 100, 100))#物体中心画十字
                                img.draw_rectangle(max_point.rect(), color = (0, 255, 0))#画矩形框
    
    
                            land_low_target = [(max_top_point[5] + max_bottom_point[5])*0.5, (max_left_point[6] + max_right_point[6])*0.5]
                            img.draw_cross(int(land_low_target[0]), int(land_low_target[1]), color = (0, 255, 0))#中心画十字
                            aim_error[0] = 0.5 + (land_low_target[0] - (roi[0] + set_range[0]/2))/set_range[0]
                            aim_error[1] = 0.5 - (land_low_target[1] - (roi[1] + set_range[1]/2))/set_range[1]
                        else:
                            aim_error[0] = 0.5
                            aim_error[1] = 0.5
                    else:
                        aim_error = [0.5,0.5]
    
    
    
    
                else:
                    temp1 = 0.5 + (max_blob[0] - (roi[0] + set_range[0]/2))/set_range[0]
                    temp2 = 0.5 - (max_blob[1] - (roi[1] + set_range[1]/2))/set_range[1]
                    if abs(temp1 - last_rol) > 0.05 or abs(temp2 - last_pit) > 0.05:
                        print('error data')
                        #log.new('Frame',frame_count,'acute change')
    
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
    
                    img.draw_cross(max_blob.cx(), max_blob.cy(), color = (0, 255, 0))#物体中心画十字
                    img.draw_rectangle(max_blob.rect(), color = (0, 255, 0))#画矩形框
    
                    if not target_find:
                        #print('target find')
                        #log.new('Frame',frame_count,'target find')
                        target_find = 1
                        target_lost = 0
    
            else:
                aim_error = [0.5, 0.5]#range = [0, 1]
    
                if not target_lost:
                    #print('target lost')
                    #log.new('Frame',frame_count,'target lost')
                    target_find = 0
                    target_lost = 1
    
            img.draw_rectangle(roi[0], roi[1], roi[2], roi[3], color = (255, 0, 0), thickness = 1, fill = False)
            #img.draw_rectangle(found_blobs_merge.rect(), color = (255, 255, 0), thickness = 1, fill = False)
            img.draw_rectangle(accurate_roi_top[0], accurate_roi_top[1], accurate_roi_top[2], accurate_roi_top[3], color = (255, 0, 0), thickness = 1, fill = False)
            img.draw_rectangle(accurate_roi_bottom[0], accurate_roi_bottom[1], accurate_roi_bottom[2], accurate_roi_bottom[3], color = (255, 0, 0), thickness = 1, fill = False)
            img.draw_rectangle(accurate_roi_left[0], accurate_roi_left[1], accurate_roi_left[2], accurate_roi_left[3], color = (255, 0, 0), thickness = 1, fill = False)
            img.draw_rectangle(accurate_roi_right[0], accurate_roi_right[1], accurate_roi_right[2], accurate_roi_right[3], color = (255, 0, 0), thickness = 1, fill = False)
            print(aim_error)
    
            udata.send(instruction = "target_location", x = aim_error[0], y = aim_error[1], z = 0)
    
            frame_count += 1
    
    
            #if frame_count % 2 :      #4帧灯闪
                #green_led.toggle()
                #red_led.toggle()
                #blue_led.toggle()
    
    
    
        time_run = pyb.elapsed_millis(start)
        #print(time_run)
        FPS = clock.fps()
        print(FPS)
    else:
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
            #log.new('Frame',frame_count,'No data')
    
        roi[0] = int( ( sensor.width() -  set_range[0] ) / 2 + ( math.sqrt( sensor.width()**2 + sensor.height()**2 ) * math.tan( rol_angle ) ) / ( 2*math.tan( view_angle_alpha / 2 ) ) * X_cor_factor )
        roi[1] = int( ( sensor.height() - set_range[1] ) / 2 +  ( math.sqrt( sensor.width()**2 + sensor.height()**2 ) * math.tan( pit_angle ) ) / ( 2*math.tan( view_angle_alpha / 2 ) ) * Y_cor_factor )
        roi[2] = set_range[0]
        roi[3] = set_range[1]
    
    
    
        if roi[0] > sensor.width() -1 or roi[0] < 1- set_range[0] or roi[1] > sensor.height() - 1 or roi[1] < 1 - set_range[1] :
            #log.new('Frame',frame_count,'bank angle over')
            continue
    
        found_blobs = img.find_blobs(red_blue_thresholds, roi=roi, x_stride = 30, y_stride = 30, pixels_threshold = 500, merge = True, margin = 20)
        max_blob = find_maxblob(found_blobs)
    
        if max_blob:
            temp1 = 0.5 + (max_blob.x() + 0.5*max_blob.w() - (roi[0] + set_range[0]/2))/set_range[0]
            temp2 = 0.5 - (max_blob.y() + 0.5*max_blob.h() - (roi[1] + set_range[1]/2))/set_range[1]
            if abs(temp1 - last_rol) > 0.05 or abs(temp2 - last_pit) > 0.05:
                print('error data')
                #log.new('Frame',frame_count,'acute change')
    
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
                #log.new('Frame',frame_count,'target find')
                target_find = 1
                target_lost = 0
    
        else:
            aim_error = [0.5, 0.5]#range = [0, 1]
    
            if not target_lost:
                print('target lost')
                #log.new('Frame',frame_count,'target lost')
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
 
