# Untitled - By: lenovo - 周六 7月 16 2022 错误：起始红框必须在视野内

import sensor, image, time
#import lcd
import math
import cmath

from uart_communication import UartCommunication

thresholds = [(0, 24, -128, 127, -57, 4)]

rol_angle = 0
pit_angle = 0


udata = UartCommunication

#lcd.init()
#lcd.rotation(2)#成倒立像，需旋转
#lcd.clear()


view_angle = 57.6#摄像头视野角度
view_angle_alpha = (view_angle*math.pi)/180#转换为弧度制

X_cor_factor = 1.1 #X偏移量修正系数
Y_cor_factor = 1.1 #Y偏移量修正系数
roi = [0, 0, 0, 0]
aim_error = [0, 0]
line_error = [90, 0]









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

    roi_center_x = roi[0] + set_range[0]/2
    roi_center_y = roi[1] + set_range[1]/2
    roi_center = (roi_center_x, roi_center_y)


    found_line_fixed_x1 = roi_center_x
    found_line_fixed_x2 = roi_center_x
    found_line_fixed_y1 = roi[1]
    found_line_fixed_y2 = roi[1] + set_range[1]
    found_line_fixed = [found_line_fixed_x1, found_line_fixed_y1, found_line_fixed_x2, found_line_fixed_y2]




    img.draw_rectangle(roi[0], roi[1], roi[2], roi[3], color = (255, 0, 0), thickness = 1, fill = False)
    #lcd.display(img)

    found_line = img.get_regression(thresholds, roi = roi)
    if found_line and found_line.magnitude() >= 8:


        '''if found_line[0] <= found_line[2]:
            found_line_fixed[2], found_line_fixed[0] = found_line[0], found_line[2]
        else:'''
        #found_line_fixed[0], found_line_fixed[2] = found_line[0], found_line[2]
        if found_line[1] <= found_line[3]:
            found_line_fixed[3], found_line_fixed[1] = found_line[1], found_line[3]
            found_line_fixed[2], found_line_fixed[0] = found_line[0], found_line[2]
        else:
            found_line_fixed[1], found_line_fixed[3] = found_line[1], found_line[3]
            found_line_fixed[0], found_line_fixed[2] = found_line[0], found_line[2]


        if (found_line.x1() - found_line.x2()) != 0 and (found_line.y1() - found_line.y2()) != 0:

            line_error[0] = (math.atan((found_line_fixed[3] - found_line_fixed[1])/(found_line_fixed[2] - found_line_fixed[0])))*180/math.pi
            if line_error[0] <= 0:
                line_error[0] = -line_error[0]
            else:
                line_error[0] = 180 - line_error[0]


            line_error[1] = ((roi_center[1] + math.tan(line_error[0]*math.pi/180)*found_line_fixed[0] - found_line_fixed[1])/math.tan(line_error[0]*math.pi/180) - roi[0])/set_range[0]
            if line_error[1] >= 1:
                line_error[1] = 1
            elif line_error[1] <= 0:
                line_error[1] = 0

        #img.draw_line(found_line.line(), color = (127, 127, 127))
        img.draw_arrow(found_line_fixed[0], found_line_fixed[1], found_line_fixed[2], found_line_fixed[3], color = (127, 127, 127), size = 30, thickness = 2)
        img.draw_cross(found_line_fixed[0], found_line_fixed[1], color = (255, 0, 0), size = 10, thickness = 2)



        print(line_error)




    #lcd.draw_string(20,50,"x:"+str(accel_array[0]))
    #lcd.draw_string(20,70,"y:"+str(accel_array[1]))
    #lcd.draw_string(20,90,"z:"+str(accel_array[2]))
    #print(clock.fps())
