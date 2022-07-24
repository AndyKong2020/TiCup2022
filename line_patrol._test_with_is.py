import sensor, image, time

import math
import cmath

from uart_communication import UartCommunication
from LOG import LOG

udata = UartCommunication
log = LOG(True)     #写日志前清空日志文件

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time = 2000)
#设置摄像头参数

thresholds = [ (0, 24, -128, 127, -57, 4) ] #阈值

rol_angle = 0
pit_angle = 0

view_angle = 57.6#摄像头视野角度
view_angle_alpha = (view_angle*math.pi)/180#转换为弧度制

X_cor_factor = 1.1 #X偏移量修正系数
Y_cor_factor = 1.1 #Y偏移量修正系数

roi = [0, 0, 0, 0]
aim_error = [0, 0]
line_error = [90, 0.5]#第一个参数是与x正方向的夹角，第二个是水平偏移量，范围0-1


set_range = [160, 160]#裁剪区域





clock = time.clock()

while True:

    clock.tick()
    img = sensor.snapshot()

    udata.send(instruction='give_me_attitude_angle')
    temp=udata.receive()
    if temp:

        rol_angle = temp[0] #读姿态角
        pit_angle = temp[1]

    else:   #没收到姿态数据
        aim_error = [0.5, 0.5]
        print('No data')
        log.new('Frame',frame_count,'No data')

    #根据姿态调整防抖的roi
    roi[0] = int( ( sensor.width() -  set_range[0] ) / 2 - ( math.sqrt( sensor.width()**2 + sensor.height()**2 ) * math.tan( rol_angle ) ) / ( 2*math.tan( view_angle_alpha / 2 ) ) * X_cor_factor )
    roi[1] = int( ( sensor.height() - set_range[1] ) / 2 -  ( math.sqrt( sensor.width()**2 + sensor.height()**2 ) * math.tan( pit_angle ) ) / ( 2*math.tan( view_angle_alpha / 2 ) ) * Y_cor_factor )
    roi[2] = set_range[0]
    roi[3] = set_range[1]

###################################################################################################
    #kh：roi的中心点坐标
    roi_center_x = roi[0] + set_range[0]/2
    roi_center_y = roi[1] + set_range[1]/2
    roi_center = (roi_center_x, roi_center_y)

    #设置直线经过修正后的两点坐标（修正后点一始终在点二下方）（这一段只是将它们初始化到中位）
    found_line_fixed_x1 = roi_center_x
    found_line_fixed_x2 = roi_center_x
    found_line_fixed_y1 = roi[1]
    found_line_fixed_y2 = roi[1] + set_range[1]
    found_line_fixed = [found_line_fixed_x1, found_line_fixed_y1, found_line_fixed_x2, found_line_fixed_y2]



    #把roi的区域用红框框出来
    img.draw_rectangle(roi[0], roi[1], roi[2], roi[3], color = (255, 0, 0), thickness = 1, fill = False)


    #从这里开始是巡线代码
    found_line = img.get_regression(thresholds, roi = roi)
    #magnitude()值越大，表示效果越好
    if found_line and found_line.magnitude() >= 8:

        #修正直线方向，让点一始终在下方
        if found_line[1] <= found_line[3]:
            found_line_fixed[3], found_line_fixed[1] = found_line[1], found_line[3]
            found_line_fixed[2], found_line_fixed[0] = found_line[0], found_line[2]
        else:
            found_line_fixed[1], found_line_fixed[3] = found_line[1], found_line[3]
            found_line_fixed[0], found_line_fixed[2] = found_line[0], found_line[2]

        #下面的计算，须在分母不为零时
        if (found_line.x1() - found_line.x2()) != 0 and (found_line.y1() - found_line.y2()) != 0:
            #计算角度偏差量
            k = (found_line_fixed[3] - found_line_fixed[1])/(found_line_fixed[2] - found_line_fixed[0]) #斜率
            line_error[0] = (math.atan(k))*180/math.pi  #角度


            #计算x轴偏差量，范围0-1，中位0.5
            line_error[1] = ((roi_center[1] + math.tan(line_error[0]*math.pi/180)*found_line_fixed[0] - found_line_fixed[1])/math.tan(line_error[0]*math.pi/180) - roi[0])/set_range[0]

            #超出范围的就取极值
            if line_error[0] <= 0:
                line_error[0] = -line_error[0]
            else:
                line_error[0] = 180 - line_error[0]

            #超出范围的就取极值
            if line_error[1] >= 1:
                line_error[1] = 1
            elif line_error[1] <= 0:
                line_error[1] = 0


        #下面几行用于调试
        #img.draw_line(found_line.line(), color = (127, 127, 127))
        img.draw_arrow(found_line_fixed[0], found_line_fixed[1], found_line_fixed[2], found_line_fixed[3], color = (127, 127, 127), size = 30, thickness = 2)
        img.draw_cross(found_line_fixed[0], found_line_fixed[1], color = (255, 0, 0), size = 10, thickness = 2)


        udata.send(instruction = "line_patrol", angle = aim_error[0], intercept = aim_error[1])
        print(line_error)




    
