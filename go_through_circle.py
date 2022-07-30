import sensor, image, time
from machine import I2C
from vl53l1x import VL53L1X
from pyb import Servo
from uart_communication import UartCommunication
udata = UartCommunication
pan_servo=Servo(2)
tilt_servo=Servo(3)

pan_servo.calibration(500,2500,1500)
tilt_servo.calibration(500,2500,1500)

rod_threshold  = (20, 71, 9, 127, -23, 50)#设置杆的阈值
roi = [0, 200, 30, 240]

pan_servo.angle(90)#云台回正
tilt_servo.angle(90)#云台回正


i2c = I2C(2)
distance = VL53L1X(i2c)#距离传感器

swerve_rod = [400, 0.43, 0.5]#绕杆参数，第一个是距离mm，第二个是杆的水平位置

#选出色块中的最大
def find_maxblob(blobs):
    max_size=1
    if blobs:
        max_blob = 0
        for blob in blobs:
            blob_size = blob.w()*blob.h()
            #色块尺寸判断
            if ( (blob_size > max_size) & (blob_size > 30)   ) :
                #色块更新
                max_blob=blob
                #最大尺寸更新
                max_size = blob.w()*blob.h()
        return max_blob


sensor.reset() # Initialize the camera sensor.
sensor.set_pixformat(sensor.RGB565) # use RGB565.
sensor.set_framesize(sensor.QVGA) # use QQVGA for speed.
sensor.skip_frames(10) # Let new settings take affect.
sensor.set_auto_whitebal(False) # turn this off.
clock = time.clock() # Tracks FPS.

while True:

    clock.tick()
    img = sensor.snapshot()
    #img.lens_corr(1.5)#畸变矫正，使用会降低画质
    rod = img.find_blobs([rod_threshold], roi = view, pixels_threshold=30)
    rod = find_maxblob(rod)#找杆

    if rod:

        #画矩形和十字
        img.draw_rectangle(rod.rect()) # rect
        img.draw_cross(rod[5], rod[6]) # cx, cy

        rod_ux = rod[5]/sensor.width()#计算杆x坐标的单位化，最左为0，最右为1

        if (0.36 < rod_ux < 0.5):#在视野中心的狭窄区域，激光测距有效

            if distance.read() < 250 or distance.read() > 650:#激光测距超限限位
                distance_limited = 420
                
            else:#未超限用原值
                distance_limited = distance.read()
                break

            swerve_rod[0] = distance_limited
            swerve_rod[1] = rod_ux
            swerve_rod[2] = 0.5

        else:#不在视野中心的狭窄区域，激光测距给中位
            swerve_rod[0] = 400
            swerve_rod[1] = rod_ux
            swerve_rod[2] = 0.5


    else:#找不到杆自旋
        swerve_rod = [400, 0.63, 0.5]

    print(swerve_rod)

    udata.send( instruction = "detour_pole", distance = swerve_rod[0], xerror = swerve_rod[1] )
    #print(clock.fps())

while True:

    clock.tick()
    img = sensor.snapshot()
    #img.lens_corr(1.5)#畸变矫正，使用会降低画质
    rod = img.find_blobs([rod_threshold], roi = view, pixels_threshold=30)
    rod = find_maxblob(rod)#找杆

    if rod:

        #画矩形和十字
        img.draw_rectangle(rod.rect()) # rect
        img.draw_cross(rod[5], rod[6]) # cx, cy

        rod_ux = rod[5]/sensor.width()#计算杆x坐标的单位化，最左为0，最右为1

        if (0.36 < rod_ux < 0.5):#在视野中心的狭窄区域，激光测距有效

            if distance.read() < 250 or distance.read() > 650:#激光测距超限限位
                distance_limited = 400
                roll = 0.5
            else:#未超限用原值
                distance_limited = distance.read()
                roll = 1

            swerve_rod[0] = distance_limited
            swerve_rod[1] = rod_ux
            swerve_rod[2] = roll

        else:#不在视野中心的狭窄区域，激光测距给中位
            swerve_rod[0] = 400
            swerve_rod[1] = rod_ux
            swerve_rod[2] = 0.5


    else:#找不到杆自旋
        swerve_rod = [400, 0.63, 0.5]

    print(swerve_rod)

    udata.send( instruction = "detour_pole", distance = swerve_rod[0], xerror = swerve_rod[1] )
    #print(clock.fps())
