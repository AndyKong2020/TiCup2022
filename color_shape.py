# Untitled - By: lenovo - 周四 7月 28 2022

import sensor, image, time
from pyb import UART, ExtInt
import pyb

from machine import I2C,Pin         #从machine模块导入I2C、Pin子模块
from ssd1306x import SSD1306_I2C     #从ssd1306模块中导入SSD1306_I2C子模块

pin6 = pyb.Pin('P6', pyb.Pin.OUT_PP)
pin6.low()
pyb.delay(1000) #P6作为地线使用

i2c = I2C(sda=Pin("P4"), scl=Pin("P5"),freq=480000 ) #I2C初始化：sda--> P0, scl --> P2,频率48MHz
oled = SSD1306_I2C(128, 64, i2c, addr=0x3c) #OLED显示屏初始化：128*64分辨率,OLED的I2C地址是0x3c

thresholds = [(7, 41, 0, 127, -33, 72), (0, 26, -34, 24, -128, 6)]
red_threshold = [(7, 41, 0, 127, -33, 72)]
blue_threshold = [(0, 26, -34, 24, -128, 6)]

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
clock = time.clock()
oled.fill(0)
oled.show()
while(True):
    color_shape_confirm = [0, 0]
    color_confidence = [0, 0]
    shape_confidence = [0, 0, 0]
    color_show = ['-', '-']
    shape_show = ['-', '-', '-']
    while True:
        clock.tick()
        img = sensor.snapshot()
        img.lens_corr(1.5)

        color_shape_detecting = [0, 0]


        found_blobs = img.find_blobs(thresholds, x_stride = 30, y_stride = 30, pixels_threshold = 500)
        max_blob = find_maxblob(found_blobs)

        if max_blob:
            img.draw_rectangle(max_blob.rect())
            red_or_not =  img.find_blobs(red_threshold, x_stride = 30, y_stride = 30, pixels_threshold = 50)
            red_or_not = find_maxblob(red_or_not)
            blue_or_not =  img.find_blobs(blue_threshold, x_stride = 30, y_stride = 30, pixels_threshold = 50)
            blue_or_not = find_maxblob(blue_or_not)
            if red_or_not and blue_or_not:
                if red_or_not.pixels() > blue_or_not.pixels():
                    color_shape_detecting[0] = 'red'
                elif blue_or_not.pixels() > red_or_not.pixels():
                    color_shape_detecting[0] = 'blue'
            elif red_or_not and not blue_or_not:
                color_shape_detecting[0] = 'red'
            elif not red_or_not and blue_or_not:
                color_shape_detecting[0] = 'blue'
            else:
                print('重新鉴定颜色')

            if red_or_not and blue_or_not:
                if color_shape_detecting[0] == 'blue':
                    if blue_or_not.density() > 0.78 and blue_or_not.solidity() > 0.85:
                        color_shape_detecting[1] = 'rect'
                    elif blue_or_not.density() > 0.6 and blue_or_not.solidity() < 0.85:
                        color_shape_detecting[1] = 'circle'
                    elif 0.4 < blue_or_not.density() < 0.5:
                        color_shape_detecting[1] = 'tri'
                    else:
                        print('重新鉴定形状')
                elif color_shape_detecting[0] == 'red':
                    if red_or_not.density() > 0.78 and red_or_not.solidity() > 0.85:
                        color_shape_detecting[1] = 'rect'
                    elif red_or_not.density() > 0.6 and red_or_not.solidity() < 0.85:
                        color_shape_detecting[1] = 'circle'
                    elif 0.4 < blue_or_not.density() < 0.5:
                        color_shape_detecting[1] = 'tri'
                    else:
                        print('重新鉴定形状')

        if color_shape_detecting[0] == 'red':
            color_confidence[0] += 1
            color_show[0] += '-'
        elif color_shape_detecting[0] == 'blue':
            color_confidence[1] += 1
            color_show[1] += '-'

        if color_shape_detecting[1] == 'tri':
            shape_confidence[0] += 1
            shape_show[0] += '-'
        elif color_shape_detecting[1] == 'rect':
            shape_confidence[1] += 1
            shape_show[1] += '-'
        elif color_shape_detecting[1] == 'circle':
            shape_confidence[2] += 1
            shape_show[2] += '-'
        oled.fill(0)
        oled.text("red:" + color_show[0], 0,  10)
        oled.text("blue:" + color_show[1], 0,  20)
        oled.text("tri:" + shape_show[0], 0,  30)
        oled.text("rect:" + shape_show[1], 0,  40)
        oled.text("circle:" + shape_show[2], 0,  50)
        oled.show()

        if color_confidence[0] + color_confidence[1] > 30 and shape_confidence[0] + shape_confidence[1] + shape_confidence[2] > 30:
            if color_confidence[0] > color_confidence[1]:
                color_shape_confirm[0] = 'red'
            elif color_confidence[1] > color_confidence[0]:
                color_shape_confirm[0] = 'blue'

            if shape_confidence[0] > shape_confidence[1] + shape_confidence[2]:
                color_shape_confirm[1] = 'tri'
            elif shape_confidence[1] > shape_confidence[0] + shape_confidence[2]:
                color_shape_confirm[1] = 'rect'
            elif shape_confidence[2] > shape_confidence[0] + shape_confidence[1]:
                color_shape_confirm[1] = 'circle'

            if color_shape_confirm[0] != 0 and color_shape_confirm[1] != 0:
                break
            else:
                print("1" )
                oled.fill(0)
                oled.show()
                color_shape_confirm = [0, 0]
                color_confidence = [0, 0]
                shape_confidence = [0, 0, 0]
                color_show = ['-', '-']
                shape_show = ['-', '-', '-']
        #print("FPS %f" % clock.fps())
    oled.fill(0)
    oled.text(color_shape_confirm[0] + ',' + color_shape_confirm[1], 0,  10)
    oled.show()
    break




