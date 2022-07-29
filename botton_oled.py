# Untitled - By: DIY大师 - 周三 7月 27 2022

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

MODE = 0
counter = 1
choosed = [0,0]
input_target = [0,0]
target1choosed = 0
frame_count = 0
frame_count_skip = -5
last_frame_count = 0

input_finished = 0
identify_finished = 0

thresholds = [(2, 79, 9, 127, 7, 63), (0, 26, -34, 24, -128, 6)]
red_threshold = [(2, 79, 9, 127, 7, 63)]
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


def button1(line):
    global counter
    global frame_count
    global frame_count_skip
    global last_frame_count

    global MODE

    if frame_count_skip > last_frame_count:
        last_frame_count = frame_count
        if not MODE:
            counter += 1
            if counter > 2:
                counter = 1
        else:
            counter += 1
            if counter > 12:
                counter = 1

        print('choose:',counter)


def button2(line):
    global counter
    global choosed
    global target1choosed
    global frame_count
    global frame_count_minus3
    global last_frame_count

    global MODE
    global input_target

    if frame_count_skip > last_frame_count:
        last_frame_count = frame_count
        if not MODE:
            MODE = counter
        else:
            if MODE==1:
                if not choosed[0]:
                    choosed[0] = counter
                    target1choosed = 1
                    print('target1 comfirmed:',choosed[0])

                if not choosed[1] and not target1choosed:
                    print('yes')
                    choosed[1] = counter
                    input_target[0] = choosed[0]
                    input_target[1] = choosed[1]
                target1choosed = 0


ext1 = ExtInt(Pin('P7'), ExtInt.IRQ_RISING    , Pin.PULL_UP  , button1)
ext2 = ExtInt(Pin('P8'), ExtInt.IRQ_RISING    , Pin.PULL_UP  , button2)

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(10)

clock = time.clock()

while(True):
    clock.tick()
    img = sensor.snapshot()

    oled.fill(0)
    if MODE==1:
        oled.text("Mode choosed: 1" ,0,0)
    elif MODE == 2:
        oled.text("Mode choosed: 2" ,0,0)
    elif MODE==0:
        oled.text("Mode setting: " + str(counter),0,0)

    if MODE==1:
        if not choosed[0]:
            oled.text("Tar1 setting: " + str(counter), 0,  20)
        else:
            oled.text("Tar1 choosed: " + str(choosed[0]), 0,  20)
        if not choosed[1] and choosed[0]:
            oled.text("Tar2 setting: " + str(counter), 0,  30)
        elif choosed[1] and choosed[0]:
            oled.text("Tar2 choosed: " + str(choosed[1]), 0,  30)
            oled.text("final: " + str(input_target[0])+','+str(input_target[1]), 0,  50)
            input_finished = 1

    oled.show()   #OLED执行显示

    if input_finished:
        break

    frame_count += 1
    frame_count_skip += 1

    if MODE==2:
        color_shape_confirm = [0, 0]
        color_confidence = [0, 0]
        shape_confidence = [0, 0, 0]
        color_show = ['-', '-']
        shape_show = ['-', '-', '-']
        oled.fill(0)
        oled.text("Ready...", 0, 0)
        oled.show()
        pyb.delay(3000)
        oled.fill(0)
        oled.show()
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
            oled.text("Mode choosed: 2" ,0,0)
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

                    color_shape_confirm = [0, 0]
                    color_confidence = [0, 0]
                    shape_confidence = [0, 0, 0]
                    color_show = ['-', '-']
                    shape_show = ['-', '-', '-']
            #print("FPS %f" % clock.fps())
        oled.fill(0)
        oled.text("Mode choosed: 2" ,0,0)
        oled.text(color_shape_confirm[0] + ',' + color_shape_confirm[1], 0,  10)
        oled.show()
        identify_finished = 1
    if identify_finished:
        break




    #print(clock.fps())





