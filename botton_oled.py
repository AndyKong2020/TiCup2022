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


counter = 1
chooesd = [0,0]
target1chooesd = 0
frame_count = 0
frame_count_skip = -5
last_frame_count = 0
def button1(line):
    global counter
    global frame_count
    global frame_count_skip
    global last_frame_count

    if frame_count_skip > last_frame_count:
        last_frame_count = frame_count
        counter += 1
        if counter > 12:
            counter = 1

        print('choose:',counter)


def button2(line):
    global counter
    global chooesd
    global target1chooesd
    global frame_count
    global frame_count_minus3
    global last_frame_count

    if frame_count_skip > last_frame_count:
        last_frame_count = frame_count
        if not chooesd[0]:
            chooesd[0] = counter
            target1chooesd = 1
            print('target1 comfirmed:',chooesd[0])

        if not chooesd[1] and not target1chooesd:
            chooesd[1] = counter

            print('target2 comfirmed:',chooesd[1])
            print('targets comfirmed:',chooesd)
        target1chooesd = 0

ext1 = ExtInt(Pin('P7'), ExtInt.IRQ_RISING    , Pin.PULL_UP  , button1)
ext2 = ExtInt(Pin('P8'), ExtInt.IRQ_RISING    , Pin.PULL_UP  , button2)

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time = 2000)

clock = time.clock()

while(True):
    clock.tick()
    img = sensor.snapshot()

    oled.fill(0)
    oled.text("choose:" + str(counter), 0,  0)
    if chooesd[0]:

        oled.text("Target1:" + str(chooesd[0]), 0,  20)
    if chooesd[1]:
        oled.text("Target2:" + str(chooesd[1]), 0,  30)
        oled.text("comfirmed:" + str(chooesd[0])+ ','+str(chooesd[1]), 0,  50)

    oled.show()   #OLED执行显示

    print(clock.fps())



    frame_count += 1
    frame_count_skip += 1
import sensor, image, time, pyb

from pid import PID
from pyb import Servo

pan_servo=Servo(2)
tilt_servo=Servo(3)

pan_servo.calibration(500,2500,1500)
tilt_servo.calibration(500,2500,1500)

red_threshold  = (11, 53, 28, 95, 6, 76)

pan_pid = PID(p=0.07, i=0, imax=90) #脱机运行或者禁用图像传输，使用这个PID
tilt_pid = PID(p=0.05, i=0, imax=90) #脱机运行或者禁用图像传输，使用这个PID
#pan_pid = PID(p=0.1, i=0, imax=90)#在线调试使用这个PID
#tilt_pid = PID(p=0.1, i=0, imax=90)#在线调试使用这个PID

sensor.reset() # Initialize the camera sensor.
sensor.set_pixformat(sensor.RGB565) # use RGB565.
sensor.set_framesize(sensor.QVGA) # use QQVGA for speed.
sensor.skip_frames(10) # Let new settings take affect.
sensor.set_auto_whitebal(False) # turn this off.
clock = time.clock() # Tracks FPS.

def find_max(blobs):
    max_size=0
    for blob in blobs:
        if blob[2]*blob[3] > max_size:
            max_blob=blob
            max_size = blob[2]*blob[3]
    return max_blob


while(True):
    clock.tick() # Track elapsed milliseconds between snapshots().
    img = sensor.snapshot() # Take a picture and return the image.

    blobs = img.find_blobs([red_threshold], pixels_threshold=100)
    if blobs:
        max_blob = find_max(blobs)
        pan_error = max_blob.cx()-img.width()/2
        tilt_error = max_blob.cy()-img.height()/2


        img.draw_rectangle(max_blob.rect()) # rect
        img.draw_cross(max_blob.cx(), max_blob.cy()) # cx, cy

        pan_output=pan_pid.get_pid(pan_error,1)/2
        tilt_output=tilt_pid.get_pid(tilt_error,1)
        print(pan_servo.angle(),tilt_servo.angle())
        pan_servo.angle(pan_servo.angle()+pan_output)

        tilt_servo.angle(tilt_servo.angle()-tilt_output)

        print('angle',tilt_servo.angle()-tilt_output)
