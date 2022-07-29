from pyb import Servo

s = Servo(1)    #启用舵机口P7，2-P8,3-P9

pin1 = Pin("A1", Pin.IN, Pin.PULL_UP)
pin2 = Pin("A2", Pin.IN, Pin.PULL_UP)
pin3 = Pin("A3", Pin.IN, Pin.PULL_UP)   #读引脚配置，之后再具体更改用哪三个引脚

def check_servo_state:      #刷新舵机输出，两个while里循环跑
    state = [0,0,0]
    state[0] = pin1.value()
    state[1] = pin2.value()
    state[2] = pin3.value()
    if state == [0,1,0] or state == [1,0,0]:
        s.angle(90)     #舵机停止命令
        break
        
    if state == [0,0,1] :
        s.angle(180)    #舵机上升命令
        break
        
    if state == [0,1,1]  :
        s.angle(0)      #舵机下降命令
        
#选择哪个模式，若引脚状态无效则卡死在此函数，直到有效引脚状态，返回1起降模式，返回2色块模式
#作为两个while执行条件      
def choose_mode:    
    state = [0,0,0]
    state[0] = pin1.value()
    state[1] = pin2.value()
    state[2] = pin3.value()
    while state  != [1,0,0] and state  != [0,0,1] and state  != [0,1,0] and state  != [0,1,1]:
        pyb.delay(10)
        state[0] = pin1.value()
        state[1] = pin2.value()
        state[2] = pin3.value()
        
    if state == [1,0,0]:    #起降
        return 1
    if state == [0,0,1] or state == [0,1,0] or state == [0,1,1]:    #色块
        return 2
        
