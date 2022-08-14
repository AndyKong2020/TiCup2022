# Untitled - By: DIY大师 - 周日 7月 24 2022

class LOG():  #SD卡记录日志
    logs = []               #记录列表
    log_count = 0           #记录数量
    log_file = '/log.txt'

    def __init__(self, clear):

        if clear:       #清空已有日志
            with open(self.log_file, "w+") as log:
                log.write('')

        with open(self.log_file, "a") as log:
            log.write('\n\n\n\nNEW LOG\n\n')          #开始追加新记录

    def new(self,*log):
        self.string = ''
        for i in log:
            self.string = self.string + str(i) + '\t'
        self.string = self.string + '\n'
        self.logs.append(self.string)
        self.log_count += 1

        if self.log_count > 20:    #记录大于50条时写入
            print('SD')
            with open(self.log_file, "a") as log:
                for i in range(0,self.log_count):
                    log.write(self.logs[i])
            self.log_count = 0
            self.logs = []