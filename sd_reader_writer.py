 log_file = '/SaveData.txt'     #记录文件
 
 with open(log_file, "w+") as log:
                log.write('')           #写入内容，覆盖全部已有记录              
                
 with open(log_file, "r") as log:
                sd_data = log.read()           #写入内容，覆盖全部已有记录          
