# 控制source_finding 运行300次, 用于统计成功率
import numpy as np
import source_finding
from tqdm import tqdm
import time
import winsound

# ---------------------------------------------------------------
total_iter = 300
robots_number = 16           # 机器人的数量
response_time = 1            # 响应时间
niche_radius = 20
errorness = 0.01             # 传感器误差
# ---------------------------------------------------------------
success_rate = []
time_consuming = []
search_step = []

def case_run(total_iter, robots_number, niche_radius, response_time, errorness):
    iterations_num = []
    count = 0 #成功率计数
    test_num = 300
    
    start_time = time.time()  # 记录开始时间
    #  ------------------------------------------------
    for i in tqdm(range(test_num)):
        iter_num, success = source_finding.find_func(total_iter, robots_number, niche_radius, response_time, errorness)
        if success == True:
            iterations_num.append(iter_num)
            count += 1
    #  ------------------------------------------------
    end_time = time.time()  # 记录结束时间
    elapsed_time = end_time - start_time  # 计算经过的时间
    # print(f"成功率={100*count/test_num}%")
    # print(f"花费时间={elapsed_time} s")
    # print(f"需迭代步数={sum(iterations_num)/len(iterations_num)}")
    return 100*count/test_num, elapsed_time, sum(iterations_num)/len(iterations_num)

robots_number_list = [3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20]
niche_radius_list = [10, 20, 30, 40, 50, 60, 70, 80, 90, 100]
response_time_list = [1, 2, 3, 4]
errorness_list = [0, 0.01, 0.05, 0.1]

# for _,errorness in enumerate(errorness_list):
#     a,b,c = case_run(total_iter, robots_number, niche_radius, response_time, errorness)
#     success_rate.append(a)
#     time_consuming.append(b)
#     search_step.append(c)
 
a,b,c = case_run(total_iter, robots_number, niche_radius, response_time, errorness)
success_rate.append(a)
time_consuming.append(b)
search_step.append(c)

winsound.Beep(1000, 500)  # Beep at 1000 Hz for 500 ms