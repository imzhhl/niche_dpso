from numpy import sin, cos, arccos, array, zeros, random, where, linalg, exp, pi, full, load, meshgrid, arange, sqrt, dot
# from obstacle_define_and_avoid import Obstacle
import re
from scipy.ndimage.morphology import distance_transform_edt as bwdist
from itertools import repeat
from tqdm import tqdm
import math
from data.input_data.fluent_data_transfer import Fluent2python
from algorithm.fitness_wind_function import Fitness
from algorithm.fitness_wind_function import Wind
from algorithm.obstacle_define_and_avoid import Obstacle
from data.output_data.figure_plot import Figure_plot
from robots.UAVs import UAV, Niche, Map
import robots.UAVs

# 一些函数的定义                   
def check_boundary(M, X):
    if M.map_x_lower < X[0, 0] < M.map_x_upper and M.map_y_lower < X[0, 1] < M.map_y_upper:
        rebound = False
    else:
        rebound = True
    return rebound

def check_plume(func, M, X, uds_t):
    # one by one
    fitness = func.fitness_func(X, uds_t)
    if fitness >= M.C_threshold:
        return True
    else:
        return False

def taboo_zone_avoid(X, V, M):
    # 有bug, 满足第二个条件时可能不满足第一个条件
    if len(M.taboo_center) != 0: # 如果已经形成了禁区
        for i in range(len(M.taboo_center)): # 循环所有禁区
            distance = linalg.norm(X - M.taboo_center[i])
            # unit_vector = (X - M.taboo_center[i])/np.linalg.norm(X - M.taboo_center[i])
            unit_vector = array((random.uniform(0,1),random.uniform(0,1))).reshape(1,2)
            # connec_vector = (M.taboo_center - X
            
            while distance < M.taboo_radius:
                
                X = X + 10*unit_vector       #当遇到搜索禁区的时候，就再随机一个新的方向
                distance = linalg.norm(X - M.taboo_center[i])     
    return X

def taboo_zone_cal_2(M):
    nrows = M.map_x_upper
    ncols = M.map_y_upper
    [x,y] = meshgrid(arange(ncols), arange(nrows))
    taboo_zone = zeros((nrows, ncols))
    
    if len(M.taboo_center) != 0: # 
        for i in range(len(M.taboo_center)): # 循环所有禁区
            t = ((x - M.taboo_center[i][0])**2 + (y - M.taboo_center[i][1])**2) < M.taboo_radius**2
            taboo_zone[t] = True
        d = bwdist(taboo_zone==0) # 计算距离禁区的距离
    else:
        d = full((nrows, ncols),float('inf'))
        
    return d.reshape(M.map_y_upper,M.map_x_upper)

def taboo_zone_cal(M):
    nrows = M.map_y_upper # rows 为区域划分的行数, 对应y坐标
    ncols = M.map_x_upper # cols 为列数, 对应x坐标
    [x,y] = meshgrid(arange(ncols), arange(nrows))
    taboo_zone = zeros((nrows, ncols))
    if len(M.taboo_center) != 0: # 
        for i in range(len(M.taboo_center)): # 循环所有禁区
            t = ((x - M.taboo_center[i][0])**2 + (y - M.taboo_center[i][1])**2) < M.taboo_radius**2
            taboo_zone[t] = 1
    # import matplotlib.pyplot as plt
    # fig, ax = plt.subplots()
    # ax.axis ([0, ncols, 0, nrows])
    # ax.imshow(taboo_zone, 'gray')
    return  taboo_zone
    
def taboo_zone_avoid_force(X, fitness, taboo_zone):
    if taboo_zone[int(X[0][1]-1),int(X[0][0]-1)] == 1:
        return array(float(0))
    else:
        return fitness
    
def taboo_zone_avoid_force_2(X, V, M, d):
    # 更科学的办法
    while d[int(X[0][1]-2),int(X[0][0]+2)] < 10: # 如果距禁区小于10
        print(f"before:{int(X[0][1]-2),int(X[0][0]+2)}")
        unit_vector = array((random.uniform(0,1),random.uniform(0,1))).reshape(1,2)
        X = X + 50*unit_vector # 随机转向反弹10m

    return X, V

def merge_arr_within_distance(arr, distance):
    merged_arr = arr.copy()  # 创建一个原数组的副本，以便进行修改

    i = 0
    while i < len(merged_arr) - 1:
        num1 = merged_arr[i]
        num2 = merged_arr[i + 1]
        if sqrt((num1[0]-num2[0])**2 + (num1[1]-num2[1])**2) < distance:
            average = [(num1[0] + num2[0]) / 2,(num1[1] + num2[1]) / 2 ]
            merged_arr[i] = average  # 替换第一个数为平均值
            del merged_arr[i + 1]  # 删除第二个数
        else:
            i += 1  # 跳到下一对数进行比较

    return merged_arr

def step_func(X, unit_vector, step):
    return X + unit_vector * step

def zigzag(M, vector, turning_num):
    sign = (-1)**turning_num
    ix = vector[0,0] * cos(sign*M.sigma) + vector[0,1] * sin(sign*M.sigma)
    iy = vector[0,1] * cos(sign*M.sigma) - vector[0,0] * sin(sign*M.sigma)
    vector = array([[ix, iy]])
    return vector

def turn_45_vector(vector):
    ix = vector[0,0] * cos(-pi/4) + vector[0,1] * sin(-pi/4)
    iy = vector[0,1] * cos(-pi/4) - vector[0,0] * sin(-pi/4)
    vector = array([[ix, iy]])
    return vector
    
def vertical_vector(vector):
    ix = vector[0,0] * cos(-pi/2) + vector[0,1] * sin(-pi/2)
    iy = vector[0,1] * cos(-pi/2) - vector[0,0] * sin(-pi/2)
    vector = array([[ix, iy]])
    return vector
    
def rebound(unit_vector):
    return -unit_vector

def turning_func(iteration):
    # 生成 3, 6, 10, 15, 21, 28, 36, 45, 54……
    n = 1000
    triangular_numbers = [(i * (i + 1)) // 2 for i in range(2, n + 2)]

    # 控制转向, 只有在第3, 6, 10, 15, 21, 28, 36, 45, 54……时才发生转向
    turning = False
    if iteration in triangular_numbers:
        turning = True
    return turning   

def inverse_velocity_update(V, X, pworst, c3, w):
    r3 = random.random(1) #随机数
    # r4 = random.random(1) #随机数
    cognitive = c3*r3*(pworst - X)
    # social =  c4*r4*(gworst -X)
    V = w*V + cognitive
    return V
    

def velocity_update(V, X, W, pbest, gbest, c_ini_1, c_end_1, c_ini_2, c_end_2, w, iter_num, total_iter):
    r1 = random.random(1) #随机数
    r2 = random.random(1) #随机数
    
    if linalg.norm(pbest - X) != 0 and linalg.norm(gbest - X) != 0 and linalg.norm(W) != 0:
        alpha_1 = arccos(dot((pbest - X)[0], (-W)[0])/(linalg.norm(pbest - X)*linalg.norm(-W)))
        alpha_2 = arccos(dot((gbest - X)[0], (-W)[0])/(linalg.norm(gbest - X)*linalg.norm(-W)))
        c1 = 2 * alpha_2/(alpha_1 + alpha_2)
        c2 = 2 * alpha_1/(alpha_1 + alpha_2)
        # c1 = (c_ini_1 + (c_end_1 - c_ini_1)*iter_num/total_iter) * alpha_2/(alpha_1 + alpha_2)
        # c2 = (c_ini_2 + (c_end_2 - c_ini_2)*iter_num/total_iter) * alpha_1/(alpha_1 + alpha_2)
        
    elif linalg.norm(pbest - X) == 0 and linalg.norm(gbest - X) != 0 and linalg.norm(W) != 0:
        alpha_2 = arccos(dot((gbest - X)[0], (-W)[0])/(linalg.norm(gbest - X)*linalg.norm(-W)))
        c1 = 0
        c2 = 2
        # c2 = (c_ini_2 + (c_end_2 - c_ini_2)*iter_num/total_iter)
        
    elif linalg.norm(pbest - X) != 0 and linalg.norm(gbest - X) == 0 and linalg.norm(W) != 0:
        alpha_1 = arccos(dot((pbest - X)[0], (-W)[0])/(linalg.norm(pbest - X)*linalg.norm(-W)))
        # c1 = (c_ini_1 + (c_end_1 - c_ini_1)*iter_num/total_iter)
        c1 = 2
        c2 = 0
    
    else:
        c1 = 2
        c2 = 2
        
        # c1 = (c_ini_1 + (c_end_1 - c_ini_1)*iter_num/total_iter)
        # c2 = (c_ini_2 + (c_end_2 - c_ini_2)*iter_num/total_iter)
        
    # w = 0.4 + (random.normal(loc=0, scale=1)/10 + random.random()/2)
    w_max = 0.9
    w_min = 0.4
    w = w_max - (w_max - w_min)*iter_num/total_iter
    cognitive = c1*r1*(pbest - X)
    social =  c2*r2*(gbest -X)
    V = w*V + cognitive + social
    

        
    # wind utilization controled by parameter "cp" (chi_theta)
    def cp(X, W, V):  #控制参数
        # 当风向与机器人运动方向相同时，控制参数cp取得最小值0
        # 当风向与机器人运动方向相反时，控制参数cp取得最大值1
        # 中间过程从0到1连续变化
        
        #算W和V的内积，粒子飞行速度和风场风向的内积
        a = dot(W[0], V[0])
        
        # 算wind的模
        wind_norm = linalg.norm(W[0])
        
        # 算V的模
        V_norm = linalg.norm(V[0])
    
        # 算W和V之间的角度的余弦值
        # 返回参数前对V_norm进行判断是否为0, 防止返回Nan
        # 此时速度太小了, 可能是因为pso固有的缺点, 在极值点处震荡
        # 也可能采样点在建筑物内部, 采用得到的速度的模为0
        if V_norm == 0 or wind_norm == 0:
            chi_theta = 1
        else:
            chi_theta = a/(wind_norm * V_norm)
        return chi_theta

    #乘控制参数
    # V = V*cp(X,W,V)
    
    return V

def position_update(X, V):
    return X + V
    
def velocity_boundary_handle(V, max_val, min_val):       
    V[V < min_val] = min_val
    V[V > max_val] = max_val
    return V

def boundary_handle(M, X, V):
    if X[0][0] < M.map_x_lower:
        X[0][0] = M.map_x_lower
        
    if X[0][1] < M.map_y_lower:
        X[0][1] = M.map_y_lower
        
    if X[0][0] > M.map_x_upper:
        X[0][0] = M.map_x_upper
        
    if X[0][1] > M.map_y_upper:
        X[0][1] = M.map_y_upper        
    
    return X, -V

def normalize(V):
    norm = linalg.norm(V)
    if norm == 0: 
        return V
    else:
        return V / norm
    
def position_initial_random_position(dimension, map_x_lower, map_x_upper, map_y_lower, map_y_upper):
    # 初始化各个粒子的位置
    position = full((1,dimension), 0)
    position[0,0] = random.uniform(map_x_lower+1,  map_x_upper-2) # x坐标(0, x_limit)
    position[0,1] = random.uniform(map_y_lower+1,  map_y_upper-2) # y坐标(0, y_limit)
    # 初始化各个粒子的位置, 并进行障碍物的壁障
    # self.position = self.obstacle.obstacle_avoid_init(self.position, self.population, obstacles_info,  self.x_limit, self.y_limit)
    return position

# 初始化位置
def position_initial_fixed_position(i): 
    if i == 0:
        return array([[335,25]])
    if i == 1:
        return array([[335,25]])
    if i == 2:
        return array([[335,25]])
    if i == 3:
        return array([[335,25]])
    if i == 4:
        return array([[335,25]])
    if i == 5:
        return array([[335,25]])
    if i == 6:
        return array([[335,25]])
    if i == 7:
        return array([[335,25]])
    if i == 8:
        return array([[335,25]])
    if i == 9:
        return array([[335,25]])
    if i == 10:
        return array([[335,25]])
    if i == 11:
        return array([[335,25]])
    if i == 12:
        return array([[335,25]])
    if i == 13:
        return array([[335,25]])
    if i == 14:
        return array([[335,25]])
    if i == 15:
        return array([[335,25]])
    if i == 16:
        return array([[335,25]])
    if i == 17:
        return array([[335,25]])
    if i == 18:
        return array([[335,25]])
    if i == 19:
        return array([[335,25]])
 
def position_initial_fixed_position_around(i): 
    if i == 0:
        return array([[107,302]])
    if i == 1:
        return array([[155,302]])
    if i == 2:
        return array([[214,302]])
    if i == 3:
        return array([[260,302]])
    if i == 4:
        return array([[90,278]])
    if i == 5:
        return array([[90,225]])
    if i == 6:
        return array([[90,166]])
    if i == 7:
        return array([[90,116]])
    if i == 8:
        return array([[107,92]])
    if i == 9:
        return array([[155,92]])
    if i == 10:
        return array([[214,92]])
    if i == 11:
        return array([[335,92]])
    if i == 12:
        return array([[280,116]])
    if i == 13:
        return array([[280,166]])
    if i == 14:
        return array([[280,225]])
    if i == 15:
        return array([[280,278]])
    if i == 16:
        return array([[335,25]])
    if i == 17:
        return array([[335,25]])
    if i == 18:
        return array([[335,25]])
    if i == 19:
        return array([[335,25]])
    
def initial_velocity(dimension, max_val, min_val):   
    velocity = random.uniform(min_val, max_val, size=(1,dimension))
    return velocity

# 判断小生境是否卡在了局部最优
def pending_check(lst):
    # 获取最后五个数
    last_five = array(lst[-5:])
    
    # 检查相邻元素的距离是否小于5
    for i in range(1, len(last_five)):
        if linalg.norm(last_five[i] - last_five[i-1]) >= 5:
            return False
    
    return True
    
def find_func(total_iter, robots_number, niche_radius, response_time, errorness):
    # 真实污染源位置
    source_1 = array([70.7, 193.8])
    source_2 = array([124.5, 263.7])
    distance_error = 50
    source_1_check = []
    source_2_check = []
        
            
    # 一些预处理参数-------------------------------------
    func = Fitness()                 # 实例化适应度函数
    wind = Wind()                    # 实例化风向函数
    # fluent2python = Fluent2python()  # 实例化fluent数据函数
    # uds_t, wind_t, grid_x, grid_y = fluent2python.file_to_array()   # 获取fluent数据
    
    
    # 记录每个小生境步最佳适应度的变化情况
    gbest_fitness_change = list(repeat([], robots_number)) # 生成包含10个列表元素的列表 
    
    # 实例化画图类
    figure = Figure_plot()
    
    # 实例化地图
    M = Map()
    O = Obstacle(rows=M.map_y_upper, cols=M.map_x_upper, dx=M.dx, dy=M.dy)
    obstacles_info = O.obstacle_define(M.obstacle_list) # 获取障碍物数组
    list_old = M.taboo_center.copy() # 判断列表是否发生变化
    taboo_zone = taboo_zone_cal(M)
    
    # 实例化几个无人机出来
    U = []
    for i in range(robots_number):
        U.append(UAV())
        U[i].NO = i
        U[i].position = position_initial_random_position(M.dimension, M.map_x_lower, M.map_x_upper, M.map_y_lower, M.map_y_upper)
        # U[i].position = position_initial_fixed_position_around(i)
        
        U[i].position_history.append(U[i].position.tolist()[0])
        U[i].velocity = initial_velocity(M.dimension, U[i].max_val, U[i].min_val)
        U[i].unit_vector = normalize(U[i].velocity)
    
    # 实例化几个小生境出来, 最多有niche_number个小生境
    N = []
    niche_number = robots_number
    for i in range(niche_number):
        N.append(Niche())
        N[i].NO = i
        
    #% 迭代过程
    exit_flag  = False # 用于直接跳出主循环用的
    success = False  # 用于判断是否溯源成功
    ####################################开始迭代####################################
    for iter_num in (range(1, total_iter+1)): 
        
        uds_t_1  = load(f'data/input_data/1200-5/uds_t_{iter_num*response_time}.npy')
        uds_t =  uds_t_1*(1+random.uniform(-errorness, errorness))
        wind_t = load(f'data/input_data/1200-5/wind_t_{iter_num*response_time}.npy')
        grid_x = load(f'data/input_data/1200-5/grid_x.npy')
        grid_y = load(f'data/input_data/1200-5/grid_y.npy')
        
        if exit_flag:
            break
            
        #---------------检测列表是否反生变化
        list_new = M.taboo_center # 判断列表是否发生变化
        if list_new != list_old:
            taboo_zone = taboo_zone_cal(M)
            list_old = list_new.copy()
        #--------------- 
        
        # =========================================================================
        # 第一阶段, 羽流发现         
        # =========================================================================
        # 循环所有的无人机
        for i in range(robots_number):
            # 计算每个无人机适应度
            U[i].fitness = func.fitness_func(U[i].position, uds_t)
            # 躲避禁区
            U[i].fitness = taboo_zone_avoid_force(U[i].position, U[i].fitness, taboo_zone)
            # 记录历史移动位置
            U[i].position_history.append(U[i].position.tolist()[0])
            U[i].fitness_history.append(U[i].fitness)
            # 羽流检测
            U[i].in_plume = check_plume(func, M, U[i].position, uds_t)                       
            # 转向, 在羽流中时则不再进行此更新
            # if U[i].turning == True and U[i].in_plume == False:
            #     U[i].unit_vector = zigzag(M, U[i].unit_vector, turn_num_2[i])
            #     turn_num_2[i] += 1
    
            # 碰壁检测
            U[i].rebound = check_boundary(M, U[i].position)
            
            # 如果rebound为false, 则执行step_func前进一步, 在羽流中时则不再进行此更新
            if U[i].rebound == False and U[i].in_plume == False:   
                # 记录历史移动位置
                U[i].position = step_func(U[i].position, U[i].unit_vector, U[i].step)
                U[i].position_history.append(U[i].position.tolist()[0])
                # 限制粒子位置
                U[i].position, U[i].velocity = boundary_handle(M, U[i].position, U[i].velocity)
                # 碰壁检测
                U[i].rebound = check_boundary(M, U[i].position)
                
                # 如果前进后碰壁, 则回退一步, 并进行20米的反弹
                if U[i].rebound == True:
                    U[i].unit_vector = -U[i].unit_vector
                    U[i].position = step_func(U[i].position, U[i].unit_vector, 1*U[i].step)
                    # 限制粒子位置
                    U[i].position, U[i].velocity = boundary_handle(M, U[i].position, U[i].velocity)
                    U[i].position_history.append(U[i].position.tolist()[0])
                    # 再随机一个初始方向
                    # V = random.uniform(size=(1,robots.UAVs.dimension))
                    # U[i].unit_vector = normalize(V)
                    
                    # 再旋转45度弹出
                    U[i].unit_vector = turn_45_vector(U[i].unit_vector)
           
        # 循环所有的无人机
        for i in range(robots_number):      
            if U[i].in_niche == False:
                # 先检查周围有没有可加入的小生境, 存入candidate_niche
                candidate_niche = []
                # 计算无人机与各小生境中心的距离是否小于半径
                for j in range(niche_number):
                    if len(N[j].menbers) != 0:
                        distance = linalg.norm(U[i].position - N[j].gbest_position)
                        if distance < niche_radius:
                            N[j].vibration = N[j].gbest_fitness * exp(-distance)
                            candidate_niche.append(N[j])
                # 比较哪个小生境的振动值大, 则加入相应的小生境  
                if len(candidate_niche) > 0:
                    largest_vibration = float('-inf')
                    for j in range(len(candidate_niche)):
                        if(candidate_niche[j].vibration > largest_vibration):
                            largest_vibration = candidate_niche[j].vibration # 寻找最大的振动值
                            largest_vibration_index = candidate_niche[j].NO  # 最大振动值的小生境
                
                    U[i].in_niche = True
                    U[i].belonged_niche_NO = largest_vibration_index
                    U[i].pbest_fitness = U[i].fitness    # 初始化
                    U[i].pbest_position = U[i].position  # 初始化
                    U[i].pworst_fitness = U[i].fitness   # 初始化
                    U[i].pworst_position = U[i].position # 初始化
                
                    
                    N[largest_vibration_index].menbers.append('UAV_' + str(i))
                    N[largest_vibration_index].UAVs_position['UAV_' + str(i)] = U[i].position # UAVs_position应该是每步变化的
                    N[largest_vibration_index].UAVs_fitness['UAV_' + str(i)] = U[i].fitness 
                    N[largest_vibration_index].UAVs_unit_vector['UAV_' + str(i)] = U[i].unit_vector                
            
            # 如果没有可加入的小生境, 每个在羽流内的无人机, 且不在小生境中, 则形成只有自己的小生境
            if U[i].in_niche == False and U[i].in_plume == True:
                U[i].in_niche = True
                U[i].belonged_niche_NO = N[i].NO
                U[i].pbest_position = U[i].position  # 初始化
                U[i].pbest_fitness = U[i].fitness    # 初始化
                U[i].pworst_position = U[i].position # 初始化
                U[i].pworst_fitness = U[i].fitness   # 初始化
                
                N[i].menbers.append('UAV_' + str(i))                       # 将无人机i加入其原本小生境i
                N[i].UAVs_position['UAV_' + str(i)] = U[i].position        # 字典赋值
                N[i].UAVs_fitness['UAV_' + str(i)] = U[i].fitness          # 字典赋值
                N[i].UAVs_unit_vector['UAV_' + str(i)] = U[i].unit_vector  # 字典赋值
                N[i].gbest_position = U[i].position  # 初始化
                N[i].gbest_fitness = U[i].fitness    # 初始化
                
                # 记录
                N[i].gbest_position_history.append(N[i].gbest_position.tolist()[0])
                N[i].gbest_fitness_history.append(N[i].gbest_fitness)
                  
        # 循环所有的小生境, 更新小生境中的参数
        for i in range(niche_number):
            if len(N[i].menbers) != 0: 
                # 确定是否需要进行反向学习
                N[i].in_inverse = pending_check(N[i].gbest_position_history) 
                
                # 更新所有小生境中各个字典的信息
                for menber in N[i].menbers:
                    b = int(re.findall("\d+",menber)[0])
                    N[i].UAVs_position['UAV_' + str(b)] = U[b].position       # UAVs_position应该是每步变化的
                    N[i].UAVs_fitness['UAV_' + str(b)] = U[b].fitness         # UAVs_fitness应该是每步变化的
                    N[i].UAVs_unit_vector['UAV_' + str(b)] = U[b].unit_vector # UAVs_unit_vector应该是每步变化的  
                    
                # 小生境中适应度值最大的无人机的索引
                i_gbest = max(N[i].UAVs_fitness, key=lambda x:N[i].UAVs_fitness[x]) 
                N[i].gbest_fitness = N[i].UAVs_fitness[i_gbest]
                N[i].gbest_position = N[i].UAVs_position[i_gbest]
                N[i].gbest_unit_vector = N[i].UAVs_unit_vector[i_gbest] 
                
    
        # =========================================================================
        # 第二阶段, 羽流追踪        
        # =========================================================================        
        # 接入PSO, 每个niche独立进行PSO  
        for i in range(niche_number):
            # 当小生境中的粒子数<2时, 不进行PSO, 此时进行单机器人搜索, 并利用风向信息逆风搜索
            if len(N[i].menbers) < N[i].gama:
                for menber in N[i].menbers:
                    b = int(re.findall("\d+",menber)[0])
                    U[b].in_pso = False
                    # 计算无人机左右两侧的浓度
                    right_direction = vertical_vector(U[b].unit_vector)
                    left_direction  = -right_direction
                    right_concentration = func.fitness_func((U[b].position + 1 * right_direction), uds_t)
                    left_concentration  = func.fitness_func((U[b].position + 1 *  left_direction), uds_t)
                    # 根据浓度差确定趋化性方向
                    if right_concentration >= left_concentration:
                        chemotaxis_vector = right_direction
                    else:
                        chemotaxis_vector = left_direction
                    # 根据风向确定趋风性方向
                    U[b].wind = wind.wind_func(U[b].position, wind_t)
                    anemotaxis_vector = -normalize(U[b].wind)
                    # 计算合方向速度方向
                    U[b].velocity = 2*chemotaxis_vector + anemotaxis_vector
                    U[b].unit_vector = normalize(U[b].velocity)
                    # 更新粒子位置
                    U[b].position = step_func(U[b].position, U[b].velocity, U[i].step)
                    # 限制粒子位置
                    U[b].position, U[b].velocity = boundary_handle(M, U[b].position, U[b].velocity)
                    # 羽流检测
                    U[b].in_plume = check_plume(func, M, U[b].position, uds_t)
                    if U[b].in_plume == False:
                        U[b].position = array(U[b].position_history[-1]).reshape(1,2)
                        U[b].velocity = -U[b].velocity
                    # 将新位置增加到列表中
                    U[b].position_history.append(U[b].position.tolist()[0])
                    # 计算更新后粒子的适应度
                    U[b].fitness = func.fitness_func(U[b].position, uds_t)
                    # 躲避禁区
                    U[b].fitness = taboo_zone_avoid_force(U[b].position, U[b].fitness, taboo_zone)
                    # 将新适应度增加到列表中
                    U[b].fitness_history.append(U[b].fitness)
    
                    # 小生境中适应度值最大的无人机的索引
                    i_pbest = max(range(len(U[b].fitness_history)), key=lambda x:U[b].fitness_history[x]) 
                    U[b].pbest_fitness = U[b].fitness_history[i_pbest]
                    U[b].pbest_position = U[b].position_history[i_pbest]
                    
                    # 小生境中适应度值最小的无人机的索引
                    i_pworst = min(range(len(U[b].fitness_history)), key=lambda x:U[b].fitness_history[x]) 
                    U[b].pworst_fitness = U[b].fitness_history[i_pworst]
                    U[b].pworst_position = U[b].position_history[i_pworst]                
                                
                
            # 当小生境中的粒子数>=?时, 进行PSO
            else:
                # # 判断小生境是否卡在了局部最优
                # if N[i].in_inverse == True:
                #     # 反向学习PSO
                #     for menber in N[i].menbers:
                #         b = int(re.findall("\d+",menber)[0])
                #         U[b].in_pso = True
                #         U[b].wind = wind.wind_func(U[b].position, wind_t)
                #         # 选择需要反向学习的粒子, 50%
                #         if b%2 ==0: #偶数
                #             # 更新小生境每个无人机的速度
                #             U[b].velocity = inverse_velocity_update(U[b].velocity,
                #                                                     U[b].position,
                #                                                     U[b].pworst_position, 
                #                                                     U[b].c3,
                #                                                     U[b].w)
                # else:
                # 正向学习PSO
                for menber in N[i].menbers:
                    b = int(re.findall("\d+",menber)[0])
                    U[b].in_pso = True
                    U[b].wind = wind.wind_func(U[b].position, wind_t)
                    
                    # 更新小生境每个无人机的速度
                    U[b].velocity = velocity_update(U[b].velocity,
                                                    U[b].position,
                                                    U[b].wind,
                                                    U[b].pbest_position, 
                                                    N[i].gbest_position,
                                                    U[b].c_ini_1,
                                                    U[b].c_end_1,
                                                    U[b].c_ini_2,
                                                    U[b].c_end_2,
                                                    U[b].w,
                                                    iter_num,
                                                    total_iter)
                    # 限制速度大小
                    U[b].velocity = velocity_boundary_handle(U[b].velocity, U[b].max_val, U[b].min_val)
                    # 更新单位向量
                    U[b].unit_vector = normalize(U[b].velocity)
                    
                    # 更新粒子位置
                    U[b].position = position_update(U[b].position, U[b].velocity)
                    # 限制粒子位置
                    U[b].position, U[b].velocity = boundary_handle(M, U[b].position, U[b].velocity)
                    # 将新位置增加到列表中
                    U[b].position_history.append(U[b].position.tolist()[0])
                    # 计算更新后粒子的适应度
                    U[b].fitness = func.fitness_func(U[b].position, uds_t)
                    # 躲避禁区
                    U[b].fitness = taboo_zone_avoid_force(U[b].position, U[b].fitness, taboo_zone)  
                    # 将新适应度增加到列表中
                    U[b].fitness_history.append(U[b].fitness)
    
                    # 更新pbest_fitness
                    if U[b].fitness >= U[b].pbest_fitness:
                        U[b].pbest_fitness = U[b].fitness
                        U[b].pbest_position = U[b].position
    
                    # 更新gbest_fitness
                    if  U[b].fitness >= N[i].gbest_fitness:
                        N[i].gbest_fitness = U[b].fitness
                        N[i].gbest_position = U[b].position
                        
                # 记录
                N[i].gbest_position_history.append(N[i].gbest_position.tolist()[0])
                N[i].gbest_fitness_history.append(N[i].gbest_fitness)
                
        # figure.niche_figure_plot(grid_x, grid_y, uds_t, U, N)
    
        #% 小生境的进化
        # 计算小生境彼此之间的距离
        niche_dist = full((niche_number,niche_number),float('inf'))
        for i in range(niche_number):
            for j in range(niche_number):
                if j == i or len(N[i].menbers) == 0 or len(N[j].menbers) == 0:
                    continue
                niche_dist[j, i] = linalg.norm(N[j].gbest_position - N[i].gbest_position)
                
        # 如果两个小生境距离很小则合并两个小生境              
        for i in range(niche_number):  
            for j in range(niche_number):      
                # 合并就是把U变一下，把N成员改一下  
                if niche_dist[j,i] < niche_radius + niche_radius:
                    # 合并列表和字典都i<-j
                    N[i].menbers = N[j].menbers + N[i].menbers
                    N[i].UAVs_position = dict(N[j].UAVs_position, **N[i].UAVs_position)
                    N[i].UAVs_fitness = dict(N[j].UAVs_fitness, **N[i].UAVs_fitness)
                    N[i].UAVs_unit_vector = dict(N[j].UAVs_unit_vector, **N[i].UAVs_unit_vector)
                    # 把小生境j中的成员属性belonged_niche_NO改为i
                    for menber in N[j].menbers:
                        b = int(re.findall("\d+",menber)[0])
                        U[b].belonged_niche_NO = i
                    # 清空小生境j
                    N[j].menbers = []
                    N[j].UAVs_position = dict()
                    N[j].UAVs_fitness = dict()
                    N[j].UAVs_unit_vector = dict()
    
        # 合并后, 更新每个小生境中的gbest_position和gbest_fitness
        for i in range(niche_number):
            if len(N[i].menbers) != 0:
                i_gbest = max(N[i].UAVs_fitness, key=lambda x:N[i].UAVs_fitness[x])
                N[i].gbest_fitness = N[i].UAVs_fitness[i_gbest]
                N[i].gbest_position = N[i].UAVs_position[i_gbest]
                N[i].gbest_unit_vector = N[i].UAVs_unit_vector[i_gbest]
    
        # 计算小生境的聚集度和允许的最大无人机数量
        for i in range(niche_number):
            if len(N[i].menbers) != 0:
                s_sum = 0 # 相似度之和
                for menber in N[i].menbers:
                    s = 1 - linalg.norm(N[i].UAVs_position[menber] - N[i].gbest_position) / niche_radius
                    s_sum = s_sum + s
                N[i].agregation = s_sum / len(N[i].menbers)          
                n_m = robots_number / M.source_number
                N[i].Ga_size = math.ceil(N[i].alpha * n_m + N[i].beta * len(N[i].menbers) * (1/(1+exp(-n_m)) - 1/(1+exp(-N[i].agregation))))
    
        # 把适度度最差的几个无人机挑出来, 干别的工作,in_plume = False, in_niche = False, 然后将其位置随机分散开干别的工作
        for i in range(niche_number):
            if len(N[i].menbers) != 0: 
                while len(N[i].menbers) > N[i].Ga_size:
                    i_gworst = min(N[i].UAVs_fitness, key=lambda x:N[i].UAVs_fitness[x])
                    N[i].menbers.remove(i_gworst)
                    del N[i].UAVs_position[i_gworst]
                    del N[i].UAVs_fitness[i_gworst]
                    del N[i].UAVs_unit_vector[i_gworst]
                    
                    U[int(re.findall("\d+",i_gworst)[0])].in_plume = False
                    U[int(re.findall("\d+",i_gworst)[0])].in_niche = False
                    U[int(re.findall("\d+",i_gworst)[0])].in_pso = False
                    # 离开小生境的粒子要随机初始化位置和速度
                    U[int(re.findall("\d+",i_gworst)[0])].position = position_initial_random_position(M.dimension, M.map_x_lower, M.map_x_upper, M.map_y_lower, M.map_y_upper)
                    U[int(re.findall("\d+",i_gworst)[0])].velocity = initial_velocity(M.dimension, U[int(re.findall("\d+",i_gworst)[0])].max_val, U[int(re.findall("\d+",i_gworst)[0])].min_val)
                    U[int(re.findall("\d+",i_gworst)[0])].unit_vector = normalize( U[i].velocity)
        
        # =========================================================================
        # 第三阶段, 羽流确认        
        # =========================================================================                              
        # 某个小生境的全局极值在一定迭代步数内变化不大,聚集度够大,适应度够大,则说明已经定位成功 N[i].agregation > N[i].epsilon and
        # 解散, 形成搜索禁区 
        for i in range(niche_number):
            if  len(N[i].menbers) >= N[i].gama and  N[i].gbest_fitness > N[i].C_threshold_g:
                # print(f"污染源坐标为：{N[i].gbest_position}")
                M.taboo_center.append([N[i].gbest_position[0][0],N[i].gbest_position[0][1]])
                
                # 合并距离较近的两个源
                M.taboo_center = merge_arr_within_distance(M.taboo_center, 10)
                                       
                # if len(M.taboo_center) == 2:
                    # exit_flag = True
                
                # 计算每个待解散粒子进入其他小生境的效用
                for menber in N[i].menbers:
                    b = int(re.findall("\d+",menber)[0])
                    # 计算效用最大的小生境
                    max_utilization = float('-inf')
                    max_utilization_niche = float('nan')
                    for j in range(niche_number):
                        if j != i and len(N[j].menbers) != 0: # 找到另一个-有成员的小生境
                            # 首先将粒子进入其他小生境的效用初始化为0
                            need = exp(-N[j].w_1*N[j].gbest_fitness + N[j].w_2*len(N[j].menbers))
                            cost = linalg.norm(N[j].gbest_position - U[b].position) 
                            N[j].utilization = need - cost
                            if N[j].utilization > max_utilization:
                                max_utilization = N[j].utilization
                                max_utilization_niche = N[j].NO
                                
                    # 将该粒子分配至效用最大的小生境            
                    U[b].belonged_niche_NO = max_utilization_niche
                    U[b].in_plume = False
                    U[b].in_pso = False
                    U[b].in_niche = False
                    # 离开小生境的粒子要随机初始化位置和速度
                    U[b].position = position_initial_random_position(M.dimension, M.map_x_lower, M.map_x_upper, M.map_y_lower, M.map_y_upper)
                    U[b].velocity = initial_velocity(M.dimension, U[b].max_val, U[b].min_val)
                    U[b].unit_vector = normalize( U[i].velocity)
                    
                    
                # 更新待加入小生境中列表和字典的信息
                if isinstance(max_utilization_niche, int):
                    N[max_utilization_niche].menbers.append('UAV_' + str(b))
                    N[max_utilization_niche].UAVs_position['UAV_' + str(b)] = U[b].position
                    N[max_utilization_niche].UAVs_fitness['UAV_' + str(b)] = U[b].fitness
                    N[max_utilization_niche].UAVs_unit_vector['UAV_' + str(b)] = U[b].unit_vector
                    
                # 原来的小生境则解散   
                N[i].menbers = []
                N[i].UAVs_position = dict()
                N[i].UAVs_fitness = dict()
                N[i].UAVs_unit_vector = dict()
                N[i].gbest_fitness = float('-inf')
                N[i].gbest_position = array([[float('-inf'),float('-inf')]])
        
        location = array(M.taboo_center)

        for j in range(len(location)):
            source_1_check.append(linalg.norm(source_1 - location[j]))
            source_2_check.append(linalg.norm(source_2 - location[j]))
        
        if len(source_1_check)>0 and len(source_2_check)>0:
            if min(source_1_check) < distance_error and min(source_2_check) < distance_error:
                success = True
                exit_flag = True
           
    # figure.niche_figure_plot(grid_x, grid_y, uds_t, U, N, M, obstacles_info, iter_num, niche_number, robots_number)
    return iter_num, success
    
# figure.figure_plot(grid_x, grid_y, uds_t, wind_t, U)

if __name__ == '__main__':
    iter_num, success = find_func()
    print(f"跑了多少步？{iter_num}，成功了吗？：{success}")
    
    





