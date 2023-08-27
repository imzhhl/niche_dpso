import sys 
import os
from numpy import array, full, random, linalg, pi

# ---------------------------------------------------------------
# total_iter = 300             # 迭代次数
# robots_number = 16           # 机器人的数量
# niche_number = robots_number # 最大小生境的数量
# response_time = 1            # 响应时间
# errorness = 0.01             # 传感器误差
# ---------------------------------------------------------------
 
class Map:
    def __init__(self):
        self.source_number = 2            # 污染源个数
        self.C_threshold = 0.01           # 进入污染羽流的阈值
        self.dimension = 2                # 搜索域的维度
        self.map_x_lower = 0              # 计算域x下限
        self.map_x_upper = 371            # 计算域x上限
        self.map_y_lower = 0              # 计算域y下限
        self.map_y_upper = 393            # 计算域y上限               
        self.sigma =  pi*5/6              # 翻折角度, 120度
        self.taboo_radius = 10            # 禁忌区半径
        self.taboo_center = []            # 初始化禁忌区的位置
        # 障碍物位置列表
        self.dx = 1.21
        self.dy = 3.95
        self.obstacle_list = array([
                                    [101.243, 106.1],
                                    [116.563, 106.1],
                                    [131.383, 106.1],
                                    [147.503, 106.1],
                                    [162.523, 106.1],
                                    [177.543, 106.1],
                                    [193.463, 106.1],
                                    [208.483, 106.1],
                                    [223.503, 106.1],
                                    [239.423, 106.1],
                                    [254.443, 106.1],
                                    [269.763, 106.1],
                                    [101.243, 126.2],
                                    [116.563, 126.2],
                                    [131.383, 126.2],
                                    [147.503, 126.2],
                                    [162.523, 126.2],
                                    [177.543, 126.2],
                                    [193.463, 126.2],
                                    [208.483, 126.2],
                                    [223.503, 126.2],
                                    [239.423, 126.2],
                                    [254.443, 126.2],
                                    [269.763, 126.2],
                                    [101.243, 146.3],
                                    [116.563, 146.3],
                                    [131.383, 146.3],
                                    [147.503, 146.3],
                                    [162.523, 146.3],
                                    [177.543, 146.3],
                                    [193.463, 146.3],
                                    [208.483, 146.3],
                                    [223.503, 146.3],
                                    [239.423, 146.3],
                                    [254.443, 146.3],
                                    [269.763, 146.3],
                                    [101.243, 166.4],
                                    [116.563, 166.4],
                                    [131.383, 166.4],
                                    [147.503, 166.4],
                                    [162.523, 166.4],
                                    [177.543, 166.4],
                                    [193.463, 166.4],
                                    [208.483, 166.4],
                                    [223.503, 166.4],
                                    [239.423, 166.4],
                                    [254.443, 166.4],
                                    [269.763, 166.4],
                                    [101.243, 186.5],
                                    [116.563, 186.5],
                                    [131.383, 186.5],
                                    [147.503, 186.5],
                                    [162.523, 186.5],
                                    [177.543, 186.5],
                                    [193.463, 186.5],
                                    [208.483, 186.5],
                                    [223.503, 186.5],
                                    [239.423, 186.5],
                                    [254.443, 186.5],
                                    [269.763, 186.5],
                                    [101.243, 206.6],
                                    [116.563, 206.6],
                                    [131.383, 206.6],
                                    [147.503, 206.6],
                                    [162.523, 206.6],
                                    [177.543, 206.6],
                                    [193.463, 206.6],
                                    [208.483, 206.6],
                                    [223.503, 206.6],
                                    [239.423, 206.6],
                                    [254.443, 206.6],
                                    [269.763, 206.6],
                                    [101.243, 226.7],
                                    [116.563, 226.7],
                                    [131.383, 226.7],
                                    [147.503, 226.7],
                                    [162.523, 226.7],
                                    [177.543, 226.7],
                                    [193.463, 226.7],
                                    [208.483, 226.7],
                                    [223.503, 226.7],
                                    [239.423, 226.7],
                                    [254.443, 226.7],
                                    [269.763, 226.7],
                                    [101.243, 246.8],
                                    [116.563, 246.8],
                                    [131.383, 246.8],
                                    [147.503, 246.8],
                                    [162.523, 246.8],
                                    [177.543, 246.8],
                                    [193.463, 246.8],
                                    [208.483, 246.8],
                                    [223.503, 246.8],
                                    [239.423, 246.8],
                                    [254.443, 246.8],
                                    [269.763, 246.8],
                                    [101.243, 266.9],
                                    [116.563, 266.9],
                                    [131.383, 266.9],
                                    [147.503, 266.9],
                                    [162.523, 266.9],
                                    [177.543, 266.9],
                                    [193.463, 266.9],
                                    [208.483, 266.9],
                                    [223.503, 266.9],
                                    [239.423, 266.9],
                                    [254.443, 266.9],
                                    [269.763, 266.9],
                                    [101.243, 287.3],
                                    [116.563, 287.3],
                                    [131.383, 287.3],
                                    [147.503, 287.3],
                                    [162.523, 287.3],
                                    [177.543, 287.3],
                                    [193.463, 287.3],
                                    [208.483, 287.3],
                                    [223.503, 287.3],
                                    [239.423, 287.3],
                                    [254.443, 287.3],
                                    [269.763, 287.3]])
    
class UAV:
    def __init__(self):
        self.step = 5                     # 机器人的步长, m

        self.NO = float('3.0')
        self.belonged_niche_NO = float('nan')
        
        self.max_val = 20
        self.min_val = -20
        
        self.c_ini_1 = 2.5
        self.c_end_1 = 0.5
        self.c_ini_2 = 0.5
        self.c_end_2 = 2.5
        
        self.w = 0.9 # 设置惯性权重
        self.c1 = 2  # 设置个体学习系数
        self.c2 = 2  # 设置全局学习系数
        self.c3 = 0.7# 设置个体反向学习系数
        self.c4 = 0.3# 设置全局反向学习系数
        
        self.wind = array([[float('-inf'),float('-inf')]]) # 粒子的风速风向信息
        
        self.fitness = float('nan')
        self.fitness_history = []
        self.position = array([[float('-inf'),float('-inf')]])
        self.position_history = []
        self.velocity = array([[float('-inf'),float('-inf')]])
        self.unit_vector = array([[float('-inf'),float('-inf')]])
        
        self.pworst_fitness = float(0)  # 粒子的最差适应度      
        self.pbest_fitness  = float(0)  # 粒子的最佳适应度
        self.pworst_position = array([[float('-inf'),float('-inf')]]) # 粒子的最差位置
        self.pbest_position = array([[float('-inf'),float('-inf')]])  # 粒子的最佳位置
        
        self.in_plume = False        
        self.in_niche = False 
        self.in_pso = False #是否在进行PSO
        self.rebound = False
        self.turning = False
        
class Niche:
    def __init__(self):
        
        self.NO = float('nan')
        self.menbers = []    # 小生境成员, 通过len(NICHE[i].menbers) != 0判断小生境是否被激活
        self.gama = 3 # 进入pso过程的机器人数量
        
        self.in_inverse = False # 是否进行反向学习
        
        self.UAVs_position = dict()           # 所有粒子的位置, 键: UAV_1, 值: array
        self.UAVs_fitness  = dict()           # 所有粒子的适应度
        self.UAVs_unit_vector = dict()        # 所有粒子的单位向量
               
        self.gbest_fitness  = float('-inf')   # 种群的最佳适应度
        self.gworst_fitness = float('-inf')   # 种群的最差适应度
        self.gbest_fitness_history = []
        
        self.gworst_position = array([[float('-inf'),float('-inf')]])    # 最差粒子的位置
        self.gbest_position  = array([[float('-inf'),float('-inf')]])    # 最佳粒子的位置
        self.gbest_position_history = []
        
        self.gbest_unit_vector = array([[float('-inf'),float('-inf')]]) # 最佳粒子的向量, 合并是时候用

        self.alpha = 0.5                # 控制参数
        self.beta = 0.9                 # 控制参数
        self.epsilon = 0.3              # 聚集度收敛判断
        self.C_threshold_g = 0.08       # 浓度收敛判断
        
        self.vibration = float('nan')   # 小生境的振动值       
        self.agregation = float('nan')  # 小生境的聚集度
        self.Ga_size = float('nan')     # 小生境中最多的机器人数量
        
        self.w_1 = 0.4                  # 效用分析参数
        self.w_2 = 0.6                  # 效用分析参数
        self.utilization = float('-inf')# 粒子的效用