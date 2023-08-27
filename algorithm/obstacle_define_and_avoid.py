import numpy as np
import math

class Obstacle:
    def __init__(self, **kwargs):
        self.rows = kwargs.pop('rows',393) # rows 为区域划分的行数, 对应y坐标
        self.cols = kwargs.pop('cols',371) # cols 为列数, 对应x坐标
        self.dx = kwargs.pop('dx',1.21) # dx 为固块长的一半
        self.dy = kwargs.pop('dy',3.95) # dx 为固块宽的一半
        
    def obstacle_define(self, obstacle_list): 
        obstacles_info = np.zeros((self.rows, self.cols))
        for i in range(obstacle_list.shape[0]):
            obstacles_info[math.floor(obstacle_list[i, 1]-self.dy ): math.ceil(obstacle_list[i, 1]+self.dy), math.floor(obstacle_list[i, 0]-self.dx) :  math.ceil(obstacle_list[i, 0]+self.dx)] = True
        return obstacles_info
    
    def obstacle_avoid_init(self, X, population, obstacles_info, x_limit, y_limit):
        for i in range(population):
            ix = np.rint( X[:,0] ).astype(int)
            iy = np.rint( X[:,1] ).astype(int)
            while obstacles_info[iy[i], ix[i]] == 1:
                
                X[i,0] = np.random.uniform(0,x_limit,size=(1)) # x坐标(0, 2000)
                X[i,1] = np.random.uniform(0,y_limit,size=(1)) # y坐标(0, 1000)
                
                ix = np.rint( X[:,0] ).astype(int)
                iy = np.rint( X[:,1] ).astype(int)
        return X
    
    #TODO 利用斥力方法实现
    # BUG 壁障速度设为0的话, 后面有分母为0的可能
    def obstacle_avoid(self, obstacles_info, X, V, x_limit, y_limit):
                     
        # 判断粒子的下一步是否在障碍物内部, 并计算距离四个边界的最小距离   
        X_new = X + V
        
        # 做防止X_new超计算域的处理
        if X_new[0, 0] <= 0:
            X_new[0, 0] = 0
            
        if X_new[0, 0] >= self.x_limit:
            X_new[0, 0] = self.x_limit-1
            
        if X_new[0, 1] <= 0:
            X_new[0, 1] = 0
            
        if X_new[0, 1] >= self.y_limit:
            X_new[0, 1] = self.y_limit-1
        
        # 生成和V相同像形状的数组, 用于存储修正后速度X_new
        V_fixed = np.zeros_like(V)
        
        # 判断粒子的下一位置是否在障碍物内
        if obstacles_info[int(X_new[0, 1]), int(X_new[0, 0])] == 1:
            temp_x_l = X_new[0, 0]
            temp_x_r = X_new[0, 0]
            temp_y_t = X_new[0, 1]
            temp_y_b = X_new[0, 1]
            
            # 当前X坐标距离左右下上边的距离, 分别存储于distance_left, distance_right, distance_bottom, distance_top
            distance_left = 0
            distance_right = 0
            distance_bottom = 0
            distance_top = 0
            
            # 计算距离障碍物左边的距离
            while(obstacles_info[int(X_new[0, 1]), int(temp_x_l)]== 1):
                temp_x_l = temp_x_l - 1
                distance_left = distance_left + 1
                
            while(obstacles_info[int(X_new[0, 1]), int(temp_x_r)] == 1):
                temp_x_r = temp_x_r + 1
                distance_right = distance_right + 1
            
            while(obstacles_info[int(temp_y_b), int(X_new[0, 0])] == 1):
                temp_y_b = temp_y_b - 1
                distance_bottom = distance_bottom + 1  
                
            while(obstacles_info[int(temp_y_t), int(X_new[0, 0])] == 1):
                temp_y_t = temp_y_t + 1
                distance_top = distance_top + 1

            distance = np.array([distance_top, distance_bottom, distance_left, distance_right])
            direction_index = np.argmin(distance)           
            min_distance = distance[direction_index]
            
            # 根据障碍物中点距离哪条边最近, 而确定修正法向量的方向, 即N
            if direction_index == 0:
                vector = np.array([0,  1]).reshape(2,1)
            if direction_index == 1:
                vector = np.array([0, -1]).reshape(2,1)
            if direction_index == 2:
                vector = np.array([-1, 0]).reshape(2,1)
            if direction_index == 3:
                vector = np.array([1,  0]).reshape(2,1)
            
            V_fixed = V + (abs(np.dot(V, vector))*vector).reshape(1,2)
             
        else:
            V_fixed = V
                
        return V_fixed 
    
