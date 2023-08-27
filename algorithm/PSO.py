import sys, os

sys.path.append('F:/ZHHL/TE_Doctor/CASES/case220927/niche_dpso/niche_dpso')

import pandas as pd
from numpy import where, zeros, append, arange, argmax, random, array
from algorithm.obstacle_define_and_avoid import Obstacle
from algorithm.fitness_function import Fitness
from data.input_data.fluent_data_transfer import Fluent2python
import robots.UAVs 

##########
class PSO:
    def __init__(self, **kwargs):
        self.c1 = kwargs.pop('c1', 2)   # 设置个体学习系数
        self.c2 = kwargs.pop('c2', 2)   # 设置全局学习系数
        self.w = kwargs.pop('w', 0.9)   # 设置惯性权重  

        self.population = kwargs.pop('population', 10)          # 初始化粒子群个体数量
        self.iter = 0                                           # 当前迭代位置, 从0开始
        self.iterations = kwargs.pop('iterations', 2000)        # 迭代次数
        self.precision = kwargs.pop('precision', 1e-2)          # 设置收敛精度
        
        self.func = Fitness()                                   # 设置适应度函数
        self.obstacle = Obstacle()                              # 设置障碍物函数
        self.fluent2python = Fluent2python()                    # 设置数据处理函数

        # Use pandas to save csv conveniently
        self.best_solution = pd.Series(index=arange(1, robots.UAVs.dimension + 2), dtype='float64')
        self.best_solution.rename(index={robots.UAVs.dimension + 1: 'Fitness'}, inplace=True)
        self.iter_solution = pd.DataFrame(index=arange(1, self.iterations + 1), columns=arange(1, robots.UAVs.dimension + 2))
        self.iter_solution.rename(columns={robots.UAVs.dimension + 1: 'Fitness'}, inplace=True)
        index = pd.MultiIndex.from_product([arange(1, self.iterations + 1), arange(1, self.population + 1)], names=['Iteration', 'Individual'])
        columns = list(range(robots.UAVs.dimension))
        self.iter_swarm_pos = pd.DataFrame(index=index, columns=columns)   # 记录粒子历史位置

    def initial_position(self, obstacles_info):
        # 初始化各个粒子的位置
        X = zeros([self.population, robots.UAVs.dimension]) # 二维数组
        X[:,0] = random.uniform(robots.UAVs.map_x_lower, robots.UAVs.map_x_upper, size = self.population) # x坐标(lower, upper)
        X[:,1] = random.uniform(robots.UAVs.map_y_lower, robots.UAVs.map_y_upper, size = self.population) # y坐标(lower, upper)
        # 初始化各个粒子的位置, 并进行障碍物的壁障
        X = self.obstacle.obstacle_avoid_init(X, self.population, obstacles_info,  robots.UAVs.map_x_upper, robots.UAVs.map_y_upper)
        return X

    def update_previous_best(self, particle_pos, particle_fit, pbest_pos, pbest_fit):
        i_pbest = where(pbest_fit > particle_fit)
        pbest_pos[i_pbest], pbest_fit[i_pbest] = particle_pos[i_pbest], particle_fit[i_pbest]
        return pbest_pos, pbest_fit

    def update_velocity(self, velocity, particle_pos, pbest_pos, gbest_pos):
        r1 = random.random((self.population, 1)) #该函数表示成population行 1列的浮点数，浮点数都是从0-1中随机。
        r2 = random.random((self.population, 1))   
        return self.w * velocity + self.c1 * r1 * (pbest_pos - particle_pos) + self.c2 * r2 * (gbest_pos - particle_pos)

    def velocity_boundary_handle(self, x, lower, upper):
        ir = where(x < lower)
        x[ir] = lower
        ir = where(x > upper)
        x[ir] = upper
        return x

    def boundary_handle(self, x):
        ix = where(x[:,0] < robots.UAVs.map_x_lower)
        x[ix,0] = 0
        ix = where(x[:,0] > robots.UAVs.map_x_upper)
        x[ix,0] = robots.UAVs.map_x_upper    
        iy = where(x[:,1] < robots.UAVs.map_y_lower)
        x[iy,1] = 0
        iy = where(x[:,1] > robots.UAVs.map_y_upper)
        x[iy,1] = robots.UAVs.map_y_upper
        return x

    def update_position(self, particle_pos, velocity):
        return particle_pos + velocity

    def run(self, obstacle_list):
        uds_t, wind_t, grid_x, grid_y = self.fluent2python.file_to_array()   # 运行一次就够了
        obstacles_info = self.obstacle.obstacle_define(obstacle_list)        # 获取障碍物数组
        particle_pos = self.initial_position(obstacles_info)                 # 位置初始化
        particle_fit = self.func.fitness_func(particle_pos, uds_t)           # 计算每个粒子的适应度
        i_best = argmax(particle_fit)                                        # 最大适应度位置索引
        pbest_pos, pbest_fit = particle_pos, particle_fit                    # 种群个体初始化
        gbest_pos, gbest_fit = particle_pos[i_best], particle_fit[i_best]    # 最佳粒子初始化
        velocity = zeros([self.population, robots.UAVs.dimension])
        self.iter = 0
        while self.iter < self.iterations:
            self.iter += 1
            self.iter_swarm_pos.loc[self.iter] = particle_pos
            self.iter_solution.loc[self.iter] = append(gbest_pos, gbest_fit)

            pbest_pos, pbest_fit = self.update_previous_best(particle_pos, particle_fit, pbest_pos, pbest_fit)
            
            velocity = self.update_velocity(velocity, particle_pos, pbest_pos, gbest_pos)
            velocity = self.velocity_boundary_handle(velocity, robots.UAVs.min_val, robots.UAVs.max_val)

            particle_pos = self.update_position(particle_pos, velocity)
            particle_pos = self.boundary_handle(particle_pos)

            particle_fit = self.func.fitness_func(particle_pos, uds_t)

            i_best = argmax(particle_fit)

            if particle_fit[i_best] > gbest_fit:
                gbest_pos, gbest_fit = particle_pos[i_best], particle_fit[i_best]

        self.best_solution.iloc[:] = append(gbest_pos, gbest_fit)    
        return gbest_pos, gbest_fit
        
if __name__ == '__main__':
    
    # 障碍物位置列表
    obstacle_list = array([
                             # [250, 250],
                             # [500, 250],
                             # [750, 250],
                             # [1000, 250],
                             # [250, 500],
                             # [500, 500],
                             # [750, 500],
                             # [1000, 500],
                             # [250, 750],
                             # [500, 750],
                             # [750, 750],
                             # [1000, 750]
                          ])
    pso = PSO()
    best_sol, best_val = pso.run(obstacle_list)
    print(pso.iter)