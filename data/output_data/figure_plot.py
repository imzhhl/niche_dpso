import matplotlib.pyplot as plt
from numpy import array
import re
import matplotlib
import matplotlib.animation as animation
from matplotlib.animation import FuncAnimation
from matplotlib.colors import LinearSegmentedColormap
import robots.UAVs
plt.ion()# 打开交互模式

matplotlib.rc("font", family='Microsoft YaHei')
font = {'family' : 'Times New Roman',
'weight' : 'normal',
'size'   : 18,
}

class Figure_plot:
    
    def __init__(self):
        pass
    
    def test_plot(self,  grid_x, grid_y, uds_t, wind_t, U):
        fig = plt.figure(figsize=(16,8))
        ax = fig.add_subplot(1,1,1)
        for i in range(robots.UAVs.robots_number):
            ax.plot(array(U[i].position_history)[:,0], array(U[i].position_history)[:,1])
            ax.scatter(array(U[i].position)[0,0], array(U[i].position)[0,1],marker='o')
        # 等值线图    
        matplotlib.pyplot.contour(grid_x, grid_y, uds_t[0,:,:],
                                  colors=list(["purple","blue","cyan", "green","black","orange","red"]),
                                  levels = [0.03, 0.04, 0.05, 0.06, 0.08, 0.17], linestyles=['-'])
        plt.xlim(0, 371)
        plt.ylim(0, 393)
        # ax.set_xlabel("x")
        # ax.set_ylabel("y")
        ax.set_title("zigzag搜索图", y=-0.2) 
        
    
    def figure_plot(self, grid_x, grid_y, uds_t, wind_t, U):
        
        fig = plt.figure(figsize=(16,12))
        
        # 污染物扩散二维等值线图
        ax = fig.add_subplot(3,2,1)
        matplotlib.pyplot.contour(grid_x, grid_y, uds_t[0,:,:],
                                  colors=list(["purple","blue","cyan", "green","black","orange","red"]),
                                  levels = [0.03, 0.04, 0.05, 0.06, 0.08, 0.17], linestyles=['-'])
        # ax.set_xlabel("x")
        # ax.set_ylabel("y")
        ax.set_title("污染物浓度场的分布", y=-0.2) 
        # ---------------------------------------------------------------------
        
        
        # 流线图
        ax = fig.add_subplot(3,2,2)
        # 确定wind_U和wind_V
        wind_U = wind_t[:,:,0]
        wind_V = wind_t[:,:,1]
        ax.streamplot(grid_x, grid_y, wind_U, wind_V, density=0.8, linewidth=1.5, color="blue", arrowsize=1.5)
        # ax.set_xlabel("x")
        # ax.set_ylabel("y")
        ax.set_title("流线图", y=-0.2)  
        # ---------------------------------------------------------------------
        
        
        # zigzag搜索图
        ax = fig.add_subplot(3,2,3)

        for i in range(robots.UAVs.robots_number):
            ax.plot(array(U[i].position_history)[:,0], array(U[i].position_history)[:,1])

        # 等值线图    
        matplotlib.pyplot.contour(grid_x, grid_y, uds_t[0,:,:],
                                  colors=list(["purple","blue","cyan", "green","black","orange","red"]),
                                  levels = [0.03, 0.04, 0.05, 0.06, 0.08, 0.17], linestyles=['-'])
        plt.xlim(0, 371)
        plt.ylim(0, 393)
        # ax.set_xlabel("x")
        # ax.set_ylabel("y")
        ax.set_title("zigzag搜索图", y=-0.2) 
        # ---------------------------------------------------------------------
        
        
        # 小生境无人机分布
        ax = fig.add_subplot(3,2,4)
        particles = []
        for i in range(robots.UAVs.robots_number):
            particles.append(U[i].position.tolist()[0])
        particles = array(particles)
        plt.scatter(particles[:,0], particles[:,1])  
        matplotlib.pyplot.contour(grid_x, grid_y, uds_t[0,:,:],
                                  colors=list(["purple","blue","cyan", "green","black","orange","red"]),
                                  levels = [0.03, 0.04, 0.05, 0.06, 0.08, 0.17], linestyles=['-'])
        plt.xlim(0, 371)
        plt.ylim(0, 393)
        # ax.set_xlabel("x")
        # ax.set_ylabel("y")
        ax.set_title("粒子分布图", y=-0.2) 
        # ---------------------------------------------------------------------
    
        
        # PSO迭代过程浓度值变化
        ax = fig.add_subplot(3,2,5)
        for i in range(robots.UAVs.robots_number):
            ax.plot(U[i].fitness_history)
        # ax.set_xlabel("x")
        # ax.set_ylabel("y")
        ax.set_title("PSO迭代过程", y=-0.2)  
        
        # plt.subplots_adjust(left=0.2,
        #             bottom=0.2,
        #             right=0.2,
        #             top=0.2,
        #             wspace=0.2,
        #             hspace=0.5)
    
    def niche_figure_plot(self, grid_x, grid_y, uds_t, U, N, M, obstacles_info, iter_num, niche_number, robots_number):
        plt.cla() # Clear the current figure
        # for i in range(robots.UAVs.robots_number):
        #     plt.plot(array(U[i].position_history)[:,0], array(U[i].position_history)[:,1], label=f"niche_{i}")
        # for i in range(robots.UAVs.niche_number): 
        #     if len(N[i].menbers) != 0: 
        #         plt.plot(array(N[i].gbest_position_history)[:,0], array(N[i].gbest_position_history)[:,1], label=f"g_best{i}")
        
        plt.contour(grid_x, grid_y, uds_t[0, :, :],
                                  colors=list(["purple","blue","cyan", "green","orange","red"]),
                                  levels = [0.01, 0.04, 0.05, 0.06, 0.08, 0.17], linestyles=['-'])
        # manager = plt.get_current_fig_manager()
        # manager.window.showMaximized() 
        
        for i in range(niche_number):
            if len(N[i].menbers) != 0: 
                for menber in N[i].menbers:
                    b = int(re.findall("\d+",menber)[0])
                    if U[b].in_pso == False: # 虽然在小生境中, 但未在PSO过程中
                        # plt.plot(array(U[b].position_history)[:,0], array(U[b].position_history)[:,1], color = 'green')
                        plt.scatter(U[b].position[0][0], U[b].position[0][1], marker = '.', s = 300, c = 'red', label = f"SINGLE_No. {b}")

        for i in range(niche_number):
            if len(N[i].menbers) != 0 and U[int(re.findall("\d+",N[i].menbers[0])[0])].in_pso == True:  # 在小生境中, 在PSO过程中
                num = len(N[i].menbers)
                plt.scatter(N[i].gbest_position_history[-1][0], N[i].gbest_position_history[-1][1], marker = '*', c = 'red', s = 300, label = f"NICHE_{i}_Include: {num}UAVs")
                # plt.plot(array(N[i].gbest_position_history)[:,0], array(N[i].gbest_position_history)[:,1],linewidth = 1.0)
                

        for b in range(robots_number):
            if U[b].in_niche == False: # 未在小生境中, 未在PSO过程中
                # plt.plot(array(U[b].position_history)[:,0], array(U[b].position_history)[:,1], color = 'green')
                plt.scatter(U[b].position[0][0], U[b].position[0][1], marker = '.', s = 300, c = 'green', label = f"SINGLE_No. {U[b].NO}")


               
        if len(M.taboo_center) != 0: # 如果已经形成了禁区
            for i in range(len(M.taboo_center)):
                circle = plt.Circle((M.taboo_center[i][0], M.taboo_center[i][1]),M.taboo_radius, edgecolor='black',facecolor='red', alpha = 0.5)
                plt.gca().add_patch(circle)
        
        # plt.contourf(grid_x, grid_y, uds_t[0,:,:], cmap='Greys',alpha=0.5)
        plt.imshow(obstacles_info,'Greys')
        plt.axis ([0, M.map_x_upper, 0, M.map_y_upper])
        
        plt.xlim(M.map_x_lower, M.map_x_upper)
        plt.ylim(M.map_y_lower, M.map_y_upper)
        plt.xlabel("x")
        plt.ylabel("y")
        plt.title(f"UAVs搜索过程, current T = {iter_num}s", fontsize=20) 
        plt.legend(loc=2, facecolor='w', framealpha=0.5)
        plt.draw()
        plt.pause(0.01)
    
