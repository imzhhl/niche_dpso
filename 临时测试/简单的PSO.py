import numpy as np
import matplotlib.pyplot as plt
import matplotlib
import csv
from scipy.interpolate import griddata
import math

matplotlib.rc("font", family='Microsoft YaHei')

# %%
# 从fluent中导出的数据导入到numpy数组中
def file_to_array():
    #x,y的坐标
    x=[]; y=[]
    #创建uds为一维数组
    uds = np.zeros(1)
    #创建W为二维数组
    wind = np.zeros((1,2)) 
    #创建两个列表
    uds_t = []  #浓度
    wind_t = []  #风向
    
    with open(r'C:\Users\zhhl_\Desktop\fft07\fluent11','r')  as  csvfile:   
        #指定分隔符为","
        plots=csv.reader(csvfile,delimiter=',')
        #循环读取文件各列
        for row in plots:
            #跳过文件前面的非数据行  
            if plots.line_num == 1:
                continue
            # 读取x和y坐标和风向
            x.append(float(row[1]))
            y.append(float(row[2]))
            #读取风向
            wind = np.append(wind,[[float(row[3]),float(row[4])]],axis=0)
            #读取浓度
            uds = np.append(uds,float(row[5]))
    
    #删掉第0个元素
    wind = np.delete(wind,0,axis=0)
    uds = np.delete(uds,0)
  
    # 如果数值过小，则视为0，防止出现非数的情况
    for i in range(len(y)):
        if uds[i] < 1e-10:
            uds[i] =0.0
    
    # 确定meshgrid的坐标范围，并离散坐标        
    xi=np.linspace(min(x),max(x), 1000)
    yi=np.linspace(min(y),max(y), 1000)
    
    # grid_x,grid_y坐标必须维数一致，且为二维
    global grid_x, grid_y
    grid_x, grid_y = np.meshgrid(xi, yi)
    
    # griddata插值，并存入uds_t列表中，列表中的元素为二维数组
    uds_t.append(griddata((x,y), uds, (grid_x,grid_y), method='linear',
                          fill_value = np.nan, rescale = False))
    # griddata插值，并存入W_t列表中，列表中的元素为三维数组
    wind_t.append(griddata((x,y), wind, (grid_x,grid_y), method='linear',
                          fill_value = np.nan, rescale = False))
    
    # 将uds_t转换为三维np数组
    uds_t = np.array(uds_t)
    
    # 将W_t转换为三维np数组
    wind_t = np.array(wind_t)
    wind_t = np.reshape(wind_t,(1000,1000,2))
    
    # 保存到文件中，下次使用是直接读取文件，避免重复上述过程(太慢！)
    np.save('uds_adjoint.npy', uds_t)
    np.save('wind.npy', wind_t)
    return (uds_t, wind_t)

#%% 显示图

def figure_plot():
    font = {'family' : 'Times New Roman',
    'weight' : 'normal',
    'size'   : 18,
    }
    
    fig = plt.figure(figsize=(10,10))
    
    ax = fig.add_subplot(2,2,1)
    # 污染物扩散二维等值线图
    matplotlib.pyplot.contour(grid_x, grid_y, uds_t[0,:,:],
                              colors=list(["purple","blue","cyan", "green","yellow","orange","red"]),
                              levels = [0.001, 0.05, 0.1, 0.2, 0.3, 0.4], linestyles=['-'])
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_title("污染物浓度场的分布")  
  
    
    ax = fig.add_subplot(2,2,2)
    # make a stream function:
    # 确定wind_U和wind_V
    wind_U = wind_t[:,:,0]
    wind_V = wind_t[:,:,1]
    ax.streamplot(grid_x, grid_y, wind_U, wind_V, density=0.8, linewidth=1.5, color="blue", arrowsize=1.5)
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_title("流线图")  


    ax = fig.add_subplot(2,2,3)
    ax.plot(X_list[:,0], X_list[:,1], label="Path", color="blue")
    ax.plot(X_list[:,0], X_list[:,1], 'b.', label='position')
    plt.xlim(0, 1000)
    plt.ylim(0, 1000)
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.legend()
    ax.set_title("UAVs搜索过程")  
    
    
    ax = fig.add_subplot(2,2,4)
    ax.plot(fitness_val_list[:], label="concentration value", color="blue")
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.legend()
    ax.set_title("PSO迭代过程")  
     
#%% PSO

##########
#求每个点的目标函数
def fitness_func(X):
    x = X[:,0]
    y = X[:,1]
    
    #二维线性插值
    z = np.zeros(X.shape[0]) 
    for i in range(len(x)):
        a=math.floor(x[i])
        b=math.floor(y[i])
        z1=uds_t[0,a,b]+(uds_t[0,a+1,b]-uds_t[0,a,b])*(x[i]-a)
        z2=uds_t[0,a,b+1]+(uds_t[0,a+1,b+1]-uds_t[0,a,b+1])*(x[i]-a)
        z[i]=z1+(z2-z1)*(y[i]-b)
    return z  #一维数组，保存了各粒子位置插值得到的污染物浓度值

##########
#求每个点的风向（二维数组）
def wind_func(X):
    x = X[:,0]
    y = X[:,1]
    #二维线性插值
    z = np.zeros((len(x),2)) 
    for i in range(len(x)):
        a=math.floor(x[i])
        b=math.floor(y[i])
        z1=wind_t[a,b,:]+(wind_t[a+1,b,:]-wind_t[a,b,:])*(x[i]-a)
        z2=wind_t[a,b+1,:]+(wind_t[a+1,b+1,:]-wind_t[a,b+1,:])*(x[i]-a)
        z[i]=z1+(z2-z1)*(y[i]-b)
    return z  #二维数组，保存了各粒子位置插值得到的U和V速度

##########
#更新速度，根据公式V(t+1)=w*V(t)+c1*r1*(pbest_i-xi)+c1*r1*(gbest_xi)
def velocity_update(V,X,W,pbest,gbest,c1,c2,w,max_val):
    size = X.shape[0]#返回矩阵X的行数
    r1 = np.random.random((size,1))#该函数表示成size行 1列的浮点数，浮点数都是从0-1中随机。
    r2 = np.random.random((size,1))   
    V = w*V + c1*r1*(pbest-X)+c2*r2*(gbest-X)#注意这里得到的是一个矩阵,列数等于未知量个数
    #乘控制参数
    V = V*cp(X,W,V)
    #这里是一个防止速度过大的处理，怕错过最理想值
    V[V<-max_val] = -max_val
    V[V>-max_val] = max_val
    return V

##########
# wind utilization controled by parameter "cp" (chi_theta)
def cp(X,W,V):  #控制参数
    size = X.shape[0]#返回矩阵X的行数,粒子的个数
    #算W和V的内积，粒子飞行速度和风场风向的内积
    a = np.zeros((size,1))
    for i in range(size):
        a[i]=np.dot(W[i, :], V[i, :])

    #算W的模
    W1=np.zeros((size,1))
    for i in range(size):
        W1[i]=math.sqrt(W[i][0]**2+W[i][1]**2)
        
    #算V的模
    V1=np.zeros((size,1))
    for i in range(size):
        V1[i]=math.sqrt(V[i][0]**2+V[i][1]**2)

    #算W和V之间的角度的余弦值
    cos=a/(W1*V1)
    
    #返回参数
    return 0.5*(1-cos)
 
##########
#更新粒子位置，根据公式X(t+1)=X(t)+V
def position_updata(X,V):
    return X+V
 
#########
def PSO():
    w = 0.9 #设置惯性权重
    c1 = 2  #设置个体学习系数
    c2 = 2  #设置全局学习系数
    dim = 2 #计算域维度
    size = 2 #初始化粒子群个体数量
    iter_num = 2000 #迭代次数
    max_val = 1 #限定最大速度

    fitness_val_list = [] #全局最理想的适应度值的变化，用于绘图
    
    #初始化各个粒子的位置
    X = np.random.uniform(0,999,size=(size,dim))
    #初始化各个粒子的速度
    V = np.random.uniform(-1,1,size=(size,dim))
    
    #风向
    W=wind_func(X)
    
    p_fitness = fitness_func(X) #得到各个个体的适应度值
    g_fitness = p_fitness.max() #全局最理想的适应度值
    
    pbest = X #初始化个体的最优位置
    gbest = X[p_fitness.argmax()] #初始化整个整体的最优位置
      
    X_list=np.array(gbest)#全局最理想的适应度值对应坐标位置的变化，用于绘图
    X_list = X_list.reshape(1, 2)
    
    #迭代
    for i in range(iter_num):
        V = velocity_update(V, X, W, pbest, gbest, c1, c2, w, max_val)
        X = position_updata(X, V)
        p_fitness2 = fitness_func(X)
        g_fitness2 = p_fitness2.max()
        
        #更新每个粒子的历史的最优位置和种群的全局最优位置
        for j in range(size):
            if p_fitness[j] < p_fitness2[j]:
                pbest[j] = X[j]
                p_fitness[j] = p_fitness2[j]
                
            if g_fitness < g_fitness2:
                gbest = X[p_fitness2.argmax()]
                g_fitness = g_fitness2
                
                fitness_val_list.append(g_fitness)
                gbest2 = gbest.reshape(1, 2)
                X_list = np.append(X_list, gbest2, axis=0)

    print("最优值是：%.5f" % fitness_val_list[-1])
    print("最优解是：x=%.5f,y=%.5f" % (gbest[0],gbest[1]))
    return fitness_val_list, X_list

#%% 运行函数并绘图
uds_t = file_to_array()[0]   #浓度
wind_t = file_to_array()[1]  #风向

fitness_val_list, X_list = PSO()

# 绘图
figure_plot()
