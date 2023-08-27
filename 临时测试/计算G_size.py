import matplotlib.pyplot as plt
import numpy as np 
alpha=0.5
beta=0.9
n_m= np.array([0,1,2,3,4,5,6,7,8,9,10])
N_a=np.array([0,1,2,3,4,5,6,7,8,9,10])
m_a = [0.1, 0.5, 0.9]
Ga_size_1 = []
Ga_size_2 = []
for i in range(3):
    Ga_size_1.append(alpha * n_m + beta * 10 * (1/(1+np.exp(-n_m)) - 1/(1+np.exp(-m_a[i]))))
    
for i in range(3):
    Ga_size_2.append(alpha * 5 + beta * N_a * (1/(1+np.exp(-5)) - 1/(1+np.exp(-m_a[i]))))
    
Ga_size_1 = np.array(Ga_size_1).T    
Ga_size_2 = np.array(Ga_size_2).T


fig = plt.figure(figsize=(20,10))

ax = fig.add_subplot(1,2,1)
my_x_ticks = np.arange(0, 11, 1)#原始数据有13个点，故此处为设置从0开始，间隔为1
plt.xticks(my_x_ticks)
my_y_ticks = np.arange(0, 11, 1)#原始数据有13个点，故此处为设置从0开始，间隔为1
plt.yticks(my_y_ticks)
plt.gcf().set_facecolor(np.ones(3)* 240 / 255)   # 生成画布的大小
ax.grid(linestyle='-.')  # 生成网格
plt.xlim(0, 10)
plt.ylim(0, 10)
ax.set_xlabel("n/m, Na = 10")
ax.set_ylabel("Ga_size")
ax.plot(N_a, Ga_size_1[:,0])
ax.plot(N_a, Ga_size_1[:,1])
ax.plot(N_a, Ga_size_1[:,2])


ax = fig.add_subplot(1,2,2)
my_x_ticks = np.arange(0, 11, 1)#原始数据有13个点，故此处为设置从0开始，间隔为1
plt.xticks(my_x_ticks)
my_y_ticks = np.arange(0, 11, 1)#原始数据有13个点，故此处为设置从0开始，间隔为1
plt.yticks(my_y_ticks)
plt.gcf().set_facecolor(np.ones(3)* 240 / 255)   # 生成画布的大小
ax.grid(linestyle='-.')  # 生成网格
plt.xlim(0, 10)
plt.ylim(0, 10)
ax.set_xlabel("N_a, n/m = 5")
ax.set_ylabel("Ga_size")
ax.plot(N_a, Ga_size_2[:,0])
ax.plot(N_a, Ga_size_2[:,1])
ax.plot(N_a, Ga_size_2[:,2])
