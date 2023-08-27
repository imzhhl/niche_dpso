import numpy as np
import math

class Wind_function:
    def __init__(self):
        pass
        
    #求每个点的风向(二维数组)
    def wind_func(self, X, wind_t):
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
    # wind utilization controled by parameter "cp" (chi_theta)
    def cp(self, X, W, V):  #控制参数
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