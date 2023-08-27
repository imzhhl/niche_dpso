import math
import numpy as np

class Fitness:
    def __init__(self, **kwargs):
        self.x_limit = kwargs.pop('x_limit', 371)
        self.y_limit = kwargs.pop('y_limit', 393)
    
    # 求每个点的适应度函数
    def fitness_func(self, X, uds_t):
        x = X[:, 0] # 粒子x坐标
        y = X[:, 1] # 粒子y坐标

        #二维线性插值
        z = np.zeros(X.shape[0]) 
        for i in range(len(x)):    
            a = math.floor(x[i]) # 对x坐标取整
            b = math.floor(y[i]) # 对y坐标取整
            
            # # 防止索引超标
            if a >= self.x_limit - 1 or b >=self.y_limit - 1:
                a = self.x_limit - 2
                b = self.y_limit - 2
            if a < 0 or b < 0:
                a = 0
                b = 0
                
            #二维线性插值, 先检索y(行), 再检索x(列)
            z1 = uds_t[0, b,   a] + (uds_t[0, b,   a+1] - uds_t[0, b,   a]) * (x[i] - a)
            z2 = uds_t[0, b+1, a] + (uds_t[0, b+1, a+1] - uds_t[0, b+1, a]) * (x[i] - a)
            z[i] = z1 + (z2 - z1) * (y[i] - b)
        
        return z  # 浮点数, 单个粒子位置X处的污染浓度值
    
class Wind:
    def __init__(self):
        self.x_limit = 371
        self.y_limit = 393
    
    # 求每个点的风向（二维数组）
    def wind_func(self, X, wind_t):
        x = X[:,0]
        y = X[:,1]
        #二维线性插值
        z = np.zeros((len(x),2)) #风向, 二维数组 
        for i in range(len(x)):
            a = math.floor(x[i])
            b = math.floor(y[i])
            
            # 防止索引超标
            if a >= self.x_limit - 1 or b >=  self.y_limit - 1:
                a = self.x_limit - 2
                b = self.y_limit - 2
            if a < 0 or b < 0:
                a = 0
                b = 0
                
            z1 = wind_t[b, a, :] + (wind_t[b, a+1, :]-wind_t[b, a, :]) * (x[i] - a)
            z2 = wind_t[b+1, a, :] + (wind_t[b+1, a+1, :]-wind_t[b+1, a, :]) * (x[i] - a)
            z[i] = z1 + (z2 - z1) * (y[i] - b)
        return z  #二维数组，保存了各粒子位置插值得到的U和V速度