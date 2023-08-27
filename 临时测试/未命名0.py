# -*- coding: utf-8 -*-
"""
Created on Tue Mar  7 17:06:03 2023

@author: zhhl_
"""
import re
import numpy as np

a=[1,2,3]
b=[3,4,5]
a+b

a = ['uav_1','uav_2','uav_3']

a.remove('uav_1')
b= 'uav_1008'
c=int(re.findall("\d+",b)[0] )

int(c[0])
a = [[]]*10
a = {'a':1,"b":2,'c':44,'d':465}
b = {'aa':1,"bb":2,'cv':44,'dd':465}
dict(a, **b)
a.pop('a')

np.argmax(a.values())
a=[1,2,3,4,5,6,7,8,9,10]
np.var(a[-50:])

a = {'a':1}
key = max(a, key=lambda x:a[x])

# 得到不同小生境中最佳的适应度值, 嵌套了两个字典
g_fitness = dict()
for i in range(niches):
    g_fitness["niche_" + str(i+1)] = dict()
    key = max(p_fitness["niche_" + str(i+1)], key=lambda x:p_fitness["niche_" + str(i+1)][x])
    g_fitness["niche_" + str(i+1)][key] = p_fitness["niche_" + str(i+1)][key]
    
V=random.uniform(-1,1,size=(1,2))
min_val=1
V[V < min_val] = min_val


list = [1]
if list:
    print('list is not empty')
    
    
lst = [1]

if lst:
    print('空列表')