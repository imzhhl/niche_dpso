from numpy import array, cos, sin, array, pi, sqrt
import matplotlib.pyplot as plt

def zigzag(iteration, vector):

    sign = (-1)**iteration
    ix = vector[0,0] * cos(alpha) + vector[0,1] * sin(sign*alpha)
    iy = vector[0,1] * cos(alpha) - vector[0,0] * sin(sign*alpha)
    vector = array([[ix, iy]])
    
    return vector

alpha = pi * 5/6

unit_vector = array([[sqrt(3),1]])
position = array([[sqrt(3),1]])
iter_num = 0
position_history = []

total = 0
turning_num = 1
temp = []
turning_num = 1

while iter_num < 1000:
    
    iter_num += 1
    total += iter_num
    if iter_num != 1:
        temp.append(total)
        
    position_history.append(position.tolist()[0])    
    # 控制转向, 只有在第3, 6, 10, 15, 21, 28, 36, 45, 54……时才发生转向
    if iter_num in temp:
        unit_vector = zigzag(turning_num, unit_vector)
        turning_num += 1
    # unit_vector = zigzag(iter_num, unit_vector)   
    position = position + unit_vector
    # print(unit_vector)
   
    
position_history = array(position_history)    
plt.plot(array(position_history)[:,0], array(position_history)[:,1])

