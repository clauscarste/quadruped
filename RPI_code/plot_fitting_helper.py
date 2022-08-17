#helper fct. to fit a polynomail curve to the gathered sensor data that relates force to drawn current
import numpy
x = [1,5,8,10,15,20,25,30]
y = [0.288235294117647,1.40196078431373,2.18921568627451,2.68382352941176,3.90343137254902,4.66617647058824,5.18333333333333,6.58333333333333]
print(numpy.polyfit(x, y, 2))
a = 2
print((-0.00221823*a*a+0.27497038*a+0.09294292)*0.15)