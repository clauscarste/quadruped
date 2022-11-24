# goal is the this script will replace the can comunication script in all its functionalites and send the necesary data
# to the simulation and retrive the necesary date from the simulation


import numpy as np
import time
N = 500
q0_start = 0;
q0_end = 1;
q1_start = -1;
q1_end = 10;
q0_0 = np.linspace(q0_start,q0_end,N)
q1_1 = [0,0.1,0.2,1,2,3,4,5,6,7,8,9,10,1,0.1,0.2,0.3]
qqqq = q1_1

def dictionary():
    ###Dictionary###
    global firstjoint
    firstjoint = [0,0,0]

    ###         ###
def countup():

    for i in range(len(qqqq)):
        a = qqqq[i]
        firstjoint[0] = a
        time.sleep(0.8)

def printit(a,b,c):
    print(a,b,c)

