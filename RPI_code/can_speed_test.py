import time
#import can_comunication

timing_list = []

def average(lst):
    return sum(lst) / len(lst)

for i in range(0,10):
    start = time.time()
    #can_comunication.can_get_voltage(5,1)
    end = time.time()
    print(end - start)
    timing_list.append(end - start)

print(average(timing_list))