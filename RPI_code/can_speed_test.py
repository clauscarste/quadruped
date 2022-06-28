import time
import can_comunication

timing_list = []
output_list = []

def average(lst):
    return sum(lst) / len(lst)

for i in range(0,10):
    start = time.perf_counter()
    output_list.append(can_comunication.can_get_voltage(5))
    elapsed = time.perf_counter() - start
    print(elapsed)
    timing_list.append(elapsed)

print(average(timing_list))
print(output_list)

