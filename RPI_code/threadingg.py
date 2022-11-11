from multiprocessing import Process, Queue
import random

def rand_num(a):
    for i in range(1,10):
        print(a, i)

if __name__ == "__main__":
    queue = Queue()

    processes = [Process(target=rand_num(1), args=())]
    processes2 = [Process(target=rand_num(2), args=())]

    for p in processes:
        p.start()

    for p in processes:
        p.join()