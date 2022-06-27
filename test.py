import matplotlib.pyplot as plt
import multiprocessing as mp
import random
import numpy
import time

def worker(q):
    plt.ion()
    ln, = plt.plot([], [])
    plt.show()

    while True:
        obj = q.get()
        n = obj + 0
        print("sub : got:"), n

        ln.set_xdata(numpy.append(ln.get_xdata(), n))
        ln.set_ydata(numpy.append(ln.get_ydata(), n))
        plt.draw()

if __name__ == '__main__':
    queue = mp.Queue()
    p = mp.Process(target=worker, args=(queue,))
    p.start()

    while True:
        n = random.random() * 5
        print("main: put:"), n
        queue.put(n)
        time.sleep(1.0)