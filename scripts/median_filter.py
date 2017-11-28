#!/usr/bin/python

import numpy as np

class medianfilter:
    def __init__(self, size = 8):
        self.size = size
        self.y_old = 0
        self.buf = np.array([0,0,0,0,0,0,0])
        self.drop = True

    def update_filter(self, x_new):
        # sort the buf
        tmp = np.copy(self.buf)
        tmp = np.append(tmp, x_new)
        tmp = np.sort(tmp)
        if (self.drop):
            tmp = tmp[0:8]
        else:
            tmp = tmp[1:9]
        self.drop = not self.drop
        self.buf = tmp
        out = np.median(self.buf)
        return out

if __name__ == "__main__":
    size = 8
    step = 10
    t = 0.0
    dt = 0.1
    t_end = 10
    x_lp = medianfilter(size)
    while (t < t_end):
        noise = 0.5*np.random.rand()
        y = x_lp.update_filter(step+noise)
        print 'time:', t, 'filtered output:', y, 'unfiltered output:', step+noise
        t += dt
