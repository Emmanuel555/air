#!/usr/bin/python

import numpy as np

class medfilter:
    def __init__(self, len = 5):
        self.len = len
        self.buffer = np.zeros(self.len)

    def update_filter(self, x_new):
        # print 'updating filter with: ', x_new
        self.buffer = np.append(self.buffer[1:],[x_new])
        # print 'new buffer: ', self.buffer
        y_new = np.median(self.buffer)
        return y_new
