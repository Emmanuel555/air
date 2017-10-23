#!/usr/bin/python

class lowpassfilter:
    def __init__(self, dt = 1.0/20.0, time_constant = 1.0):
        self.dt = dt
        self.T = time_constant
        self.y_old = 0
        self.alpha = self.dt/ (self.T+self.dt)

    def update_filter(self, x_new):
        y_new = self.alpha * x_new + (1-self.alpha) * self.y_old
        self.y_old = y_new
        return y_new

if __name__ == "__main__":
    t = 0.0
    dt = 1.0/20.0
    t_end = 5.0
    time_constant = 1.0
    x_lp = lowpassfilter(dt, time_constant)
    while (t < t_end):
        y = x_lp.update_filter(10)
        print 'time:', t, 'filtered output:', y
        t += dt
