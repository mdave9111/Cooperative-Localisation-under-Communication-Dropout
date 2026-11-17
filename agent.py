import numpy as np
import matplotlib.pyplot as plt
from estimators import fixed_gain_update

class Agent:
    R = 0.12 #radius of wheel
    L = 0.30 #length of axle
    def __init__(self,heading,is_anchor,x=0.0,y=0.0):
        self.is_anchor = is_anchor
        self.theta_l = 0.0
        self.theta_r = 0.0
        self.w_l = 0.0
        self.w_r = 0.0
        ## ground truth values
        self.s_lc = 0.0
        self.s_rc = 0.0
        self.xc = x
        self.yc = y
        self.heading = heading ## wrt north
        ## dead reckoning values (where rover thinks it is)
        self.s_lc_dr = 0.0
        self.s_rc_dr = 0.0
        self.x_dr = x
        self.y_dr = y
        self.heading_dr = heading

        #initial values
        self.t_prev = None
        self.theta_lprev = None
        self.theta_rprev = None
        self.w_lprev = None
        self.w_rprev = None
        self.xcprev = None
        self.ycprev = None
        self.x_drprev = None
        self.y_drprev = None

        self.x_hist = [x]
        self.y_hist = [y]
        self.x_dr_hist = [x]
        self.y_dr_hist = [y]


    def noise(self):
        if not self.is_anchor: ## inject noise as this agent doesn't have gps
            mu = 0
            sigma = 0.003
            noise_l = np.random.normal(mu,sigma) # this creates positive or negative noise value which we add on
            noise_r = np.random.normal(mu,sigma)
            self.s_lc_dr = self.s_lc + noise_l
            self.s_rc_dr = self.s_rc + noise_r

        else:
            self.s_lc_dr = self.s_lc
            self.s_rc_dr = self.s_rc




    def tick(self,theta_l,theta_r,tick):
        if self.t_prev is None: ## makes sure no division by 0 on first go
            self.theta_l = theta_l
            self.theta_r = theta_r
            self.t_prev = tick
            return

        dt = tick - self.t_prev
        self.update_prev() #sets the 1 previous values correctly
        self.theta_l = theta_l
        self.theta_r = theta_r
        self.coord_calc()
        self.w_l = (self.theta_l - self.theta_lprev)/dt
        self.w_r = (self.theta_r - self.theta_rprev)/dt
        self.t_prev = tick
        return self.w_l, self.w_r

    def update_prev(self):
        self.theta_lprev = self.theta_l # sets the previous theta before hand
        self.theta_rprev = self.theta_r
        self.w_lprev = self.w_l
        self.w_rprev = self.w_r
        self.xcprev = self.xc
        self.ycprev = self.yc
        self.x_drprev = self.x_dr
        self.y_drprev = self.y_dr


    def coord_calc(self):
        ## this is the ground truth measurements (actual location)
        dtheta_l = self.theta_l - self.theta_lprev
        dtheta_r = self.theta_r - self.theta_rprev
        self.s_lc = Agent.R*(dtheta_l) # incremental displacements
        self.s_rc = Agent.R*(dtheta_r)
        dheading = (self.s_lc - self.s_rc)/Agent.L # delta(heading)
        midheading = self.heading + dheading/2.0
        self.heading += dheading
        self.sc = 0.5*(self.s_lc + self.s_rc) # midpoitn displacement
        self.xc = self.xcprev + (np.sin(midheading))*self.sc
        self.yc = self.ycprev + (np.cos(midheading))*self.sc
        self.noise() ##injects noise

        ## this is the noisy dead reckoning measurements

        dheading_dr = (self.s_lc_dr - self.s_rc_dr) / Agent.L
        midheading_dr = self.heading_dr + dheading_dr / 2.0
        self.heading_dr += dheading_dr
        sc_dr = 0.5 * (self.s_lc_dr + self.s_rc_dr)
        self.x_dr = self.x_drprev + np.sin(midheading_dr) * sc_dr
        self.y_dr = self.y_drprev + np.cos(midheading_dr) * sc_dr

    def get_coords(self):
        return self.x_dr,self.y_dr,self.is_anchor ## for anchors xdr,ydr = xc,yc

    def correct_coords(self,packet): ## This will correct the coordinates of the non anchors
        fixed_gain_update(self,packet)


    def record_history(self):
        self.x_hist.append(self.xc)
        self.y_hist.append(self.yc)
        self.x_dr_hist.append(self.x_dr)
        self.y_dr_hist.append(self.y_dr)