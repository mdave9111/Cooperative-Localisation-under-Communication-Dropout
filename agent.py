import numpy as np
import matplotlib.pyplot as plt

class Agent:
    R = 0.12 #radius of wheel
    L = 0.30 #length of axle
    def __init__(self,heading,is_anchor):
        self.is_anchor = is_anchor
        self.theta_l = 0.0
        self.theta_r = 0.0
        self.w_l = 0.0
        self.w_r = 0.0
        ## ground truth values
        self.s_lc = 0.0
        self.s_rc = 0.0
        self.xc = 0.0
        self.yc = 0.0
        self.heading = heading ## wrt north
        ## dead reckoning values (where rover thinks it is)
        self.s_lc_dr = 0.0
        self.s_rc_dr = 0.0
        self.x_dr = 0.0
        self.y_dr = 0.0
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
        
    



def main():
    np.random.seed(42) ## adds a predicatbale seed for randomness?
    t = np.linspace(0,10,1000)
    theta_l = 4*t
    theta_r = 5*t
    
    agent1 = Agent(heading=1.2,is_anchor=True) # this is where the rover thinks it is (it doesnt have gnss)
    x_true_hist,y_true_hist = [],[]
    x_dr_hist, y_dr_hist = [],[]

    plt.ion()
    fig,ax = plt.subplots(figsize=(7,7))
    
    for tick_no,tick in enumerate(t):
        agent1.tick(theta_l[tick_no],theta_r[tick_no],tick)
        x_true_hist.append(agent1.xc)
        y_true_hist.append(agent1.yc)
        x_dr_hist.append(agent1.x_dr)
        y_dr_hist.append(agent1.y_dr)

        # Redraw every 3rd frame so animation runs smoothly
        if tick_no % 10 == 0:
            ax.cla()
            ax.plot(x_true_hist, y_true_hist, "b-", linewidth=2, label="Path")
            ax.plot(x_dr_hist, y_dr_hist, "r--", linewidth=2, label="Dead Reckoning")
            ax.scatter(agent1.xc, agent1.yc, color="blue", s=60, zorder=5)
            ax.scatter(agent1.x_dr, agent1.y_dr, color="red", s=60, zorder=5)

            err = np.hypot(agent1.xc - agent1.x_dr, agent1.yc - agent1.y_dr)
            ax.set_xlim(-3.0, 2.0)
            ax.set_ylim(-1.0, 4.0)
            ax.set_aspect("equal")
            ax.grid(True, linestyle="--", alpha=0.5)
            ax.set_xlabel("X (meters)")
            ax.set_ylabel("Y (meters)")
            ax.set_title(f"t = {tick:.2f}s | Drift Error: {err:.2f}m")
            ax.legend(loc="upper right")
            plt.pause(0.01)

    plt.ioff()
    plt.show()




if __name__ == "__main__":
    main()