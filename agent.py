import numpy as np
import matplotlib.pyplot as plt

class Agent:
    R = 0.12 #radius of wheel
    L = 0.30 #length of axle
    def __init__(self,heading):
        self.theta_l = 0.0
        self.theta_r = 0.0
        self.w_l = 0.0
        self.w_r = 0.0
        ## ground truth values
        self.s_lc = 0.0
        self.xc = 0.0
        self.yc = 0.0
        self.heading = heading ## wrt north
        self.dheading = 0.0
        self.midheading = 0.0

        #initial values
        self.t_prev = None
        self.theta_lprev = None
        self.theta_rprev = None
        self.w_lprev = None
        self.w_rprev = None
        self.xcprev = None
        self.ycprev = None


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


    def coord_calc(self):
        ## this is the dead reckoning/odometry with noisy encoder measurements
        self.s_lc = Agent.R*(self.theta_l - self.theta_lprev) # incremental displacements
        self.s_rc = Agent.R*(self.theta_r - self.theta_rprev)
        self.dheading = (self.s_lc - self.s_rc)/Agent.L # delta(heading)
        self.midheading = self.heading + self.dheading/2.0
        self.heading += self.dheading
        self.sc = 0.5*(self.s_lc + self.s_rc) # midpoitn displacement
        self.xc = self.xcprev + (np.sin(self.midheading))*self.sc
        self.yc = self.ycprev + (np.cos(self.midheading))*self.sc

        
        

    def turn(self):
        pass




def main():
    t = np.linspace(0,10,1000)
    theta_l = 4*t
    theta_r = 5*t
    
    agent1 = Agent(heading=1.2)
    x_hist,y_hist = [],[]

    plt.ion()
    fig,ax = plt.subplots(figsize=(7,7))
    
    for tick_no,tick in enumerate(t):
        print(agent1.tick(theta_l[tick_no],theta_r[tick_no],tick))
        x_hist.append(agent1.xc)
        y_hist.append(agent1.yc)

        # Redraw every 3rd frame so animation runs smoothly
        if tick_no % 10 == 0:
            ax.cla()
            ax.plot(x_hist, y_hist, "b-", linewidth=2, label="Path")
            ax.scatter(agent1.xc, agent1.yc, color="blue", s=60, zorder=5)

            # Red arrow shows which way the rover is facing (wrt North)
            ax.quiver(
                agent1.xc,
                agent1.yc,
                np.sin(agent1.heading),
                np.cos(agent1.heading),
                color="red",
                scale=12,
                width=0.008,
                zorder=6,
                label="Heading",
            )

            ax.set_xlim(-3.0, 1.0)
            ax.set_ylim(-0.5, 3.5)
            ax.set_aspect("equal")
            ax.grid(True, linestyle="--", alpha=0.5)
            ax.set_xlabel("X (meters)")
            ax.set_ylabel("Y (meters)")
            ax.set_title(f"Rover Kinematics | t = {tick:.2f}s")
            ax.legend(loc="upper right")
            plt.pause(0.01)

    plt.ioff()
    plt.show()




if __name__ == "__main__":
    main()