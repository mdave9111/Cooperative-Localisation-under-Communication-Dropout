import matplotlib.pyplot as plt
import numpy as np
from agent import Agent

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