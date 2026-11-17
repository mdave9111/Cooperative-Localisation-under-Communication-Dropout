import matplotlib.pyplot as plt
import numpy as np
from agent import Agent
from estimators import Packet

def main():
    np.random.seed(7) ## "shuffles the deck in a specific way so that you pick the same numbers of the deck each time you run sim"
    t = np.linspace(0,10,1000)
    
    anchor1 = Agent(heading=1.2,is_anchor=True) # this is where the rover thinks it is (it doesnt have gnss)
    anchor2 = Agent(heading=2.3,is_anchor=True)
    rover1 = Agent(heading=1.2,is_anchor=False,x=2.3,y=3.4)
    rover2 = Agent(heading=1.2,is_anchor=False,x=1.2,y=1.4)
    rover3 = Agent(heading=1.2,is_anchor=False,x=2.7,y=0.5)
    rover4 = Agent(heading=1.2,is_anchor=False,x=-2.5,y=1.1)
    agents = [anchor1,anchor2,rover1,rover2,rover3,rover4]

    theta_l = [1.2 * t,2.3 * t, 4.0 * t, 4.5 * t, 3.8 * t, 4.2 * t] 
    theta_r = [0.5 * t,1.3 * t, 5.0 * t, 4.2 * t, 4.4 * t, 4.8 * t]
    

    plt.ion()
    fig,ax = plt.subplots(figsize=(7,7))
    
    for tick_no,tick in enumerate(t):
        # move all 6 agents every tick
        for i,agent in enumerate(agents):
            agent.tick(theta_l[i][tick_no],theta_r[i][tick_no],tick)

        # get feedback and coordinates here#
        active_links = []
        for receiver in agents:
            if receiver.is_anchor: # dont need to correct coordinates if it already has gps
                continue
            for sender in agents:
                if receiver is sender: # don't study relationship between the same receiver and sender
                    continue
                packet = Packet(sender, receiver, max_range=6.0, p=0.3, sigma_range=0.05)
                if packet.valid:
                    receiver.correct_coords(packet)
                    active_links.append((sender, receiver))

        # record the post correction history
        for agent in agents:
            agent.record_history()
            


        # 4. Plot every 15th frame
        if tick_no % 15 == 0:
            ax.cla()

            # Draw active radio pings on this tick
            for sender, receiver in active_links:
                ax.plot([sender.xc, receiver.xc], [sender.yc, receiver.yc],
                        color="green", linestyle=":", linewidth=0.8, alpha=0.5)

            for idx, agent in enumerate(agents):
                if agent.is_anchor:
                    ax.scatter(agent.xc, agent.yc, marker="^", color="gold",
                            edgecolor="black", s=140, zorder=6,
                            label="Anchor" if idx == 0 else "")
                else:
                    ax.plot(agent.x_hist, agent.y_hist, "b-", linewidth=1.5,
                            label="True Path" if idx == 2 else "")
                    ax.plot(agent.x_dr_hist, agent.y_dr_hist, "r--", linewidth=1.5,
                            label="Estimated Path" if idx == 2 else "")
                    ax.scatter(agent.xc, agent.yc, color="blue", s=40, zorder=5)
                    ax.scatter(agent.x_dr, agent.y_dr, color="red", s=40, zorder=5)

            mean_err = np.mean([np.hypot(r.xc - r.x_dr, r.yc - r.y_dr) for r in agents[2:]])
            ax.set_xlim(-3.0, 4.0)
            ax.set_ylim(-2.0, 5.0)
            ax.set_aspect("equal")
            ax.grid(True, linestyle="--", alpha=0.5)
            ax.set_xlabel("X (meters)")
            ax.set_ylabel("Y (meters)")
            ax.set_title(f"t = {tick:.2f}s | Mean Rover Error: {mean_err:.2f}m")
            ax.legend(loc="upper right")
            plt.pause(0.01)

    plt.ioff()
    plt.show()


if __name__ == "__main__":
    main()