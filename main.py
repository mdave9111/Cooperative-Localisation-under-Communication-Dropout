import matplotlib.pyplot as plt
import numpy as np
from agent import Agent
from estimators import Packet, ekf_update

def run_simulation(seed_no,p_drop,mode):

    np.random.seed(seed_no) ## "shuffles the deck in a specific way so that you pick the same numbers of the deck each time you run sim"
    
    t = np.linspace(0,10,1000)
    rmse_array = []
    anchor1 = Agent(heading=1.2,is_anchor=True) # this is where the rover thinks it is (it doesnt have gnss)
    anchor2 = Agent(heading=2.3,is_anchor=True)
    rover1 = Agent(heading=1.2,is_anchor=False,x=2.3,y=3.4)
    rover2 = Agent(heading=1.2,is_anchor=False,x=1.2,y=1.4)
    rover3 = Agent(heading=1.2,is_anchor=False,x=2.7,y=0.5)
    rover4 = Agent(heading=1.2,is_anchor=False,x=-2.5,y=1.1)
    agents = [anchor1,anchor2,rover1,rover2,rover3,rover4]

    theta_l = [1.2 * t,2.3 * t, 4.0 * t, 4.5 * t, 3.8 * t, 4.2 * t] 
    theta_r = [0.5 * t,1.3 * t, 5.0 * t, 4.2 * t, 4.4 * t, 4.8 * t]

    for tick_no,tick in enumerate(t):
        # move all 6 agents every tick
        for i,agent in enumerate(agents):
            agent.tick(theta_l[i][tick_no],theta_r[i][tick_no],tick)
        # get feedback and coordinates here 
        for receiver in agents:
            if receiver.is_anchor: # dont need to correct coordinates if it already has gps
                continue
            for sender in agents:
                if receiver is sender: # don't study relationship between the same receiver and sender
                    continue
                packet = Packet(sender, receiver, max_range=6.0, p=p_drop, sigma_range=0.05)
                if mode == "fg":
                    receiver.correct_coords(packet)
                elif mode == "ekf":
                    ekf_update(receiver, packet)


        # record the post correction history
        for agent in agents:
            agent.record_history()

    rover_rmse = []
    for rover in agents[2:]:
        dx = np.array(rover.x_hist) - np.array(rover.x_dr_hist)
        dy = np.array(rover.y_hist) - np.array(rover.y_dr_hist)
        rover_rmse.append(np.sqrt(np.mean(dx**2 + dy**2)))
    
    return np.mean(rover_rmse)



def main():
    rmse_fg_plot = []
    rmse_ekf_plot = []
    dr_seeds = []

    for seed_no in range(0,30):
        dr_seeds.append(run_simulation(seed_no, p_drop=1.0, mode="dr"))
    
    rmse_dr_plot = [np.mean(dr_seeds)]*21

    for p in np.linspace(0,1,21):
        rmse_arr_total = []
        ekf_seeds = []
        for seed_no in range(0,30):
            rmse_seeds = run_simulation(seed_no,p,mode="fg") # fixed gain update
            rmse_arr_total.append(rmse_seeds)

            ekf_seed = run_simulation(seed_no, p, mode="ekf")
            ekf_seeds.append(ekf_seed)

        mean_rmse_fg = np.mean(rmse_arr_total)
        rmse_fg_plot.append(mean_rmse_fg)

        mean_rmse_ekf = np.mean(ekf_seeds)
        rmse_ekf_plot.append(mean_rmse_ekf)

    plt.figure(figsize=(8, 5))
    plt.plot(np.linspace(0,1,21), rmse_dr_plot, "r--", linewidth=2, label="Dead Reckoning (Open-Loop)")
    plt.plot(np.linspace(0,1,21), rmse_fg_plot, "b-o", linewidth=2, label="Fixed-Gain vs Dead Reckoning Estimator")
    plt.plot(np.linspace(0,1,21), rmse_ekf_plot, "g-s", linewidth=2, label="Extended Kalman Filter (EKF)")
    plt.xlabel("Packet Drop Probability (p)")
    plt.ylabel("Mean Non-Anchor RMSE (meters)")
    plt.title("Cooperative Localization Benchmark (30 Seeds per p)")
    plt.xlim(0.0, 1.0)
    plt.ylim(bottom=0)
    plt.grid(True, linestyle="--", alpha=0.5)
    plt.legend()
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()