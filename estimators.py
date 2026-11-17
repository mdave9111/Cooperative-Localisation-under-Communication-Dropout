import numpy as np
import random as rnd

class Packet:

    def __init__(self,sender,receiver,max_range=6.0,p=0.3,sigma_range=0.05):
        self.sender = sender
        self.receiver = receiver
        self.valid = False

        self.d_true = np.hypot(receiver.xc - sender.xc, receiver.yc - sender.yc) ## (actual distance)
        
        if self.d_true > max_range: # checks if the TOF distance is less than 6m (otherwise it won't be able to send the packet)
            return

        ## now we must use the probability of packet drop and see if the packet is received successfully or not
        if np.random.rand() < p: # if packet ends up dropping don't relay info
            return
        
        # define estimated distance
        self.sender_x,self.sender_y,self.sender_is_anchor = sender.get_coords()
        self.receiver_x_dr,self.receiver_y_dr,self.receiver_is_anchor = receiver.get_coords()
        self.d_est = np.hypot(self.receiver_x_dr-self.sender_x,self.receiver_y_dr-self.sender_y) ##uses the estimated coordinates of receiver to calculate distance

        ## successful transmission
        self.d_meas = self.d_true + np.random.normal(0, sigma_range) ## this is the TOF sensor's slight random noise
        self.derror = self.d_meas - self.d_est ## we will use this to calculate the actual coordinates of the non anchors
        self.valid = True

def fixed_gain_update(agent,packet): #this is hte response that adjusts the rover's position estimation using derror with some coefficient K: 0<K<1 depending on trust of source
    if agent.is_anchor or not packet.valid or packet.d_est <1e-6:
        return
    # get unit vector pointed towards the anchor from the estimated location (this is direction along which we will update coords):

    ux = (agent.x_dr - packet.sender_x)/packet.d_est
    uy = (agent.y_dr - packet.sender_y)/packet.d_est

    gain = 0.45 if packet.sender_is_anchor else 0.08 #this sets the trustworthiness to 45 percent if anchor is transmitting the coordinates otherwise its 8 percent

    # actually correcting coordinate equations
    agent.x_dr += (gain*packet.derror)*ux #in brackets is the distance we want to correct along unit vector
    agent.y_dr += (gain*packet.derror)*uy

        


        
