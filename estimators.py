import numpy as np
from agent import Agent
import random as rnd

class Packet:

    def __init__(self,sender,receiver,max_range=6.0,p=0.3,sigma_range=0.05):
        self.sender = sender
        self.receiver = receiver
        self.valid = False

        # define measured distance
        self.sender_x,self.sender_y = sender.xc,sender.yc
        self.receiver_x_dr,self.receiver_y_dr,self.receiver_is_anchor = receiver.get_coords()
        self.receiver_x,self.receiver_y = receiver.xc,receiver.yc
        
        self.d_true = np.hypot(receiver.xc - sender.xc, receiver.yc - sender.yc) ## (actual distance)

        
        if self.d_true > max_range: # checks if the TOF distance is less than 6m (otherwise it won't be able to send the packet)
            return

        ## now we must use the probability of packet drop and see if the packet is received successfully or not
        if np.random.rand() < p: # if packet ends up dropping don't relay info
            return
        ## successful transmission
        self.d_est = np.hypot(self.receiver_x_dr-self.sender_x,self.receiver_y_dr-self.sender_y) ##uses the estimated coordinates of receiver to calculate distance
        self.d_meas = self.d_true + np.random.normal(0, sigma_range) ## this is the TOF sensor's slight random noise
        self.derror = self.d_meas - self.d_est ## we will use this to calculate the actual coordinates of the non anchors
        self.valid = True

        
