from cmath import inf
import math
from threading import Timer
import numpy
import scipy
import pandas
import matplotlib.pyplot as plt

######################################################################################
class AEB_Controller:
    def __init__(self, relative_dist, ego_vel, ACC_set_speed, ttc, ego_acc):
        # used for definition
        self.relative_dist = relative_dist
        self.ego_vel = ego_vel
        self.ACC_set_speed = ACC_set_speed
        self.ttc = ttc
        self.ego_acc = ego_acc
        # parameters which we won't need outside the aeb block --- constants
        self.AEB_PB1 = 2
        self.AEB_PB2 = 3
        self.AEB_FB = 3.9
        self.Kp = 1
        self.Kd = 0
        self.Ki = 1
        self.min_def_dist = 1.5
        self.AEB_Headway_Offset = 3
        self.FCW_Reaction_Time = 1.2
        self.FCW_Driver_Deacc = 4
        self.acceleration = 0
        # parameters  which keep on updated every iteration
        self.FCW_Stopping_Time = inf
        self.PB1_Stopping_Time = inf
        self.PB2_Stopping_Time = inf
        self.FB_Stopping_Time = inf
        self.stop_bool = False
        self.deacc = 0
        self.aeb_status = False
        self.fcw_status = False
        self.velocity_offset = 0
        self.velocity_offset_controlled = 0
        self.ACC_state = 1
        self.ACC_enable = 1
        self.AEB_state = "Default"
         
        

    # check for ttc > 0        we don't need ttc for acc or cc we need it only for aeb 
    def TTC_non_positive(self):
        if (self.ttc <= 0) or (self.ego_vel <= 0):
            self.stop_bool = True
        else:
            self.stop_bool = False
        return self.stop_bool

    def PID(self, error, current_time):  
        I = 0  
        control_var = 0                                
        for i in range(current_time):           
            #print("i = {}. type = {}".format(i, type(i)))
            #print(error[i])
            P = (self.Kp)*error[i]
            I = I + (self.Ki)*error[i]*dt           
            D = (self.Kd)*(error[i]-error[i-1])/dt   

            control_var = P + I + D
        return control_var

    def acc_controller(self, current_time):
        ACC_acc = 0
        self.velocity_offset = self.ACC_set_speed - self.ego_vel
        error[current_time] = self.velocity_offset
        self.velocity_offset_controlled = self.PID(error, current_time)
        sd = (2*self.ego_vel)+self.min_def_dist                             
        y = self.relative_dist - sd
        error_y[current_time] = y

        if self.ACC_state == 1 and y <= 0:                                                                  
            self.ACC_state = 2
        elif self.ACC_state == 2 and (y >= 0.2*sd and j > 100):                       
            self.ACC_state = 1
        elif self.ACC_state == 2 and (y <= 0 and j > 100):
            self.ACC_state = 3
        elif self.ACC_state == 3 and y >= 0.2*sd:
            self.ACC_state = 2

        if self.ACC_state == 1:
            ACC_acc = self.PID(error, current_time)
        elif self.ACC_state == 2:
            for j in range(1, 101):
                j += 1
                self.ACC_state += 0.05
        elif self.ACC_state == 3:
            ACC_acc = self.PID(error_y, current_time)

        if ACC_acc > 1:
            ACC_acc = 1
        elif ACC_acc < -0:
            ACC_acc = 0

        if self.ACC_enable > 0:
            self.acceleration = ACC_acc
        else:
            self.acceleration = self.velocity_offset_controlled

        return self.acceleration

    def stopping_time(self):
        reaction_time1 = (self.ego_vel/self.FCW_Driver_Deacc)*1.63
        self.FCW_Stopping_Time = self.FCW_Reaction_Time + reaction_time1
        return self.FCW_Stopping_Time

    def stopping_time_calc(self):
        self.PB1_Stopping_Time = self.ego_vel/self.AEB_PB1
        self.PB2_Stopping_Time = self.ego_vel/self.AEB_PB2
        self.FB_Stopping_Time = self.ego_vel/self.AEB_FB
        return self.PB1_Stopping_Time, self.PB2_Stopping_Time, self.FB_Stopping_Time

    def AEB_state_machine(self):
        if (self.ttc > 0) : 
            if self.AEB_state == "Default" and (self.ttc < self.FCW_Stopping_Time):
                self.AEB_state = "FCW"
                self.aeb_status = 0
                self.fcw_status = 1
                self.deacc = 0
            elif self.AEB_state == "FCW" and (self.ttc > 1.2*self.FCW_Stopping_Time):
                self.AEB_state = "Default"
                self.aeb_status = 0
                self.fcw_status = 0
                self.deacc = 0
            elif self.AEB_state == "FCW" and (self.ttc <= self.PB1_Stopping_Time):
                self.AEB_state = "PB1"
                self.aeb_status = 1
                self.fcw_status = 1
                self.deacc = self.AEB_PB1
            elif self.AEB_state == "PB1" and (self.ttc < self.PB2_Stopping_Time):
                self.AEB_state = "PB2"
                self.aeb_status = 1
                self.fcw_status = 1
                self.deacc = self.AEB_PB2
            elif self.AEB_state == "PB1" and self.stop_bool is True:
                self.AEB_state = "Default"
                self.aeb_status = 0
                self.fcw_status = 0
                self.deacc = 0
            elif self.AEB_state == "PB2" and (self.ttc < self.FB_Stopping_Time):
                self.AEB_state = "FB"
                self.aeb_status = 1
                self.fcw_status = 1
                self.deacc = self.AEB_FB
            elif self.AEB_state == "PB2" and self.stop_bool is True:
                self.AEB_state = "Default"
                self.aeb_status = 0
                self.fcw_status = 0
                self.deacc = 0
            elif self.AEB_state == "FB" and ((self.stop_bool is True) and (abs(self.ttc) > abs(1.2*self.FCW_Stopping_Time))):
                self.AEB_state = "Default"
                self.aeb_status = 0
                self.fcw_status = 0
                self.deacc = 0
        
            return self.deacc, self.aeb_status, self.fcw_status
        else: 
            a3 = "TTC negative! You have crashed! "
            print(a3)
            a2 = 0
            a1 = 0
            return a1, a2, a3
    
    def MOO_Dist_Calc(self):
        dist_MOO = min(dist_ObjDet,dist_Seg)
        return dist_MOO
    

############################################################################
class Brake_Control:

    def __init__(self, ACC_deacc, deceleration, driver_brake):
        self.ACC_deacc = -1*ACC_deacc
        self.deceleration = deceleration
        self.driver_brake = driver_brake
        self.brake = 2

          
    def final_decel(self):
        if self.ACC_deacc > 0:
            self.ACC_deacc = self.ACC_deacc
        else:
            self.ACC_deacc = 0
        brake_final = max(self.brake, self.ACC_deacc, self.deceleration)
        return brake_final

#######################################################################
class Throttle_control:
    
    def __init__(self, aeb_status, acc):
        self.aeb_status = aeb_status
        self.acc = acc
        self.acc_bool = True
        self.AEB_or_acc_bool = True
        self.throttle = 0

        
    def switch(self):
        
        # check if acceleration is positive or negative
        if self.acc <= 0:
            self.acc_bool = True
        else:
            self.acc_bool = False

        # check the aeb status
        if (self.aeb_status is True) or (self.acc_bool is True):
            self.AEB_or_acc_bool = True
        else:
            self.AEB_or_acc_bool = False

        # return the throttle command
        if (self.AEB_or_acc_bool is True):
            self.throttle = 0
        else:
            self.throttle = self.acc
        return self.throttle
##########################################################################################

####################################################################
############   Main function ###########
####################################################################

if __name__ == '__main__':

    # inputs and parameters before simulation starts 
    end_time = 10             # set simulation end time
    dt = 0.1                    # set the resolution    
    error = numpy.zeros(100)
    error_y = numpy.zeros(100)
    initial_rel_dist = 100
    initial_ego_vel = 0
    initial_acc_set_speed = 10
    initial_ttc = 50
    initial_ego_acc = 1

    # initialise an object at the beginning of the simulation. 
    # same object keeps on updating as the simulation runs          
    aeb = AEB_Controller(initial_rel_dist, initial_ego_vel, initial_acc_set_speed, initial_ttc, initial_ego_acc)
    ego_vel = initial_ego_vel
    ego_acc = initial_ego_acc
    # all the code below goes in a for or while loop with iterator as "current_time"
    ############
    for current_time in range(100):
        #print(current_time)
        # define the inputs to be taken from perception module 
        rel_dist = 100 - current_time 
        
        #ego_vel = None 
        acc_set_speed = 10 
        ttc = 50 - current_time*dt 
        #ego_acc = None 
        driver_brake = 0 
        
        print(rel_dist, ttc)
    

        # update the aeb attributes using the above inputs 
        aeb.relative_dist = rel_dist
        aeb.ego_vel = ego_vel
        aeb.ACC_set_speed = acc_set_speed
        aeb.ttc = ttc
        aeb.ego_acc = ego_acc  

        # perform the functions using the updated attributes
        # other attributes will get updated as the function runs
        aeb.stop_bool = aeb.TTC_non_positive()
        aeb.FCW_Stopping_Time = aeb.stopping_time()
        aeb.PB1_Stopping_Time, aeb.PB2_Stopping_Time, aeb.FB_Stopping_Time = aeb.stopping_time_calc()
        acc_acceleration = aeb.acc_controller(current_time)
        aeb_deceleration, aeb_status, fcw_status = aeb.AEB_state_machine()
                        
        brakecontrol = Brake_Control(acc_acceleration, aeb_deceleration, driver_brake)
        brake_command = brakecontrol.final_decel()
        throttle = Throttle_control(aeb_status, acc_acceleration)
        throttle_command = throttle.switch()
        ego_acc = throttle_command
        ego_vel += (throttle_command )*dt

        print("brake = {}".format(brake_command))
        print("throttle = {}".format(throttle_command))
    





