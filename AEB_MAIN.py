#!/usr/bin/env python3

import rospy
import numpy
import aeb 

def execute(aeb):
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
    return throttle_command, brake_command

def publisher1('/msg1', '/msg2'):
    pub1 = rospy.Publisher('t_b', Twist, queue_size = 1)    
    move = Twist()
    move.linear.x = t
    move.linear.y = b
    pub1.publish(move)
    return 0 

def callback (msg1, msg2, current_time, dt):
    # take the values from the perception module 
    # store them in - rel_dist, ego_vel, acc_set_speed, ego_acc, ttc, driver_brake 
    t, b = execute(aeb)   # updates aeb parameters according to current inputs, gives throttle command and brake as outputs
    publisher1(t, b)
    current_time += 1 



rospy.init_node('aeb_implementor')
current_time = None    # set it to 0 as the initial time  
dt = None              # set it to whatever we'd like the resolution
sub = rospy.Subscriber('/msg1', '/msg2', '/topic', callback)
rospy.spin()