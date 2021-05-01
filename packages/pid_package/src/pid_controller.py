#!/usr/bin/env python3

import time
import math

class PidController
    def __init__(self):
        self.last_time = time.time() 
        self.last_integral = ()
        self.last_error = ()        
        self.kp = ()
        self.ki = ()
        self.kd = ()
    
    def Gains(self, p_kp, p_ki, p_kd):
        self.kp = p_kp
        self.ki = p_ki
        self.kd = p_kd
        
    def Calculations(self, p_error, p_time):
        dt = p_time - self.last_time
        integral = self.last_integral + (p_error*dt)
        derivative = (p_error - self.last_error) / dt
        control = self.kp*p_error + self.kin*integral + self.kd*derivative)
        
        self.last_error = p_error
        self.last_time = p_time
        self.last_integral = integral
        return control
