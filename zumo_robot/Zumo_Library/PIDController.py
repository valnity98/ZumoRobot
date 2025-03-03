#!/usr/bin/env python3

'''
 ------------------------------------------------------------------------------
 File: Zumo328PPID.py
 Author: Mutasem Bader
 Description:
     This file contains the implementation of the PID controller for the Zumo robot.
     The PID controller adjusts the motor speeds based on the measured position 
     error between the target and measured position, utilizing proportional 
     and derivative control with integral control.
 
     Usage:
     - Instantiate the class with the maximum speed and and PID gains 
       (kp, kd and ki) to update the motor speeds.
     - Call `control_speed` with the measured position, target position.
     - Use `get_left_speed` and `get_right_speed` to retrieve the calculated motor speeds.
 
 ------------------------------------------------------------------------------ 
'''


import time

class Zumo328PPID:
    def __init__(self, kp, ki, kd, max_speed=200.0, aktiv=True, integral_limit=(-50.0, 50.0)):
        """
        Initialize the PID controller for the Zumo robot.
        
        Args:
        max_speed (float): Maximum speed for the motors. Default is 200.0.
        integral_limit (tuple): Tuple specifying the minimum and maximum limits for the integral term. Default is (-50.0, 50.0).
        """
        self.max_speed = max_speed  # Maximum speed of the motors
        self.integral_limit = integral_limit  # Limits for the integral term
        self._left_speed = 0  # Initial left motor speed
        self._right_speed = 0  # Initial right motor speed
        self._prevT = time.time()  # Previous time (for deltaT calculation)
        self._last_error = 0  # Last error value
        self._integral = 0.0  # Integral term
        self._filtered_derivative = 0.0  # Filtered derivative term
        self._filter_coefficient = 0.1  # Coefficient for low-pass filter
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.aktiv = aktiv

    def control_speed(self, measured_position, target_position):
        """
        Calculate and update the motor speeds based on the PID control.
        
        Args:
        measured_position (float): The current measured position of the robot.
        target_position (float): The desired target position of the robot.
        kp (float): The proportional gain for the PID controller.
        ki (float): The integral gain for the PID controller.
        kd (float): The derivative gain for the PID controller.
        aktiv (bool): Flag to indicate whether the deltaT should be active.
        """
        error = measured_position - target_position  # Calculate the error (distance from target position)

        # Only update if the PID controller is active
        if self.aktiv:
            currT = time.time()  # Get the current time in seconds
            deltaT = currT - self._prevT  # Calculate the time difference
            
            # Prevent division by zero in case deltaT is too small
            if deltaT <= 0.0:
                deltaT = 1e-6  # Set to a small value if deltaT is too small to avoid division by zero

            self._prevT = currT  # Update the previous time

            # Update integral term
            self._integral += error * deltaT

            # Apply Anti-Windup by clamping the integral term
            self._integral = max(self.integral_limit[0], min(self._integral, self.integral_limit[1]))


        else:
            deltaT = 1
            self._integral = 0

        
        # Update derivative term with low-pass filtering
        raw_derivative = (error - self._last_error) / deltaT
        self._filtered_derivative += self._filter_coefficient * (raw_derivative - self._filtered_derivative)

        # Calculate the speed difference using the PID formula (P, I, and D terms)
        speed_difference = (self.kp * error) + (self.ki * self._integral) + (self.kd * self._filtered_derivative)
        self._last_error = error  # Update the last error value

        # Update the motor speeds based on the speed difference
        self._left_speed = self.max_speed + speed_difference 
        self._right_speed = self.max_speed - speed_difference 

        # Limit the motor speeds to be within the range [0, max_speed]
        self._left_speed = max(0, min(self._left_speed, self.max_speed))
        self._right_speed = max(0, min(self._right_speed, self.max_speed))

    def get_left_speed(self):
        """Returns the current left motor speed."""
        return self._left_speed
    
    def get_right_speed(self):
        """Returns the current right motor speed."""
        return self._right_speed




