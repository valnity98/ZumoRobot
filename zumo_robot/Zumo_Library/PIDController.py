#!/usr/bin/env python3

'''
 ------------------------------------------------------------------------------
 File: Zumo328PPID.py
 Author: Mutasem Bader
 Description:
     This file contains the implementation of the PID controller for the Zumo robot.
     The PID controller adjusts the motor speeds based on the measured position 
     error between the target and measured position, utilizing proportional 
     and derivative control (without integral control).
 
     Usage:
     - Instantiate the class with the maximum speed.
     - Call `control_speed` with the measured position, target position, and PID gains 
       (kp and kd) to update the motor speeds.
     - Use `get_left_speed` and `get_right_speed` to retrieve the calculated motor speeds.
 
 ------------------------------------------------------------------------------ 
'''

import time

class Zumo328PPID:
    def __init__(self, max_speed=200.0):
        """
        Initialize the PID controller for the Zumo robot.
        
        Args:
        max_speed (float): Maximum speed for the motors. Default is 200.0.
        """
        self.max_speed = max_speed  # Maximum speed of the motors
        self._left_speed = 0  # Initial left motor speed
        self._right_speed = 0  # Initial right motor speed
        self._prevT = time.time()  # Previous time (for deltaT calculation)
        self._last_error = 0  # Last error value

    def control_speed(self, measured_position, target_position, kp, kd, aktiv=True):
        """
        Calculate and update the motor speeds based on the PID control.
        
        Args:
        measured_position (float): The current measured position of the robot.
        target_position (float): The desired target position of the robot.
        kp (float): The proportional gain for the PID controller.
        kd (float): The derivative gain for the PID controller.
        aktiv (bool): Flag to indicate whether the PID controller should be active.
        """
        error = measured_position - target_position  # Calculate the error (distance from target position)

        # Only update if the PID controller is active
        if aktiv:
            currT = time.time()  # Get the current time in seconds
            deltaT = currT - self._prevT  # Calculate the time difference
            
            # Prevent division by zero in case deltaT is too small
            if deltaT <= 0.0:
                deltaT = 1e-6  # Set to a small value if deltaT is too small to avoid division by zero

            self._prevT = currT  # Update the previous time

            # Calculate the speed difference using the PID formula (P and D terms)
            speed_difference = (kp * error) + (kd * ((error - self._last_error) / deltaT))
            self._last_error = error  # Update the last error value

            # Update the motor speeds based on the speed difference
            self._left_speed = self.max_speed + speed_difference
            self._right_speed = self.max_speed - speed_difference

            # Limit the motor speeds to be within the range [0, max_speed]
            self._left_speed = max(0, min(self._left_speed, self.max_speed))
            self._right_speed = max(0, min(self._right_speed, self.max_speed))
        else:
            # If not active, leave motor speeds as they are (or reset to 0)
            self._left_speed = 0
            self._right_speed = 0

    def get_left_speed(self):
        """Returns the current left motor speed."""
        return self._left_speed
    
    def get_right_speed(self):
        """Returns the current right motor speed."""
        return self._right_speed
