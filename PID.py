#!/usr/bin/env python
# coding: utf-8
#  PID control
# *****************************************************************#
#                      Position type PID system                   #
# *****************************************************************#
class PositionalPID:
    def __init__(self, P, I, D):
        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.SystemOutput = 0.0
        self.ResultValueBack = 0.0
        self.PidOutput = 0.0
        self.Iterm = 0.0
        self.preErr = 0.0
        self.outMin = -400 # 2-axis
        self.outMax = 400  # 2-axis
    # Set PID controller parameters
    def SetStepSignal(self, StepSignal):
		# Clamped output and Integral term
        Err = StepSignal - self.SystemOutput
        KpWork = self.Kp * Err
        self.Iterm += self.Ki * Err
        #KdWork = self.Kd * (Err - self.preErr)
        KdWork = self.Kd * (self.SystemOutput - self.preErr)

        if (self.Iterm > self.outMax): self.Iterm = self.outMax
        elif(self.Iterm < self.outMin): self.Iterm= self.outMin

        #self.PidOutput = KpWork + self.Iterm + KdWork
        self.PidOutput = KpWork + self.Iterm - KdWork # subtract Kdterm to remove derivative kick

        if(self.PidOutput > self.outMax): self.PidOutput = self.outMax;
        elif(self.PidOutput < self.outMin): self.PidOutput = self.outMin;

        #self.preErr = Err
        self.preErr = self.SystemOutput
        #print(f'P={KpWork:4.1f}, I{self.Iterm:4.1f}, D{KdWork:4.1f}, out{self.PidOutput:4.1f}, err{Err:4.1f}')

    # Set InertiaTime as inertial time constant in the first-order inertial link system
    def SetInertiaTime(self, InertiaTime, SampleTime):
        self.SystemOutput = (InertiaTime * self.ResultValueBack + \
                             SampleTime * self.PidOutput) / (SampleTime + InertiaTime)
        self.ResultValueBack = self.SystemOutput
