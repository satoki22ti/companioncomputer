#install some libs
import math
from logging import Logger
from .config import (
        WinchingConfiguration as Config,
        EventConfiguration as EventConfig,
        TelemetryLoggingConfiguration as LoggingConfig
        )
from .telemetry_logger import TelemetryLogger
from winch_control.motor_driver.velocity_driver import VelocityDriver
from winch_control.motor_driver.types import State as MotorState, Telemetry
from winch_control.calculators.acceleration_calculator import AccelerationCalculator

class ASR:
    """ASR, Anti-Swing Restaction is an algorithm to minimise the swing amplitude while the retraction
    base: minimal retraction velocity, sigma: sensitivity """

    def __init__(self, max_rectaction_vel, count, lpf, logger: Logger):
        self.lpf = lpf
        self.max_vel = max_rectaction_vel
        self.tension_old = 0.0
        self.tension_lfp = 0.0
        self.flag = 0
        self.tension_list = []
        self.count  = count
        self.logger= logger
        self.len = 0

    def tension_update(self, new_tension) ->None:
        tension_old = self.tension_lpf
        new_tension = self.lpf* new_tension + (1-self.lpf)* tension_old # apply eponential smoother
        self.tension_list.append(new_tension)


    
    def ASR(self, len) -> float:
        self.len = len

        if self.len > 0.2 and self.len < 5.0:
            # ASR
            #self.logger.debug('CONTROL: ASR activated')
            if self.flag(self.flag == 0):
                T = self.tension_list[-1]

            else:
                self.flag = self.flag - 1

            if T < math.average(self.tension_list): #when velocity is expected to be high
                #self.logger.debug('CONTROL: ASR slow swing detected')
                retract = self.max_vel
                self.flag = self.count # avoid tension calculation for a while
                
                return retract

            else: # when velocity is high
                #self.logger.debug('CONTROL: ASR fast swing detected')
                return self.max_vel/1.5
                

            

        else:
            return self.max_vel