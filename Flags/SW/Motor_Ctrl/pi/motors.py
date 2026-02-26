

from dataclasses import dataclass
from typing import List, Tuple

# I2C library
from smbus import SMBus
import RPi.GPIO as GPIO

import time
from threading import Thread

#IFACE = 'GPIO'
IFACE = 'I2C'


#FIXME For now, velocity is not in °/s but °/30ms, so need this constant here.
# When acceleration and velocity are normalized,
# then this module would not need this constant.
GUI_LOOP_PERIOD = 0.03 # 30ms

@dataclass
class MotorState:
    angle: float
    velocity: float
    acceleration: float
    pattern_time: float
    sub_patterns: List[str]
    color: str
    width: int
    active_instrument: str = None

if IFACE == 'GPIO':
    N_MOTORS = 1
elif IFACE == 'I2C':
    N_MOTORS = 20

# 28BYJ-48
# 2ms/step
# 500 steps/s
# 2048 steps/rev
# 4.096 s/rev
# v = 88°/s

# 2ms is max for 28BYJ-48
MOTOR_LOOP_PERIOD = 2e-3

I2C_ADDR = 0x20 # I2C addr of PCF8574
MOTORS_PER_I2C_BUS = 10

class MotorsDriver:
    
    def _open_i2c_bus(self, bus_i):
        try:
            self.bus[bus_i] = SMBus(bus_i) # bus index of the 40pin header
        except FileNotFoundError as e:
            print(f'WARNING: Cannot open I2C bus No {bus_i}!')
            print(f'Please run install_RPi.sh or install_JetsonNano.sh and reboot.')
            print('Original error:')
            print(e)

    def __init__(self):
        if IFACE == 'GPIO':
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(10, GPIO.OUT)
            GPIO.setup(11, GPIO.OUT)
            GPIO.setup(12, GPIO.OUT)
            GPIO.setup(13, GPIO.OUT)
        elif IFACE == 'I2C':
            self.bus = [None, None]
            for bus_i in range(N_MOTORS//MOTORS_PER_I2C_BUS):
                self._open_i2c_bus(bus_i)
            # Set all to 0
            for i0 in range(0, N_MOTORS, 2):
                self._write_i2c_motor_pair(i0, 0x0, 0x0)

        #TODO Maybe in °? Or at least getters with degrees.
        # In steps units.
        self.current = [0.0]*N_MOTORS
        self.target = [0.0]*N_MOTORS
        self.velocity = [0.0]*N_MOTORS

        def step_loop():
            i = 0
            while True:
                # Calculate next step.
                for i in range(N_MOTORS):
                    # How much to move in 2ms
                    delta = self.velocity[i]*MOTOR_LOOP_PERIOD

                    diff = self.target[i] - self.current[i]

                    if diff != 0:
                        if abs(diff) <= delta:
                            self.current[i] = self.target[i]
                        else:
                            #FIXME Should not do this, but seems that velocity is absolute.
                            if diff > 0:
                                self.current[i] += abs(delta)
                            else:
                                self.current[i] -= abs(delta)
                self._send_to_motors()
                
                time.sleep(MOTOR_LOOP_PERIOD)
        
        Thread(target=step_loop, daemon=True).start()

    def _write_i2c_motor_pair(self, i0, data0, data1):
        bus_i = i0//MOTORS_PER_I2C_BUS
        i_pair = i0%MOTORS_PER_I2C_BUS >> 1
        data = (data1 << 4) | data0
        bus = self.bus[bus_i]
        if bus:
            bus.write_byte(I2C_ADDR + i_pair, data)

    def _send_to_motors(self):
        if IFACE == 'GPIO':
            for i in range(N_MOTORS):
                i_step = round(self.current[i])
                # coil could be 0001, 0010, 0100, 1000
                coil = 1 << (i_step & 0x3)
                GPIO.output(10, coil>>0 & 1)# Take 0th bit and turn on/off the 0th coil.
                GPIO.output(11, coil>>1 & 1)
                GPIO.output(12, coil>>2 & 1)
                GPIO.output(13, coil>>3 & 1)
        elif IFACE == 'I2C':
            for i0 in range(0, N_MOTORS, 2):
                i1 = i0+1
                step0 = round(self.current[i0])
                step1 = round(self.current[i1])
                # coil could be 0001, 0010, 0100, 1000
                coil0 = 1 << (step0 & 0x3)
                coil1 = 1 << (step1 & 0x3)
                self._write_i2c_motor_pair(i0, coil0, coil1)



    def update(self, motor_idx, motor_state):
        if motor_idx < 0 or motor_idx >= N_MOTORS:
            raise IndexError(f'Motor index is in range [0, N_MOTORS)!')
        
        a = motor_state.angle
        #FIXME This angle is °/30ms, converting it to °/s
        v = motor_state.velocity/0.03

        # Because it is too fast for 28BYJ-48, we scale speed and angle
        scale = 1/10
        v *= scale
        a *= scale

        # If no v do it fastest possible.
        if v == 0:
            v = 87

        # Limit max speed.
        if v > 87:
            print('WARN: Too fast')
            v = 87

        # Angle to step.
        sa = a/360*2048
        sv = v/360*2048

        self.target[motor_idx] = sa
        self.velocity[motor_idx] = sv


        if False:
            print(a)
            print(v)
            print(sa)
            print(sv)
            print()
