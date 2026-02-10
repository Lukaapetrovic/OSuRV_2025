#!/usr/bin/env python3


from motors import *
import time

motor_state = MotorState(
    angle=0.0,
    velocity=0.0,
    acceleration=0.0,
    pattern_time=0.0,
    sub_patterns=[],
    color='black',
    width=2,
    active_instrument=None
)
motors_driver = MotorsDriver()


'''
# Move 0th motor
motor_state.angle = 300.0
motor_state.velocity = 25.0
motors_driver.update(0, motor_state)
time.sleep(1)

motor_state.angle = 0.0
motor_state.velocity = 10.0
motors_driver.update(0, motor_state)
time.sleep(1)
'''


# Move all 20 motors
for i in range(20):
    motor_state.angle = 300.0
    motor_state.velocity = 25.0
    motors_driver.update(i, motor_state)
time.sleep(1)

for i in range(20):
    motor_state.angle = 0.0
    motor_state.velocity = 10.0
    motors_driver.update(i, motor_state)
time.sleep(1)


print('End')