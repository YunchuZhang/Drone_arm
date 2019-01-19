import sys
import time

import navio.pwm
import navio.util
import datetime

navio.util.check_apm()

#PWM_OUTPUT = 1
PWM_OUTPUT_0  = 0
PWM_OUTPUT_1  = 1
PWM_OUTPUT_2  = 2
PWM_OUTPUT_3  = 3
SERVO_MIN = 1.250 #ms
SERVO_MAX = 1.750 #ms
SERVO_NOM = 1.500 #ms
SERVO_NOM_1 = 1.350
SERVO_STOP = 0.000

def loop_for(seconds, func, *args):
    endTime = datetime.datetime.now() + datetime.timedelta(seconds=seconds)

    while True:
        if datetime.datetime.now() >= endTime:
            break
        func(*args)



def calibration_ESC():
    #pwm =  navio.pwm.PWM(PWM_OUTPUT)
    pwm0 = navio.pwm.PWM(PWM_OUTPUT_0)
    pwm1 = navio.pwm.PWM(PWM_OUTPUT_1)
    pwm2 = navio.pwm.PWM(PWM_OUTPUT_2)
    pwm3 = navio.pwm.PWM(PWM_OUTPUT_3)
   # pwm.initialize()
   # pwm.set_period(50)
   # pwm.enable()
    pwm0.initialize()
    pwm1.initialize()
    pwm2.initialize()
    pwm3.initialize()

    pwm0.set_period(50)
    pwm1.set_period(50)
    pwm2.set_period(50)
    pwm3.set_period(50)

    pwm0.enable()
    pwm1.enable()
    pwm2.enable()
    pwm3.enable()

    print("SERVO_MAX")
    loop_for(5, pwm0.set_duty_cycle, SERVO_MAX)
    print("SERVO_MIN")
    loop_for(5, pwm0.set_duty_cycle, SERVO_MIN)
    print("Stop")
    loop_for(2, pwm0.set_duty_cycle, SERVO_STOP)


    print("SERVO_MAX")
    loop_for(5, pwm1.set_duty_cycle, SERVO_MAX)
    print("SERVO_MIN")
    loop_for(5, pwm1.set_duty_cycle, SERVO_MIN)
    print("Stop")
    loop_for(2, pwm1.set_duty_cycle, SERVO_STOP)

    #pwm2 = navio.pwm.PWM(PWM_OUTPUT_2)
    #pwm2.initialize()
    #pwm2.set_period(50)
    #pwm2.enable()
    print("SERVO_MAX")
    loop_for(5, pwm2.set_duty_cycle, SERVO_MAX)
    print("SERVO_MIN")
    loop_for(5, pwm2.set_duty_cycle, SERVO_MIN)
    print("Stop")
    loop_for(2, pwm2.set_duty_cycle, SERVO_STOP)

    print("SERVO_MAX")
    loop_for(5, pwm3.set_duty_cycle, SERVO_MAX)
    print("SERVO_MIN")
    loop_for(5, pwm3.set_duty_cycle, SERVO_MIN)
    print("Stop")
    loop_for(2, pwm3.set_duty_cycle, SERVO_STOP)



   # print("SERVO_MIN_AGAIN")
   # loop_for(8, pwm.set_duty_cycle, SERVO_MIN)
   # print("SERVO_NOM")
   # loop_for(2, pwm.set_duty_cycle, SERVO_NOM_1)
   # print("SERVO_NOM_CHANGE")
   # loop_for(2, pwm.set_duty_cycle, SERVO_NOM)
   # print("Stop")
   # loop_for(2, pwm.set_duty_cycle, SERVO_STOP)

calibration_ESC()


