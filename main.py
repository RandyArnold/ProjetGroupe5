#!/usr/bin/env pybricks-micropython
#from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B, SpeedPercent, MoveTank
#from ev3dev2.sensor import INPUT_1
#from ev3dev2.sensor.lego import TouchSensor, InfraredSensor
#from ev3dev2.led import Leds
import time
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, InfraredSensor, TouchSensor, UltrasonicSensor, GyroSensor
from pybricks.parameters import  Port, Stop, Direction, Button, Color
import os


import array
from math import *


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# Create your objects here.
ev3 = EV3Brick()

sens = UltrasonicSensor(Port.S1)
gyro = GyroSensor(Port.S4)

LeftMotor = Motor(Port.A)
RightMotor = Motor(Port.B)

#gyro_360 = 325
gyro_360 = 340

time_360 = 4700

# Write your program here.
motor_speed = 200

def scan(nb_measures, current_x, current_y, current_rotation, max_scan_distance) :
    #initialisation d'une liste vide 
    scan_results=[]
    gyro.reset_angle(0)
    #pour chaque étape

    rotation_time = time_360/nb_measures
    now = time.time()
    for i in range(0,nb_measures):
        #on mesure la distance 
        distance = sens.distance()
        scan_results.append(distance)
        #On calcule les coordonnées 
        #x = int(distance*cos((2*pi*(i+current_rotation))/nb_measures))+current_x
        #y = int(distance*sin((2*pi*(i+current_rotation))/nb_measures))+current_y
        teta = 2*pi*current_rotation/nb_measures
        x_a = distance*cos((2*pi*i)/nb_measures)
        y_a = distance*sin((2*pi*i)/nb_measures)
        x_rot=x_a*cos(teta)-y_a*sin(teta)
        y_rot=x_a*sin(teta)+y_a*cos(teta)
        x=x_rot+current_x
        y=y_rot+current_y

        # LeftMotor.run_time(motor_speed, rotation_time, then=Stop.HOLD, wait=False)
        # RightMotor.run_time(-motor_speed, rotation_time, then=Stop.HOLD, wait=True)

        while(gyro.angle() < (gyro_360/nb_measures)*(i+1)):
           LeftMotor.run(motor_speed) 
           RightMotor.run(-motor_speed)
        if(distance<max_scan_distance): 
            with open('measures.txt', 'a') as the_file:
                the_file.write(str(x)+','+str(y)+'\n') 
    return scan_results

def findPath(scan_results, travel_distance, security_distance_margin) :
    next_rotation = 0
    for scan_result in scan_results:
        if scan_result<travel_distance+security_distance_margin:
            next_rotation+=1
        else :
            return int(next_rotation)

    #return False

def rotation(next_rotation, current_rotation, nb_measures):
    current_rotation+=next_rotation
    gyro.reset_angle(0)

    rotation_time = time_360*next_rotation/nb_measures

    # LeftMotor.run_time(motor_speed, rotation_time, then=Stop.HOLD, wait=False)
    # RightMotor.run_time(-motor_speed, rotation_time, then=Stop.HOLD, wait=True)

    while(gyro.angle() < (gyro_360/nb_measures)*(next_rotation)):
        LeftMotor.run(motor_speed) 
        RightMotor.run(-motor_speed)
    return int(current_rotation)

def moveForward(nb_measures, current_x, current_y, current_rotation, travel_distance):
    LeftMotor.run_angle(200, -360, then=Stop.HOLD, wait=False)
    RightMotor.run_angle(200, -360, then=Stop.HOLD, wait=True)
    current_x+=travel_distance*cos(2*pi*current_rotation/nb_measures)
    current_y+=travel_distance*sin(2*pi*current_rotation/nb_measures)
    return [current_x, current_y]
    

def execute(nb_measures, wheels_diameter, security_distance_margin, execution_time, max_scan_distance):
    current_x=0
    current_y=0
    current_rotation=int(0)
    travel_distance=pi*wheels_diameter
    start_time = time.time()
    execution_time = abs(start_time+execution_time)
    cycles_nb=15
    counter=0
    for i in range(0, cycles_nb):
        scan_results = scan(nb_measures, current_x, current_y, current_rotation, max_scan_distance)
        next_rotation = findPath(scan_results, travel_distance, security_distance_margin)
        #if(next_rotation != False): 
         #   rotation(next_rotation, current_rotation)
          #  moveForward(nb_measures, current_x, current_y, current_rotation, travel_distance)
        #else: 
        #    break
        current_rotation = rotation(next_rotation, current_rotation, nb_measures)           
        new_coordonates = moveForward(nb_measures, current_x, current_y, current_rotation, travel_distance)
        current_x = new_coordonates[0]
        current_y = new_coordonates[1]
        with open('robotposition.txt', 'a') as the_file:
            the_file.write(str(current_x)+','+str(current_y)+'\n')
        with open('robotrotation.txt', 'a') as the_file:
            the_file.write(str(current_rotation)+'\n')
        counter+=1
        
def timeTest(execution_time): 
    start_time = time.time()
    execution_time = abs(start_time+execution_time)
    with open('measures.txt', 'a') as the_file:
        the_file.write(str(start_time)+'\n')
    with open('measures.txt', 'a') as the_file:
       the_file.write(str(execution_time)+'\n')

def do_a_360_time(execution_time): 
    gyro.reset_angle(0)
   
    LeftMotor.run_time(motor_speed, execution_time, then=Stop.HOLD, wait=False)
    RightMotor.run_time(-motor_speed, execution_time, then=Stop.HOLD, wait=True)

def do_a_360_gyro(test_angle): 
    gyro.reset_angle(0)
    while(gyro.angle() < test_angle):
        LeftMotor.run(motor_speed) 
        RightMotor.run(-motor_speed)

execute(32, 55, 450, 60000000000, 600)

#do_a_360_gyro(325)

#do_a_360(13463)

#timeTest(60000)
           
#moveForward()

##scan(360,0,0,0)

#scan(16,0,0,0)
