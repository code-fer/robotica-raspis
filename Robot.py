#!/usr/bin/python
# -*- coding: UTF-8 -*-
from __future__ import print_function # use python 3 syntax but make it compatible with python 2
from __future__ import division       #                           ''

import brickpi3 # import the BrickPi3 drivers
import time     # import the time library for the sleep function
import sys
import math
import numpy as np

# tambien se podria utilizar el paquete de threading
from multiprocessing import Process, Value, Array, Lock



class Robot:
    def __init__(self, init_position=[0.0, 0.0, 0.0]):
        """
        Initialize basic robot params. \

        Initialize Motors and Sensors according to the set up in your robot
        """



######## UNCOMMENT and FILL UP all you think is necessary (following the suggested scheme) ########

        # Robot construction parameters
        #self.R = ?? 5.5(d) --> 2.75(r)
        #self.L = ?? 10.30
        #self. ...


        ##################################################
        # Motors and sensors setup
        
        # Create an instance of the BrickPi3 class. BP will be the BrickPi3 object.
        self.BP = brickpi3.BrickPi3()

        # Configure sensors, for example a touch sensor.
        #self.BP.set_sensor_type(self.BP.PORT_1, self.BP.SENSOR_TYPE.TOUCH)

        # reset encoder B and C (or all the motors you are using)
        self.MOT_DCHA_PORT = self.BP.PORT_B
        self.MOT_IZQ_PORT = self.BP.PORT_C
        self.BP.offset_motor_encoder(self.MOT_DCHA_PORT,
            self.BP.get_motor_encoder(self.MOT_DCHA_PORT))
        self.BP.offset_motor_encoder(self.MOT_IZQ_PORT,
            self.BP.get_motor_encoder(self.MOT_IZQ_PORT))

        ##################################################
        # odometry shared memory values
        self.x = Value('d', init_position[0])
        self.y = Value('d', init_position[1])
        self.th = Value('d', init_position[2])
        self.v = Value('d',0.0)
        self.w = Value('d',0.0)
        self.finished = Value('b',1) # boolean to show if odometry updates are finished

        # if we want to block several instructions to be run together, we may want to use an explicit Lock
        self.lock_odometry = Lock()

        # odometry update period --> UPDATE value!
        self.P = 1.0

        """"FLAGS GLOBALES (CONFIGURACION HARDWARE)"""
        #GYRO_PORT = ...
        self.RADIO_RUEDAS = 27.5 #mm
        self.SEP_RUEDAS = 103.0 #mm

    def setSpeed(self, v,w):
        """ To be filled - These is all dummy sample code """
        print("setting speed to %.2f %.2f" % (v, w))

        # compute the speed that should be set in each motor ...

        #speedPower = 100
        #BP.set_motor_power(BP.PORT_B + BP.PORT_C, speedPower)

        matr_trans=np.array([1/self.RADIO_RUEDAS,self.SEP_RUEDAS/(2*self.RADIO_RUEDAS)],[1/self.RADIO_RUEDAS,-self.SEP_RUEDAS/(2*self.RADIO_RUEDAS)])
        vc=np.array([v,w])
        wr=matr_trans@vc #vel_ruedas=[w_mot_izq, w_mot_dcho]

        speedDPS_left = math.degrees(wr[1])
        speedDPS_right = math.degrees(wr[0])

        #aplicar velocudad a los motores
        self.BP.set_motor_dps(self.MOT_IZQ_PORT, speedDPS_left)
        self.BP.set_motor_dps(self.MOT_DCHA_PORT, speedDPS_right)
        print(wr[0])

        self.lock_odometry.acquire()
        v = self.v.value
        w = self.w.value
        self.lock_odometry.release()


    def readSpeed(self):
        self.lock_odometry.acquire()
        v = self.v.value
        w = self.w.value
        self.lock_odometry.release()

        return v,w

    def readOdometry(self):
        """ Returns current value of odometry estimation """
        self.lock_odometry.acquire()
        x = self.x.value
        y = self.y.value
        th = self.th.value
        self.lock_odometry.release()
        return x, y, th

    def startOdometry(self):
        """ This starts a new process/thread that will be updating the odometry periodically """
        self.finished.value = False
        self.p = Process(target=self.updateOdometry, args=()) #additional_params?))
        self.p.start()
        print("PID: ", self.p.pid)

    # You may want to pass additional shared variables besides the odometry values and stop flag
    def updateOdometry(self): #, additional_params?):
        """ To be filled ...  """

        motor_izq_ant = self.BP.get_motor_encoder(self.MOT_IZQ_PORT)
        motor_dcha_ant = self.BP.get_motor_encoder(self.MOT_DCHA_PORT)


        while not self.finished.value:
            # current processor time in a floating point value, in seconds
            tIni = time.clock()

            # compute updates

            motor_izq_act = self.BP.get_motor_encoder(self.MOT_IZQ_PORT)
            motor_dcha_act = self.BP.get_motor_encoder(self.MOT_DCHA_PORT)

            speedDPS_left = (motor_izq_act - motor_izq_ant) /self.P
            speedDPS_right = (motor_dcha_act - motor_dcha_ant) /self.P

            motor_izq_ant = motor_izq_act
            motor_izq_ant = motor_izq_act

            wi = np.radians(speedDPS_left)
            wd = np.radians(speedDPS_right)

            wr = np.array([wd,wi])

            matr_trans=np.array([self.RADIO_RUEDAS/2, self.RADIO_RUEDAS/2], [self.RADIO_RUEDAS/self.SEP_RUEDAS, -self.RADIO_RUEDAS/self.SEP_RUEDAS])
            vc = matr_trans@wr #vc=[vel_lin, vel_ang]

            v = vc[0]
            w = vc[1]

            ######## UPDATE FROM HERE with your code (following the suggested scheme) ########
            sys.stdout.write("Dummy update of odometry ...., X=  %d, \
                Y=  %d, th=  %d \n" %(self.x.value, self.y.value, self.th.value) )
            #print("Dummy update of odometry ...., X=  %.2f" %(self.x.value) )

            # update odometry uses values that require mutex
            # (they are declared as value, so lock is implicitly done for atomic operations, BUT =+ is NOT atomic)

            # Operations like += which involve a read and write are not atomic.
            with self.x.get_lock():
                self.x.value+=1

            # to "lock" a whole set of operations, we can use a "mutex"
            self.lock_odometry.acquire()
            if w == 0:
                self.x.value += v*self.P * math.cos(self.th.value)
                self.y.value += v*self.P * math.sin(self.th.value)
            else:
                self.x.value += (v/w) * (math.sin(self.th.value + w*self.P) - math.sin(self.th.value))
                self.y.value += (v/w) * (-math.cos(self.th.value + w*self.P) + math.cos(self.th.value))
            x = self.x.value
            y = self.y.value
            th = self.th.value
            self.lock_odometry.release()

            try:
                # Each of the following BP.get_motor_encoder functions returns the encoder value
                # (what we want to store).
                sys.stdout.write("Reading encoder values .... \n")
                #[encoder1, encoder2] = [self.BP.get_motor_encoder(self.BP.PORT_B),
                #    self.BP.get_motor_encoder(self.BP.PORT_C)]
            except IOError as error:
                #print(error)
                sys.stdout.write(error)

            #sys.stdout.write("Encoder (%s) increased (in degrees) B: %6d  C: %6d " %
            #        (type(encoder1), encoder1, encoder2))


            # save LOG
            # Need to decide when to store a log with the updated odometry ...

            ######## UPDATE UNTIL HERE with your code ########


            tEnd = time.clock()
            time.sleep(self.P - (tEnd-tIni))

        #print("Stopping odometry ... X= %d" %(self.x.value))
        sys.stdout.write("Stopping odometry ... X=  %.2f, \
                Y=  %.2f, th=  %.2f \n" %(self.x.value, self.y.value, self.th.value))


    # Stop the odometry thread.
    def stopOdometry(self):
        self.finished.value = True
        #self.BP.reset_all()

robot = Robot()
robot.setSpeed(3,4)