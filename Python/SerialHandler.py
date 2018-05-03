import Params
from HelperFunctions import rescale

import threading
import serial
from time import sleep


class SerialHandler(threading.Thread):
    def __init__(self, messageLogger, robot):
        self.messageLogger = messageLogger
        self.robot = robot

        # Threading/timing vars
        threading.Thread.__init__(self)
        self.event = threading.Event()
        self.dt = 0.05  # 50 ms

        # Input vars
        self.ser = None
        self.serialOK = False
        self.serialDisconnected = False


    def run(self):
        while not self.event.isSet():
            if not self.serialOK:
                try:
                    self.ser = serial.Serial(Params.serialPort, Params.serialBaudRate)
                    self.messageLogger.log("Serial port " + Params.serialPort + " connected")
                    self.serialOK = True
                    self.serialDisconnected = False
                except serial.SerialException:
                    self.serialOK = False
                    if self.serialDisconnected == False:
                        self.messageLogger.log("Serial port " + Params.serialPort + " not connected")
                        self.serialDisconnected = True
                    sleep(2)
            else:
                self.poll()
            self.event.wait(self.dt)


    def stop(self):
        self.event.set()


    def poll(self):
        writeStr = ""
        speed = 100  # Fixed speed for now
        # Legs
        for i, leg in enumerate(self.robot.legs):
            #print("leg:", leg.id, "angles:", leg.angles)
            n = len(leg.angles)
            for j in range(0, n):
                if (i % 2 == 0):
                    # Left side
                    # Joint 2 needs its direction inverted
                    if j == 1: # Joint 2 (zero-indexed)
                        x = int( rescale(-leg.angles[j], -180.0, 180.0, 0, 1023) )
                    else:
                        x = int( rescale(leg.angles[j], -180.0, 180.0, 0, 1023) )
                else:
                    # Right side
                    # All joints except for 1 and 5 are mirrors of left side
                    if (j == 2) or (j == 3):  # Joints 3 & 4 (zero-indexed)
                        x = int( rescale(-leg.angles[j], -180.0, 180.0, 0, 1023) )
                    else:  # Joints 1, 2 & 5 (zero-indexed)
                        x = int( rescale(leg.angles[j], -180.0, 180.0, 0, 1023) )
                writeStr += str(leg.joints[j].id) + " " + str(x) + " " + str(speed) + " "

        # Spine
        # Joint 1
        j = 0
        x = int( rescale(self.robot.spine.angles[j], -180.0, 180.0, 0, 1023) )
        writeStr += str(self.robot.spine.joints[j].id) + " " + str(x) + " " + str(speed) + " "
        # Skip dummy joint
        # Joint 2 (inverted)
        j = 2
        x = int( rescale(-self.robot.spine.angles[j], -180.0, 180.0, 0, 1023) )
        writeStr += str(self.robot.spine.joints[j].id) + " " + str(x) + " " + str(speed) + "\n"

        # Write
        self.send(writeStr)


    def send(self, msg):
        #print("msg: ", msg)
        try:
            self.ser.write(msg.encode("utf-8"))
        except serial.SerialException:
            self.messageLogger.log("Serial write error")
            self.serialOK = False


    def closeSerial(self):
        if self.serialOK:
            self.ser.flush()
            self.ser.close()
            self.messageLogger.log("Serial port closed")
