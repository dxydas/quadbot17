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
        speed = 150  # Fixed speed for now
        # Legs
        for i, leg in enumerate(self.robot.legs):
            #print("leg:", leg.id, ", angles:", leg.angles)
            n = len(leg.angles)
            for j in range(0, n):
                if (i % 2 == 0):
                    # Left side
                    sd = self.robot.leftLegServoDirections[j]
                else:
                    # Right side
                    sd = self.robot.rightLegServoDirections[j]
                so = self.robot.legServoOffsets[j]
                x = int( rescale( (leg.angles[j] + so)*sd, -150.0, 150.0, 0, 1023 ) )
                writeStr += str(leg.joints[j].id) + " " + str(x) + " " + str(speed) + " "
        # Spine
        #print("spine angles:", self.robot.spine.angles)
        n = len(self.robot.spine.angles)
        for j in range(0, n, 2):  # Skip dummy joint
            sd = self.robot.spineServoDirections[j]
            so = self.robot.spineServoOffsets[j]
            x = int( rescale( (self.robot.spine.angles[j] + so)*sd, -150.0, 150.0, 0, 1023 ) )
            writeStr += str(self.robot.spine.joints[j].id) + " " + str(x) + " " + str(speed)
            if (j == n - 1):
                writeStr += "\n"
            else:
                writeStr += " "
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
