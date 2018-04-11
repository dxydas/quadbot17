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
        self.ser = 0
        self.port = "/dev/ttyUSB0"
        self.serialOK = False
        self.serialDisconnected = False


    def run(self):
        while not self.event.isSet():
            if not self.serialOK:
                try:
                    self.ser = serial.Serial(self.port, 38400)
                    self.messageLogger.log("Serial port " + self.port + " connected")
                    self.serialOK = True
                    self.serialDisconnected = False
                except serial.SerialException:
                    self.serialOK = False
                    if self.serialDisconnected == False:
                        self.messageLogger.log("Serial port " + self.port + " not connected")
                        self.serialDisconnected = True
                    sleep(2)
            else:
                pollSerial()
            self.event.wait(self.dt)


    def stop(self):
        self.event.set()


    def pollSerial():
        writeStr = ""
        for i, leg in enumerate(self.robot.legs):
            #print("leg:", leg.id, "angles:", leg.angles)
            for j in range(len(leg.angles)):
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
                writeStr += str(leg.joints[j].id) + "," + str(x)
                if (i == 3) and ( j == (len(leg.angles) - 1) ):
                    writeStr += "\n"
                else:
                    writeStr += ","
        #print("writeStr: ", writeStr)
        try:
            self.ser.write(writeStr)
        except serial.SerialException:
            self.messageLogger.log("Serial write error")
            self.ser.close()
            self.serialOK = False


    def closeSerial(self):
        if self.serialOK:
            self.ser.close()
