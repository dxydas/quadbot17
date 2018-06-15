# Serial port and baud rate
# /dev/ttyUSB0: ArbotiX-M
# /dev/ttyACM0: OpenCM 9.04 with OpenCM 485 Expansion Board
serialPort = "/dev/ttyACM0"
serialBaudRate = 38400

# Screen size var
# 1: HD screen
# 2: 4K screen
scsz = 2

# Canvas width/height
canvasW = scsz*585
canvasH = scsz*380

# Default font
defaultFont = ("System", 12)

# Graphical representation
# 0: None
# 1: 2D
# 2: 3D
gui = 1

# Dummy adjustment while IMU is not present
spineJointsDummyAdjustment = True

# Flag to show/hide target widgets
showTargets = True

# Flag that indicates a load targets timer is still running
loadTargetsTimerRunning = False

# Input mode select vars
numOfModes = 5
inputModeSelect = 0

# Input "force" vars
inputForceMax = 1000.0
dragForceCoef = 5.0
