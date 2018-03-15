# Yes, yes, globals are bad

from Tkinter import Text, END


targetsHome = None
targets = None
speeds = None
selectedLeg = 0
selectedInput = 0
showTargets = True


def logMessage(msg):
    messageBox.insert(END, msg + "\n")


def messageBoxModifiedCallback(self):
    messageBox.see(END)
    messageBox.edit_modified(False)
