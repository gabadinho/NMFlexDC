# Nanomotion FlexDC controller support

# Load motor record
dbLoadTemplate("flexdc.substitutions")

# Configure asyn IP address
drvAsynIPPortConfigure("NMCTRL", "192.1168.100.1:4000")

# Load asyn record
dbLoadRecords("$(ASYN)/db/asynRecord.db", "P=FLEXDC:, R=ASYN1, PORT=NMCTRL, ADDR=0, OMAX=256, IMAX=256")

# Turn on asyn trace
asynSetTraceMask("NMCTRL", 0, 0x03)
asynSetTraceIOMask("NMCTRL", 0, 0x04)

# NMFlexDCCreateController(portName, asynPort, numAxes, movingPollingRate, idlePollingRate)
NMFlexDCCreateController("NMFLEXDC", "NMCTRL", 2, 50, 200)

# Turn off asyn trace
asynSetTraceMask("NMCTRL", 0, 0x01)
asynSetTraceIOMask("NMCTRL", 0, 0x00)
