# motorNMFlexDC
EPICS asyn motor support for the Nanomotion FlexDC motion controller.

Please refer to the provided example IOC, flexdcIOC, for specifics.

### Usage, in short:
1. Create ethernet asyn port:
	```drvAsynIPPortConfigure("NMCTRL", "192.168.100.10:4000")```
2. Configure FlexDC to above asyn port:
	```NMFlexDCCreateController("NMFLEXDC", "NMCTRL", 2, 50, 100)```
3. Load asynMotor DTYP motor record(s):
	```dbLoadTemplate("flexdc.substitutions")```

The ```NMFlexDCCreateController``` command follows the usual API ```(portName, asynPortName, numAxes, movingPollingRate, idlePollingRate)```.

