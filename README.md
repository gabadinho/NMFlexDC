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

### Extra records:
- ```$(P)$(M)_HOMR_CMD```
Which macro to use when HOMR field is _'put_ (disabled, reverse limit-switch, home index mark).
- ```$(P)$(M)_HOMF_CMD```
Which macro to use when HOMF field is _'put_ (disabled, forward limit-switch, home index mark).
- ```$(P)$(M)_HOMS_CMD```
Status of the homing macro (11th value of parameters array),
- ```$(P)$(M)_RST_CMD```
Kills running macros, turns off power to the motors, and resets the controller (note: despite this field being attached to each axis, all actions are controller-wide).

### Limitations:
- Calibration, PID loop filters, I/O, etc., are not implemented!
- Homing feature relies on the macros provided by Nanomotion to be loaded and configured on the controller.

