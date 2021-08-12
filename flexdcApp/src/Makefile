TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#==================================================
# build a support library

LIBRARY_IOC += flexdcMotor

# install flexdcMotor.dbd into <top>/dbd
DBD += flexdcMotor.dbd

INC += FlexDCMotorDriver.h

# specify all source files to be compiled and added to the library
flexdcMotor_SRCS += FlexDCMotorDriver.h

flexdcMotor_LIBS += motor
flexdcMotor_LIBS += asyn

flexdcMotor_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

