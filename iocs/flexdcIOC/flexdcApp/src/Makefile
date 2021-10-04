TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#=============================
# Build the IOC application

PROD_IOC = flexdc
# flexdc.dbd will be created and installed
DBD += flexdc.dbd

# flexdc.dbd will be made up from these files:
flexdc_DBD += base.dbd

# Include dbd files from all support applications:
#ifdef ASYN
flexdc_DBD += asyn.dbd
flexdc_DBD += drvAsynSerialPort.dbd
flexdc_DBD += drvAsynIPPort.dbd
#endif
flexdc_DBD += motorSupport.dbd
flexdc_DBD += flexdcMotor.dbd

# Add all the support libraries needed by this IOC
flexdc_LIBS += flexdcMotor
flexdc_LIBS += motor
#ifdef ASYN
flexdc_LIBS += asyn
#endif
#ifdef SNCSEQ
flexdc_LIBS += seq pv
#endif

# flexdc_registerRecordDeviceDriver.cpp derives from flexdc.dbd
flexdc_SRCS += flexdc_registerRecordDeviceDriver.cpp

# Build the main IOC entry point on workstation OSs.
flexdc_SRCS_DEFAULT += flexdcMain.cpp
flexdc_SRCS_vxWorks += -nil-

# Add support from base/src/vxWorks if needed
#flexdc_OBJS_vxWorks += $(EPICS_BASE_BIN)/vxComLibrary

# Finally link to the EPICS Base libraries
flexdc_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

