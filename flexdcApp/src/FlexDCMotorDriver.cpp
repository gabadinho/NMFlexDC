/*
FILENAME...   FlexDCMotorDriver.cpp
USAGE...      Motor driver support (model 3, asyn) for the Nanomotion FlexDC controller

Jose Gabadinho
April 2021
*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include <iocsh.h>
#include <epicsThread.h>

#include <asynOctetSyncIO.h>

#include <asynMotorController.h>
#include <asynMotorAxis.h>

#include "FlexDCMotorDriver.h"

#include <epicsExport.h>



static const char *driverName = "NanomotionFlexDC";

/** Creates a new FlexDCController object.
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] asynPortName      The name of the drvAsynIPPPort that was created previously to connect to the Flex DC controller
  * \param[in] numAxes           The number of axes that this controller supports, overwritten to 2
  * \param[in] movingPollPeriod  The time between polls when any axis is moving
  * \param[in] idlePollPeriod    The time between polls when no axis is moving
  */
FlexDCController::FlexDCController(const char *portName, const char *asynPortName, int numAxes, double movingPollPeriod, double idlePollPeriod)
    :asynMotorController(portName, 2, NUM_FLEXDC_PARAMS, 
                         asynOctetMask, 
                         asynOctetMask,
                         ASYN_CANBLOCK | ASYN_MULTIDEVICE, 
                         1, /* autoconnect */
                         0, 0) /* Default priority and stack size */ {
    int axis;
    asynStatus status;
    char eos[10];
    int eos_len;
    static const char *functionName = "FlexDCController";

    createParam(AXIS_RDBD_PARAMNAME, asynParamFloat64, &driverRetryDeadband);

    numAxes = 2; // Force two-axes regardless of what user says

    // Connect to FlexDC controller
    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s: Creating Nanomotion FlexDC controller %s to asyn %s with %d axes\n", driverName, functionName, portName, asynPortName, numAxes);
    status = pasynOctetSyncIO->connect(asynPortName, 0, &pasynUserController_, NULL);
    if (status) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: cannot connect to Nanomotion FlexDC controller\n", driverName, functionName);
    }

    pasynOctetSyncIO->getInputEos(pasynUserController_, eos, 10, &eos_len);
    if (!eos_len) {
        // UPDATE THIS!!!
        asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s: Setting input acknowledgement to CR\n", driverName, functionName);
        pasynOctetSyncIO->setInputEos(pasynUserController_, "\r", 1);
    }

    pasynOctetSyncIO->getOutputEos(pasynUserController_, eos, 10, &eos_len);
    if (!eos_len) {
        // UPDATE THIS!!!
        asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s: Setting output acknowledgement to CR\n", driverName, functionName);
        pasynOctetSyncIO->setOutputEos(pasynUserController_, "\r", 1);
    }

    // Create the axis objects
    for (axis=0; axis<numAxes; axis++) {
        new FlexDCAxis(this, axis);
    }

    startPoller(movingPollPeriod, idlePollPeriod, 2);
}

/** Reports on status of the driver
  * \param[in] fp The file pointer on which report information will be written
  * \param[in] level The level of report detail desired
  *
  * If details > 0 then information is printed about each axis.
  * After printing controller-specific information calls asynMotorController::report()
  */
void FlexDCController::report(FILE *fp, int level) {
    asynStatus status = asynError;

    fprintf(fp, "Nanomotion FlexDC motor controller %s, numAxes=%d, moving poll period=%f, idle poll period=%f\n", this->portName, numAxes_, movingPollPeriod_, idlePollPeriod_);

    if (level > 0) {
        // Retrieve controller version
        sprintf(this->outString_, CTRL_VER_CMD);
        status = this->writeReadController();
        if (status == asynSuccess) {
            // FORCE ZERO-ENDED STRING...?!
            fprintf(fp, "  version=%s\n", this->inString_);
        } else {
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "Unable to retrieve FlexDC %s controller version\n", this->portName);
        }
    }

    // Call the base class method
    asynMotorController::report(fp, level);
}

/** Returns a pointer to an FlexDCAxis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] pasynUser asynUser structure that encodes the axis index number. */
FlexDCAxis* FlexDCController::getAxis(asynUser *pasynUser) {
    return static_cast<FlexDCAxis*>(asynMotorController::getAxis(pasynUser));
}

/** Returns a pointer to an FlexDCAxis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] axisNo Axis index number. */
FlexDCAxis* FlexDCController::getAxis(int axisNo) {
    return static_cast<FlexDCAxis*>(asynMotorController::getAxis(axisNo));
}



// These are the FlexDCAxis methods

/** Creates a new FlexDCAxis object.
  * \param[in] pC Pointer to the FlexDCController to which this axis belongs. 
  * \param[in] axisNo Index number of this axis, range 0 to pC->numAxes_-1.
  */
FlexDCAxis::FlexDCAxis(FlexDCController *pC, int axisNo): asynMotorAxis(pC, axisNo), pC_(pC) {
    this->setIntegerParam(pC_->motorStatusHomed_, 0);
  
    callParamCallbacks();
}

/** Reports on status of the driver
  * \param[in] fp The file pointer on which report information will be written
  * \param[in] level The level of report detail desired
  *
  * If details > 0 then information is printed about each axis.
  * After printing controller-specific information calls asynMotorAxis::report()
  */
void FlexDCAxis::report(FILE *fp, int level) {
    if (level > 0) {
      // print as much status as known...
      // also velocity
    } else {
       fprintf(fp,
            "  axis %d\n",
            this->axisNo_);
    }

    asynMotorAxis::report(fp, level);
}

asynStatus FlexDCAxis::move(double position, int relative, double minVelocity, double maxVelocity, double acceleration) {
    asynStatus status = asynError;

    // must check is powered off

    return callParamCallbacks();
}

asynStatus FlexDCAxis::home(double minVelocity, double maxVelocity, double acceleration, int forwards) {
    asynStatus status = asynError;

    return callParamCallbacks();
}

asynStatus FlexDCAxis::stop(double acceleration) {
    asynStatus status = asynError;

    sprintf(pC_->outString_, "%s", AXIS_STOP_CMD[this->axisNo_]);
    status = pC_->writeController();
    setStatusProblem(status);

    return callParamCallbacks();
}

asynStatus FlexDCAxis::setPosition(double position) {
    asynStatus status = asynError;

    sprintf(pC_->outString_, AXIS_FORCEPOS_CMD[this->axisNo_], (long)position);
    status = pC_->writeController();
    setStatusProblem(status);

    return callParamCallbacks();
}

/** Polls the axis.
  * This function reads the controller position, encoder position, the limit status, the moving status, 
  * and the drive power-on status.  It does not current detect following error, etc. but this could be
  * added.
  * It calls setIntegerParam() and setDoubleParam() for each item that it polls,
  * and then calls callParamCallbacks() at the end.
  * \param[out] moving A flag that is set indicating that the axis is moving (1) or done (0). */
asynStatus FlexDCAxis::poll(bool *moving) { 
    asynStatus status = asynError, final_status = asynSuccess;

    sprintf(pC_->outString_, "%s", AXIS_GETPOS_CMD[this->axisNo_]);
    status = pC_->writeReadController();
    if (status == asynSuccess) {
        this->readbackPosition = atol(pC_->inString_);
        setDoubleParam(pC_->motorPosition_, this->readbackPosition);
    } else {
        final_status = asynError;
    }

    sprintf(pC_->outString_, "%s", AXIS_POSERR_CMD[this->axisNo_]);
    status = pC_->writeReadController();
    if (status == asynSuccess) {
        this->positionError = atol(pC_->inString_);
    } else {
        final_status = asynError;
    }

    sprintf(pC_->outString_, "%s", AXIS_MOTIONSTAT_CMD[this->axisNo_]);
    status = pC_->writeReadController();
    if (status == asynSuccess) {
        this->motionStatus = atoi(pC_->inString_);
    } else {
        final_status = asynError;
    }

    sprintf(pC_->outString_, "%s", AXIS_ENDMOTREAS_CMD[this->axisNo_]);
    status = pC_->writeReadController();
    if (status == asynSuccess) {
        this->endMotionReason = atoi(pC_->inString_);
    } else {
        final_status = asynError;
    }

    sprintf(pC_->outString_, "%s", AXIS_MOTORFAULT_CMD[this->axisNo_]);
    status = pC_->writeReadController();
    if (status == asynSuccess) {
        this->motorFault = atoi(pC_->inString_);
    } else {
        final_status = asynError;
    }

    sprintf(pC_->outString_, "%s", AXIS_MACRO_RESULT_CMD[this->axisNo_]);
    status = pC_->writeReadController();
    if (status == asynSuccess) {
        this->macroResult = atoi(pC_->inString_);
    } else {
        final_status = asynError;
    }

    setStatusProblem(final_status);

    return callParamCallbacks();
}

void FlexDCAxis::setStatusProblem(asynStatus status) {
    int status_problem;

    pC_->getIntegerParam(this->axisNo_, pC_->motorStatus_, &status_problem);
    if ((status != asynSuccess) && (!status_problem)) {
        this->setIntegerParam(pC_->motorStatusProblem_, 1);
    }
    if ((status == asynSuccess) && (status_problem)) {
        this->setIntegerParam(pC_->motorStatusProblem_, 0);
    }
}



/** Creates a new FlexDCController object.
  * Configuration command, called directly or from iocsh
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] asynPortName      The name of the drvAsynIPPort/drvAsynSerialPortConfigure that was created previously to connect to the Nanomotion controller 
  * \param[in] numAxes           The number of axes that this controller supports 
  * \param[in] movingPollPeriod  The time in ms between polls when any axis is moving
  * \param[in] idlePollPeriod    The time in ms between polls when no axis is moving 
  */
extern "C" int NMFlexDCCreateController(const char *portName, const char *asynPortName, int numAxes,  int movingPollPeriod, int idlePollPeriod) {
    new FlexDCController(portName, asynPortName, numAxes, movingPollPeriod/1000., idlePollPeriod/1000.);
    return asynSuccess;
}

/** Code for iocsh registration */
static const iocshArg NMFlexDCCreateControllerArg0 = { "Port name", iocshArgString };
static const iocshArg NMFlexDCCreateControllerArg1 = { "Asyn port name", iocshArgString };
static const iocshArg NMFlexDCCreateControllerArg2 = { "Number of axes", iocshArgInt };
static const iocshArg NMFlexDCCreateControllerArg3 = { "Moving poll period (ms)", iocshArgInt };
static const iocshArg NMFlexDCCreateControllerArg4 = { "Idle poll period (ms)", iocshArgInt };
static const iocshArg * const NMFlexDCCreateControllerArgs[] = { &NMFlexDCCreateControllerArg0,
                                                                 &NMFlexDCCreateControllerArg1,
                                                                 &NMFlexDCCreateControllerArg2,
                                                                 &NMFlexDCCreateControllerArg3,
                                                                 &NMFlexDCCreateControllerArg4 };
static const iocshFuncDef NMFlexDCCreateControllerDef = { "NMFlexDCCreateController", 5, NMFlexDCCreateControllerArgs };
static void NMFlexDCCreateControllerCallFunc(const iocshArgBuf *args) {
    NMFlexDCCreateController(args[0].sval, args[1].sval, args[2].ival, args[3].ival, args[4].ival);
}

static void NMFlexDCControllerRegister(void) {
    iocshRegister(&NMFlexDCCreateControllerDef, NMFlexDCCreateControllerCallFunc);
}

extern "C" {
    epicsExportRegistrar(NMFlexDCControllerRegister);
}
