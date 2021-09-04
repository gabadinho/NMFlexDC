/*
FILENAME...   FlexDCMotorDriver.cpp
USAGE...      Motor driver support (model 3, asyn) for the Nanomotion FlexDC controller

Jose G.C. Gabadinho
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
  *
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] asynPortName      The name of the drvAsynIPPPort that was created previously to connect to the Flex DC controller
  * \param[in] numAxes           The number of axes that this controller supports (discarded and overwritten to 2)
  * \param[in] movingPollPeriod  The time between polls when any axis is moving
  * \param[in] idlePollPeriod    The time between polls when no axis is moving
  */
FlexDCController::FlexDCController(const char *portName, const char *asynPortName, int numAxes, double movingPollPeriod, double idlePollPeriod)
    :asynMotorController(portName, 2, NUM_FLEXDC_PARAMS, 
                         asynOctetMask, // || asynFloat64Mask,
                         asynOctetMask, // || asynFloat64Mask,
                         ASYN_CANBLOCK | ASYN_MULTIDEVICE, 
                         1, /* autoconnect */
                         0, 0) /* Default priority and stack size */ {
    int axis;
    asynStatus status;
    char eos[10];
    int eos_len;
    static const char *functionName = "FlexDCController";

    createParam(AXIS_RDBD_PARAMNAME, asynParamFloat64, &driverRetryDeadband);
    createParam(AXIS_HOMRMACRO_PARAMNAME, asynParamInt32, &driverHomeReverseMacro);
    createParam(AXIS_HOMFMACRO_PARAMNAME, asynParamInt32, &driverHomeForwardMacro);
    createParam(AXIS_HOMS_PARAMNAME, asynParamInt32, &driverHomeStatus);

    numAxes = 2; // Force two-axes regardless of what user says

    // Connect to FlexDC controller
    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s: Creating Nanomotion FlexDC controller %s to asyn %s with %d axes\n", driverName, functionName, portName, asynPortName, numAxes);
    status = pasynOctetSyncIO->connect(asynPortName, 0, &pasynUserController_, NULL);
    if (status) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: cannot connect to Nanomotion FlexDC controller\n", driverName, functionName);
    }

    pasynOctetSyncIO->getInputEos(pasynUserController_, eos, 10, &eos_len);
    if (!eos_len) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s: Setting input acknowledgement to '>'\n", driverName, functionName);
        pasynOctetSyncIO->setInputEos(pasynUserController_, ">", 1);
    }

    pasynOctetSyncIO->getOutputEos(pasynUserController_, eos, 10, &eos_len);
    if (!eos_len) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s: Setting output acknowledgement to CR LF\n", driverName, functionName);
        pasynOctetSyncIO->setOutputEos(pasynUserController_, "\r\n", 2);
    }

    // Create the axis objects
    for (axis=0; axis<numAxes; axis++) {
        new FlexDCAxis(this, axis);
    }

    startPoller(movingPollPeriod, idlePollPeriod, 2);
}

/** Reports on status of the driver
  *
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
            fprintf(fp, "  version = %s\n", this->inString_);
        } else {
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "Unable to retrieve FlexDC %s controller version\n", this->portName);
        }
    }

    // Call the base class method
    asynMotorController::report(fp, level);
}

/** Returns a pointer to an FlexDCAxis object.
  *
  * \param[in] pasynUser asynUser structure that encodes the axis index number
  *
  * \return FlexDCAxis object or NULL if the axis number encoded in pasynUser is invalid
  */
FlexDCAxis* FlexDCController::getAxis(asynUser *pasynUser) {
    return static_cast<FlexDCAxis*>(asynMotorController::getAxis(pasynUser));
}

/** Returns a pointer to an FlexDCAxis object.
  *
  * \param[in] axisNo Axis index number
  *
  * \return FlexDCAxis object or NULL if the axis number encoded in pasynUser is invalid
  */
FlexDCAxis* FlexDCController::getAxis(int axisNo) {
    return static_cast<FlexDCAxis*>(asynMotorController::getAxis(axisNo));
}



// These are the FlexDCAxis methods

/** Creates a new FlexDCAxis object.
  *
  * \param[in] pC Pointer to the FlexDCController to which this axis belongs
  * \param[in] axisNo Index number of this axis, range 0 to pC->numAxes_-1
  */
FlexDCAxis::FlexDCAxis(FlexDCController *pC, int axisNo): asynMotorAxis(pC, axisNo), pC_(pC) {
    this->motionStatus = 0;
    this->motorFault = 0;
    this->endMotionReason = 0;
    this->macroResult = 0;
    this->positionError = 0;
    this->positionReadback = 0;
    this->isMotorOn = 0;

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
    long speed;
    int homr_type, homf_type;

    if (level > 0) {
        sprintf(pC_->outString_, AXIS_GETSPEED_CMD, CTRL_AXES[this->axisNo_]);
        if (pC_->writeReadController() == asynSuccess) {
            speed = atol(pC_->inString_);
        } else {
            speed = 0;
        }

        pC_->getIntegerParam(this->axisNo_, pC_->driverHomeReverseMacro, &homr_type);
        pC_->getIntegerParam(this->axisNo_, pC_->driverHomeForwardMacro, &homf_type);

        fprintf(fp,
            "  axis %d\n"
            "  switched on = %d\n"
            "  last speed = %ld\n"
            "  fault = %x\n"
            "  motion end = %s\n"
            "  homr type = %d\n"
            "  homf type = %d\n"
            "  macro res.= %d\n",
            this->axisNo_,
            this->isMotorOn,
            speed,
            this->motorFault,
            MOTION_END_REASON[this->endMotionReason],
            homr_type,
            homf_type,
            this->macroResult
        );

    } else {
       fprintf(fp,
            "  axis %d\n",
            this->axisNo_);
    }

    asynMotorAxis::report(fp, level);
}

asynStatus FlexDCAxis::move(double position, int relative, double minVelocity, double maxVelocity, double acceleration) {
    asynStatus status = asynSuccess;
    long target = (long)position;
    char mot = CTRL_AXES[this->axisNo_];

    if (this->macroResult == EXECUTING) {
        status = haltHomingMacro();
    }
    if ((status == asynSuccess) && (this->motionStatus != 0)) {
        status = stopMotor();
    }
    if (status == asynSuccess) {
        if (relative) {
            target += this->positionReadback;
        }
        setIntegerParam(pC_->motorStatusDone_, 0);

        // SET SPEED!!!
        asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW, "Moving FlexDC %s axis %d to %ld at velocity %f\n", pC_->portName, this->axisNo_, target, maxVelocity);
        sprintf(pC_->outString_, AXIS_MOVE_CMD, mot, mot, mot, mot, target, mot);
        status = pC_->writeController();
    }

    setStatusProblem(status);

    return callParamCallbacks();
}

asynStatus FlexDCAxis::home(double minVelocity, double maxVelocity, double acceleration, int forwards) {
    asynStatus status = asynSuccess;
    char mot = CTRL_AXES[this->axisNo_];
    int hom_type;

    if (this->macroResult == EXECUTING) {
        status = haltHomingMacro();
    }
    if ((status == asynSuccess) && (this->motionStatus != 0)) {
        status = stopMotor();
    }
    if (status == asynSuccess) {
        if (forwards) {
            pC_->getIntegerParam(this->axisNo_, pC_->driverHomeForwardMacro, &hom_type);
            if (hom_type > DISABLED) {
                sprintf(pC_->outString_, HOMF_MACRO[hom_type], mot);
                status = pC_->writeController();
            } else {
                asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "Forward-homing of FlexDC %s axis %d is disabled!\n", pC_->portName, this->axisNo_);
            }
        } else {
            pC_->getIntegerParam(this->axisNo_, pC_->driverHomeReverseMacro, &hom_type);
            if (hom_type > DISABLED) {
                sprintf(pC_->outString_, HOMR_MACRO[hom_type], mot);
                status = pC_->writeController();
            } else {
                asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "Reverse-homing of FlexDC %s axis %d is disabled!\n", pC_->portName, this->axisNo_);
            }
        }
    }

    setStatusProblem(status);

    return callParamCallbacks();
}

asynStatus FlexDCAxis::stop(double acceleration) {
    asynStatus status = asynError;

    if (this->macroResult == EXECUTING) {
        haltHomingMacro();
    }
    status = stopMotor();
    setStatusProblem(status);

    return callParamCallbacks();
}

asynStatus FlexDCAxis::setPosition(double position) {
    asynStatus status = asynError;

    if ((this->macroResult == EXECUTING) || (this->motionStatus != 0)) {
        asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "Due to ongoing motion of FlexDC %s axis %d, readback position will not be overriden!\n", pC_->portName, this->axisNo_);
    } else {
        sprintf(pC_->outString_, AXIS_FORCEPOS_CMD, CTRL_AXES[this->axisNo_], (long)position);
        status = pC_->writeController();
    }

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
    int at_limit, motion_status;
    bool valid_motion_status = false, valid_macro_result = false;
    double rdbd, mres;

    sprintf(pC_->outString_, AXIS_GETPOS_CMD, CTRL_AXES[this->axisNo_]);
    status = pC_->writeReadController();
    if (status == asynSuccess) {
        this->positionReadback = atol(pC_->inString_);
        setDoubleParam(pC_->motorPosition_, this->positionReadback);
    } else {
        final_status = asynError;
    }

    sprintf(pC_->outString_, AXIS_ISPOWERED_CMD, CTRL_AXES[this->axisNo_]);
    status = pC_->writeReadController();
    if (status == asynSuccess) {
        this->isMotorOn = atol(pC_->inString_);
    } else {
        final_status = asynError;
    }

    sprintf(pC_->outString_, AXIS_MOTIONSTATUS_CMD, CTRL_AXES[this->axisNo_]);
    status = pC_->writeReadController();
    if (status == asynSuccess) {
        motion_status = atoi(pC_->inString_);
        valid_motion_status = true;
        if ((this->motionStatus != 0) && (motion_status == 0)) {
            asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW, "FlexDC %s axis %d has stopped\n", pC_->portName, this->axisNo_);
            setIntegerParam(pC_->motorStatusDone_, 1);
        }
        *moving = (motion_status!=0);
        this->motionStatus = motion_status;
    } else {
        final_status = asynError;
    }

    sprintf(pC_->outString_, AXIS_MACRO_RESULT_CMD, CTRL_AXES[this->axisNo_]);
    status = pC_->writeReadController();
    if (status == asynSuccess) {
        this->macroResult = atoi(pC_->inString_);
        valid_macro_result = true;
        setIntegerParam(pC_->driverHomeStatus, this->macroResult);
    } else {
        final_status = asynError;
    }

    sprintf(pC_->outString_, AXIS_POSERR_CMD, CTRL_AXES[this->axisNo_]);
    status = pC_->writeReadController();
    if (status == asynSuccess) {
        this->positionError = atol(pC_->inString_);

        if ((valid_macro_result) && (this->macroResult != 0) && (valid_motion_status) && (motion_status == 0)) {
            pC_->getDoubleParam(this->axisNo_, pC_->driverRetryDeadband, &rdbd);
            pC_->getDoubleParam(this->axisNo_, pC_->motorResolution_, &mres);
            rdbd = rdbd/mres;

            printf("MRES=%f RDBD=%f\n",mres,rdbd);

                // check here if motor should be switched off after a movement?
                // retrieve retry-deadband
                // rdbd is from record in user units
                // positionError is in ticks!!!
        }
    } else {
        final_status = asynError;
    }

    sprintf(pC_->outString_, AXIS_MOTIONEND_CMD, CTRL_AXES[this->axisNo_]);
    status = pC_->writeReadController();
    if (status == asynSuccess) {
        this->endMotionReason = atoi(pC_->inString_);

        pC_->getIntegerParam(this->axisNo_, pC_->motorStatusLowLimit_, &at_limit);
        if ( (this->endMotionReason == HARD_RLS) && (!at_limit)) {
            setIntegerParam(pC_->motorStatusLowLimit_, 1);
            switchMotorPower(0);
        } else if ( (this->endMotionReason != HARD_RLS) && (at_limit)) {
            setIntegerParam(pC_->motorStatusLowLimit_, 0);
        }

        pC_->getIntegerParam(this->axisNo_, pC_->motorStatusHighLimit_, &at_limit);
        if ( (this->endMotionReason == HARD_FLS) && (!at_limit)) {
            setIntegerParam(pC_->motorStatusHighLimit_, 1);
            switchMotorPower(0);
        } else if ( (this->endMotionReason != HARD_FLS) && (at_limit)) {
            setIntegerParam(pC_->motorStatusHighLimit_, 0);
        }

    } else {
        final_status = asynError;
    }

    sprintf(pC_->outString_, AXIS_MOTORFAULT_CMD, CTRL_AXES[this->axisNo_]);
    status = pC_->writeReadController();
    if (status == asynSuccess) {
        this->motorFault = atoi(pC_->inString_);
    } else {
        final_status = asynError;
    }

    setStatusProblem(final_status);

    return callParamCallbacks();
}

/** Raises the motor record problem status.
  *
  * \param[in] status Last operation status
  */
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


/** Switches the motor power on or off.
  *
  * \param[in] on 1 to switch motor on, 0 to switch motor off
  */
asynStatus FlexDCAxis::switchMotorPower(bool on) {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW, "Switching FlexDC %s axis %d power to %d\n", pC_->portName, this->axisNo_, on);
    sprintf(pC_->outString_, AXIS_POWER_CMD, CTRL_AXES[this->axisNo_], on);
    return pC_->writeController();
}

asynStatus FlexDCAxis::stopMotor() {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW, "Stop motion on FlexDC %s axis %d\n", pC_->portName, this->axisNo_);
    sprintf(pC_->outString_, AXIS_STOP_CMD, CTRL_AXES[this->axisNo_]);
    return pC_->writeController();
}


asynStatus FlexDCAxis::haltHomingMacro() {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW, "Halting FlexDC %s axis %d homing macro\n", pC_->portName, this->axisNo_);
    sprintf(pC_->outString_, AXIS_MACRO_HALT_CMD, CTRL_AXES[this->axisNo_]);
    return pC_->writeController();
}



/** Creates a new FlexDCController object.
  * Configuration command, called directly or from iocsh.
  *
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] asynPortName      The name of the drvAsynIPPort/drvAsynSerialPortConfigure that was created previously to connect to the Nanomotion controller 
  * \param[in] numAxes           The number of axes that this controller supports 
  * \param[in] movingPollPeriod  The time in ms between polls when any axis is moving
  * \param[in] idlePollPeriod    The time in ms between polls when no axis is moving 
  *
  * \return Always asynSuccess
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
