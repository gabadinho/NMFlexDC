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

#include "FlexDCMotorDriver.h"

#include <iocsh.h>
#include <epicsThread.h>

#include <asynOctetSyncIO.h>

#include <epicsExport.h>
#include <epicsThread.h>



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
                         0,
                         0,
                         ASYN_CANBLOCK | ASYN_MULTIDEVICE, 
                         1, /* autoconnect */
                         0, 0) /* Default priority and stack size */ {
    int axis;
    asynStatus status;
    char eos[10];
    int eos_len;
    static const char *functionName = "FlexDCController";

    createParam(AXIS_MRES_PARAMNAME, asynParamFloat64, &driverMotorRecResolution);
    createParam(AXIS_RDBD_PARAMNAME, asynParamFloat64, &driverRetryDeadband);
    createParam(AXIS_HOMR_PARAMNAME, asynParamInt32, &driverHomeReverseMacro);
    createParam(AXIS_HOMF_PARAMNAME, asynParamInt32, &driverHomeForwardMacro);
    createParam(AXIS_HOMS_PARAMNAME, asynParamInt32, &driverHomeStatus);
    createParam(CTRL_RST_PARAMNAME,  asynParamInt32, &driverResetController);

    numAxes = 2; // Force two-axes regardless of what user says

    // Connect to FlexDC controller
    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s: Creating Nanomotion FlexDC controller %s to asyn %s with %d axes\n", driverName, functionName, portName, asynPortName, numAxes);
    status = pasynOctetSyncIO->connect(asynPortName, 0, &pasynUserController_, NULL);
    if (status) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Cannot connect to Nanomotion FlexDC controller at asyn %s\n", driverName, functionName, asynPortName);
    }

    pasynOctetSyncIO->getInputEos(pasynUserController_, eos, 10, &eos_len);
    if (!eos_len) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s: Setting input acknowledgement of %s to '>'\n", driverName, functionName, portName);
        pasynOctetSyncIO->setInputEos(pasynUserController_, ">", 1);
    }

    pasynOctetSyncIO->getOutputEos(pasynUserController_, eos, 10, &eos_len);
    if (!eos_len) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s: Setting output acknowledgement of %s to CR LF\n", driverName, functionName, portName);
        pasynOctetSyncIO->setOutputEos(pasynUserController_, "\r\n", 2);
    }

    // Create the axis objects
    for (axis=0; axis<numAxes; axis++) {
        new FlexDCAxis(this, axis);
    }

    startPoller(movingPollPeriod, idlePollPeriod, 2);
}

/** Called when asyn clients call pasynInt32->write().
  * Extracts the function and axis number from pasynUser.
  * Sets the value in the parameter library.
  * If the function is motorStop_ then it calls pAxis->stop().
  * If the function is motorUpdateStatus_ then it does a poll and forces a callback.
  * Calls any registered callbacks for this pasynUser->reason and address.  
  * Motor drivers will reimplement this function if they support 
  * controller-specific parameters on the asynInt32 interface. They should call this
  * base class method for any parameters that are not controller-specific.
  *
  * \param[in] pasynUser asynUser structure that encodes the reason and address.
  * \param[in] value     Value to write.
  *
  * \return Result of callParamCallbacks() call or asynMotorController::writeInt32()
  */
asynStatus FlexDCController::writeInt32(asynUser *pasynUser, epicsInt32 value) {
    int function = pasynUser->reason;
    asynMotorAxis *p_axis;
    asynStatus status = asynSuccess;
    const char* functionName = "writeInt32";

    p_axis = getAxis(pasynUser);
    if (p_axis) {
        if (function == driverResetController) {
            p_axis->setIntegerParam(function, value);

            sprintf(this->outString_, CTRL_RESET_CMD);
            writeController();

            status = p_axis->callParamCallbacks();
        } else {
            status = asynMotorController::writeInt32(pasynUser, value);
        }
    } else {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "Unable to retrieve FlexDC %s axis from asynUser in %s\n", this->portName, functionName);
        status = asynError;
    }

    return status;
}


/** Reports on status of the driver.
  * If level > 0 then error message, controller version is printed.
  *
  * \param[in] fp    The file pointer on which report information will be written
  * \param[in] level The level of report detail desired
  */
void FlexDCController::report(FILE *fp, int level) {
    asynStatus status = asynError;

    fprintf(fp, "Nanomotion FlexDC motor controller %s, numAxes=%d, moving poll period=%f, idle poll period=%f\n", this->portName, numAxes_, movingPollPeriod_, idlePollPeriod_);

    if (level > 0) {
        // Retrieve controller version
        sprintf(this->outString_, CTRL_VER_CMD);
        status = writeReadController();
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
    this->endMotionReason = MOTOR_OFF;
    this->macroResult = FAIL_NO_INDEX_FOUND;
    this->positionError = 0;
    this->positionReadback = 0;
    this->isMotorOn = false;

    setIntegerParam(pC_->motorStatusHomed_, 0);
    setIntegerParam(pC_->motorStatusHasEncoder_, 1);
    setIntegerParam(pC_->motorClosedLoop_, 1);
    setStatusProblem(asynSuccess);

    callParamCallbacks();
}

/** Reports on status of the axis.
  * If level > 0 then detailed axis information (on, speed, fault, end-motion reason, etc.) is printed.
  *
  * \param[in] fp The file pointer on which report information will be written
  * \param[in] level The level of report detail desired
  */
void FlexDCAxis::report(FILE *fp, int level) {
    long speed;
    int homr_type, homf_type;

    if (level > 0) {
        buildGenericGetCommand(pC_->outString_, AXIS_GETSPEED_CMD, this->axisNo_);
        if (pC_->writeReadController() == asynSuccess) {
            speed = atol(pC_->inString_);
        } else {
            speed = -1;
        }

        pC_->getIntegerParam(this->axisNo_, pC_->driverHomeReverseMacro, &homr_type);
        pC_->getIntegerParam(this->axisNo_, pC_->driverHomeForwardMacro, &homf_type);

        fprintf(fp,
            "  axis %d\n"
            "    motion status = %x\n"
            "    motion end = %s\n"
            "    motor fault = %x\n"
            "    switched on = %d\n"
            "    pos.error = %ld\n"
            "    last speed = %ld\n"
            "    homr type = %d\n"
            "    homf type = %d\n"
            "    macro res.= %d\n",
            this->axisNo_,
            this->motionStatus,
            MOTION_END_REASON[this->endMotionReason],
            this->motorFault,
            this->isMotorOn,
            this->positionError,
            speed,
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

/** Moves the axis to a different target position.
  * Warning: stops any ongoing move or home actions!
  *
  * \param[in] position      The desired target position
  * \param[in] relative      1 for relative position
  * \param[in] minVelocity   Motion parameter
  * \param[in] maxVelocity   Motion parameter
  * \param[in] acceleration  Motion parameter
  *
  * \return Result of callParamCallbacks() call
  */
asynStatus FlexDCAxis::move(double position, int relative, double minVelocity, double maxVelocity, double acceleration) {
    asynStatus status = asynSuccess;
    long target = (long)position;
    int speed = (long)maxVelocity;

    if (this->macroResult == EXECUTING) {
        status = haltHomingMacro();
        shortWait();
    }
    if ((status == asynSuccess) && (this->motionStatus != 0)) {
        status = stopMotor();
        shortWait();
    }
    if (status == asynSuccess) {
        asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW, "Moving FlexDC %s axis %d to %ld at velocity %d\n", pC_->portName, this->axisNo_, target, speed);

        setIntegerParam(pC_->motorStatusDone_, 0);

        buildMoveCommand(pC_->outString_, this->axisNo_, target, relative, speed);
        status = pC_->writeController();
        if (status != asynSuccess) {
            setIntegerParam(pC_->motorStatusDone_, 1);
        }
    }

    setStatusProblem(status);

    return callParamCallbacks();
}

/** Starts the axis homing macro, as defined in the HOMR_CMD or HOMF_CMD records.
  * Warning: stops any ongoing move or home actions!
  *
  * \param[in] minVelocity   Motion parameter
  * \param[in] maxVelocity   Motion parameter
  * \param[in] acceleration  Motion parameter
  * \param[in] forwards      1 if user wants to home forward, 0 for reverse
  *
  * \return Result of callParamCallbacks() call
  */
asynStatus FlexDCAxis::home(double minVelocity, double maxVelocity, double acceleration, int forwards) {
    asynStatus status = asynSuccess;
    int hom_type;

    if (this->macroResult == EXECUTING) {
        status = haltHomingMacro();
        shortWait();
    }
    if ((status == asynSuccess) && (this->motionStatus != 0)) {
        status = stopMotor();
        shortWait();
    }

    if (status == asynSuccess) {
        if (forwards) {
            pC_->getIntegerParam(this->axisNo_, pC_->driverHomeForwardMacro, &hom_type);
        } else {
            pC_->getIntegerParam(this->axisNo_, pC_->driverHomeReverseMacro, &hom_type);
        }

        if (hom_type > DISABLED) {
            if (forwards) {
                asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW, "Forward-homing FlexDC %s axis %d with type %d\n", pC_->portName, this->axisNo_, hom_type);
            } else {
                asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW, "Reverse-homing FlexDC %s axis %d with type %d\n", pC_->portName, this->axisNo_, hom_type);
            }

            setIntegerParam(pC_->motorStatusDone_, 0);
            setIntegerParam(pC_->motorStatusHome_, 1);
            setIntegerParam(pC_->motorStatusHomed_, 0);

            buildHomeMacroCommand(pC_->outString_, this->axisNo_, forwards, static_cast<flexdcHomeMacro>(hom_type));
            status = pC_->writeController();
            if (status != asynSuccess) {
                setIntegerParam(pC_->motorStatusHome_, 0);
                setIntegerParam(pC_->motorStatusDone_, 1);
            }

        } else {
            if (forwards) {
                asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "Forward-homing of FlexDC %s axis %d is disabled!\n", pC_->portName, this->axisNo_);
            } else {
                asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "Reverse-homing of FlexDC %s axis %d is disabled!\n", pC_->portName, this->axisNo_);
            }
            status = asynError;
        }
    }

    setStatusProblem(status);

    return callParamCallbacks();
}

/** Stops an ongoing motion.
  *
  * \param[in] acceleration  Motion parameter
  *
  * \return Result of callParamCallbacks() call
  */
asynStatus FlexDCAxis::stop(double acceleration) {
    asynStatus status = asynError;

    if (this->macroResult == EXECUTING) {
        haltHomingMacro();
    }
    status = stopMotor();
    setStatusProblem(status);

    return callParamCallbacks();
}

/** Forces the axis readback position to some value.
  * Warning: if motor is movinr or homing, nothing is done!
  *
  * \param[in] position  The desired readback position
  *
  * \return Result of callParamCallbacks() call
  */
asynStatus FlexDCAxis::setPosition(double position) {
    asynStatus status = asynError;

    if ((this->macroResult == EXECUTING) || (this->motionStatus != 0)) {
        asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "Due to ongoing motion of FlexDC %s axis %d, readback position will not be overriden!\n", pC_->portName, this->axisNo_);
    } else {
        buildSetPositionCommand(pC_->outString_, this->axisNo_, position);
        status = pC_->writeController();
    }

    setStatusProblem(status);

    return callParamCallbacks();
}

/** Polls the axis.
  * Reads the states, limits, readback, etc. and calls setIntegerParam() or setDoubleParam() for each item that it polls.
  * If motor is stopped and the position error is less than RDBD field, then motor is switched off.
  *
  * \param[out] moving A flag that is set indicating that the axis is moving (1) or done (0)
  *
  * \return Result of callParamCallbacks() call
  */
asynStatus FlexDCAxis::poll(bool *moving) { 
    asynStatus status = asynError, final_status = asynSuccess;
    int at_limit, is_homing;
    int status_done;
    bool valid_motion_status = false, valid_macro_result = false, valid_ispowered = false;

    buildGenericGetCommand(pC_->outString_, AXIS_GETPOS_CMD, this->axisNo_);
    status = pC_->writeReadController();
    if (status == asynSuccess) {
        this->positionReadback = atol(pC_->inString_);
        setDoubleParam(pC_->motorEncoderPosition_, this->positionReadback);
        setDoubleParam(pC_->motorPosition_, this->positionReadback);
    } else {
        final_status = asynError;
    }

    buildGenericGetCommand(pC_->outString_, AXIS_ISPOWERED_CMD, this->axisNo_);
    status = pC_->writeReadController();
    if (status == asynSuccess) {
        this->isMotorOn = atol(pC_->inString_);
        valid_ispowered = true;
        setIntegerParam(pC_->motorStatusPowerOn_, this->isMotorOn);
    } else {
        final_status = asynError;
    }

    buildGenericGetCommand(pC_->outString_, AXIS_MOTIONSTATUS_CMD, this->axisNo_);
    status = pC_->writeReadController();
    if (status == asynSuccess) {
        this->motionStatus = atoi(pC_->inString_);
        valid_motion_status = true;
    } else {
        final_status = asynError;
    }

    buildGenericGetCommand(pC_->outString_, AXIS_MACRO_RESULT_CMD, this->axisNo_);
    status = pC_->writeReadController();
    if (status == asynSuccess) {
        this->macroResult = static_cast<flexdcMacroResult>(atoi(pC_->inString_));
        valid_macro_result = true;
        setIntegerParam(pC_->driverHomeStatus, this->macroResult);

        pC_->getIntegerParam(this->axisNo_, pC_->motorStatusHome_, &is_homing);
        if ((this->macroResult != EXECUTING) && (is_homing)) {
            pC_->setIntegerParam(pC_->motorStatusHome_, 0);

            if (this->macroResult == OK) {
                asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW, "FlexDC %s axis %d is now homed\n", pC_->portName, this->axisNo_);
                setIntegerParam(pC_->motorStatusHomed_, 1);
            } else {
                asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW, "FlexDC %s axis %d failed to home with error code %d!\n", pC_->portName, this->axisNo_, this->macroResult);
            }
        }
    } else {
        final_status = asynError;
    }

    buildGenericGetCommand(pC_->outString_, AXIS_MOTIONEND_CMD, this->axisNo_);
    status = pC_->writeReadController();
    if (status == asynSuccess) {
        this->endMotionReason = static_cast<flexdcMotionEndReason>(atoi(pC_->inString_));

        pC_->getIntegerParam(this->axisNo_, pC_->motorStatusLowLimit_, &at_limit);
        if ((this->endMotionReason == HARD_RLS) && (!at_limit)) {
            asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW, "FlexDC %s axis %d at low limit switch\n", pC_->portName, this->axisNo_);
            setIntegerParam(pC_->motorStatusLowLimit_, 1);
            switchMotorPower(false);
        } else if ((this->endMotionReason != HARD_RLS) && (this->endMotionReason != MOTOR_OFF) && (at_limit)) {
            setIntegerParam(pC_->motorStatusLowLimit_, 0);
        }

        pC_->getIntegerParam(this->axisNo_, pC_->motorStatusHighLimit_, &at_limit);
        if ((this->endMotionReason == HARD_FLS) && (!at_limit)) {
            asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW, "FlexDC %s axis %d at high limit switch\n", pC_->portName, this->axisNo_);
            setIntegerParam(pC_->motorStatusHighLimit_, 1);
            switchMotorPower(false);
        } else if ((this->endMotionReason != HARD_FLS) && (this->endMotionReason != MOTOR_OFF) && (at_limit)) {
            setIntegerParam(pC_->motorStatusHighLimit_, 0);
        }

    } else {
        final_status = asynError;
    }

    buildGenericGetCommand(pC_->outString_, AXIS_POSERR_CMD, this->axisNo_);
    status = pC_->writeReadController();
    if (status == asynSuccess) {
        this->positionError = atol(pC_->inString_);

        // Power off motor when not moving/homing and position-error is lower than retry-deadband
        if ((valid_macro_result) && (valid_motion_status) && (valid_ispowered)) {
            setMotionDone(this->motionStatus, this->macroResult, this->isMotorOn, this->positionError);
        }
    } else {
        final_status = asynError;
    }

    buildGenericGetCommand(pC_->outString_, AXIS_MOTORFAULT_CMD, this->axisNo_);
    status = pC_->writeReadController();
    if (status == asynSuccess) {
        this->motorFault = atoi(pC_->inString_);
    } else {
        final_status = asynError;
    }

    pC_->getIntegerParam(this->axisNo_, pC_->motorStatusDone_, &status_done);
    *moving = !status_done;

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
        setIntegerParam(pC_->motorStatusProblem_, 1);
    }
    if ((status == asynSuccess) && (status_problem)) {
        setIntegerParam(pC_->motorStatusProblem_, 0);
    }
}

/** Updates the motor record to indicate that a motion has finished.
  *
  * \param[in] motion_status  0 if stopped
  * \param[in] macro_result   see flexdcMacroResult
  * \param[in] power_on       true is motor power and PID loop is switched on, false otherwise
  * \param[in] pos_error      difference between target and readback position
  *
  * \return Result of writeController() call, or asynSuccess if nothing to do
  */
asynStatus FlexDCAxis::setMotionDone(int motion_status, flexdcMacroResult macro_result, bool power_on, long pos_error) {
    asynStatus status = asynSuccess;
    int status_done, allowed_error;
    double rdbd, mres;

    pC_->getIntegerParam(this->axisNo_, pC_->motorStatusDone_, &status_done);
    if (!status_done) {
        if ((macro_result != EXECUTING) && (motion_status == 0) && (power_on)) {
            pC_->getDoubleParam(this->axisNo_, pC_->driverRetryDeadband, &rdbd);
            pC_->getDoubleParam(this->axisNo_, pC_->driverMotorRecResolution, &mres);
            allowed_error = (int)(rdbd/mres);

            if (labs(pos_error) <= allowed_error) {
                asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW, "FlexDC %s axis %d motion is within error margin, switching off motor\n", pC_->portName, this->axisNo_);
                setIntegerParam(pC_->motorStatusDone_, 1);
                status = switchMotorPower(false);
            }
        } else if ((macro_result != EXECUTING) && (motion_status == 0)) {
            setIntegerParam(pC_->motorStatusDone_, 1);
        }
    }

    return status;
}

/** Switches the motor power on or off.
  *
  * \param[in] on 1 to switch motor on, 0 to switch motor off
  *
  * \return Result of writeController() call
  */
asynStatus FlexDCAxis::switchMotorPower(bool on) {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW, "Switching FlexDC %s axis %d power to %d\n", pC_->portName, this->axisNo_, on);
    buildMotorPowerCommand(pC_->outString_, this->axisNo_, on);
    return pC_->writeController();
}

/** Stops a motion.
  *
  * \return Result of writeController() call
  */
asynStatus FlexDCAxis::stopMotor() {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW, "Stop motion on FlexDC %s axis %d\n", pC_->portName, this->axisNo_);
    buildStopCommand(pC_->outString_, this->axisNo_);
    return pC_->writeController();
}

/** Stops a homing macro.
  *
  * \return Result of writeController() call
  */
asynStatus FlexDCAxis::haltHomingMacro() {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW, "Halting FlexDC %s axis %d homing macro\n", pC_->portName, this->axisNo_);
    buildHaltMacroCommand(pC_->outString_, this->axisNo_);
    return pC_->writeController();
}

/** Performs a short epicsThreadSleep().
  *
  */
void FlexDCAxis::shortWait() {
    epicsThreadSleep(0.1);
}

/** All the following methods generate a command string to be sent to the controller.
  *
  */
bool FlexDCAxis::buildMoveCommand(char *buffer, int axis, double position, bool relative, double velocity) {
    char mot = CTRL_AXES[axis];
    if ((!buffer) || (axis<0) || (axis>1)) {
        return false;
    }
    if (relative) {
        sprintf(buffer, AXIS_MOVEREL_CMD, mot, mot, mot, mot, (int)velocity, mot, (long)position, mot);
    } else {
        sprintf(buffer, AXIS_MOVEABS_CMD, mot, mot, mot, mot, (int)velocity, mot, (long)position, mot);
    }
    return true;
}

bool FlexDCAxis::buildSetPositionCommand(char *buffer, int axis, double position) {
    if ((!buffer) || (axis<0) || (axis>1)) {
        return false;
    }
    sprintf(buffer, AXIS_FORCEPOS_CMD, CTRL_AXES[axis], (long)position);
    return true;
}

bool FlexDCAxis::buildStopCommand(char *buffer, int axis) {
    if ((!buffer) || (axis<0) || (axis>1)) {
        return false;
    }
    sprintf(buffer, AXIS_STOP_CMD, CTRL_AXES[axis]);
    return true;
}

bool FlexDCAxis::buildHaltMacroCommand(char *buffer, int axis) {
    if ((!buffer) || (axis<0) || (axis>1)) {
        return false;
    }
    //sprintf(buffer, AXIS_MACRO_HALT_CMD, CTRL_AXES[axis]);
    sprintf(buffer, AXIS_MACRO_KILLINIT_CMD, CTRL_AXES[axis], CTRL_AXES[axis]);
    return true;
}

bool FlexDCAxis::buildMotorPowerCommand(char *buffer, int axis, bool on) {
    if ((!buffer) || (axis<0) || (axis>1)) {
        return false;
    }
    sprintf(buffer, AXIS_POWER_CMD, CTRL_AXES[axis], on);
    return true;
}

bool FlexDCAxis::buildHomeMacroCommand(char *buffer, int axis, bool forwards, flexdcHomeMacro home_type) {
    if ((!buffer) || (axis<0) || (axis>1) || (home_type==DISABLED)) {
        return false;
    }
    if (forwards) {
        sprintf(buffer, HOMF_MACRO[home_type], CTRL_AXES[axis], CTRL_AXES[axis]);
    } else {
        sprintf(buffer, HOMR_MACRO[home_type], CTRL_AXES[axis], CTRL_AXES[axis]);
    }
    return true;
}

bool FlexDCAxis::buildGenericGetCommand(char *buffer, const char *command_format, int axis) {
    if ((!buffer) || (!command_format) || (axis<0) || (axis>1)) {
        return false;
    }
    sprintf(buffer, command_format, CTRL_AXES[axis]);
    return true;
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
