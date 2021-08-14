/*
FILENAME...   FlexDCMotorDriver.h
USAGE...      Motor driver support (model 3, asyn) for the Nanomotion FlexDC controller

Jose Gabadinho
April 2021
*/

#ifndef _FLEXDCMOTORDRIVER_H_
#define _FLEXDCMOTORDRIVER_H_

#include "asynMotorController.h"
#include "asynMotorAxis.h"



#define MAX_FLEXDC_STRING_SIZE 80

#define AXIS_RDBD_PARAMNAME "MOTOR_RDBD"



const char CTRL_RESET_CMD[] = "XRS";

const char CTRL_VER_CMD[] = "XVR";

const char* AXIS_MOVE_CMD[]     = { "XMM=0;XSM=0;XAP=%d;XBG", "YMM=0;YSM=0;YAP=%d;YBG" };
const char* AXIS_FORCEPOS_CMD[] = { "XPS=%ld", "YPS=%ld" };

const char* AXIS_GETPOS_CMD[]   = { "XPS", "YPS" };
const char* AXIS_POSERR_CMD[]   = { "XPE", "YPE" };

const char* AXIS_MOTIONSTAT_CMD[]   = { "XMS", "YMS" };
const char* AXIS_ENDMOTREAS_CMD[]   = { "XEM", "YEM" };
const char* AXIS_MOTORFAULT_CMD[]   = { "XMF", "YMF" };

const char* AXIS_STOP_CMD[] = { "XST", "YST" };

const char* AXIS_GETSPEED_CMD[] = { "XSP", "YSP" };
const char* AXIS_SETSPEED_CMD[] = { "XSP=%d", "YSP=%d" };

const char* AXIS_POWER_CMD[]     = { "XMO=%d", "YMO=%d" };
const char* AXIS_ISPOWERED_CMD[] = { "XMO", "YMO" };

const char* AXIS_MACRO_RESULT_CMD[] = { "XPA[11]", "YPA[11]" };


enum flexdcMotionEndReason {
    IN_MOTION,
    NORMAL,
    HARD_FLS,
    HARD_RLS,
    SOFT_HL,
    SOFT_LL,
    MOTOR_FAULT,
    USER_STOP,
    MOTOR_OFF,
    BAD_PARAM
};

enum flexdcMacroResult {
    EXECUTING,
    OK,
    FAIL_NO_INDEX_FOUND=5,
    FAIL_TOO_MANY_FOUND,
    FAIL_GET_OFF_INPUT=9
};

enum flexdcMotionStatus {
    IDLE,
    MOVING,
    STOPPED,
    ACCEL,
    DECEL
};



class FlexDCAxis: public asynMotorAxis {

public:
    FlexDCAxis(class FlexDCController *pC, int axis);

    // These are the methods we override from the base class
    void report(FILE *fp, int level);

    asynStatus move(double position, int relative, double min_velocity, double max_velocity, double acceleration);
    asynStatus home(double minVelocity, double maxVelocity, double acceleration, int forwards);
    asynStatus stop(double acceleration);
    asynStatus setPosition(double position);

    asynStatus poll(bool *moving);

    // Specific class methods
    void setStatusProblem(asynStatus status);

private:
    FlexDCController *pC_; // Pointer to the asynMotorController to which this axis belongs

    int motionStatus;
    int endMotionReason;
    int macroResult;
    int motorFault;
    long positionError;
    long readbackPosition;
    bool motorOn;

friend class FlexDCController;
};



class FlexDCController: public asynMotorController {

public:
    FlexDCController(const char *portName, const char *asynPortName, int numAxes, double movingPollPeriod, double idlePollPeriod);

    // These are the methods we override from the base class
    void report(FILE *fp, int level);

    FlexDCAxis* getAxis(asynUser *pasynUser);
    FlexDCAxis* getAxis(int axisNo);

protected:
    int driverRetryDeadband;
#define NUM_FLEXDC_PARAMS 1

private:

friend class FlexDCAxis;
};

#endif // _FLEXDCMOTORDRIVER_H_
