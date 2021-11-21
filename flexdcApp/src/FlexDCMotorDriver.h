/*
FILENAME...   FlexDCMotorDriver.h
USAGE...      Motor driver support (model 3, asyn) for the Nanomotion FlexDC controller

Jose G.C. Gabadinho
April 2021
*/

#ifndef _FLEXDCMOTORDRIVER_H_
#define _FLEXDCMOTORDRIVER_H_

#include <asynMotorController.h>
#include <asynMotorAxis.h>



#define AXIS_MRES_PARAMNAME "MOTOR_MRES"
#define AXIS_RDBD_PARAMNAME "MOTOR_RDBD"
#define AXIS_HOMR_PARAMNAME "MOTOR_HOMR"
#define AXIS_HOMF_PARAMNAME "MOTOR_HOMF"
#define AXIS_HOMS_PARAMNAME "MOTOR_HOMS"
#define CTRL_RST_PARAMNAME  "CTRL_RESET"



const char CTRL_AXES[] = { 'X', 'Y' };

const char CTRL_VER_CMD[] = "XVR";

const char CTRL_RESET_CMD[] = "XQK;YQK;AMO=0;XRS";

const char AXIS_MOVEABS_CMD[]  = "%cMO=1;%cMM=0;%cSM=0;%cSP=%d;%cAP=%ld;%cBG";
const char AXIS_MOVEREL_CMD[]  = "%cMO=1;%cMM=0;%cSM=0;%cSP=%d;%cRP=%ld;%cBG";
const char AXIS_FORCEPOS_CMD[] = "%cPS=%ld";

const char AXIS_GETPOS_CMD[] = "%cPS";
const char AXIS_POSERR_CMD[] = "%cPE";

const char AXIS_MOTIONSTATUS_CMD[] = "%cMS";
const char AXIS_MOTIONEND_CMD[]    = "%cEM";
const char AXIS_MOTORFAULT_CMD[]   = "%cMF";

const char AXIS_STOP_CMD[] = "%cST";

const char AXIS_GETSPEED_CMD[] = "%cSP";
const char AXIS_SETSPEED_CMD[] = "%cSP=%d";

const char AXIS_POWER_CMD[]     = "%cMO=%d";
const char AXIS_ISPOWERED_CMD[] = "%cMO";

const char AXIS_MACRO_RESULT_CMD[] = "%cPA[11]";

const char AXIS_MACRO_HALT_CMD[]     = "%cQH";
const char AXIS_MACRO_KILLINIT_CMD[] = "%cQK;%cQI";



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

enum flexdcHomeMacro {
    DISABLED,
    HOME_LS,
    HOME_IDX
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

    // Class-wide methods
    static bool updateAxisReadbackPosition(asynStatus status, const char *reply, long& readback, asynStatus *asyn_error);
    static bool updateAxisMotorPower(asynStatus status, const char *reply, bool& motor_power, asynStatus *asyn_error);
    static bool updateAxisMotionStatus(asynStatus status, const char *reply, int& motion_stat, asynStatus *asyn_error);
    static bool updateAxisMacroResult(asynStatus status, const char *reply, flexdcMacroResult& macro_res, asynStatus *asyn_error);
    static bool updateAxisMotionEnd(asynStatus status, const char *reply, flexdcMotionEndReason& motion_end, asynStatus *asyn_error);
    static bool updateAxisPositionError(asynStatus status, const char *reply, long& pos_error, asynStatus *asyn_error);
    static bool updateAxisMotorFault(asynStatus status, const char *reply, int& mot_fault, asynStatus *asyn_error);

    static bool buildMoveCommand(char *buffer, int axis, double position, bool relative, double velocity);
    static bool buildSetPositionCommand(char *buffer, int axis, double position);
    static bool buildStopCommand(char *buffer, int axis);
    static bool buildHaltMacroCommand(char *buffer, int axis);
    static bool buildMotorPowerCommand(char *buffer, int axis, bool on);
    static bool buildHomeMacroCommand(char *buffer, int axis, bool forwards, flexdcHomeMacro home_type);
    static bool buildGenericGetCommand(char *buffer, const char *command_format, int axis);

    static bool issigneddigit(const char *buffer) {
        size_t buf_len = strlen(buffer);
        if (buf_len == 1) return isdigit(*buffer);
        if (buf_len > 1) return isdigit(*buffer) || (*buffer=='-' && isdigit(*++buffer));
        return false;
    }

protected:
    // Specific class methods
    virtual void setStatusProblem(asynStatus status);

    virtual asynStatus setMotionDone(int motion_status, flexdcMacroResult macro_result, bool power_on, long pos_error);

    virtual asynStatus switchMotorPower(bool on);
    virtual asynStatus stopMotor();
    virtual asynStatus haltHomingMacro();
    virtual void shortWait();

    virtual asynStatus getIntegerParam(int index, epicsInt32 *value);
    virtual asynStatus getDoubleParam(int index, double *value);

    virtual void log(int reason, const char *format, ...);

    FlexDCController *pC_; // Pointer to the asynMotorController to which this axis belongs

private:
    int motionStatus;
    int motorFault;
    flexdcMotionEndReason endMotionReason;
    flexdcMacroResult macroResult;
    long positionError;
    long positionReadback;
    bool isMotorOn;

friend class FlexDCController;
};



class FlexDCController: public asynMotorController {

public:
    FlexDCController(const char *portName, const char *asynPortName, int numAxes, double movingPollPeriod, double idlePollPeriod);

    // These are the methods we override from the base class
    asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);

    void report(FILE *fp, int level);

    FlexDCAxis* getAxis(asynUser *pasynUser);
    FlexDCAxis* getAxis(int axisNo);

protected:
    virtual void log(int reason, const char *format, ...);

    int driverMotorRecResolution;
    int driverRetryDeadband;
    int driverHomeReverseMacro;
    int driverHomeForwardMacro;
    int driverHomeStatus;
    int driverResetController;
#define NUM_FLEXDC_PARAMS 6

private:

friend class FlexDCAxis;
};

#endif // _FLEXDCMOTORDRIVER_H_
