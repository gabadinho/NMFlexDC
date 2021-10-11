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
const char* MOTION_END_REASON[] = {
    "IN_MOTION",
    "NORMAL",
    "HARD_FLS",
    "HARD_RLS",
    "SOFT_HL",
    "SOFT_LL",
    "MOTOR_FAULT",
    "USER_STOP",
    "MOTOR_OFF",
    "BAD_PARAM"
};

enum flexdcMacroResult {
    EXECUTING,
    OK,
    FAIL_NO_INDEX_FOUND=5,
    FAIL_TOO_MANY_FOUND,
    FAIL_GET_OFF_INPUT=9
};
const char* MACRO_RESULT[] = {
    "EXECUTING",
    "OK",
    "OTHER2",
    "OTHER3",
    "OTHER4",
    "FAIL_NO_INDEX_FOUND",
    "FAIL_TOO_MANY_FOUND",
    "OTHER7",
    "OTHER8",
    "FAIL_GET_OFF_INPUT"
};

enum flexdcHomeMacro {
    DISABLED,
    HOME_LS,
    HOME_IDX
};
const char* HOMR_MACRO[] = {
    "",
    "%cQE,#HINRI%c",
    "%cQE,#HINX_%c"
};
const char* HOMF_MACRO[] = {
    "",
    "%cQE,#HINFI%c",
    "%cQE,#HINX_%c"
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
    static bool buildMoveCommand(char *buffer, int axis, double position, bool relative, double velocity);
    static bool buildSetPositionCommand(char *buffer, int axis, double position);
    static bool buildStopCommand(char *buffer, int axis);
    static bool buildHaltMacroCommand(char *buffer, int axis);
    static bool buildMotorPowerCommand(char *buffer, int axis, bool on);
    static bool buildHomeMacroCommand(char *buffer, int axis, bool forwards, flexdcHomeMacro home_type);
    static bool buildGenericGetCommand(char *buffer, const char *command_format, int axis);

protected:
    // Specific class methods
    void setStatusProblem(asynStatus status);

    asynStatus setMotionDone(int motion_status, flexdcMacroResult macro_result, bool power_on, long pos_error);

    asynStatus switchMotorPower(bool on);
    asynStatus stopMotor();
    asynStatus haltHomingMacro();
    void shortWait();

private:
    FlexDCController *pC_; // Pointer to the asynMotorController to which this axis belongs

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
