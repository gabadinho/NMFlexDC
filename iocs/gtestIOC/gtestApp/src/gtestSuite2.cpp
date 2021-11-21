#include <gtest/gtest.h>

#include "FlexDCMotorDriver.h"



TEST(ReplyParse, AsynErrorStatus) {
    char reply[] = "1000";
    long rb;
    asynStatus asyn_error = asynSuccess;
    bool res = FlexDCAxis::updateAxisReadbackPosition(asynError, reply, rb, &asyn_error);
    ASSERT_EQ(false, res);
    ASSERT_EQ(asynError, asyn_error);
}

TEST(ReplyParse, ZeroReadback) {
    char reply[] = "0";
    long rb;
    asynStatus asyn_error = asynSuccess;
    bool res = FlexDCAxis::updateAxisReadbackPosition(asynSuccess, reply, rb, &asyn_error);
    ASSERT_EQ(true, res);
    ASSERT_EQ(0, rb);
    ASSERT_EQ(asynSuccess, asyn_error);
}

TEST(ReplyParse, PositiveReadback) {
    char reply[] = "1000";
    long rb;
    asynStatus asyn_error = asynSuccess;
    bool res = FlexDCAxis::updateAxisReadbackPosition(asynSuccess, reply, rb, &asyn_error);
    ASSERT_EQ(true, res);
    ASSERT_EQ(1000, rb);
    ASSERT_EQ(asynSuccess, asyn_error);
}

TEST(ReplyParse, NegativeReadback) {
    char reply[] = "-2500";
    long rb;
    asynStatus asyn_error = asynSuccess;
    bool res = FlexDCAxis::updateAxisReadbackPosition(asynSuccess, reply, rb, &asyn_error);
    ASSERT_EQ(true, res);
    ASSERT_EQ(-2500, rb);
    ASSERT_EQ(asynSuccess, asyn_error);
}



TEST(ReplyParse, MotorPowerOff) {
    char reply[] = "0";
    bool mp;
    asynStatus asyn_error = asynSuccess;
    bool res = FlexDCAxis::updateAxisMotorPower(asynSuccess, reply, mp, &asyn_error);
    ASSERT_EQ(true, res);
    ASSERT_EQ(false, mp);
    ASSERT_EQ(asynSuccess, asyn_error);
}

TEST(ReplyParse, MotorPowerOn) {
    char reply[] = "1";
    bool mp;
    asynStatus asyn_error = asynSuccess;
    bool res = FlexDCAxis::updateAxisMotorPower(asynSuccess, reply, mp, &asyn_error);
    ASSERT_EQ(true, res);
    ASSERT_EQ(true, mp);
    ASSERT_EQ(asynSuccess, asyn_error);
}

TEST(ReplyParse, MotorPowerError) {
    char reply[] = "2";
    bool mp;
    asynStatus asyn_error = asynSuccess;
    bool res = FlexDCAxis::updateAxisMotorPower(asynSuccess, reply, mp, &asyn_error);
    ASSERT_EQ(false, res);
    ASSERT_EQ(asynError, asyn_error);
}



TEST(ReplyParse, CharMotionStatus) {
    char reply[] = "A";
    int ms;
    asynStatus asyn_error = asynSuccess;
    bool res = FlexDCAxis::updateAxisMotionStatus(asynSuccess, reply, ms, &asyn_error);
    ASSERT_EQ(false, res);
    ASSERT_EQ(asynError, asyn_error);
}



TEST(ReplyParse, OverflowMacroResult) {
    char reply[] = "10";
    flexdcMacroResult mr;
    asynStatus asyn_error = asynSuccess;
    bool res = FlexDCAxis::updateAxisMacroResult(asynSuccess, reply, mr, &asyn_error);
    ASSERT_EQ(false, res);
    ASSERT_EQ(asynError, asyn_error);
}

TEST(ReplyParse, MacroResultOk) {
    char reply[] = "1";
    flexdcMacroResult mr;
    asynStatus asyn_error = asynSuccess;
    bool res = FlexDCAxis::updateAxisMacroResult(asynSuccess, reply, mr, &asyn_error);
    ASSERT_EQ(true, res);
    ASSERT_EQ(OK, mr);
}



TEST(ReplyParse, OverflowMotionEndReason) {
    char reply[] = "10";
    flexdcMotionEndReason me;
    asynStatus asyn_error = asynSuccess;
    bool res = FlexDCAxis::updateAxisMotionEnd(asynSuccess, reply, me, &asyn_error);
    ASSERT_EQ(false, res);
    ASSERT_EQ(asynError, asyn_error);
}

TEST(ReplyParse, MotionEndReasonHLS) {
    char reply[] = "2";
    flexdcMotionEndReason me;
    asynStatus asyn_error = asynSuccess;
    bool res = FlexDCAxis::updateAxisMotionEnd(asynSuccess, reply, me, &asyn_error);
    ASSERT_EQ(true, res);
    ASSERT_EQ(HARD_FLS, me);
}



TEST(ReplyParse, EmptyPositionError) {
    char reply[] = "";
    long pe;
    asynStatus asyn_error = asynSuccess;
    bool res = FlexDCAxis::updateAxisPositionError(asynError, reply, pe, &asyn_error);
    ASSERT_EQ(false, res);
    ASSERT_EQ(asynError, asyn_error);
}



TEST(ReplyParse, EmptyMotorFault) {
    char reply[] = "";
    int mf;
    asynStatus asyn_error = asynSuccess;
    bool res = FlexDCAxis::updateAxisMotorFault(asynError, reply, mf, &asyn_error);
    ASSERT_EQ(false, res);
    ASSERT_EQ(asynError, asyn_error);
}

