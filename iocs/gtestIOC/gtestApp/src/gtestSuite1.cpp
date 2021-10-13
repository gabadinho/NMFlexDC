#include <gtest/gtest.h>

#include "FlexDCMotorDriver.h"



#define STRING_BUFFER_SIZE 256



TEST(CommandBuild, MoveAbs_0_1000000_20000) {
    char buffer[STRING_BUFFER_SIZE];
    bool res = FlexDCAxis::buildMoveCommand(buffer, 0, 1000000, false, 20000);
    ASSERT_EQ(true, res);
    ASSERT_STREQ("XMO=1;XMM=0;XSM=0;XSP=20000;XAP=1000000;XBG", buffer);
}

TEST(CommandBuild, MoveRel_1_min25000_1000) {
    char buffer[STRING_BUFFER_SIZE];
    bool res = FlexDCAxis::buildMoveCommand(buffer, 1, -25000, true, 1000);
    ASSERT_EQ(true, res);
    ASSERT_STREQ("YMO=1;YMM=0;YSM=0;YSP=1000;YRP=-25000;YBG", buffer);
}

TEST(CommandBuild, MoveAbs_2_0_2000) {
    char buffer[STRING_BUFFER_SIZE] = "MyBuffer";
    bool res = FlexDCAxis::buildMoveCommand(buffer, 2, 0, false, 2000);
    ASSERT_EQ(false, res);
    ASSERT_STREQ("MyBuffer", buffer);
}



TEST(CommandBuild, SetPosition_0_100) {
    char buffer[STRING_BUFFER_SIZE];
    bool res = FlexDCAxis::buildSetPositionCommand(buffer, 0, 100);
    ASSERT_EQ(true, res);
    ASSERT_STREQ("XPS=100", buffer);
}

TEST(CommandBuild, SetPosition_0_min1253) {
    char buffer[STRING_BUFFER_SIZE];
    bool res = FlexDCAxis::buildSetPositionCommand(buffer, 0, -1253);
    ASSERT_EQ(true, res);
    ASSERT_STREQ("XPS=-1253", buffer);
}

TEST(CommandBuild, SetPosition_0_pi) {
    char buffer[STRING_BUFFER_SIZE];
    bool res = FlexDCAxis::buildSetPositionCommand(buffer, 0, 3.1415);
    ASSERT_EQ(true, res);
    ASSERT_STREQ("XPS=3", buffer);
}

TEST(CommandBuild, SetPosition_0_0) {
    char buffer[STRING_BUFFER_SIZE];
    bool res = FlexDCAxis::buildSetPositionCommand(buffer, 0, 0);
    ASSERT_EQ(true, res);
    ASSERT_STREQ("XPS=0", buffer);
}

TEST(CommandBuild, SetPosition_1_1) {
    char buffer[STRING_BUFFER_SIZE];
    bool res = FlexDCAxis::buildSetPositionCommand(buffer, 1, 1);
    ASSERT_EQ(true, res);
    ASSERT_STREQ("YPS=1", buffer);
}

TEST(CommandBuild, SetPosition_2_20000) {
    char buffer[STRING_BUFFER_SIZE] = "MyBuffer";
    bool res = FlexDCAxis::buildSetPositionCommand(buffer, 2, 20000);
    ASSERT_EQ(false, res);
    ASSERT_STREQ("MyBuffer", buffer);
}



TEST(CommandBuild, Stop_0) {
    char buffer[STRING_BUFFER_SIZE];
    bool res = FlexDCAxis::buildStopCommand(buffer, 0);
    ASSERT_EQ(true, res);
    ASSERT_STREQ("XST", buffer);
}

TEST(CommandBuild, Stop_1) {
    char buffer[STRING_BUFFER_SIZE];
    bool res = FlexDCAxis::buildStopCommand(buffer, 1);
    ASSERT_EQ(true, res);
    ASSERT_STREQ("YST", buffer);
}

TEST(CommandBuild, Stop_2) {
    char buffer[STRING_BUFFER_SIZE] = "MyBuffer";
    bool res = FlexDCAxis::buildStopCommand(buffer, 2);
    ASSERT_EQ(false, res);
    ASSERT_STREQ("MyBuffer", buffer);
}



TEST(CommandBuild, HaltMacro_0) {
    char buffer[STRING_BUFFER_SIZE];
    bool res = FlexDCAxis::buildHaltMacroCommand(buffer, 0);
    ASSERT_EQ(true, res);
    ASSERT_STREQ("XQK;XQI", buffer);
}

TEST(CommandBuild, HaltMacro_1) {
    char buffer[STRING_BUFFER_SIZE];
    bool res = FlexDCAxis::buildHaltMacroCommand(buffer, 1);
    ASSERT_EQ(true, res);
    ASSERT_STREQ("YQK;YQI", buffer);
}

TEST(CommandBuild, HaltMacro_min1) {
    char buffer[STRING_BUFFER_SIZE] = "MyBuffer";
    bool res = FlexDCAxis::buildHaltMacroCommand(buffer, -1);
    ASSERT_EQ(false, res);
    ASSERT_STREQ("MyBuffer", buffer);
}



TEST(CommandBuild, MotorPower_Off_0) {
    char buffer[STRING_BUFFER_SIZE];
    bool res = FlexDCAxis::buildMotorPowerCommand(buffer, 0, false);
    ASSERT_EQ(true, res);
    ASSERT_STREQ("XMO=0", buffer);
}

TEST(CommandBuild, MotorPower_On_0) {
    char buffer[STRING_BUFFER_SIZE];
    bool res = FlexDCAxis::buildMotorPowerCommand(buffer, 0, true);
    ASSERT_EQ(true, res);
    ASSERT_STREQ("XMO=1", buffer);
}

TEST(CommandBuild, MotorPower_Off_1) {
    char buffer[STRING_BUFFER_SIZE];
    bool res = FlexDCAxis::buildMotorPowerCommand(buffer, 1, false);
    ASSERT_EQ(true, res);
    ASSERT_STREQ("YMO=0", buffer);
}

TEST(CommandBuild, MotorPower_On_1) {
    char buffer[STRING_BUFFER_SIZE];
    bool res = FlexDCAxis::buildMotorPowerCommand(buffer, 1, true);
    ASSERT_EQ(true, res);
    ASSERT_STREQ("YMO=1", buffer);
}

TEST(CommandBuild, MotorPower_On_2) {
    char buffer[STRING_BUFFER_SIZE] = "MyBuffer";
    bool res = FlexDCAxis::buildMotorPowerCommand(buffer, 2, true);
    ASSERT_EQ(false, res);
    ASSERT_STREQ("MyBuffer", buffer);
}

TEST(CommandBuild, MotorPower_Off_3) {
    char buffer[STRING_BUFFER_SIZE] = "MyBuffer";
    bool res = FlexDCAxis::buildMotorPowerCommand(buffer, 3, false);
    ASSERT_EQ(false, res);
    ASSERT_STREQ("MyBuffer", buffer);
}





TEST(CommandBuild, HomeMacro_0_For_Lim) {
    char buffer[STRING_BUFFER_SIZE];
    bool res = FlexDCAxis::buildHomeMacroCommand(buffer, 0, true, HOME_LS);
    ASSERT_EQ(true, res);
    ASSERT_STREQ("XQE,#HINFIX", buffer);
}

TEST(CommandBuild, HomeMacro_0_For_Idx) {
    char buffer[STRING_BUFFER_SIZE];
    bool res = FlexDCAxis::buildHomeMacroCommand(buffer, 0, true, HOME_IDX);
    ASSERT_EQ(true, res);
    ASSERT_STREQ("XQE,#HINX_X", buffer);
}

TEST(CommandBuild, HomeMacro_0_For_Disa) {
    char buffer[STRING_BUFFER_SIZE] = "MyBuffer";
    bool res = FlexDCAxis::buildHomeMacroCommand(buffer, 0, true, DISABLED);
    ASSERT_EQ(false, res);
    ASSERT_STREQ("MyBuffer", buffer);
}

TEST(CommandBuild, HomeMacro_1_Rev_Lim) {
    char buffer[STRING_BUFFER_SIZE];
    bool res = FlexDCAxis::buildHomeMacroCommand(buffer, 1, false, HOME_LS);
    ASSERT_EQ(true, res);
    ASSERT_STREQ("YQE,#HINRIY", buffer);
}

TEST(CommandBuild, HomeMacro_1_Rev_Idx) {
    char buffer[STRING_BUFFER_SIZE];
    bool res = FlexDCAxis::buildHomeMacroCommand(buffer, 1, false, HOME_IDX);
    ASSERT_EQ(true, res);
    ASSERT_STREQ("YQE,#HINX_Y", buffer);
}

TEST(CommandBuild, HomeMacro_1_Rev_Disa) {
    char buffer[STRING_BUFFER_SIZE] = "MyBuffer";
    bool res = FlexDCAxis::buildHomeMacroCommand(buffer, 1, false, DISABLED);
    ASSERT_EQ(false, res);
    ASSERT_STREQ("MyBuffer", buffer);
}

TEST(CommandBuild, HomeMacro_2_For_Lim) {
    char buffer[STRING_BUFFER_SIZE] = "MyBuffer";
    bool res = FlexDCAxis::buildHomeMacroCommand(buffer, 2, true, HOME_LS);
    ASSERT_EQ(false, res);
    ASSERT_STREQ("MyBuffer", buffer);
}



TEST(CommandBuild, GetSpeed_0) {
    char buffer[STRING_BUFFER_SIZE];
    bool res = FlexDCAxis::buildGenericGetCommand(buffer, AXIS_GETSPEED_CMD, 0);
    ASSERT_EQ(true, res);
    ASSERT_STREQ("XSP", buffer);
}

TEST(CommandBuild, GetSpeed_1) {
    char buffer[STRING_BUFFER_SIZE];
    bool res = FlexDCAxis::buildGenericGetCommand(buffer, AXIS_GETSPEED_CMD, 1);
    ASSERT_EQ(true, res);
    ASSERT_STREQ("YSP", buffer);
}



TEST(CommandBuild, IsPowered_0) {
    char buffer[STRING_BUFFER_SIZE];
    bool res = FlexDCAxis::buildGenericGetCommand(buffer, AXIS_ISPOWERED_CMD, 0);
    ASSERT_EQ(true, res);
    ASSERT_STREQ("XMO", buffer);
}

TEST(CommandBuild, IsPowered_1) {
    char buffer[STRING_BUFFER_SIZE];
    bool res = FlexDCAxis::buildGenericGetCommand(buffer, AXIS_ISPOWERED_CMD, 1);
    ASSERT_EQ(true, res);
    ASSERT_STREQ("YMO", buffer);
}



TEST(CommandBuild, MacroResult_0) {
    char buffer[STRING_BUFFER_SIZE];
    bool res = FlexDCAxis::buildGenericGetCommand(buffer, AXIS_MACRO_RESULT_CMD, 0);
    ASSERT_EQ(true, res);
    ASSERT_STREQ("XPA[11]", buffer);
}

TEST(CommandBuild, MacroResult_1) {
    char buffer[STRING_BUFFER_SIZE];
    bool res = FlexDCAxis::buildGenericGetCommand(buffer, AXIS_MACRO_RESULT_CMD, 1);
    ASSERT_EQ(true, res);
    ASSERT_STREQ("YPA[11]", buffer);
}



TEST(CommandBuild, PositionError_0) {
    char buffer[STRING_BUFFER_SIZE];
    bool res = FlexDCAxis::buildGenericGetCommand(buffer, AXIS_POSERR_CMD, 0);
    ASSERT_EQ(true, res);
    ASSERT_STREQ("XPE", buffer);
}

TEST(CommandBuild, PositionError_1) {
    char buffer[STRING_BUFFER_SIZE];
    bool res = FlexDCAxis::buildGenericGetCommand(buffer, AXIS_POSERR_CMD, 1);
    ASSERT_EQ(true, res);
    ASSERT_STREQ("YPE", buffer);
}



TEST(CommandBuild, MotionStatus_0) {
    char buffer[STRING_BUFFER_SIZE];
    bool res = FlexDCAxis::buildGenericGetCommand(buffer, AXIS_MOTIONSTATUS_CMD, 0);
    ASSERT_EQ(true, res);
    ASSERT_STREQ("XMS", buffer);
}

TEST(CommandBuild, MotionStatus_1) {
    char buffer[STRING_BUFFER_SIZE];
    bool res = FlexDCAxis::buildGenericGetCommand(buffer, AXIS_MOTIONSTATUS_CMD, 1);
    ASSERT_EQ(true, res);
    ASSERT_STREQ("YMS", buffer);
}



TEST(CommandBuild, MotionEndReason_0) {
    char buffer[STRING_BUFFER_SIZE];
    bool res = FlexDCAxis::buildGenericGetCommand(buffer, AXIS_MOTIONEND_CMD, 0);
    ASSERT_EQ(true, res);
    ASSERT_STREQ("XEM", buffer);
}

TEST(CommandBuild, MotionEndReason_1) {
    char buffer[STRING_BUFFER_SIZE];
    bool res = FlexDCAxis::buildGenericGetCommand(buffer, AXIS_MOTIONEND_CMD, 1);
    ASSERT_EQ(true, res);
    ASSERT_STREQ("YEM", buffer);
}



TEST(CommandBuild, MotorFault_0) {
    char buffer[STRING_BUFFER_SIZE];
    bool res = FlexDCAxis::buildGenericGetCommand(buffer, AXIS_MOTORFAULT_CMD, 0);
    ASSERT_EQ(true, res);
    ASSERT_STREQ("XMF", buffer);
}

TEST(CommandBuild, MotorFault_1) {
    char buffer[STRING_BUFFER_SIZE];
    bool res = FlexDCAxis::buildGenericGetCommand(buffer, AXIS_MOTORFAULT_CMD, 1);
    ASSERT_EQ(true, res);
    ASSERT_STREQ("YMF", buffer);
}

