#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "FlexDCMotorDriver.h"



class MockFlexDCAxis: public FlexDCAxis {
public:
    MockFlexDCAxis(FlexDCController *ctrl): FlexDCAxis(ctrl, 0) { this->pC_->shuttingDown_=1; } // Accessing controller's instance variable to prevent polling

    void log(int reason, const char *format, ...) {}

    MOCK_METHOD(asynStatus, switchMotorPower, (bool), (override));
    MOCK_METHOD(asynStatus, getDoubleParam, (int, double*), (override));
    MOCK_METHOD(asynStatus, setIntegerParam, (int, int), (override));
    MOCK_METHOD(void, setStatusProblem, (asynStatus), (override));
    MOCK_METHOD(asynStatus, callParamCallbacks, (), (override));

    asynStatus setMotionDone(int motion_status, flexdcMacroResult macro_result, bool power_on, long pos_error) {
        return FlexDCAxis::setMotionDone(motion_status, macro_result, power_on, pos_error);
    }
};



/** Creating a shared global dummy controller.
  * Violates isolation principle of unit-testing, but otherwise it core-dumps...
  */
FlexDCController dummy_ctrl("fake_ctrl", "fake_conn", 2, 1, 1);



TEST(setMotionDoneMock, AlreadyOff) {
    MockFlexDCAxis dummy_axis(&dummy_ctrl);
    EXPECT_CALL(dummy_axis, setIntegerParam(testing::_, 1));
    dummy_axis.setMotionDone(0, OK, false, 10);
}

TEST(setMotionDoneMock, ErrorTooLarge) {
    MockFlexDCAxis dummy_axis(&dummy_ctrl);
    EXPECT_CALL(dummy_axis, getDoubleParam(testing::_, testing::_)).Times(2);;
    dummy_axis.setMotionDone(0, OK, true, 10);
}

TEST(setMotionDoneMock, ShouldStop) {
    MockFlexDCAxis dummy_axis(&dummy_ctrl);
    EXPECT_CALL(dummy_axis, getDoubleParam(testing::_, testing::_)).Times(2);;
    EXPECT_CALL(dummy_axis, switchMotorPower(false));
    EXPECT_CALL(dummy_axis, setIntegerParam(testing::_, 1));
    dummy_axis.setMotionDone(0, OK, true, 0);
}

TEST(setMotionDoneMock, StillMoving) {
    MockFlexDCAxis dummy_axis(&dummy_ctrl);
    dummy_axis.setMotionDone(1, OK, true, 10);
}

TEST(setMotionDoneMock, MacroRunning) {
    MockFlexDCAxis dummy_axis(&dummy_ctrl);
    dummy_axis.setMotionDone(0, EXECUTING, true, 10);
}

