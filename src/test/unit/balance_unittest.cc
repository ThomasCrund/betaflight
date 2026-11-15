/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software and/or
 * modify this software under the terms of the GNU General Public License
 * as published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 */

#include <cmath>

#include "gtest/gtest.h"

extern "C" {
#include "common/maths.h"
#include "flight/balance.h"
}

TEST(BalanceControlTest, AxisAngleRemainsContinuousAcrossVertical)
{
    const float angle89 = DEGREES_TO_RADIANS(89.0f);
    const float angle91 = DEGREES_TO_RADIANS(91.0f);

    EXPECT_NEAR(89.0f, balanceAxisAngleDegrees(sinf(angle89), cosf(angle89)), 0.01f);
    EXPECT_NEAR(91.0f, balanceAxisAngleDegrees(sinf(angle91), cosf(angle91)), 0.01f);
}

TEST(BalanceControlTest, AngleErrorUsesShortestRotation)
{
    EXPECT_FLOAT_EQ(-91.0f, balanceAngleErrorDegrees(90.0f, -179.0f));
    EXPECT_FLOAT_EQ(91.0f, balanceAngleErrorDegrees(-90.0f, 179.0f));
}

TEST(BalanceControlTest, FlatStartSelectsConfiguredDefaultPitchEdge)
{
    balanceOrientation_t orientation = balanceSelectOrientation(3.0f, -2.0f, 90.0f, 15.0f);
    EXPECT_TRUE(orientation.fromFlat);
    EXPECT_EQ(BALANCE_AXIS_PITCH, orientation.axis);
    EXPECT_FLOAT_EQ(90.0f, orientation.targetAngle);

    orientation = balanceSelectOrientation(3.0f, -2.0f, -90.0f, 15.0f);
    EXPECT_TRUE(orientation.fromFlat);
    EXPECT_EQ(BALANCE_AXIS_PITCH, orientation.axis);
    EXPECT_FLOAT_EQ(-90.0f, orientation.targetAngle);
}

TEST(BalanceControlTest, SideStartSelectsNearestSignedAxis)
{
    balanceOrientation_t orientation = balanceSelectOrientation(82.0f, 4.0f, 90.0f, 15.0f);
    EXPECT_FALSE(orientation.fromFlat);
    EXPECT_EQ(BALANCE_AXIS_PITCH, orientation.axis);
    EXPECT_FLOAT_EQ(90.0f, orientation.targetAngle);

    orientation = balanceSelectOrientation(-84.0f, 3.0f, 90.0f, 15.0f);
    EXPECT_EQ(BALANCE_AXIS_PITCH, orientation.axis);
    EXPECT_FLOAT_EQ(-90.0f, orientation.targetAngle);

    orientation = balanceSelectOrientation(5.0f, 81.0f, 90.0f, 15.0f);
    EXPECT_EQ(BALANCE_AXIS_ROLL, orientation.axis);
    EXPECT_FLOAT_EQ(90.0f, orientation.targetAngle);

    orientation = balanceSelectOrientation(-4.0f, -83.0f, 90.0f, 15.0f);
    EXPECT_EQ(BALANCE_AXIS_ROLL, orientation.axis);
    EXPECT_FLOAT_EQ(-90.0f, orientation.targetAngle);
}

TEST(BalanceControlTest, IntermediateStartSelectsAnEdgeWithoutAnAttitudeLockout)
{
    balanceOrientation_t orientation = balanceSelectOrientation(45.0f, 8.0f, 90.0f, 15.0f);
    EXPECT_FALSE(orientation.fromFlat);
    EXPECT_EQ(BALANCE_AXIS_PITCH, orientation.axis);
    EXPECT_FLOAT_EQ(90.0f, orientation.targetAngle);

    orientation = balanceSelectOrientation(-5.0f, -45.0f, 90.0f, 15.0f);
    EXPECT_FALSE(orientation.fromFlat);
    EXPECT_EQ(BALANCE_AXIS_ROLL, orientation.axis);
    EXPECT_FLOAT_EQ(-90.0f, orientation.targetAngle);
}

TEST(BalanceControlTest, DesiredRateLimitsLargeAngleSteps)
{
    EXPECT_FLOAT_EQ(80.0f, balanceDesiredRate(90.0f, 0.012f, 0.003f, 80.0f));
    EXPECT_FLOAT_EQ(-80.0f, balanceDesiredRate(-90.0f, 0.012f, 0.003f, 80.0f));
    EXPECT_FLOAT_EQ(20.0f, balanceDesiredRate(5.0f, 0.012f, 0.003f, 80.0f));
}

TEST(BalanceControlTest, ControllerCombinesAngleAndRateFeedback)
{
    EXPECT_NEAR(0.04f, balanceControllerOutput(81.0f, 79.0f, 0.0f,
        0.02f, 0.003f, 80.0f, 0.25f, 0.35f, 0.20f, 0.20f,
        1.0f, 1.7f, 45.0f, 30.0f, 0.0f, 1.0f), 0.0001f);
    EXPECT_NEAR(0.01f, balanceControllerOutput(81.0f, 79.0f, 10.0f,
        0.02f, 0.003f, 80.0f, 0.25f, 0.35f, 0.20f, 0.20f,
        1.0f, 1.7f, 45.0f, 30.0f, 0.0f, 1.0f), 0.0001f);
}

TEST(BalanceControlTest, ControllerClampsByUpperPairThrustDirection)
{
    EXPECT_FLOAT_EQ(0.25f, balanceControllerOutput(81.0f, 60.0f, 0.0f,
        0.02f, 0.003f, 500.0f, 0.25f, 0.35f, 0.20f, 0.20f,
        1.0f, 1.7f, 10.0f, 30.0f, 0.0f, 1.0f));
    EXPECT_FLOAT_EQ(-0.35f, balanceControllerOutput(81.0f, 100.0f, 0.0f,
        0.02f, 0.003f, 500.0f, 0.25f, 0.35f, 0.20f, 0.20f,
        1.0f, 1.7f, 10.0f, 30.0f, 0.0f, 1.0f));

    // The same physical thrust limits still apply when balancing on the
    // opposite edge, where controller-effort signs are reversed.
    EXPECT_FLOAT_EQ(-0.25f, balanceControllerOutput(-81.0f, -60.0f, 0.0f,
        0.02f, 0.003f, 500.0f, 0.25f, 0.35f, 0.20f, 0.20f,
        1.0f, 1.7f, 10.0f, 30.0f, 0.0f, 1.0f));
    EXPECT_FLOAT_EQ(0.35f, balanceControllerOutput(-81.0f, -100.0f, 0.0f,
        0.02f, 0.003f, 500.0f, 0.25f, 0.35f, 0.20f, 0.20f,
        1.0f, 1.7f, 10.0f, 30.0f, 0.0f, 1.0f));
}

TEST(BalanceControlTest, ControllerScalesEffortByUpperPairThrustDirection)
{
    EXPECT_NEAR(-0.034f, balanceControllerOutput(81.0f, 82.0f, 0.0f,
        0.02f, 0.003f, 80.0f, 0.25f, 0.35f, 0.20f, 0.20f,
        1.0f, 1.7f, 45.0f, 30.0f, 0.0f, 1.0f), 0.0001f);
    EXPECT_NEAR(0.034f, balanceControllerOutput(-81.0f, -82.0f, 0.0f,
        0.02f, 0.003f, 80.0f, 0.25f, 0.35f, 0.20f, 0.20f,
        1.0f, 1.7f, 45.0f, 30.0f, 0.0f, 1.0f), 0.0001f);
}

TEST(BalanceControlTest, DynamicMotionPreservesDirectionalOutputLimits)
{
    EXPECT_FLOAT_EQ(-0.60f, balanceControllerOutput(90.0f, 70.0f, 150.0f,
        0.012f, 0.003f, 80.0f, 0.35f, 1.0f, 0.25f, 0.60f,
        2.0f, 4.0f, 45.0f, 30.0f, 0.0f, 1.0f));
    EXPECT_FLOAT_EQ(0.25f, balanceControllerOutput(90.0f, 70.0f, -120.0f,
        0.012f, 0.003f, 80.0f, 0.35f, 1.0f, 0.25f, 0.60f,
        2.0f, 4.0f, 45.0f, 30.0f, 0.0f, 1.0f));

    // Physical thrust direction, rather than controller sign, selects the
    // limit on the opposite edge too.
    EXPECT_FLOAT_EQ(0.60f, balanceControllerOutput(-90.0f, -70.0f, -150.0f,
        0.012f, 0.003f, 80.0f, 0.35f, 1.0f, 0.25f, 0.60f,
        2.0f, 4.0f, 45.0f, 30.0f, 0.0f, 1.0f));
    EXPECT_FLOAT_EQ(-0.25f, balanceControllerOutput(-90.0f, -70.0f, 120.0f,
        0.012f, 0.003f, 80.0f, 0.35f, 1.0f, 0.25f, 0.60f,
        2.0f, 4.0f, 45.0f, 30.0f, 0.0f, 1.0f));

    // Far from the target and nearly stationary, full directional recovery
    // authority remains available.
    EXPECT_FLOAT_EQ(0.35f, balanceControllerOutput(90.0f, 20.0f, 0.0f,
        0.012f, 0.004f, 80.0f, 0.35f, 1.0f, 0.25f, 0.60f,
        2.0f, 4.0f, 45.0f, 30.0f, 0.0f, 1.0f));
    EXPECT_FLOAT_EQ(-1.0f, balanceControllerOutput(90.0f, 150.0f, 0.0f,
        0.012f, 0.004f, 80.0f, 0.35f, 1.0f, 0.25f, 0.60f,
        2.0f, 4.0f, 45.0f, 30.0f, 0.0f, 1.0f));
}

TEST(BalanceControlTest, TemporaryLimitCapsBackAndCooldownOutput)
{
    EXPECT_FLOAT_EQ(-0.25f, balanceControllerOutput(90.0f, 170.0f, 0.0f,
        0.012f, 0.003f, 80.0f, 0.35f, 1.0f, 0.25f, 0.60f,
        2.0f, 4.0f, 45.0f, 30.0f, 0.0f, 0.25f));
    EXPECT_FLOAT_EQ(0.25f, balanceControllerOutput(-90.0f, -170.0f, 0.0f,
        0.012f, 0.003f, 80.0f, 0.35f, 1.0f, 0.25f, 0.60f,
        2.0f, 4.0f, 45.0f, 30.0f, 0.0f, 0.25f));
}

TEST(BalanceControlTest, FloorUsesLaunchPowerAndIntermediateUsesDynamicLimit)
{
    EXPECT_FLOAT_EQ(0.35f, balanceControllerOutput(90.0f, 0.0f, 0.0f,
        0.012f, 0.003f, 80.0f, 0.35f, 1.0f, 0.25f, 0.60f,
        2.0f, 4.0f, 45.0f, 30.0f, 0.0f, 1.0f));
    EXPECT_FLOAT_EQ(0.25f, balanceControllerOutput(90.0f, 45.0f, 0.0f,
        0.012f, 0.003f, 80.0f, 0.35f, 1.0f, 0.25f, 0.60f,
        2.0f, 4.0f, 45.0f, 30.0f, 0.0f, 1.0f));
    EXPECT_FLOAT_EQ(0.0f, balanceControllerOutput(90.0f, 45.0f, 80.0f,
        0.012f, 0.003f, 80.0f, 0.35f, 1.0f, 0.25f, 0.60f,
        2.0f, 4.0f, 45.0f, 30.0f, 0.0f, 1.0f));
}

TEST(BalanceControlTest, IntegralLearnsOnlyNearTargetAndCannotWindUp)
{
    float integral = 0.0f;
    for (int i = 0; i < 1000; i++) {
        integral = balanceIntegralUpdate(integral, -5.0f, 0.0f,
            0.005f, 0.30f, 15.0f, 30.0f, 0.001f, true, false);
    }
    EXPECT_NEAR(-0.025f, integral, 0.0001f);

    EXPECT_FLOAT_EQ(integral, balanceIntegralUpdate(integral, -20.0f, 0.0f,
        0.005f, 0.30f, 15.0f, 30.0f, 1.0f, true, false));
    EXPECT_FLOAT_EQ(integral, balanceIntegralUpdate(integral, -5.0f, 40.0f,
        0.005f, 0.30f, 15.0f, 30.0f, 1.0f, true, false));

    integral = 0.0f;
    for (int i = 0; i < 1000; i++) {
        integral = balanceIntegralUpdate(integral, 15.0f, 0.0f,
            0.050f, 0.30f, 15.0f, 30.0f, 0.01f, true, false);
    }
    EXPECT_FLOAT_EQ(0.30f, integral);
    EXPECT_FLOAT_EQ(0.0f, balanceIntegralUpdate(integral, 0.0f, 0.0f,
        0.005f, 0.30f, 15.0f, 30.0f, 0.001f, false, false));
}

TEST(BalanceControlTest, IntegralResetsWhenFrameReturnsFlat)
{
    EXPECT_FLOAT_EQ(0.0f, balanceIntegralUpdate(-0.161f, 90.0f, 0.0f,
        0.005f, 0.30f, 15.0f, 30.0f, 0.001f, true, true));
}

TEST(BalanceControlTest, InitialLaunchMustReachNearTargetBeforeRecoveryTracking)
{
    balanceRecoveryTracker_t tracker = {};
    bool launchComplete = false;

    for (int i = 0; i < 20; i++) {
        launchComplete = balanceLaunchCompleteUpdate(launchComplete, 90.0f, 15.0f);
        EXPECT_FALSE(balanceRecoveryTrackerUpdate(&tracker,
            90.0f, launchComplete, 45.0f, 5.0f, 1.0f, 0.5f, 0.1f));
    }
    EXPECT_FALSE(launchComplete);
    EXPECT_FALSE(tracker.tracking);

    launchComplete = balanceLaunchCompleteUpdate(launchComplete, 15.0f, 15.0f);
    EXPECT_TRUE(launchComplete);
    EXPECT_FALSE(balanceRecoveryTrackerUpdate(&tracker,
        15.0f, launchComplete, 45.0f, 5.0f, 1.0f, 0.5f, 0.1f));

    // Once reached, the latch stays set and a later fall starts recovery
    // tracking normally.
    launchComplete = balanceLaunchCompleteUpdate(launchComplete, 90.0f, 15.0f);
    EXPECT_TRUE(launchComplete);
    EXPECT_FALSE(balanceRecoveryTrackerUpdate(&tracker,
        90.0f, launchComplete, 45.0f, 5.0f, 1.0f, 0.5f, 0.1f));
    EXPECT_TRUE(tracker.tracking);
}

TEST(BalanceControlTest, RecoveryCooldownLimitsStalledAttemptsAndRetries)
{
    balanceRecoveryTracker_t tracker = {};

    EXPECT_FALSE(balanceRecoveryTrackerUpdate(&tracker,
        70.0f, true, 45.0f, 5.0f, 1.0f, 0.5f, 0.1f));
    for (int i = 0; i < 9; i++) {
        EXPECT_FALSE(balanceRecoveryTrackerUpdate(&tracker,
            69.0f, true, 45.0f, 5.0f, 1.0f, 0.5f, 0.1f));
    }
    EXPECT_TRUE(balanceRecoveryTrackerUpdate(&tracker,
        69.0f, true, 45.0f, 5.0f, 1.0f, 0.5f, 0.1f));
    EXPECT_TRUE(balanceRecoveryTrackerUpdate(&tracker,
        69.0f, true, 45.0f, 5.0f, 1.0f, 0.5f, 0.5f));
    EXPECT_FALSE(balanceRecoveryTrackerUpdate(&tracker,
        69.0f, true, 45.0f, 5.0f, 1.0f, 0.5f, 0.1f));

    // Returning near the target clears a pending attempt immediately.
    EXPECT_FALSE(balanceRecoveryTrackerUpdate(&tracker,
        20.0f, true, 45.0f, 5.0f, 1.0f, 0.5f, 0.1f));
    EXPECT_FALSE(tracker.tracking);
}

TEST(BalanceControlTest, RecoveryProgressRestartsTheTimeout)
{
    balanceRecoveryTracker_t tracker = {};
    EXPECT_FALSE(balanceRecoveryTrackerUpdate(&tracker,
        90.0f, true, 45.0f, 5.0f, 1.0f, 0.5f, 0.1f));
    for (int i = 0; i < 8; i++) {
        EXPECT_FALSE(balanceRecoveryTrackerUpdate(&tracker,
            89.0f, true, 45.0f, 5.0f, 1.0f, 0.5f, 0.1f));
    }
    EXPECT_FALSE(balanceRecoveryTrackerUpdate(&tracker,
        84.0f, true, 45.0f, 5.0f, 1.0f, 0.5f, 0.1f));
    EXPECT_FLOAT_EQ(0.0f, tracker.stalledTime);
}

TEST(BalanceControlTest, SoftStartRampsControllerAuthority)
{
    EXPECT_FLOAT_EQ(0.0f, balanceSoftStartScale(0.0f, 0.3f));
    EXPECT_FLOAT_EQ(0.5f, balanceSoftStartScale(0.15f, 0.3f));
    EXPECT_FLOAT_EQ(1.0f, balanceSoftStartScale(0.3f, 0.3f));
    EXPECT_FLOAT_EQ(1.0f, balanceSoftStartScale(1.0f, 0.0f));
}

TEST(BalanceControlTest, EntryFailureWaitsAndRetriesWithoutAuxCycle)
{
    balanceState_e state = BALANCE_STATE_IDLE;

    state = balanceControllerNextState(state, true, true, false);
    EXPECT_EQ(BALANCE_STATE_IDLE, state);

    state = balanceControllerNextState(state, true, true, true);
    EXPECT_EQ(BALANCE_STATE_ACTIVE, state);
}

TEST(BalanceControlTest, PreArmRequestWaitsInsteadOfLockingOut)
{
    balanceState_e state = BALANCE_STATE_IDLE;

    state = balanceControllerNextState(state, true, false, true);
    EXPECT_EQ(BALANCE_STATE_IDLE, state);

    state = balanceControllerNextState(state, true, true, true);
    EXPECT_EQ(BALANCE_STATE_ACTIVE, state);
}

TEST(BalanceControlTest, ActiveModeDoesNotHaveAnAngleAbort)
{
    const balanceState_e state = balanceControllerNextState(
        BALANCE_STATE_ACTIVE, true, true, true);
    EXPECT_EQ(BALANCE_STATE_ACTIVE, state);
}

TEST(BalanceControlTest, HardwareFailureStillLocksOutSafely)
{
    balanceState_e state = balanceControllerNextState(
        BALANCE_STATE_ACTIVE, true, false, true);
    EXPECT_EQ(BALANCE_STATE_LOCKED_OUT, state);

    state = balanceControllerNextState(state, true, true, false);
    EXPECT_EQ(BALANCE_STATE_LOCKED_OUT, state);

    state = balanceControllerNextState(state, true, true, true);
    EXPECT_EQ(BALANCE_STATE_ACTIVE, state);
}

TEST(BalanceControlTest, MotorDrivePrioritisesUpperPair)
{
    EXPECT_FLOAT_EQ(0.2f, balanceMotorDrive(0.2f, 1.0f, 81.0f, 0.25f));
    EXPECT_FLOAT_EQ(-0.05f, balanceMotorDrive(0.2f, -1.0f, 81.0f, 0.25f));

    EXPECT_FLOAT_EQ(0.05f, balanceMotorDrive(0.2f, 1.0f, -81.0f, 0.25f));
    EXPECT_FLOAT_EQ(-0.2f, balanceMotorDrive(0.2f, -1.0f, -81.0f, 0.25f));
}

TEST(BalanceControlTest, MotorOutputMapsDshotAndAnalogReverseRanges)
{
    EXPECT_FLOAT_EQ(1550.0f, balanceMotorOutput(
        0.5f, 0.0f, 50.0f, 2000.0f, 1047.0f, 1100.0f, true));
    EXPECT_FLOAT_EQ(548.5f, balanceMotorOutput(
        -0.5f, 0.0f, 50.0f, 2000.0f, 1047.0f, 1100.0f, true));

    EXPECT_FLOAT_EQ(1770.0f, balanceMotorOutput(
        0.5f, 1500.0f, 1000.0f, 2000.0f, 1460.0f, 1540.0f, false));
    EXPECT_FLOAT_EQ(1230.0f, balanceMotorOutput(
        -0.5f, 1500.0f, 1000.0f, 2000.0f, 1460.0f, 1540.0f, false));
    EXPECT_FLOAT_EQ(1500.0f, balanceMotorOutput(
        0.005f, 1500.0f, 1000.0f, 2000.0f, 1460.0f, 1540.0f, false));
}
