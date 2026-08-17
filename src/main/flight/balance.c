/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software and/or
 * modify this software under the terms of the GNU General Public License
 * as published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 */

#include <math.h>

#include "platform.h"

#include "build/debug.h"

#include "common/axis.h"
#include "common/maths.h"

#include "config/feature.h"

#include "fc/core.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "flight/failsafe.h"
#include "flight/imu.h"
#include "flight/pid.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "sensors/gyro.h"
#include "sensors/sensors.h"

#include "balance.h"

#define BALANCE_GAIN_SCALE 0.001f

PG_REGISTER_WITH_RESET_TEMPLATE(balanceConfig_t, balanceConfig, PG_BALANCE_CONFIG, 4);

PG_RESET_TEMPLATE(balanceConfig_t, balanceConfig,
    .angle = 90,
    .pGain = 12,
    .dGain = 3,
    .maxRate = 80,
    .maxOutputForward = 35,
    .maxOutputReverse = 100,
    .maxOutputDynamicForward = 25,
    .maxOutputDynamicReverse = 60,
    .outputGainForward = 200,
    .outputGainReverse = 400,
    .iGain = 5,
    .iLimit = 30,
    .iZone = 15,
    .iRate = 30,
    .lowerMotorPercent = 0,
    .entryAngle = 15,
    .entryRate = 100,
    .entryThrottle = 5,
    .recoveryAngle = 45,
    .recoveryRate = 30,
    .recoveryMinProgress = 5,
    .recoveryTimeoutMs = 1000,
    .recoveryCooldownMs = 1000,
    .recoveryOutputLimit = 25,
    .backAngle = 135,
    .backOutputLimit = 25,
    .startupRampMs = 300
);

static FAST_DATA_ZERO_INIT balanceState_e balanceState;
static FAST_DATA_ZERO_INIT float balanceOutput;
static FAST_DATA_ZERO_INIT float balanceAngle;
static FAST_DATA_ZERO_INIT float balanceTargetAngle;
static FAST_DATA_ZERO_INIT balanceAxis_e balanceAxis;
static FAST_DATA_ZERO_INIT bool balanceOrientationLatched;
static FAST_DATA_ZERO_INIT float balanceIntegralOutput;
static FAST_DATA_ZERO_INIT float balanceActiveTime;
static FAST_DATA_ZERO_INIT balanceRecoveryTracker_t balanceRecoveryTracker;
static FAST_DATA_ZERO_INIT bool balanceLaunchComplete;

static bool balanceHardwareReady(bool motorAllocationSupported)
{
    return motorAllocationSupported
        && ARMING_FLAG(ARMED)
        && featureIsEnabled(FEATURE_3D)
        && sensors(SENSOR_ACC)
        && !failsafeIsActive();
}

bool balanceModeUpdate(bool motorAllocationSupported)
{
    const bool requested = IS_RC_MODE_ACTIVE(BOXBALANCE);
    float pitchAngle = 0.0f;
    float rollAngle = 0.0f;

#ifdef USE_ACC
    const float cosTilt = getCosTiltAngle();
    pitchAngle = balanceAxisAngleDegrees(getSinPitchAngle(), cosTilt);
    rollAngle = balanceAxisAngleDegrees(rMat.m[NWU_U][Y], cosTilt);
#endif

    if (!requested) {
        balanceOrientationLatched = false;
    } else if (!balanceOrientationLatched) {
        const balanceOrientation_t orientation = balanceSelectOrientation(
            pitchAngle,
            rollAngle,
            balanceConfig()->angle,
            balanceConfig()->entryAngle);
        balanceAxis = orientation.axis;
        balanceTargetAngle = orientation.targetAngle;
        balanceOrientationLatched = true;
    }

    balanceAngle = balanceAxis == BALANCE_AXIS_ROLL ? rollAngle : pitchAngle;
    const int selectedRateAxis = balanceAxis == BALANCE_AXIS_ROLL ? FD_ROLL : FD_PITCH;
    const float selectedRate = gyro.gyroADCf[selectedRateAxis];
    const float angleError = balanceAngleErrorDegrees(balanceTargetAngle, balanceAngle);
    const bool targetIsEdgeLike = fabsf(balanceTargetAngle) >= 45.0f;
    const bool hardwareReady = targetIsEdgeLike && balanceHardwareReady(motorAllocationSupported);

    // There is intentionally no attitude entry zone or angle abort. The
    // selected orientation is latched wherever AUX is enabled. Retain only
    // rate and throttle checks to prevent an unsafe handover while moving.
    const float entryRate = MAX(fabsf(gyro.gyroADCf[FD_ROLL]), fabsf(gyro.gyroADCf[FD_PITCH]));
    const bool entryAllowed = entryRate <= balanceConfig()->entryRate
        && calculateThrottlePercentAbs() <= balanceConfig()->entryThrottle;

    const balanceState_e previousState = balanceState;
    balanceState = balanceControllerNextState(balanceState, requested,
        hardwareReady, entryAllowed);

    const float dt = pidGetDT();
    bool recoveryCooldown = false;
    bool backLimited = false;
    if (balanceState == BALANCE_STATE_ACTIVE) {
        if (previousState != BALANCE_STATE_ACTIVE) {
            balanceIntegralOutput = 0.0f;
            balanceActiveTime = 0.0f;
            balanceRecoveryTrackerReset(&balanceRecoveryTracker);
            balanceLaunchComplete = false;
        }

        balanceActiveTime += dt;
        const float absoluteError = fabsf(angleError);
        balanceLaunchComplete = balanceLaunchCompleteUpdate(
            balanceLaunchComplete,
            absoluteError,
            balanceConfig()->iZone);

        // A holding bias learned around vertical can point the wrong way after
        // the frame falls. Clear it on the flat so the next launch gets the
        // complete configured recovery authority.
        const bool frameIsFlat = fabsf(pitchAngle) <= balanceConfig()->entryAngle
            && fabsf(rollAngle) <= balanceConfig()->entryAngle;
        balanceIntegralOutput = balanceIntegralUpdate(
            balanceIntegralOutput,
            angleError,
            selectedRate,
            balanceConfig()->iGain * BALANCE_GAIN_SCALE,
            balanceConfig()->iLimit * 0.01f,
            balanceConfig()->iZone,
            balanceConfig()->iRate,
            dt,
            true,
            frameIsFlat);

        // Initial launch is intentionally uninterrupted. The no-progress
        // watchdog becomes active only after this BALANCE activation has
        // reached the near-target I zone at least once.
        recoveryCooldown = balanceRecoveryTrackerUpdate(
            &balanceRecoveryTracker,
            absoluteError,
            balanceLaunchComplete,
            balanceConfig()->recoveryAngle,
            balanceConfig()->recoveryMinProgress,
            balanceConfig()->recoveryTimeoutMs * 0.001f,
            balanceConfig()->recoveryCooldownMs * 0.001f,
            dt);

        float temporaryOutputLimit = 1.0f;
        if (recoveryCooldown) {
            temporaryOutputLimit = balanceConfig()->recoveryOutputLimit * 0.01f;
        }
        backLimited = fabsf(balanceAngle) >= balanceConfig()->backAngle;
        if (backLimited) {
            temporaryOutputLimit = MIN(temporaryOutputLimit,
                balanceConfig()->backOutputLimit * 0.01f);
        }

        balanceOutput = balanceControllerOutput(
            balanceTargetAngle,
            balanceAngle,
            selectedRate,
            balanceConfig()->pGain * BALANCE_GAIN_SCALE,
            balanceConfig()->dGain * BALANCE_GAIN_SCALE,
            balanceConfig()->maxRate,
            balanceConfig()->maxOutputForward * 0.01f,
            balanceConfig()->maxOutputReverse * 0.01f,
            balanceConfig()->maxOutputDynamicForward * 0.01f,
            balanceConfig()->maxOutputDynamicReverse * 0.01f,
            balanceConfig()->outputGainForward * 0.01f,
            balanceConfig()->outputGainReverse * 0.01f,
            balanceConfig()->recoveryAngle,
            balanceConfig()->recoveryRate,
            balanceIntegralOutput,
            temporaryOutputLimit);
        balanceOutput *= balanceSoftStartScale(
            balanceActiveTime,
            balanceConfig()->startupRampMs * 0.001f);
    } else {
        balanceOutput = 0.0f;
        balanceIntegralOutput = 0.0f;
        balanceActiveTime = 0.0f;
        balanceRecoveryTrackerReset(&balanceRecoveryTracker);
        balanceLaunchComplete = false;
    }

    DEBUG_SET(DEBUG_BALANCE, 0, balanceState);
    DEBUG_SET(DEBUG_BALANCE, 1, lrintf(balanceAngle * 10.0f));
    DEBUG_SET(DEBUG_BALANCE, 2, lrintf(angleError * 10.0f));
    DEBUG_SET(DEBUG_BALANCE, 3, lrintf(selectedRate));
    DEBUG_SET(DEBUG_BALANCE, 4, lrintf(balanceOutput * 1000.0f));
    // Bit 0: requested, bit 1: no-progress cooldown, bit 2: back-angle cap,
    // bit 3: initial launch has not yet reached the near-target zone.
    const bool initialLaunch = balanceState == BALANCE_STATE_ACTIVE && !balanceLaunchComplete;
    DEBUG_SET(DEBUG_BALANCE, 5, requested | (recoveryCooldown << 1) | (backLimited << 2) | (initialLaunch << 3));
    int orientationCode = balanceAxis == BALANCE_AXIS_ROLL ? 2 : 1;
    if (balanceTargetAngle < 0.0f) {
        orientationCode = -orientationCode;
    }
    DEBUG_SET(DEBUG_BALANCE, 6, hardwareReady ? orientationCode : 0);
    // Desired rate can be reconstructed from error/P/D/max-rate; log the new
    // integral holding bias here instead so it can be tuned from Blackbox.
    DEBUG_SET(DEBUG_BALANCE, 7, lrintf(balanceIntegralOutput * 1000.0f));

    return balanceState == BALANCE_STATE_ACTIVE;
}

bool balanceModeIsActive(void)
{
    return balanceState == BALANCE_STATE_ACTIVE;
}

balanceState_e balanceModeGetState(void)
{
    return balanceState;
}

float balanceModeGetOutput(void)
{
    return balanceOutput;
}

float balanceModeGetAngle(void)
{
    return balanceAngle;
}

float balanceModeGetTargetAngle(void)
{
    return balanceTargetAngle;
}

balanceAxis_e balanceModeGetAxis(void)
{
    return balanceAxis;
}
