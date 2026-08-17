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

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "sensors/gyro.h"
#include "sensors/sensors.h"

#include "balance.h"

#define BALANCE_GAIN_SCALE 0.001f

PG_REGISTER_WITH_RESET_TEMPLATE(balanceConfig_t, balanceConfig, PG_BALANCE_CONFIG, 3);

PG_RESET_TEMPLATE(balanceConfig_t, balanceConfig,
    .angle = 90,
    .pGain = 12,
    .dGain = 3,
    .maxRate = 80,
    .maxOutputForward = 35,
    .maxOutputReverse = 100,
    .maxOutputDynamic = 40,
    .outputGainForward = 200,
    .outputGainReverse = 400,
    .lowerMotorPercent = 0,
    .entryAngle = 15,
    .entryRate = 100,
    .entryThrottle = 5,
    .recoveryAngle = 45,
    .recoveryRate = 30
);

static FAST_DATA_ZERO_INIT balanceState_e balanceState;
static FAST_DATA_ZERO_INIT float balanceOutput;
static FAST_DATA_ZERO_INIT float balanceAngle;
static FAST_DATA_ZERO_INIT float balanceTargetAngle;
static FAST_DATA_ZERO_INIT balanceAxis_e balanceAxis;
static FAST_DATA_ZERO_INIT bool balanceOrientationLatched;

static bool balanceHardwareReady(bool motorAllocationSupported)
{
    return motorAllocationSupported
        && ARMING_FLAG(ARMED)
        && featureIsEnabled(FEATURE_3D)
        && sensors(SENSOR_ACC)
        && !failsafeIsActive();
}

bool FAST_CODE balanceModeUpdate(bool motorAllocationSupported)
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

    balanceState = balanceControllerNextState(balanceState, requested,
        hardwareReady, entryAllowed);

    if (balanceState == BALANCE_STATE_ACTIVE) {
        balanceOutput = balanceControllerOutput(
            balanceTargetAngle,
            balanceAngle,
            selectedRate,
            balanceConfig()->pGain * BALANCE_GAIN_SCALE,
            balanceConfig()->dGain * BALANCE_GAIN_SCALE,
            balanceConfig()->maxRate,
            balanceConfig()->maxOutputForward * 0.01f,
            balanceConfig()->maxOutputReverse * 0.01f,
            balanceConfig()->maxOutputDynamic * 0.01f,
            balanceConfig()->outputGainForward * 0.01f,
            balanceConfig()->outputGainReverse * 0.01f,
            balanceConfig()->recoveryAngle,
            balanceConfig()->recoveryRate);
    } else {
        balanceOutput = 0.0f;
    }

    DEBUG_SET(DEBUG_BALANCE, 0, balanceState);
    DEBUG_SET(DEBUG_BALANCE, 1, lrintf(balanceAngle * 10.0f));
    DEBUG_SET(DEBUG_BALANCE, 2, lrintf(angleError * 10.0f));
    DEBUG_SET(DEBUG_BALANCE, 3, lrintf(selectedRate));
    DEBUG_SET(DEBUG_BALANCE, 4, lrintf(balanceOutput * 1000.0f));
    DEBUG_SET(DEBUG_BALANCE, 5, requested);
    int orientationCode = balanceAxis == BALANCE_AXIS_ROLL ? 2 : 1;
    if (balanceTargetAngle < 0.0f) {
        orientationCode = -orientationCode;
    }
    DEBUG_SET(DEBUG_BALANCE, 6, hardwareReady ? orientationCode : 0);
    const float desiredRate = balanceDesiredRate(
        angleError,
        balanceConfig()->pGain * BALANCE_GAIN_SCALE,
        balanceConfig()->dGain * BALANCE_GAIN_SCALE,
        balanceConfig()->maxRate);
    DEBUG_SET(DEBUG_BALANCE, 7, lrintf(desiredRate));

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
