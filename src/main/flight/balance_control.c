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

#include "common/maths.h"

#include "balance.h"

#define BALANCE_MOTOR_DEADBAND 0.01f

float balanceAxisAngleDegrees(float sinAxisAngle, float cosTilt)
{
    // Unlike the normal Euler extraction, atan2 remains continuous as the
    // craft approaches and crosses vertical in the selected contact plane.
    return RADIANS_TO_DEGREES(atan2_approx(sinAxisAngle, cosTilt));
}

float balanceAngleErrorDegrees(float targetAngle, float currentAngle)
{
    float error = targetAngle - currentAngle;
    if (error > 180.0f) {
        error -= 360.0f;
    } else if (error < -180.0f) {
        error += 360.0f;
    }
    return error;
}

balanceOrientation_t balanceSelectOrientation(float pitchAngle, float rollAngle,
    float defaultPitchAngle, float flatThreshold)
{
    const bool fromFlat = fabsf(pitchAngle) <= flatThreshold
        && fabsf(rollAngle) <= flatThreshold;
    const float targetMagnitude = fabsf(defaultPitchAngle);

    balanceOrientation_t orientation = {
        .axis = BALANCE_AXIS_PITCH,
        .targetAngle = copysignf(targetMagnitude, defaultPitchAngle),
        .fromFlat = fromFlat,
    };

    if (fromFlat) {
        return orientation;
    }

    const bool selectRoll = fabsf(rollAngle) > fabsf(pitchAngle);
    const float selectedAngle = selectRoll ? rollAngle : pitchAngle;
    orientation.axis = selectRoll ? BALANCE_AXIS_ROLL : BALANCE_AXIS_PITCH;
    orientation.targetAngle = copysignf(targetMagnitude, selectedAngle);
    return orientation;
}

float balanceDesiredRate(float angleError, float pGain, float dGain, float maxRate)
{
    if (dGain <= 0.0f || maxRate <= 0.0f) {
        return 0.0f;
    }

    // This preserves the original PD response close to the target, while
    // limiting the approach velocity when the angle error is large. Without
    // this limit the floor-start logs reached roughly 300 degrees/second
    // before the 3D motors had time to reverse and brake.
    return constrainf(angleError * pGain / dGain, -maxRate, maxRate);
}

float balanceControllerOutput(float targetAngle, float currentAngle, float pitchRate,
    float pGain, float dGain, float maxRate,
    float maxOutputForward, float maxOutputReverse, float maxOutputDynamic,
    float outputGainForward, float outputGainReverse,
    float recoveryAngle, float recoveryRate)
{
    const float angleError = balanceAngleErrorDegrees(targetAngle, currentAngle);
    const float desiredRate = balanceDesiredRate(angleError, pGain, dGain, maxRate);
    const float unscaledOutput = (desiredRate - pitchRate) * dGain;

    // The upper motor pair has the same selected-axis coefficient sign as the
    // target angle. Account for that sign so these limits consistently
    // describe the actual 3D thrust direction on all four edges.
    const bool positiveEffortUsesForwardThrust = targetAngle >= 0.0f;
    const bool effortUsesForwardThrust = (unscaledOutput >= 0.0f) == positiveEffortUsesForwardThrust;
    const float outputGain = effortUsesForwardThrust ? outputGainForward : outputGainReverse;
    const float output = unscaledOutput * outputGain;
    float maxPositiveOutput = positiveEffortUsesForwardThrust ? maxOutputForward : maxOutputReverse;
    float maxNegativeOutput = positiveEffortUsesForwardThrust ? maxOutputReverse : maxOutputForward;

    // Full directional power is reserved for starting or recovering while the
    // craft is nearly stationary and far from the target. During motion, or
    // close to vertical, cap both directions to avoid the 25%/100% bang-bang
    // reversals seen in the floor tests.
    const bool dynamicControl = fabsf(angleError) <= recoveryAngle
        || fabsf(pitchRate) > recoveryRate;
    if (dynamicControl) {
        maxPositiveOutput = MIN(maxPositiveOutput, maxOutputDynamic);
        maxNegativeOutput = MIN(maxNegativeOutput, maxOutputDynamic);
    }

    return constrainf(output, -maxNegativeOutput, maxPositiveOutput);
}

balanceState_e balanceControllerNextState(balanceState_e state, bool requested,
    bool hardwareReady, bool entryAllowed)
{
    if (!requested) {
        return BALANCE_STATE_IDLE;
    }

    if (!hardwareReady) {
        return BALANCE_STATE_LOCKED_OUT;
    }

    switch (state) {
    case BALANCE_STATE_IDLE:
        // Wait for a safe rate/throttle without forcing the pilot to cycle AUX.
        // Attitude is deliberately not an entry condition: an orientation is
        // selected and latched at any angle, including approximately 45 degrees.
        return entryAllowed ? BALANCE_STATE_ACTIVE : BALANCE_STATE_IDLE;
    case BALANCE_STATE_ACTIVE:
        // Once active, angle alone never locks the mode out. This permits
        // recovery through flat and from either side of the selected edge.
        return BALANCE_STATE_ACTIVE;
    case BALANCE_STATE_LOCKED_OUT:
    default:
        return BALANCE_STATE_LOCKED_OUT;
    }
}

float balanceMotorDrive(float effort, float axisCoefficient, float targetAngle,
    float lowerMotorScale)
{
    const float targetSign = targetAngle >= 0.0f ? 1.0f : -1.0f;
    const bool upperMotor = axisCoefficient * targetSign > 0.0f;
    const float pairScale = upperMotor ? 1.0f : lowerMotorScale;
    const float axisDirection = axisCoefficient > 0.0f ? 1.0f : -1.0f;
    return effort * axisDirection * pairScale;
}

float balanceMotorOutput(float signedDrive, float disarmOutput, float motorOutputLow,
    float motorOutputHigh, float deadbandMotor3dLow, float deadbandMotor3dHigh,
    bool reverseRangeIncreases)
{
    const float magnitude = constrainf(fabsf(signedDrive), 0.0f, 1.0f);
    if (magnitude < BALANCE_MOTOR_DEADBAND) {
        return disarmOutput;
    }

    if (signedDrive > 0.0f) {
        return deadbandMotor3dHigh
            + magnitude * (motorOutputHigh - deadbandMotor3dHigh);
    }

    if (reverseRangeIncreases) {
        return motorOutputLow
            + magnitude * (deadbandMotor3dLow - motorOutputLow);
    }

    return deadbandMotor3dLow
        + magnitude * (motorOutputLow - deadbandMotor3dLow);
}
