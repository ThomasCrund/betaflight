/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software and/or
 * modify this software under the terms of the GNU General Public License
 * as published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "pg/pg.h"

typedef enum {
    BALANCE_STATE_IDLE = 0,
    BALANCE_STATE_ACTIVE,
    BALANCE_STATE_LOCKED_OUT,
} balanceState_e;

typedef enum {
    BALANCE_AXIS_PITCH = 0,
    BALANCE_AXIS_ROLL,
} balanceAxis_e;

typedef struct balanceOrientation_s {
    balanceAxis_e axis;
    float targetAngle;
    bool fromFlat;
} balanceOrientation_t;

typedef struct balanceConfig_s {
    int8_t angle;                       // target magnitude; sign selects the default pitch edge from flat
    uint8_t pGain;                      // output fraction per degree of angle error, x1000
    uint8_t dGain;                      // output fraction per degree/second, x1000
    uint16_t maxRate;                   // maximum commanded selected-axis rate, degrees/second
    uint8_t maxOutputForward;           // maximum upper-pair forward 3D output, percent
    uint8_t maxOutputReverse;           // maximum upper-pair reverse 3D output, percent
    uint8_t maxOutputDynamic;           // maximum output while moving or near the target, percent
    uint16_t outputGainForward;         // forward controller-effort multiplier, percent
    uint16_t outputGainReverse;         // reverse controller-effort multiplier, percent
    uint8_t lowerMotorPercent;          // lower motor effort as a percentage of upper motor effort
    uint8_t entryAngle;                 // flat detection threshold used when choosing the default edge, degrees
    uint16_t entryRate;                 // maximum selected-axis rate allowed when engaging, degrees/second
    uint8_t entryThrottle;              // maximum absolute 3D throttle allowed when engaging, percent
    uint8_t recoveryAngle;              // full static recovery output is allowed beyond this error, degrees
    uint16_t recoveryRate;              // maximum rate considered static for full recovery output, degrees/second
} balanceConfig_t;

PG_DECLARE(balanceConfig_t, balanceConfig);

// Called at motor update rate. The selected edge is latched on the AUX rising
// edge and is not reconsidered until AUX is cycled.
bool balanceModeUpdate(bool motorAllocationSupported);

bool balanceModeIsActive(void);
balanceState_e balanceModeGetState(void);
float balanceModeGetOutput(void);
float balanceModeGetAngle(void);
float balanceModeGetTargetAngle(void);
balanceAxis_e balanceModeGetAxis(void);

// Pure helpers exposed for focused controller tests and future simulation work.
float balanceAxisAngleDegrees(float sinAxisAngle, float cosTilt);
float balanceAngleErrorDegrees(float targetAngle, float currentAngle);
balanceOrientation_t balanceSelectOrientation(float pitchAngle, float rollAngle,
    float defaultPitchAngle, float flatThreshold);
float balanceDesiredRate(float angleError, float pGain, float dGain, float maxRate);
float balanceControllerOutput(float targetAngle, float currentAngle, float pitchRate,
    float pGain, float dGain, float maxRate,
    float maxOutputForward, float maxOutputReverse, float maxOutputDynamic,
    float outputGainForward, float outputGainReverse,
    float recoveryAngle, float recoveryRate);
balanceState_e balanceControllerNextState(balanceState_e state, bool requested,
    bool hardwareReady, bool entryAllowed);
float balanceMotorDrive(float effort, float axisCoefficient, float targetAngle,
    float lowerMotorScale);
float balanceMotorOutput(float signedDrive, float disarmOutput, float motorOutputLow,
    float motorOutputHigh, float deadbandMotor3dLow, float deadbandMotor3dHigh,
    bool reverseRangeIncreases);
