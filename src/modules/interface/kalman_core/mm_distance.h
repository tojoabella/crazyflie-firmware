/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--'  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2021 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 */

#pragma once

#include "kalman_core.h"

/**
 * @file mm_distance.h
 * @brief TWR distance measurement model interface for the Kalman core.
 *
 * - Pipeline role: converts @ref distanceMeasurement_t packets (range to a known anchor)
 *   into scalar EKF updates. Called from @ref estimator_kalman.c when the localization
 *   service enqueues `MeasurementTypeDistance`.
 * - Key structs: uses @ref distanceMeasurement_t (anchor coordinates, measured distance,
 *   standard deviation).
 * - Key functions: @ref kalmanCoreUpdateWithDistance().
 * - Frames/units: anchor coordinates and Crazyflie position are expressed in the global
 *   frame [m]; distances are meters.
 * - Notes on gating: relies on anchor geometry (derivative of range equation); invalid
 *   Jacobians are avoided by falling back to unit vectors when predicted range is zero.
 */

/**
 * @brief Fuse a single range measurement toward a known anchor into the EKF position.
 *
 * Builds the Jacobian from the current state to the measured distance and feeds it to
 * @ref kalmanCoreScalarUpdate().
 *
 * @param this Kalman core data.
 * @param d Distance measurement (anchor position, measured range and std dev).
 */
void kalmanCoreUpdateWithDistance(kalmanCoreData_t* this, distanceMeasurement_t *d);
