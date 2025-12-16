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
 * @file mm_flow.h
 * @brief Optical flow measurement model interface.
 *
 * - Pipeline role: consumes @ref flowMeasurement_t packets coming from the PMW3901 deck,
 *   converts them to world-frame velocity constraints and runs two scalar updates.
 * - Key structs: @ref flowMeasurement_t (pixel deltas, dt, std dev) and @ref Axis3f gyro
 *   sample for yaw-rate de-rotation.
 */

/**
 * @brief Fuse optical flow pixel deltas into the EKF velocity states.
 *
 * @param this Kalman core data.
 * @param flow Optical flow measurement (pixel deltas per frame).
 * @param gyro Most recent gyro sample (rad/s) for body rotation compensation.
 */
void kalmanCoreUpdateWithFlow(kalmanCoreData_t* this, const flowMeasurement_t *flow, const Axis3f *gyro);
