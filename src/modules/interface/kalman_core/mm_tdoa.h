/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--'  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2023 Bitcraze AB
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
#include "outlierFilterTdoa.h"

/**
 * @file mm_tdoa.h
 * @brief Time-Difference-of-Arrival measurement model interface.
 *
 * - Pipeline role: converts @ref tdoaMeasurement_t packets (difference of range
 *   measurements between two anchors) into scalar EKF updates and runs the online
 *   integrator-based outlier filter before accepting the residual.
 * - Key structs: @ref tdoaMeasurement_t contains anchor positions, measured
 *   distance difference and measurement variance; @ref OutlierFilterTdoaState_t
 *   tracks the gating state.
 * - Frames/units: anchor/state positions are in the world frame [m], distance
 *   differences are meters.
 */

/**
 * @brief Fuse a UWB TDoA measurement and optionally reject outliers.
 *
 * @param this Kalman core data.
 * @param tdoa Measurement packet (two anchors, distance difference and std dev).
 * @param nowMs Current timestamp [ms], used for outlier filter timing.
 * @param outlierFilterState Integrator filter context shared across updates.
 */
void kalmanCoreUpdateWithTdoa(kalmanCoreData_t* this, tdoaMeasurement_t *tdoa, const uint32_t nowMs, OutlierFilterTdoaState_t* outlierFilterState);
