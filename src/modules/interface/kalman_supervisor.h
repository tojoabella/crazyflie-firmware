/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2019 Bitcraze AB
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
 *
 * kalman_supervisor.h - Supervises the kalman estimator and makes sure it
 * stays within bounds.
 */

#pragma once

/**
 * @file kalman_supervisor.h
 * @brief Public guard-rail interface that keeps the EKF inside reasonable bounds.
 *
 * - Pipeline role: @ref estimator_kalman.c calls
 *   @ref kalmanSupervisorIsStateWithinBounds() every loop and triggers a reset
 *   if it returns false.
 * - Key structs: reuses @ref kalmanCoreData_t to inspect position/velocity.
 * - Frames/units: same as @ref kalmanCoreData_t (world meters for position, body m/s for velocity).
 */

#include <stdbool.h>
#include "kalman_core.h"

/**
 * @brief Determine if the EKF state remains inside configured position/velocity bounds.
 *
 * @param this Kalman core data inspected by the supervisor.
 * @return true when everything is within limits, false otherwise (which triggers a reset).
 */
bool kalmanSupervisorIsStateWithinBounds(const kalmanCoreData_t* this);
