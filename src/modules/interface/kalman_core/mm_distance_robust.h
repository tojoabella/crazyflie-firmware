/** 
 * This robust M-estimation-based Kalman filter was originally implemented in
 * work by the Dynamic Systems Lab (DSL) at the University of Toronto
 * Institute for Aerospace Studies (UTIAS) and the Vector Institute for
 * Artificial Intelligence, Toronto, ON, Canada.
 *
 * It can be cited as:
 * \verbatim
   @ARTICLE{Zhao2021Learningbased,
    author={Zhao, Wenda and Panerati, Jacopo and Schoellig, Angela P.},
    title={Learning-based Bias Correction for Time Difference of Arrival
           Ultra-wideband Localization of Resource-constrained Mobile Robots},
    journal={IEEE Robotics and Automation Letters},
    year={2021},
    publisher={IEEE}}
 * \endverbatim
 *
 */

#pragma once

#include "kalman_core.h"

/**
 * @file mm_distance_robust.h
 * @brief Public interface for the robust TWR measurement model.
 *
 * - Pipeline role: used when the `kalman.robustTwr` parameter is non-zero to fuse
 *   distance measurements using IRLS and G-M weighting (see @ref mm_distance_robust.c).
 * - Key structs: @ref distanceMeasurement_t (anchor position, measured distance).
 * - Notes on frames/units: meters in the world frame, same as the standard distance model.
 */

/**
 * @brief M-estimation based robust Kalman filter update for UWB TWR measurements.
 *
 * @param this Kalman core data.
 * @param d Distance measurement packet to fuse.
 */
void kalmanCoreRobustUpdateWithDistance(kalmanCoreData_t* this, distanceMeasurement_t *d); 
