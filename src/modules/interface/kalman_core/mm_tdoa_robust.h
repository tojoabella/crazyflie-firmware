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
#include "outlierFilterTdoa.h"

/**
 * @file mm_tdoa_robust.h
 * @brief Robust TDoA measurement model interface.
 *
 * - Pipeline role: provides the entry point for the IRLS based TDoA fusion when
 *   `kalman.robustTdoa` is non-zero. Shares the same outlier filter context as
 *   the standard model.
 */

/**
 * @brief M-estimation based robust Kalman filter update for UWB TDoA measurements.
 *
 * @param this Kalman core data.
 * @param tdoa Measurement packet to fuse.
 * @param outlierFilterState Shared TDoA outlier filter context.
 */
void kalmanCoreRobustUpdateWithTdoa(kalmanCoreData_t* this, tdoaMeasurement_t *tdoa, OutlierFilterTdoaState_t* outlierFilterState);
