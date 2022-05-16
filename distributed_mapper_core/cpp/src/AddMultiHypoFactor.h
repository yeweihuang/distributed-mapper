//
// Created by yewei on 5/12/22.
//

#pragma once

#include <gtsam/geometry/Rot3.h>
#include <gtsam/slam/BetweenFactor.h>

#include "dcsam/DCMixtureFactor.h"
#include "dcsam/DCSAM.h"
#include "dcsam/DiscretePriorFactor.h"

#include "BetweenChordalFactor.h"

typedef gtsam::noiseModel::Diagonal::shared_ptr NoisePointer;
const double tol = 10e-3;
const double sigmaNullHypo = 8.0;

const NoisePointer pose_noiseNullHypo3 =
    gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(3) <<
    sigmaNullHypo, sigmaNullHypo, sigmaNullHypo).finished());

const NoisePointer pose_noiseNullHypo6 =
    gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) <<
    sigmaNullHypo, sigmaNullHypo, sigmaNullHypo, sigmaNullHypo, sigmaNullHypo, sigmaNullHypo).finished());

void addDCMixtureFactor(gtsam::Key key_s, gtsam::Key key_t,
                          vector<gtsam::Rot3> odom_arr, vector<NoisePointer> meas_noise,
                            int discrete_id, dcsam::DiscreteValues &initial_discrete,
                              gtsam::DiscreteFactorGraph &d_graph, dcsam::DCFactorGraph &dc_graph);

void addDCMixtureFactor(gtsam::Key key_s, gtsam::Key key_t,
                        vector<gtsam::Pose3> odom_arr, vector<NoisePointer> meas_noise,
                        int discrete_id, dcsam::DiscreteValues &initial_discrete,
                        gtsam::DiscreteFactorGraph &d_graph, dcsam::DCFactorGraph &dc_graph);

gtsam::Rot3 vector2Rot3(gtsam::Vector v);