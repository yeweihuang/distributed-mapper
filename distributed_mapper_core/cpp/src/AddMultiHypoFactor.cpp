//
// Created by yewei on 5/12/22.
//

#include "AddMultiHypoFactor.h"

using namespace std;
using namespace dcsam;

using gtsam::symbol_shorthand::X;
using gtsam::symbol_shorthand::D;

/* ************************************************************************* */
void addDCMixtureFactor(gtsam::Key key_s, gtsam::Key key_t,
                          vector<gtsam::Rot3> odom_arr, vector<NoisePointer> meas_noise,
                            int discrete_id, dcsam::DiscreteValues &initial_discrete,
                              gtsam::DiscreteFactorGraph &d_graph, dcsam::DCFactorGraph &dc_graph){
  int m_num = odom_arr.size();
  gtsam::KeyVector keys;
  keys.push_back(key_s);
  keys.push_back(key_t);

  //add a discrete element
  const size_t cardinality = m_num + 1;
  gtsam::DiscreteKey dk(D(discrete_id), cardinality);
  std::vector<double> prior_dk_class;
  for (int i = 0; i<m_num; i++){
      prior_dk_class.push_back((1 - tol)/double(m_num));
  }
  prior_dk_class.push_back(tol);
  DiscretePriorFactor dki(dk, prior_dk_class);

  d_graph.push_back(dki);

  initial_discrete[dk.first] = discrete_id%m_num;
  std::vector<gtsam::BetweenFactor<gtsam::Rot3>> factorComponents;

  for (int i = 0; i<m_num; i++){
    gtsam::Rot3 odom_r = odom_arr[i];
    gtsam::BetweenFactor<gtsam::Rot3> fbw(key_s, key_t, odom_r, meas_noise[i]);
    factorComponents.push_back(fbw);
  }

  //add null hypothesis
  gtsam::BetweenFactor<gtsam::Rot3> fbw_null
          (key_s, key_t, odom_arr[0], pose_noiseNullHypo3);
  factorComponents.push_back(fbw_null);
//
  DCMixtureFactor<gtsam::BetweenFactor<gtsam::Rot3>> dcMixture(keys, {dk}, factorComponents);

  dc_graph.push_back(dcMixture);
}

/* ************************************************************************* */
gtsam::Rot3 vector2Rot3(gtsam::Vector v){
  gtsam::Rot3 r = gtsam::Rot3(v(0), v(1), v(2),
                              v(3), v(4), v(5),
                              v(6), v(7), v(8) );
  return r;
}

/* ************************************************************************* */
void addDCMixtureFactor(gtsam::Key key_s, gtsam::Key key_t,
                        vector<gtsam::Pose3> odom_arr, vector<NoisePointer> meas_noise,
                        int discrete_id, dcsam::DiscreteValues &initial_discrete,
                        gtsam::DiscreteFactorGraph &d_graph, dcsam::DCFactorGraph &dc_graph){
  int m_num = odom_arr.size();
  gtsam::KeyVector keys;
  keys.push_back(key_s);
  keys.push_back(key_t);

  //add a discrete element
  const size_t cardinality = m_num + 1;
  gtsam::DiscreteKey dk(D(discrete_id), cardinality);
  std::vector<double> prior_dk_class;
  for (int i = 0; i<m_num; i++){
    prior_dk_class.push_back((1 - tol)/double(m_num));
  }
  prior_dk_class.push_back(tol);
  DiscretePriorFactor dki(dk, prior_dk_class);

  d_graph.push_back(dki);

  initial_discrete[dk.first] = discrete_id%m_num;
  std::vector<gtsam::BetweenFactor<gtsam::Pose3>> factorComponents;

  for (int i = 0; i<m_num; i++){
    gtsam::Pose3 odom_r = odom_arr[i];
    gtsam::BetweenFactor<gtsam::Pose3> fbw(key_s, key_t, odom_r, meas_noise[i]);
    factorComponents.push_back(fbw);
  }

  //add null hypothesis
  gtsam::BetweenFactor<gtsam::Pose3> fbw_null
      (key_s, key_t, odom_arr[0], pose_noiseNullHypo6);
  factorComponents.push_back(fbw_null);
//
  DCMixtureFactor<gtsam::BetweenChordalFactor<gtsam::Pose3>> dcMixture(keys, {dk}, factorComponents);

  dc_graph.push_back(dcMixture);
}