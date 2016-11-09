// gtsam-examples matlab wrapper declarations

// gtsam deceleration
class gtsam::Point2;
class gtsam::Pose2;

class gtsam::GaussianFactorGraph;
class gtsam::Values;
virtual class gtsam::noiseModel::Base;
virtual class gtsam::NonlinearFactor;
virtual class gtsam::NonlinearFactorGraph;
virtual class gtsam::NoiseModelFactor;


namespace gtsamexamples {

// GPS factor for Pose2
#include <cpp/GPSPose2Factor.h>

virtual class GPSPose2Factor : gtsam::NoiseModelFactor {
  GPSPose2Factor(size_t poseKey, const gtsam::Point2& m, gtsam::noiseModel::Base* model);
};

} // namespace gtsamexamples

