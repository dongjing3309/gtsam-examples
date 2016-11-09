/**
 * @file functions.cpp
 * @brief functions for expressions
 * @date Nov 8, 2016
 * @author Jing Dong
 */

#include "functions.h"

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose2.h>


namespace gtsamexamples {

/* ************************************************************************** */
gtsam::Point2 projectPose2(const gtsam::Pose2& pose, gtsam::OptionalJacobian<2,3> H) {
  
  // jacobian: left 2x2 identity + right 2x1 zero 
  if (H) *H = (gtsam::Matrix23() << 1.0, 0.0, 0.0, 
                                    0.0, 1.0, 0.0).finished();
  
  // return translation
  return gtsam::Point2(pose.x(), pose.y());
}

} // namespace gtsamexamples
