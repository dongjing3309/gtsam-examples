/**
 * @file functions.h
 * @brief functions for expressions
 * @date Nov 8, 2016
 * @author Jing Dong
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose2.h>


// you can custom namespace (if needed for your project)
namespace gtsamexamples {

// function project Pose2 to Point2
gtsam::Point2 projectPose2(const gtsam::Pose2& pose, 
    gtsam::OptionalJacobian<2,3> H = boost::none);


} // namespace gtsamexamples
