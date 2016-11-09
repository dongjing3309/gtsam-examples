/**
 * @file Pose2GPSExample.cpp
 * @brief 2D example with custom 'GPS' factor
 * @date Nov 8, 2016
 * @author Jing Dong
 */

/**
 * A simple 2D pose-graph SLAM with 'GPS' measurement
 * The robot moves from x1 to x3, with odometry information between each pair. 
 * each step has an associated 'GPS' measurement by GPSPose2Factor
 * The graph strcuture is shown:
 * 
 *  g1   g2   g3
 *  |    |    |
 *  x1 - x2 - x3
 */

// custom GPS factor
#include "../GPSPose2Factor.h"

// GTSAM headers
#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>


using namespace std;
using namespace gtsam;
using namespace gtsamexamples;

int main(int argc, char** argv) {

  // Create a factor graph container
  NonlinearFactorGraph graph;

  // odometry measurement noise model (covariance matrix)
  noiseModel::Diagonal::shared_ptr odomModel = noiseModel::Diagonal::Sigmas(Vector3(0.5, 0.5, 0.1));

  // Add odometry factors
  // Create odometry (Between) factors between consecutive poses
  // robot makes 90 deg right turns at x3 - x5
  graph.add(BetweenFactor<Pose2>(Symbol('x', 1), Symbol('x', 2), Pose2(5, 0, 0), odomModel));
  graph.add(BetweenFactor<Pose2>(Symbol('x', 2), Symbol('x', 3), Pose2(5, 0, 0), odomModel));

  // 2D 'GPS' measurement noise model, 2-dim
  noiseModel::Diagonal::shared_ptr gpsModel = noiseModel::Diagonal::Sigmas(Vector2(1.0, 1.0));

  // Add the GPS factors
  // note that there is NO prior factor needed at first pose, since GPS provides
  // the global positions (and rotations given more than 1 GPS measurements)
  graph.add(GPSPose2Factor(Symbol('x', 1), Point2(0, 0), gpsModel));
  graph.add(GPSPose2Factor(Symbol('x', 2), Point2(5, 0), gpsModel));
  graph.add(GPSPose2Factor(Symbol('x', 3), Point2(10, 0), gpsModel));
  
  // print factor graph
  graph.print("\nFactor Graph:\n"); 


  // initial varible values for the optimization
  // add random noise from ground truth values
  Values initials;
  initials.insert(Symbol('x', 1), Pose2(0.2, -0.3, 0.2));
  initials.insert(Symbol('x', 2), Pose2(5.1, 0.3, -0.1));
  initials.insert(Symbol('x', 3), Pose2(9.9, -0.1, -0.2));
  
  // print initial values
  initials.print("\nInitial Values:\n"); 


  // Use Gauss-Newton method optimizes the initial values
  GaussNewtonParams parameters;
  
  // print per iteration
  parameters.setVerbosity("ERROR");
  
  // optimize!
  GaussNewtonOptimizer optimizer(graph, initials, parameters);
  Values results = optimizer.optimize();
  
  // print final values
  results.print("Final Result:\n");


  // Calculate marginal covariances for all poses
  Marginals marginals(graph, results);
  
  // print marginal covariances
  cout << "x1 covariance:\n" << marginals.marginalCovariance(Symbol('x', 1)) << endl;
  cout << "x2 covariance:\n" << marginals.marginalCovariance(Symbol('x', 2)) << endl;
  cout << "x3 covariance:\n" << marginals.marginalCovariance(Symbol('x', 3)) << endl;

  return 0;
}
