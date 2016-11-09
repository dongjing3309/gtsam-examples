/**
 * @file Pose2SLAMExpressionExample.cpp
 * @brief 2D SLAM example using Expression
 * @date Nov 7, 2016
 * @author Jing Dong
 */

/**
 * A simple 2D pose-graph SLAM
 * The robot moves from x1 to x5, with odometry information between each pair. 
 * the robot moves 5 each step, and makes 90 deg right turns at x3 - x5
 * At x5, there is a *loop closure* between x2 is avaible
 * The graph strcuture is shown:
 * 
 *  p-x1 - x2 - x3
 *         |    |
 *         x5 - x4 
 */


// Extra headers for using expression
#include <gtsam/slam/expressions.h>
#include <gtsam/nonlinear/ExpressionFactorGraph.h>

// Regular headers
#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>


using namespace std;
using namespace gtsam;

int main(int argc, char** argv) {

  // expression factor graph container
  ExpressionFactorGraph graph;
  
  // expressions needed for varibles
  // in GTSAM we use the naming convention that expressions (and types) have an underscore at the end
  Pose2_ x1_(Symbol('x', 1)), x2_(Symbol('x', 2)), x3_(Symbol('x', 3)), 
         x4_(Symbol('x', 4)), x5_(Symbol('x', 5));

  // Add a prior on the first pose, setting it to the origin
  // The prior is needed to fix/align the whole trajectory at world frame
  // A prior factor consists of a mean value and a noise model (covariance matrix)
  noiseModel::Diagonal::shared_ptr priorModel = noiseModel::Diagonal::Sigmas(Vector3(1.0, 1.0, 0.1));
  graph.addExpressionFactor(x1_, Pose2(0, 0, 0), priorModel);

  // odometry measurement noise model (covariance matrix)
  noiseModel::Diagonal::shared_ptr odomModel = noiseModel::Diagonal::Sigmas(Vector3(0.5, 0.5, 0.1));

  // Add odometry factors
  // Create odometry (Between) factors between consecutive poses
  // robot makes 90 deg right turns at x3 - x5
  graph.addExpressionFactor(between(x1_, x2_), Pose2(5, 0, 0), odomModel);
  graph.addExpressionFactor(between(x2_, x3_), Pose2(5, 0, -M_PI_2), odomModel);
  graph.addExpressionFactor(between(x3_, x4_), Pose2(5, 0, -M_PI_2), odomModel);
  graph.addExpressionFactor(between(x4_, x5_), Pose2(5, 0, -M_PI_2), odomModel);

  // loop closure measurement noise model
  noiseModel::Diagonal::shared_ptr loopModel = noiseModel::Diagonal::Sigmas(Vector3(0.5, 0.5, 0.1));

  // Add the loop closure constraint
  graph.addExpressionFactor(between(x5_, x2_), Pose2(5, 0, -M_PI_2), loopModel);
  
  // print factor graph
  graph.print("\nFactor Graph:\n"); 


  // initial varible values for the optimization
  // add random noise from ground truth values
  Values initials;
  initials.insert(Symbol('x', 1), Pose2(0.2, -0.3, 0.2));
  initials.insert(Symbol('x', 2), Pose2(5.1, 0.3, -0.1));
  initials.insert(Symbol('x', 3), Pose2(9.9, -0.1, -M_PI_2 - 0.2));
  initials.insert(Symbol('x', 4), Pose2(10.2, -5.0, -M_PI + 0.1));
  initials.insert(Symbol('x', 5), Pose2(5.1, -5.1, M_PI_2 - 0.1));
  
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
  cout << "x4 covariance:\n" << marginals.marginalCovariance(Symbol('x', 4)) << endl;
  cout << "x5 covariance:\n" << marginals.marginalCovariance(Symbol('x', 5)) << endl;

  return 0;
}
