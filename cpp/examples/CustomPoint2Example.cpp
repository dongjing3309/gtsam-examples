/**
 * @file CustomPoint2Example.cpp
 * @brief 2D example with custom Point2c type
 * @date Nov 8, 2016
 * @author Jing Dong
 */

/**
 * A simple 2D pose-graph open-loop example
 * The robot moves from x1 to x5 with 2 stepsize, with odometry information between each pair. 
 * prior factor on first state, remainings are open-loop
 * The graph strcuture is shown:
 * 
 *  p-x1 - x2 - x3 - x4 - x5
 */

// custom Point2 type
#include "../Point2c.h"

// GTSAM headers
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>


using namespace std;
using namespace gtsam;
using namespace gtsamexamples;


int main(int argc, char** argv) {

  // Create a factor graph container
  NonlinearFactorGraph graph;

  // first state prior noise model (covariance matrix)
  noiseModel::Diagonal::shared_ptr priorModel = noiseModel::Diagonal::Sigmas(Vector2(0.2, 0.2));
  
  // add prior factor on first state (at origin)
  graph.add(PriorFactor<Point2c>(Symbol('x', 1), Point2c(0, 0), priorModel));

  // odometry measurement noise model (covariance matrix)
  noiseModel::Diagonal::shared_ptr odomModel = noiseModel::Diagonal::Sigmas(Vector2(0.5, 0.5));

  // Add odometry factors
  // Create odometry (Between) factors between consecutive point2c
  graph.add(BetweenFactor<Point2c>(Symbol('x', 1), Symbol('x', 2), Point2c(2, 0), odomModel));
  graph.add(BetweenFactor<Point2c>(Symbol('x', 2), Symbol('x', 3), Point2c(2, 0), odomModel));
  graph.add(BetweenFactor<Point2c>(Symbol('x', 3), Symbol('x', 4), Point2c(2, 0), odomModel));
  graph.add(BetweenFactor<Point2c>(Symbol('x', 4), Symbol('x', 5), Point2c(2, 0), odomModel));
  
  // print factor graph
  graph.print("\nFactor Graph:\n"); 


  // initial varible values for the optimization
  // add random noise from ground truth values
  Values initials;
  initials.insert(Symbol('x', 1), Point2c(0.2, -0.3));
  initials.insert(Symbol('x', 2), Point2c(2.1, 0.3));
  initials.insert(Symbol('x', 3), Point2c(3.9, -0.1));
  initials.insert(Symbol('x', 4), Point2c(5.9, -0.3));
  initials.insert(Symbol('x', 5), Point2c(8.2, 0.1));
  
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
