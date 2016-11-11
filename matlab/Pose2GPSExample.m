% @brief 2D example with custom 'GPS' factor
% @date Nov 8, 2016
% @author Jing Dong

% A simple 2D pose-graph SLAM with 'GPS' measurement
% The robot moves from x1 to x3, with odometry information between each pair. 
% each step has an associated 'GPS' measurement by GPSPose2Factor
% The graph strcuture is shown:
% 
%  g1   g2   g3
%  |    |    |
%  x1 - x2 - x3

clear
close all

import gtsam.*
import gtsamexamples.*


% Create a factor graph container
graph = NonlinearFactorGraph;

% odometry measurement noise model (covariance matrix)
odomModel = noiseModel.Diagonal.Sigmas([0.5, 0.5, 0.1]');

% Add odometry factors
% Create odometry (Between) factors between consecutive poses
% robot makes 90 deg right turns at x3 - x5
graph.add(BetweenFactorPose2(symbol('x', 1), symbol('x', 2), Pose2(5, 0, 0), odomModel));
graph.add(BetweenFactorPose2(symbol('x', 2), symbol('x', 3), Pose2(5, 0, 0), odomModel));

% 2D 'GPS' measurement noise model, 2-dim
gpsModel = noiseModel.Diagonal.Sigmas([1.0, 1.0]');

% Add the GPS factors
% note that there is NO prior factor needed at first pose, since GPS provides
% the global positions (and rotations given more than 1 GPS measurements)
graph.add(GPSPose2Factor(symbol('x', 1), Point2(0, 0), gpsModel));
graph.add(GPSPose2Factor(symbol('x', 2), Point2(5, 0), gpsModel));
graph.add(GPSPose2Factor(symbol('x', 3), Point2(10, 0), gpsModel));
  
% print factor graph
graph.print('\nFactor Graph:\n'); 


% initial varible values for the optimization
% add random noise from ground truth values
initials = Values;
initials.insert(symbol('x', 1), Pose2(0.2, -0.3, 0.2));
initials.insert(symbol('x', 2), Pose2(5.1, 0.3, -0.1));
initials.insert(symbol('x', 3), Pose2(9.9, -0.1, -0.2));
  
% print initial values
initials.print('\nInitial Values:\n'); 


% Use Gauss-Newton method optimizes the initial values
parameters = GaussNewtonParams;
  
% print per iteration
parameters.setVerbosity('ERROR');
  
% optimize!
optimizer = GaussNewtonOptimizer(graph, initials, parameters);
results = optimizer.optimize();
  
% print final values
results.print('Final Result:\n');


% Calculate marginal covariances for all poses
marginals = Marginals(graph, results);

% plot result trajectory
figure(1)
hold on, axis equal

% black lines are odometry
% black ellipse are estimated covariance on x-y plane
marginals = Marginals(graph, results);
plot2DTrajectory(results, [], marginals);

% print marginal covariances
cov_x1 = marginals.marginalCovariance(symbol('x', 1))
cov_x2 = marginals.marginalCovariance(symbol('x', 2))
cov_x3 = marginals.marginalCovariance(symbol('x', 3))

