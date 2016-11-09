% @brief 2D SLAM example
% @date Nov 7 2016
% @author Jing Dong

% A simple 2D pose-graph SLAM
% The robot moves from x1 to x5, with odometry information between each pair. 
% the robot moves 5 each step, and makes 90 deg right turns at x3 - x5
% At x5, there is a *loop closure* between x2 is avaible
% The graph strcuture is shown:
%  
%   p-x1 - x2 - x3
%          |    |
%          x5 - x4 

clear
close all

import gtsam.*


% Create a factor graph container
graph = NonlinearFactorGraph;

% Add a prior on the first pose, setting it to the origin
% The prior is needed to fix/align the whole trajectory at world frame
% A prior factor consists of a mean value and a noise model (covariance matrix)
priorModel = noiseModel.Diagonal.Sigmas([1.0, 1.0, 0.1]');
graph.add(PriorFactorPose2(symbol('x', 1), Pose2(0, 0, 0), priorModel));

% odometry measurement noise model (covariance matrix)
odomModel = noiseModel.Diagonal.Sigmas([0.5, 0.5, 0.1]');

% Add odometry factors
% Create odometry (Between) factors between consecutive poses
% robot makes 90 deg right turns at x3 - x5
graph.add(BetweenFactorPose2(symbol('x', 1), symbol('x', 2), Pose2(5, 0, 0), odomModel));
graph.add(BetweenFactorPose2(symbol('x', 2), symbol('x', 3), Pose2(5, 0, -pi/2), odomModel));
graph.add(BetweenFactorPose2(symbol('x', 3), symbol('x', 4), Pose2(5, 0, -pi/2), odomModel));
graph.add(BetweenFactorPose2(symbol('x', 4), symbol('x', 5), Pose2(5, 0, -pi/2), odomModel));

% loop closure measurement noise model
loopModel = noiseModel.Diagonal.Sigmas([0.5, 0.5, 0.1]');

% Add the loop closure constraint
graph.add(BetweenFactorPose2(symbol('x', 5), symbol('x', 2), Pose2(5, 0, -pi/2), loopModel));
  
% print factor graph
graph.print('\nFactor Graph:\n'); 


% initial varible values for the optimization
% add random noise from ground truth values
initials = Values;
initials.insert(symbol('x', 1), Pose2(0.2, -0.3, 0.2));
initials.insert(symbol('x', 2), Pose2(5.1, 0.3, -0.1));
initials.insert(symbol('x', 3), Pose2(9.9, -0.1, -pi/2 - 0.2));
initials.insert(symbol('x', 4), Pose2(10.2, -5.0, -pi + 0.1));
initials.insert(symbol('x', 5), Pose2(5.1, -5.1, pi/2 - 0.1));
  
% print initial values
initials.print('\nInitial Values:\n'); 


% Use Gauss-Newton method optimizes the initial values
parameters = GaussNewtonParams;

% print per iteration
parameters.setVerbosity('ERROR');
  
% optimize!
optimizer = GaussNewtonOptimizer(graph, initials, parameters);
results = optimizer.optimizeSafely();
  
% print final values
results.print('Final Result:\n');


% Calculate marginal covariances for all poses
marginals = Marginals(graph, results);

% plot result trajectory
figure(1)
hold on

% black lines are odometry
% red line is loop closure
% black ellipse are estimated covariance on x-y plane
marginals = Marginals(graph, results);
plot2DTrajectory(results, [], marginals);
plot([results.atPose2(symbol('x', 5)).x; results.atPose2(symbol('x', 2)).x], ...
    [results.atPose2(symbol('x', 5)).y; results.atPose2(symbol('x', 2)).y], 'r-');
  
% print marginal covariances
cov_x1 = marginals.marginalCovariance(symbol('x', 1))
cov_x2 = marginals.marginalCovariance(symbol('x', 2))
cov_x3 = marginals.marginalCovariance(symbol('x', 3))
cov_x4 = marginals.marginalCovariance(symbol('x', 4))
cov_x5 = marginals.marginalCovariance(symbol('x', 5))


