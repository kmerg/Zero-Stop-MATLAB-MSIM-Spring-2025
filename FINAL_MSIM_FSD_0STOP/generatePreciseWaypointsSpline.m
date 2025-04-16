function smoothWaypoints = generatePreciseWaypointsSpline(waypoints, numPoints)
% generatePreciseWaypointsSpline - Generates a smooth trajectory using a cubic spline.
%
% Syntax:
%   smoothWaypoints = generatePreciseWaypointsSpline(waypoints, numPoints)
%
% Input:
%   waypoints  - An N×3 matrix of (dense) waypoints.
%   numPoints  - (Optional) The number of points for the smoothed trajectory.
%                Default is 200.
%
% Output:
%   smoothWaypoints - An M×3 matrix representing the smoothed trajectory.
%
% This function fits a cubic spline to the waypoints to obtain a smooth path.

if nargin < 2 || isempty(numPoints)
    numPoints = 30;
end

% Parameterize the input waypoints based on cumulative distance.
dists = [0; cumsum(sqrt(sum(diff(waypoints,1,1).^2,2)))];
t = dists / dists(end);

% New parameter values for the smooth path.
tSmooth = linspace(0, 1, numPoints)';

% Fit a cubic spline for each coordinate.
xSmooth = spline(t, waypoints(:,1), tSmooth);
ySmooth = spline(t, waypoints(:,2), tSmooth);
zSmooth = spline(t, waypoints(:,3), tSmooth);

smoothWaypoints = [xSmooth, ySmooth, zSmooth];
end
