function denseWaypoints = generateWaypoints(coarseWaypoints, numPoints)
% generateWaypoints - Generates a dense set of waypoints from coarse waypoints.
%
% Syntax:
%   denseWaypoints = generateWaypoints(coarseWaypoints, numPoints)
%
% Input:
%   coarseWaypoints - An N×3 matrix of coarse waypoints.
%   numPoints       - (Optional) The desired number of dense waypoints.
%                     Default is 100.
%
% Output:
%   denseWaypoints  - An M×3 matrix of interpolated waypoints.
%
% This function uses cubic interpolation to densify the provided waypoints.

if nargin < 2 || isempty(numPoints)
    numPoints = 10;
end

% Parameterize the coarse waypoints based on cumulative distance.
dists = [0; cumsum(sqrt(sum(diff(coarseWaypoints,1,1).^2,2)))];
t = dists / dists(end);

% Define the new interpolation parameter values.
tInterp = linspace(0, 1, numPoints)';

% Interpolate for each coordinate using cubic spline interpolation.
xInterp = interp1(t, coarseWaypoints(:,1), tInterp, 'spline');
yInterp = interp1(t, coarseWaypoints(:,2), tInterp, 'spline');
zInterp = interp1(t, coarseWaypoints(:,3), tInterp, 'spline');

denseWaypoints = [xInterp, yInterp, zInterp];
end
