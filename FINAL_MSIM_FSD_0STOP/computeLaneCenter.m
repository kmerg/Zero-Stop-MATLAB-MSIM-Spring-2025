function laneCenter = computeLaneCenter(leftBoundary, rightBoundary)
% computeLaneCenter - Computes the lane center from left and right boundary points.
%
% Syntax:
%   laneCenter = computeLaneCenter(leftBoundary, rightBoundary)
%
% Input:
%   leftBoundary  - An N×3 matrix of left boundary waypoints.
%   rightBoundary - An N×3 matrix of right boundary waypoints.
%
% Output:
%   laneCenter    - An N×3 matrix representing the centerline of the lane.
%
% If left and right boundaries have different number of points, this function
% interpolates them to the same parameterization before computing the average.
%
% If boundaries are empty or not provided, laneCenter will simply be empty.

if nargin < 2 || isempty(leftBoundary) || isempty(rightBoundary)
    error('Both left and right boundaries must be provided.');
end

% Determine parameterization based on the longer boundary.
nLeft = size(leftBoundary,1);
nRight = size(rightBoundary,1);
nPoints = max(nLeft, nRight);
t = linspace(0, 1, nPoints)';

% Interpolate left boundary if needed.
if nLeft < nPoints
    tLeft = linspace(0, 1, nLeft)';
    leftBoundary = interp1(tLeft, leftBoundary, t, 'spline');
end

% Interpolate right boundary if needed.
if nRight < nPoints
    tRight = linspace(0, 1, nRight)';
    rightBoundary = interp1(tRight, rightBoundary, t, 'spline');
end

% Compute the lane center as the average of the boundaries.
laneCenter = (leftBoundary + rightBoundary) / 2;
end
