function [  ] = rangeDataListener(handle, event )
%rangeDataListener Invoked when new laser range data arrives.
% Updates a global variable storing the ranges array and a flag that
% indicates that new range data has arrived.

global ranges;
global newRanges;

ranges = event.data.ranges;
newRanges = 1;

end

