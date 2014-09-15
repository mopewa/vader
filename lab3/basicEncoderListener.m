function [  ] = basicEncoderListener(handle, event )
%basicEncoderListener Invoked when new encoder data arrives.
% Updates global varibles (must be in workspace) with left and right
% encoder values and the timeStamp.

global leftEncoder;
global rightEncoder;
global timeStamp;

timeStamp = event.data.header.stamp.secs + ...
    (event.data.header.stamp.nsecs/1000000000);
leftEncoder = event.data.left;
rightEncoder = event.data.right;


end

