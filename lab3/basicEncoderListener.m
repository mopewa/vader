function [  ] = basicEncoderListener(handle, event )
%basicEncoderListener Invoked when new encoder data arrives.
% Updates global varibles (must be in workspace) with left and right
% encoder values and the timeStamp.

global leftEncoder;
global rightEncoder;
global timeStamp;


if (event.data.left == 0 || event.data.right == 0)
    disp('Encoder Error Ignore Reading');
else 
    timeStamp = event.data.header.stamp.secs + ...
        (event.data.header.stamp.nsecs/1000000000);
    leftEncoder = event.data.left;
    rightEncoder = event.data.right;
end

end

