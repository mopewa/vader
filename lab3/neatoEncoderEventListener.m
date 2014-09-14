function [ ] = neatoEncoderEventListener (handle,event)
%NeatoEncoderEventListener Invoked when new encoder data arrives.
% A MATLAB event listener for the Neato Robot. Invoked when 
% encoder data arrives.

%% Setup -- help persistent for more info
persistent leftEncoder;
persistent timeStamp;
persistent initialTime;
persistent timer;

%% Case 2: Encoder events, event time tags
    % Reminder -- can't run sections in functions! Comment out unwanted 
    % case before running listener!
%{  
if isempty(leftEncoder)
    timeStamp = event.data.header.stamp.secs + ...
    (event.data.header.stamp.nsecs/1000000000);
    leftEncoder = event.data.left;
    initialTime = timeStamp;
    timeStamp = timeStamp - initialTime; %normalize
else
    lastTime = timeStamp;
    lastEncoder = leftEncoder;
    timeStamp = event.data.header.stamp.secs + ...
    (event.data.header.stamp.nsecs/1000000000) - initialTime;
    if (leftEncoder > .1) % noise filter
        leftEncoder = event.data.left;  
        ds = abs(lastEncoder - leftEncoder);
        dt = abs(lastTime - timeStamp);
              
        V = (ds/dt)/1000;   % in m/s
        plot(timeStamp,V, 'x')
    end 
end
%}

%% Case 3: Encoder events, Matlab clock
if isempty(leftEncoder)
    timer = tic;
    timeStamp = toc(timer);
    leftEncoder = event.data.left;
else
    lastTime = timeStamp;
    lastEncoder = leftEncoder;
    timeStamp = toc(timer);
    if (leftEncoder > .1) % noise filter
        leftEncoder = event.data.left;  
        ds = abs(lastEncoder - leftEncoder);
        dt = abs(lastTime - timeStamp);
              
        V = (ds/dt)/1000;   % in m/s
        plot(timeStamp,V, 'x')
    end
end

end