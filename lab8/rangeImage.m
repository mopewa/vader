classdef rangeImage < handle
    %rangeImage Stores a 1D range image and provides related services.
    
    properties(Constant)
        maxUsefulRange = 2.0;
        minUsefulRange = 0.05;
        maxRangeForTarget = 1.0;
    end
    
    properties(Access = public)
        rArray = [];
        tArray = [];
        xArray = [];
        yArray = [];
        numPix;
    end
    
    methods(Access = public)
        function obj = rangeImage(ranges,skip,cleanFlag)
            % Constructs a rangeImage for the supplied data.
            % Converts the data to rectangular coordinates
            if(nargin == 3)
                n=0;
                for i=1:skip:length(ranges)
                    n = n + 1;
                    obj.rArray(n) = ranges(i);
                    obj.tArray(n) = (i-1)*(pi/180);
                    obj.xArray(n) = ranges(i)*cos(obj.tArray(n));
                    obj.yArray(n) = ranges(i)*sin(obj.tArray(n));
                end
                obj.numPix = n;
                if cleanFlag; obj.removeBadPoints(); end;
            end
        end
        
        function removeBadPoints(obj)
            % takes all points above and below two range thresholds
            % out of the arrays. This is a convenience but the result
            % should not be used by any routine that expects the points
            % to be equally separated in angle. The operation is done
            % inline and removed data is deleted.
            badPoints = find(obj.rArray < obj.minUsefulRange | ...
                obj.rArray > obj.maxRangeForTarget);
            obj.rArray(badPoints) = [];
            obj.tArray(badPoints) = [];
            obj.xArray(badPoints) = [];
            obj.yArray(badPoints) = [];
        end
        
        function plotRvsTh(obj, maxRange)
            % plot the range image after removing all points exceeding
            % maxRange
            obj.maxRangeForTarget = maxRange;
            removeBadPoints();
            plot(obj.rArray, obj.tArray);
        end
        
        function plotXvsY(obj, maxRange)
            % plot the range image after removing all points exceeding
            % maxRange
            obj.maxRangeForTarget = maxRange;
            removeBadPoints();
            plot(obj.xArray, obj.yArray);
        end
        
        function [err num th] = findLineCandidate(obj,middle,maxLen)
            % Find the longest sequence of pixels centered at pixel
            % “middle” whose endpoints are separated by a length less
            % than the provided maximum. Return the line fit error, the
            % number of pixels participating, and the angle of
            % the line relative to the sensor.
            len = 0;
            before = middle;
            after = middle;
            numpixels = 1;
            while (len < maxLen)
                before = dec(before);
                after = inc(after);
                [xbefore, ybefore, ~] = irToXy(obj.tArray(before), obj.rArray(before));
                [xafter, yafter, ~] = irToXy(obj.tArray(after), obj.rArray(after));
                len = sqrt((xbefore-xafter)^2 + (ybefore-yafter)^2);
                numpixels = numpixels + 2;
            end
            
            while (obj.rArray(dec(before)) == 0 || obj.rArray(inc(after)) == 0)
                before = inc(before);
                after = dec(after);
                numpixels = numpixels - 2;
            end
            
            num = numpixels;
            th = atan2(yafter-ybefore, xafter-xbefore);
            
            lineEquation = polyfit([xbefore, xafter], [ybefore, yafter], 1);
            
            xActual = obj.xArray(before, after);
            yActual = obj.yArray(before, after);
            
            xLine = linspace(xbefore, xafter, numpixels);
            yLine = polyval(lineEquation, xLine);
            
            r = obj.rArray(before, after);
            toIgnore = find(r == 0);
            
            xActual(toIgnore) = [];
            yActual(toIgnore) = [];
            xLine(toIgnore) = [];
            yLine(toIgnore) = [];          
            
            xError = (xLine - xActual).^2;
            yError = (yLine - yActual).^2;
            
            err = 0;
            for i=1:length(xLine)
                err = err + sqrt(xError(i) + yError(i));
            end
            err = err/numpixels;
        end
        
        function num = numPixels(obj)
            num = obj.numPix;
        end
        
        % Modulo arithmetic on nonnegative integers. MATLABs choice to
        % have matrix indices start at 1 has implications for
        % calculations relating to the position of a number in the
        % matrix. In particular, if you want an operation defined on
        % the sequence of numbers 1 2 3 4 that wraps around, the
        % traditional modulus operations will not work correctly.
        % The trick is to convert the index to 0 1 2 3 4, do the
        % math, and convert back.
        
        function out = inc(obj,in)
            % increment with wraparound over natural numbers
            out = indexAdd(obj,in,1);
        end
        
        function out = dec(obj,in)
            % decrement with wraparound over natural numbers
            out = indexAdd(obj,in,-1);
        end
        
        function out = indexAdd(obj,a,b)
            % add with wraparound over natural numbers. First number
            % “a” is "natural" meaning it >=1. Second number is signed.
            % Convert a to 0:3 and add b (which is already 0:3).
            % Convert the result back by adding 1.
            out = mod((a-1)+b,obj.numPix)+1;
        end
        
        function [ x, y, theta] = irToXy(i, r)
            % irToXy finds position and bearing of a range pixel endpoint
            % Finds the position and bearing of the endpoint of a range pixel in the plane.
            start_angle = 0;
            angle_spacing = 1;
            
            theta = start_angle + (i-1) * angle_spacing;
            
            x = r * cos(theta * (pi/180));
            
            y = r * sin(theta * (pi/180));
            
            % theta needs to be from -180 to 180
            
            if (theta > 180)
                theta = theta - 360;
            end
        end
    end
end