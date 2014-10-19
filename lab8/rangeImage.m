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
        originalRanges = [];
        originalSkip = 1;
        numPix;
    end
    
    methods(Access = public)
        function obj = rangeImage(ranges,skip,cleanFlag)
            % Constructs a rangeImage for the supplied data.
            % Converts the data to rectangular coordinates
            if(nargin == 3)
                obj.originalRanges = ranges;
                obj.originalSkip = skip;
                n=0;
                for i=1:skip:length(ranges)
                    n = n + 1;
                    obj.rArray(n) = ranges(i);
                    obj.tArray(n) = (i-1)*(pi/180);
                    obj.xArray(n) = ranges(i)*cos(obj.tArray(n));
                    obj.yArray(n) = ranges(i)*sin(obj.tArray(n));
                end
                obj.numPix = n;
                if cleanFlag; obj.removeBadPoints(obj.maxUsefulRange); end;
            end
        end
        
        function reconstructPoints(obj)
            for i=1:obj.originalSkip:length(obj.originalRanges)
                obj.rArray(i) = obj.originalRanges(i);
                obj.tArray(i) = (i-1)*(pi/180);
                obj.xArray(i) = obj.originalRanges(i)*cos(obj.tArray(i));
                obj.yArray(i) = obj.originalRanges(i)*sin(obj.tArray(i));
            end
        end
        
        function removeBadPoints(obj, maxRange)
            % takes all points above and below two range thresholds
            % out of the arrays. This is a convenience but the result
            % should not be used by any routine that expects the points
            % to be equally separated in angle. The operation is done
            % inline and removed data is deleted.
            badPoints = find(obj.rArray < obj.minUsefulRange | ...
                obj.rArray > maxRange);
            obj.rArray(badPoints) = [];
            obj.tArray(badPoints) = [];
            obj.xArray(badPoints) = [];
            obj.yArray(badPoints) = [];
        end
        
        function plotRvsTh(obj, maxRange)
            % plot the range image after removing all points exceeding
            % maxRange
            obj.removeBadPoints(maxRange);
            plot(obj.rArray, obj.tArray)
        end
        
        function plotXvsY(obj, maxRange)
            % plot the range image after removing all points exceeding
            % maxRange
            obj.removeBadPoints(maxRange);
            plot(obj.xArray, obj.yArray)
        end
        
        function [err, num, th] = findLineCandidate(obj,middle,maxLen)
            % Find the longest sequence of pixels centered at pixel
            % “middle” whose endpoints are separated by a length less
            % than the provided maximum. Return the line fit error, the
            % number of pixels participating, and the angle of
            % the line relative to the sensor.
           %{
            if (length(obj.rArray) ~= length(obj.originalRanges))
                obj.reconstructPoints();
            end
            %}
            %  search in both directions, two pixels at a time, to find two
            % points separated by a maximum length less than maxLen
            len = 0;
            leftIndex = middle;
            rightIndex = middle;
            numpixels = 1;
            while (len <= maxLen)
                leftIndex = obj.dec(leftIndex);
                rightIndex = obj.inc(rightIndex);
                xLeft = obj.xArray(leftIndex);
                yLeft = obj.yArray(leftIndex);
                xRight = obj.xArray(rightIndex);
                yRight = obj.yArray(rightIndex);
                len = sqrt((xLeft-xRight)^2 + (yLeft-yRight)^2);
                numpixels = numpixels + 2;
                if (numpixels > length(obj.xArray))
                    numpixels = numpixels - 2;
                    leftIndex = obj.inc(leftIndex);
                    rightIndex = obj.dec(rightIndex);
                    xLeft = obj.xArray(leftIndex);
                    yLeft = obj.yArray(leftIndex);
                    xRight = obj.xArray(rightIndex);
                    yRight = obj.yArray(rightIndex);
                    len = sqrt((xLeft-xRight)^2 + (yLeft-yRight)^2);
                    break;
                end
            end
            
            % make sure endpoints aren't zero
            while (len > maxLen || obj.rArray(leftIndex) == 0 || obj.rArray(rightIndex) == 0)
                leftIndex = obj.inc(leftIndex);
                rightIndex = obj.dec(rightIndex);
                xLeft = obj.xArray(leftIndex);
                yLeft = obj.yArray(leftIndex);
                xRight = obj.xArray(rightIndex);
                yRight = obj.yArray(rightIndex);
                len = sqrt((xLeft-xRight)^2 + (yLeft-yRight)^2);
                numpixels = numpixels - 2;
            end
            
            num = numpixels;
            th = atan2(yRight-yLeft, xRight-xLeft);
            
            % figure out line equation using 2 endpoints
            lineEquation = polyfit([xLeft, xRight], [yLeft, yRight], 1);
            
            % make arrays of actual x and y values
            if (leftIndex > rightIndex)
                xActual = [obj.xArray(leftIndex:360), obj.xArray(1:rightIndex)];
                yActual = [obj.yArray(leftIndex:360), obj.yArray(1:rightIndex)];
                r = [obj.rArray(leftIndex:360), obj.rArray(1:rightIndex)];
            else
                xActual = obj.xArray(leftIndex:rightIndex);
                yActual = obj.yArray(leftIndex:rightIndex);
                r = obj.rArray(leftIndex:rightIndex);
            end
            
            % make arrays of line x and y values
            xLine = linspace(xLeft, xRight, numpixels);
            yLine = polyval(lineEquation, xLine);
            
            toIgnore = find(r == 0);
            
            xActual(toIgnore) = [];
            yActual(toIgnore) = [];
            xLine(toIgnore) = [];
            yLine(toIgnore) = [];
            
            xError = (xLine - xActual).^2;
            yError = (yLine - yActual).^2;
            
            err = sum((xError + yError).^(1/2));
            
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
    end
end