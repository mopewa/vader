classdef rangeImage < handle
    %rangeImage Stores a 1D range image and provides related services.
    
    properties(Constant)
        maxUsefulRange = 1.5;
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
                if cleanFlag 
                    obj.removeBadPoints(obj.maxUsefulRange);
                    obj.numPix = length(obj.rArray);
                end
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
        
        function [bestX, bestY, bestTh] = getBestLine(obj, maxLen)
            bestErr = intmax;
            bestTh = 0;
            bestI = 0;
            bestLen = 0;
            bestX = 0;
            bestY = 0;
            for i = 1:size(obj.tArray,2)
                [err, ~, th, len, x, y] = obj.findLineCandidate(i, maxLen)
                disp([num2str(err), ' ', num2str(bestErr), ' ', num2str(obj.rArray(i))]);
                if (err < bestErr && len > bestLen && obj.rArray(i) > 0 && obj.rArray(i) < 1.5)
                    disp(['***********found better********** ', int2str(i)]);
%                     disp(obj.rArray(i));
                    bestErr = err;
                    bestTh = th;
                    bestI = i;
                    bestLen = len;
                    bestX = x;
                    bestY = y;
                end
            end
%             if bestI == 0
%                 x = 0;
%                 y = 0;
%                 theta = 0;
%             else
%                 x = obj.xArray(bestI);
%                 y = obj.yArray(bestI);
%                 theta = bestTh;
%             end
        end
        
        function [err, num, th, len, x, y] = findLineCandidate(obj,middle,maxLen)
            % Find the longest sequence of pixels centered at pixel
            % â€œmiddleâ€? whose endpoints are separated by a length less
            % than the provided maximum. Return the line fit error, the
            % number of pixels participating, and the angle of
            % the line relative to the sensor.
            
            %  search in both directions, two pixels at a time, to find two
            % points separated by a maximum length less than maxLen
            len = 0;
            leftIndex = middle;
            rightIndex = middle;
            numpixels = 1;
            while (len < maxLen)
                leftIndex = obj.dec(leftIndex);
                rightIndex = obj.inc(rightIndex);
                xLeft = obj.xArray(leftIndex);
                yLeft = obj.yArray(leftIndex);
                xRight = obj.xArray(rightIndex);
                yRight = obj.yArray(rightIndex);
                len = sqrt((xLeft-xRight)^2 + (yLeft-yRight)^2)
                numpixels = numpixels + 2;
                if (numpixels > length(obj.xArray))
                    numpixels = numpixels - 2;
                    leftIndex = obj.inc(leftIndex);
                    rightIndex = obj.dec(rightIndex);
                    break;
                end
                len < maxLen
            end
            
            disp(['for ', num2str(obj.tArray(middle)*180/pi), ' len is ', num2str(len), ' with some pixels ', num2str(numpixels)]);

            if (numpixels > 1)
                numpixels = numpixels-2;
                leftIndex = obj.inc(leftIndex);
                rightIndex = obj.dec(rightIndex);
            end
            % make sure endpoints aren't zero
            while (obj.rArray(leftIndex) == 0 || obj.rArray(rightIndex) == 0)
                leftIndex = obj.inc(leftIndex);
                rightIndex = obj.dec(rightIndex);
                numpixels = numpixels - 2;
                if (leftIndex == rightIndex)
                    num = 0;
                    err = NaN;
                    th = 1;
                    disp('failing here');
                    return;
                end
            end
            
            xLeft = obj.xArray(leftIndex);
            yLeft = obj.yArray(leftIndex);
            xRight = obj.xArray(rightIndex);
            yRight = obj.yArray(rightIndex);
            len = sqrt((xLeft-xRight)^2 + (yLeft-yRight)^2);
            
            disp(['for ', num2str(obj.tArray(middle)*180/pi), ' len is ', num2str(len), ' with some pixels ', num2str(numpixels)]);

            if (len > 2*maxLen || numpixels == 1)
                num = 0;
                err = NaN;
                th = 1;
                disp('returning here');
                return;
            end
            
            num = numpixels;
%             disp([xLeft, xRight, yLeft, yRight]);
            th = atan2(yRight-yLeft, xRight-xLeft);
            thPos = th+pi/2;
            thNeg = th-pi/2;
%             disp([obj.tArray(middle), thPos, thNeg]);
            direc = obj.tArray(middle);
            posErr = abs(thPos-direc);
            if posErr > pi
                posErr = 2*pi-posErr;
            end
            negErr = abs(thNeg-direc);
            if negErr > pi
                negErr = 2*pi-negErr;
            end
            if (posErr < negErr)
                th = thPos;
            else
                th = thNeg;
            end
            
            % figure out line equation using 2 endpoints
            [lineEquation, S, mu] = polyfit([xLeft, xRight], [yLeft, yRight], 1);
            
            % make arrays of actual x and y values
            if (leftIndex > rightIndex)
                xActual = [obj.xArray(leftIndex:obj.numPix), obj.xArray(1:rightIndex)];
                yActual = [obj.yArray(leftIndex:obj.numPix), obj.yArray(1:rightIndex)];
                r = [obj.rArray(leftIndex:obj.numPix), obj.rArray(1:rightIndex)];
            else
                xActual = obj.xArray(leftIndex:rightIndex);
                yActual = obj.yArray(leftIndex:rightIndex);
                r = obj.rArray( leftIndex:rightIndex);
            end
            
            % make arrays of line x and y values
            xLine = linspace(xLeft, xRight, numpixels);
            yLine = polyval(lineEquation, xLine, S, mu);
            
            toIgnore = find(r == 0);
            
            xActual(toIgnore) = [];
            yActual(toIgnore) = [];
            xLine(toIgnore) = [];
            yLine(toIgnore) = [];
            
            xError = (xLine - xActual).^2;
            yError = (yLine - yActual).^2;
            
            err = sum((xError + yError).^(1/2));
            
            err = err/(numpixels-2);
            x = (xLeft+xRight)/2;
            y = (yLeft+yRight)/2;
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
            % â€œaâ€? is "natural" meaning it >=1. Second number is signed.
            % Convert a to 0:3 and add b (which is already 0:3).
            % Convert the result back by adding 1.
            out = mod((a-1)+b,obj.numPix)+1;
        end
    end
end