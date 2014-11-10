classdef lineMapLocalizer < handle
    % mapLocalizer A class to match a range scan against a map in
    % order to find the true location of the range scan relative to
    % the map.
    properties(Constant)
        maxError = 0.1; % 15 cm
        minPts = 5; % min # of points that must match
        body = vaderBot.bodyGraph(); % set of points used to graph the robot
    end
    
    properties(Access = private)
    end
    
    properties(Access = public)
        lines_p1 = [];
        lines_p2 = [];
        gain = 0.0;
        errThresh = 0.0;
        gradThresh = 0.0;
        % Values used by Kelly:
        % gain = 0.01;
        % errThresh = 0.001;
        % gradThresh = 0.0005;
    end
    
    methods(Static)
        function [rad2, po] = closestPointOnLineSegment(pt,p1,p2)
            % Given set of points and a line segment, returns the
            % closest point and square of distance to segment for
            % each point. If the closest point is an endpoint, returns
            % infinity for rad2 because such points are bad for
            % lidar matching localization.
            %
            % [rad2, po] = CLOSESTPOINTONLINESEGMENT(pt,p1,p2)
            %
            % pi - Array of points of size 2 x n.
            % p1 - Column of size 2, endpoint of segment.
            % p2 - Column of size 2, endpoint of segment.
            %
            % rad2 - Squared distance to closest point on segment.
            % po - Closest points on segment. Same size as pt.
            v1 = bsxfun(@minus,pt,p1);
            v2 = p2-p1;
            v3 = bsxfun(@minus,pt,p2);
            v1dotv2 = bsxfun(@times,v1,v2);
            v1dotv2 = sum(v1dotv2,1);
            v2dotv2 = sum(v2.*v2);
            v3dotv2 = bsxfun(@times,v3,v2);
            v3dotv2 = sum(v3dotv2,1);
            nPoints = size(pt,2);
            rad2 = zeros(1,nPoints);
            po = zeros(2,nPoints);
            % Closest is on segment
            flag1 = v1dotv2 > 0.0 & v3dotv2 < 0.0;
            if any(flag1)
                scale = v1dotv2/v2dotv2;
                temp = bsxfun(@plus,v2*scale,[p1(1) ; p1(2)]);
                po(:,flag1) = temp(:,flag1);
                dx = pt(1,flag1)-po(1,flag1);
                dy = pt(2,flag1)-po(2,flag1);
                rad2(flag1) = dx.*dx+dy.*dy;
            end
            % Closest is first endpoint
            flag2 = v1dotv2 <= 0.0;
            if any(flag2)
                temp = bsxfun(@times,ones(2,sum(flag2)),[p1(1);
                    p1(2)]);
                po(:,flag2) = temp;
                rad2(flag2) = inf;
            end
            % Closest is second endpoint
            flag3 = ones(1,nPoints) & ~flag1 & ~flag2;
            if any(flag3)
                temp = bsxfun(@times,ones(2,sum(flag3)),[p2(1);
                    p2(2)]);
                po(:,flag3) = temp;
                rad2(flag3) = inf;
            end
        end
        
        
    end
    
    methods
        function obj = lineMapLocalizer(lines_p1,lines_p2,gain,errThresh,gradThresh)
            % create a lineMapLocalizer
            % lines_p1 is a 2xn vector with the coordinates of one endpoint of
            % each line
            % lines_p2 is a 2xn vector with the coordinates of the other
            % endpoint of each line
            obj.lines_p1 = lines_p1;
            obj.lines_p2 = lines_p2;
            obj.gain = gain;
            obj.errThresh = errThresh;
            obj.gradThresh = gradThresh;
        end
        
        function ro2 = closestSquaredDistanceToLines(obj,pt)
            % Find the squared shortest distance from pt to any line
            % segment in the supplied list of line segments.
            % pt is an array of 2d points
            
            % throw away homogenous flag
            pt = pt(1:2,:);
            r2Array = zeros(size(obj.lines_p1,2),size(pt,2));
            for i = 1:size(obj.lines_p1,2)
                [r2Array(i,:) , ~] = lineMapLocalizer.closestPointOnLineSegment(pt,...
                    obj.lines_p1(:,i),obj.lines_p2(:,i));
            end
            ro2 = min(r2Array,[],1);
        end
        
        function ids = throwOutliers(obj,pose,ptsInModelFrame)
            % Find ids of outliers in a scan.
            worldPts = pose.bToA()*ptsInModelFrame;
            r2 = obj.closestSquaredDistanceToLines(worldPts);
            ids = find(sqrt(r2) > obj.maxError);
        end
        
        function avgErr = fitError(obj,pose,ptsInModelFrame)
            % Find the standard deviation of perpendicular distances of
            % all points to all lines
            % transform the points
            worldPts = pose.bToA()*ptsInModelFrame;
            
            r2 = obj.closestSquaredDistanceToLines(worldPts);
            r2(r2 == Inf) = [];
            err = sum(r2);
            num = length(r2);
            if(num >= lineMapLocalizer.minPts)
                avgErr = sqrt(err)/num;
            else
                % not enough points to make a guess
                avgErr = inf;
            end
        end
       
        function [errPlus0,J] = getJacobian(obj,poseIn,modelPts)
            % Computes the gradient of the error function
            errPlus0 = fitError(obj,poseIn,modelPts);
            
            eps = 0.001;
            dx = [eps ; 0.0 ; 0.0];
            dy = [0.0; eps; 0.0];
            dt = [0.0; 0.0; eps];
            
            newPoseX = pose(poseIn.getPose+dx);
            newPoseY = pose(poseIn.getPose+dy);
            newPoseT = pose(poseIn.getPose+dt);
            
            errX = fitError(obj, newPoseX, modelPts);
            errY = fitError(obj, newPoseY, modelPts);
            errT = fitError(obj, newPoseT, modelPts);
            
            J = [1/eps * (errX-errPlus0), 1/eps * (errY-errPlus0), 1/eps * (errT-errPlus0)];
        end
        
        function [success, outPose] = refinePose(obj,inPose,ptsInModelFrame,maxIters)
            % refine robot pose in world (inPose) based on lidar
            % registration. Terminates if maxIters iterations is
            % exceeded or if insufficient points match the lines.
            % Even if the minimum is not found, outPose will contain
            % any changes that reduced the fit error. Pose changes that
            % increase fit error are not included and termination
            % occurs thereafter.
            clf;
            inPose = pose(vaderBot.senToWorld(inPose));
            plot(inPose.x, inPose.y, 'rx');hold on;
            ids = obj.throwOutliers(inPose, ptsInModelFrame);
            ptsInModelFrame(:,ids)=[];
            [err, grad] = obj.getJacobian(inPose, ptsInModelFrame);
            
            % for plotting a small box at the point's current location
            %             xPts = [-.01,-.01,.01,.01,-.01];
            %             yPts = [.01,-.01,-.01,.01,.01];
            
            success = 1;
            for i = 1:maxIters
                if abs(err) < obj.errThresh || norm(grad) < obj.gradThresh
                    success = 0;
                    break;
                end
                
                change = -obj.gain*grad;
                inPose = pose(inPose.x+change(1), inPose.y+change(2), inPose.th+change(3));
                [err, grad] = obj.getJacobian(inPose, ptsInModelFrame);
                
                if (isnan(inPose.th))
                    disp('In Refine NaN');
                    -obj.gain*grad
                end
            end
%             plot(inPose.x, inPose.y, 'xg');
            outPose = pose(vaderBot.robToWorld(inPose));
%             temp = outPose.bToA()*obj.body;
%             plot([0 0], [0 1.2192], 'r');hold on;
%             plot([0 1.2192], [0 0], 'r');hold on;
%             plot(temp(1, :), temp(2, :));hold on;
%             axis equal;
%             pause(.1);
        end
    end
end