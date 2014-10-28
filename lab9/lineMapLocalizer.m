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
        
        function [rad2 , po] = closestPointOnLineSegment(pi,p1,p2)
            % Find point po on a line segment p1-p2 closest to a given
            % point pi and return the closest point and the square of
            % the distance to it. The line segment has endpoints p1
            % and p2 (column vectors) and the point is pi. If the
            % closest point is an endpoint, returns infinity for rad2
            % because such points are bad for lidar matching
            % localization.
            dx13 = pi(1) - p1(1);
            dy13 = pi(2) - p1(2);
            dx12 = p2(1) - p1(1);
            dy12 = p2(2) - p1(2);
            dx23 = pi(1) - p2(1);
            dy23 = pi(2) - p2(2);
            v1 = [dx13 ; dy13];
            v2 = [dx12 ; dy12];
            v3 = [dx23 ; dy23];
            v1dotv2 = dot(v1,v2);
            v2dotv2 = dot(v2,v2);
            v3dotv2 = dot(v3,v2);
            if v1dotv2 > 0.0 && v3dotv2 < 0.0 % Closest is on segment
                scale = v1dotv2/v2dotv2;
                po = v2*scale + [p1(1) ; p1(2)];
                dx = pi(1)-po(1);
                dy = pi(2)-po(2);
                rad2 = dx*dx+dy*dy;
            elseif v1dotv2 <= 0.0 % Closest is first endpoint
                po = [p1(1) ; p1(2)];
                rad2 = inf;
            else % Closest is second endpoint
                po = [p2(1) ; p2(2)];
                rad2 = inf;
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
        
        function ro2 = closestSquaredDistanceToLines(obj,pi)
            % Find the squared shortest distance from pi to any line
            % segment in the supplied list of line segments. p1 is the
            % array of start point and p2 is the array of end points.
            ro2 = inf;
            for i = 1:size(obj.lines_p1,2)
                [r2 , ~] = lineMapLocalizer.closestPointOnLineSegment(pi,...
                    obj.lines_p1(:,i),obj.lines_p2(:,i));
                if(r2 < ro2); ro2 = r2; end;
            end
        end
        
        function ids = throwOutliers(obj,pose,ptsInModelFrame)
            % Find ids of outliers in a scan.
            ids = [];
            worldPts = pose.bToA()*ptsInModelFrame;
            for i = 1:size(worldPts,2)
                r2 = obj.closestSquaredDistanceToLines(worldPts(:,i));
                if(sqrt(r2) > obj.maxError)
                    ids = [ids i];
                end
            end
        end
        
        function avgErr = fitError(obj,pose,ptsInModelFrame,printErr)
            % Find the standard deviation of perpendicular distances of
            % all points to all lines
            % pose is the believed robot pose
            % ptsInModelFrame must be in homogenous form
            % printErr is a printing boolean flag
            
            worldPts = pose.bToA()*ptsInModelFrame;
            
            err = 0.0;
            num = 0;
            for i = 1:size(worldPts,2)
                r2 = obj.closestSquaredDistanceToLines(worldPts(:,i));
                if(r2 == Inf)
                    continue;
                end
                err = err + r2;
                num = num + 1;
                if(printErr)
                    fprintf('i: %d x:%f y:%f val:%f\n',i,...
                        worldPts(1,i),worldPts(2,i),r2);
                end
            end
%             printf('num = %d\n', num);
%             size(worldPts, 2)
            if(num > lineMapLocalizer.minPts)
                avgErr = sqrt(err)/num;
            else
                avgErr = inf;
            end
        end
        
        function [errPlus0,J] = getJacobian(obj,poseIn,modelPts)
            % Computes the gradient of the error function
            errPlus0 = fitError(obj,poseIn,modelPts,false);
            
            eps = 0.001;
            dx = [eps ; 0.0 ; 0.0];
            dy = [0.0; eps; 0.0];
            dt = [0.0; 0.0; eps];
            
            newPoseX = pose(poseIn.getPose+dx);
            newPoseY = pose(poseIn.getPose+dy);
            newPoseT = pose(poseIn.getPose+dt);
            
            errX = fitError(obj, newPoseX, modelPts, false);
            errY = fitError(obj, newPoseY, modelPts, false);
            errT = fitError(obj, newPoseT, modelPts, false);
            
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
                
            end
            plot(inPose.x, inPose.y, 'xg');
            outPose = pose(vaderBot.robToWorld(inPose));
            temp = outPose.bToA()*obj.body;
            plot([0 0], [0 1.2192], 'r');hold on;
            plot([0 1.2192], [0 0], 'r');hold on;
            plot(temp(1, :), temp(2, :));hold on;
            axis equal;
            pause(.1);
        end
    end
end