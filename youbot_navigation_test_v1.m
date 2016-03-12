function [q, q_ref] = youbot_navigation_test_v1()
close all;
disp('Program started');
%vrep = remApi('remoteApi', 'extApi.h');
vrep=remApi('remoteApi');
vrep.simxFinish(-1);
id = vrep.simxStart('127.0.0.1', 19997, true, true, 2000, 5);
if id < 0,
    disp('Failed connecting to remote API server. Exiting.');
    vrep.delete();
    return;
end
fprintf('Connection %d to remote API server open.\n', id);

% Make sure we close the connexion whenever the script is interrupted.
cleanupObj = onCleanup(@() cleanup_vrep(vrep, id));

% This will only work in "continuous remote API server service"
% See http://www.v-rep.eu/helpFiles/en/remoteApiServerSide.htm
res = vrep.simxStartSimulation(id, vrep.simx_opmode_oneshot_wait);
% We're not checking the error code - if vrep is not run in continuous remote
% mode, simxStartSimulation could return an error.
% vrchk(vrep, res);

% Retrieve all handles, and stream arm and wheel joints, the robot's pose,
% the Hokuyo, and the arm tip pose.
h = youbot_init(vrep, id);
h = youbot_hokuyo_init(vrep, h);

% Let a few cycles pass to make sure there's a value waiting for us next time
% we try to get a joint angle or the robot pose with the simx_opmode_buffer
% option.
pause(.2);

% Constants:

timestep = .05;
wheelradius = 0.0937/2; % This value may be inaccurate. Check before using.

% Min max angles for all joints:
armJointRanges = [-2.9496064186096,2.9496064186096;
    -1.5707963705063,1.308996796608;
    -2.2863812446594,2.2863812446594;
    -1.7802357673645,1.7802357673645;
    -1.5707963705063,1.5707963705063 ];

startingJoints = [0,30.91*pi/180,52.42*pi/180,72.68*pi/180,0];

% In this demo, we move the arm to a preset pose:
pickupJoints = [90*pi/180, 19.6*pi/180, 113*pi/180, -41*pi/180, 0*pi/180];

% Tilt of the Rectangle22 box
r22tilt = -44.56/180*pi;


% Parameters for controlling the youBot's wheels:
forwBackVel = 0; % Speed according to x axis
leftRightVel = 0; % Speed according to y axis
rotVel = 0; % Speed around z axis
prevOri = 0; prevLoc = 0;

disp('Starting robot');

% Set the arm to its starting configuration:
res = vrep.simxPauseCommunication(id, true); vrchk(vrep, res);
for i = 1:5,
    res = vrep.simxSetJointTargetPosition(id, h.armJoints(i),...
        startingJoints(i),...
        vrep.simx_opmode_oneshot);
    vrchk(vrep, res, true);
end
res = vrep.simxPauseCommunication(id, false); vrchk(vrep, res);

% Make sure everything is settled before we start
pause(2);

[res, homeGripperPosition] = ...
    vrep.simxGetObjectPosition(id, h.ptip,...
    h.armRef,...
    vrep.simx_opmode_buffer);
vrchk(vrep, res, true);
fsm = 'rotate';

RTr = [-1 0 0 7.5; 0 -1 0 7.5; 0 0 1 0; 0 0 0 1]; % matrix to have the ref in the lower left corner
side_map = 15;
cell = 0.125;
map = zeros(side_map/cell, side_map/cell);
path = [];
while true,
    tic
    if vrep.simxGetConnectionId(id) == -1,
        error('Lost connection to remote API.');
    end
    
    [res, youbotPos] = vrep.simxGetObjectPosition(id, h.ref, -1,...
        vrep.simx_opmode_buffer); %-1 is to retrieve the absolute position of the object
    vrchk(vrep, res, true);
    [res, youbotEuler] = vrep.simxGetObjectOrientation(id, h.ref, -1,...
        vrep.simx_opmode_buffer);
    vrchk(vrep, res, true);
    
    % Read data from the Hokuyo sensor:
    [pts, contacts] = youbot_hokuyo(vrep, h, vrep.simx_opmode_buffer); 
    %We receive the seen points and the obstacles as coordinates in comparison to the youbot
    youbot2absTrans = transl(youbotPos) * trotx(youbotEuler(1)) * troty(youbotEuler(2)) * trotz(youbotEuler(3));
    pts_abs = homtrans(youbot2absTrans, pts(:, contacts));
    pts2 = [pts_abs; ones(1, size(pts_abs, 2))];
    % RTr = [-1 0 0 7.5; 0 1 0 7.5; 0 0 -1 0; 0 0 0 1];
    pts_final = RTr * pts2;
    pts_final = ceil(pts_final/cell);
    map(sub2ind(size(map), pts_final(2,:), pts_final(1,:))) = 1; % % http://nl.mathworks.com/company/newsletters/articles/matrix-indexing-in-matlab.html
    [row, col] = find(map(:,:) == 1);
    subplot(111)
    hold on;
    plot(col,row,'.r');
    hold off;
    xlim([1,15/cell]);
    ylim([1,15/cell]);
    drawnow;
    
    angl = -pi/2;
    
    if strcmp(fsm, 'rotate'),
        rotVel = angdiff(angl, youbotEuler(3));
        if (abs(angdiff(angl, youbotEuler(3))) < .1/180*pi) && (abs(angdiff(prevOri, youbotEuler(3))) < .01/180*pi),
            rotVel = 0;
            fsm = 'drive';
        end
        prevOri = youbotEuler(3);
    elseif strcmp(fsm, 'drive'),     
        if isempty(path) || any(map(sub2ind(size(map), path(:,2), path(:,1)))) % If no path or a point of the path in which we want to go crosses a wall
            
            forwBackVel = 0; % Robot must not move while computing because we loose VREP comm during this time
            %youbotPos
            start = RTr*([youbotPos.'; 1]);
            start = ceil(start(1:2)/cell).';
            goal = [116,84];
            % Use of DT algorithm to move
            fprintf('Computing a new path \n');
            dx = DXform(map);
            dx.plan(goal) 
            path = dx.path(start);
            path = [start; path];
            via = [path((path(1:end-1,1) ~= path(2:end,1)) & (path(1:end-1,2) ~= path(2:end,2)),:);goal];
            if isempty(path)
                q = mstraj([start;goal], [0.8,0.8], [] ,start, timestep, 0);
            else
                q = mstraj(via, [0.8,0.8], [] ,start(1,:), timestep, 0);
            end
            %q = mstraj(path, [0.8,0.8], [] ,[start(1), start(2)], timestep, 0);
            q = [q, zeros(length(q),1), 4*ones(length(q),1)].';
            q_ref = (RTr\q)*cell;
            fprintf('Computing a new path finished \n');
            subplot(111)
            hold on;
            plot(path(:,1),path(:,2),'.g');
            hold off;
            drawnow;
            step_move = 1;
        end
        
%        forwBackVel = -(youbotPos(1)+3.167);
        
%         if (youbotPos(1)+3.167 < .001) && (abs(youbotPos(1)-prevLoc) < .001),
%             forwBackVel = 0;
%             vrep.simxSetObjectOrientation(id, h.rgbdCasing, h.ref,...
%                 [0 0 pi/4], vrep.simx_opmode_oneshot);
%             for i = 1:5,
%                 res = vrep.simxSetJointTargetPosition(id, h.armJoints(i), pickupJoints(i),...
%                     vrep.simx_opmode_oneshot);
%                 vrchk(vrep, res, true);
%             end
% 
%             fsm = 'finished';
%         end
        if step_move < size(q_ref,2)
            forwBackVel = (q_ref(1,step_move+1)-q_ref(1,step_move))/timestep;
            leftRightVel = -(q_ref(2,step_move+1)-q_ref(2,step_move))/timestep;
            step_move = step_move + 1;
        else
            
            fsm = 'finished';
        end
        
        prevLoc = youbotPos(1);
    elseif strcmp(fsm, 'finished'),
        pause(3);
        break;
    else
        error(sprintf('Unknown state %s.', fsm));
    end
    
    % Update wheel velocities
    h = youbot_drive(vrep, h, forwBackVel, leftRightVel, rotVel);
    
    % Make sure that we do not go faster that the simulator
    elapsed = toc;
    timeleft = timestep-elapsed;
    if (timeleft > 0),
        pause(min(timeleft, .01));
    end
end

end % main function
