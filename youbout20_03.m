function [q, q_ref] = youbout20_03()
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
% PID rotation
%ku_r = 7; % me first found good
ku_r = 5;
Tu_r = 1.8;
kp_r = 0.6*ku_r;
ki_r = 2*kp_r/Tu_r;
kd_r = kp_r*Tu_r/8;
prevErrRot = 0;

% PID drive
%kp = 10; % Maybe too much oscillation
ku_d = 6.7;
Tu_d = 2;
kp_d = 0.6*ku_d;
ki_d = 2*kp_d/Tu_d;
kd_d = kp_d*Tu_d/8;
prevErrDr = 0;

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
fsm = 'path';

%RTr = [-1 0 0 7.5; 0 -1 0 7.5; 0 0 1 0; 0 0 0 1]; % matrix to have the ref in the lower left corner
RTr = [-1 0 7.5; 0 -1 7.5; 0 0 1];
side_map = 15;
cell = 0.125;
map = zeros(side_map/cell, side_map/cell);
path = [];
step_path = ceil(3/cell);
cnt_rotate = 0;
cnt_drive = 0;
cnt_lateral = 0;
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
    %pts2 = [pts_abs; ones(1, size(pts_abs, 2))];
    % RTr = [-1 0 0 7.5; 0 1 0 7.5; 0 0 -1 0; 0 0 0 1];
    pts_final = homtrans(RTr,pts_abs(1:2,:));
    %pts_final = RTr * pts2;
    pts_final = ceil(pts_final/cell);
    for i = 1:1:size(pts_final, 2)
        if pts_final(1, i) > floor(15/cell-2)
            pts_final(1, i) = floor(15/cell-2);
        end
        if pts_final(2, i) > floor(15/cell-2)
            pts_final(2, i) = floor(15/cell-2);
        end
        if pts_final(1,i) < 3
            pts_final(1,i) = 3;
        end
        if pts_final(2,i)<3
            pts_final(2,i)=3;
        end
    end
    StructEl = ones(3,3);
    map2 = zeros(size(map));
    map2(sub2ind(size(map), pts_final(2,:), pts_final(1,:))) = 1; % % http://nl.mathworks.com/company/newsletters/articles/matrix-indexing-in-matlab.html
    map2 = idilate(map2, StructEl);
    index_dilate = ((map2-map) == 1);
    map(index_dilate) = 1;
    
    youbot_abs = ceil( (homtrans(RTr,[youbotPos(1); youbotPos(2)]))/cell);
    [row, col] = find(map(:,:) == 1);
    subplot(111)
    
    plot(col,row,'.r');
    if ~(isempty(path))
        hold on;
        plot(path(:,1),path(:,2),'.g');
        plot(via(cnt,1),via(cnt,2), '.k');
    end
    hold on;
    plot(youbot_abs(1),youbot_abs(2),'.m');
    hold off;
    xlim([1,15/cell]);
    ylim([1,15/cell]);
    drawnow;
    
    % If there is an obstacle on the path
    if (isempty(path) || any(map(sub2ind(size(map), path(:,2), path(:,1)))) ) && ~strcmp(fsm, 'rotate')
        forwBackVel = 0; % Robot must not move while computing because we loose VREP comm during this time
        leftRightVel = 0;
        rotVel = 0;
        h = youbot_drive(vrep, h, forwBackVel, leftRightVel, rotVel);
        fsm = 'path';
    end
    
    if strcmp(fsm, 'path'),
        %start = RTr*([youbotPos.'; 1]);
        forwBackVel = 0; % Robot must not move while computing because we loose VREP comm during this time
        leftRightVel = 0;
        rotVel = 0;
        h = youbot_drive(vrep, h, forwBackVel, leftRightVel, rotVel);
        start = homtrans(RTr, [youbotPos(1);youbotPos(2)]);
        start = ceil(start/cell).';
        
        %goal = [116,84];
        goal = [30,20];
        %goal = [20,195];
        % Use of DT algorithm to move
        fprintf('Computing a new path \n');
        dx = DXform(map);
        dx.plan(goal)
        path = dx.path(start);
        
        % To only take the pivots points
        %path = [start;path;goal];
        %angles = (path(2:end,2)-path(1:end-1,2))./(path(2:end,1)-path(1:end-1,1));
        %ind_angles = [false(1); angles(2:end) ~= angles(1:end-1)];
        %via = path(ind_angles,:);
        %via = [via; goal]
        
        % To take every x meters a point
        path = [start;path;goal];
        step_path = ceil(1/cell);
        angles = (path(2:end,2)-path(1:end-1,2))./(path(2:end,1)-path(1:end-1,1));
        ind_angles = [false(1); angles(2:end) ~= angles(1:end-1)];
        ind_angles = find(ind_angles == 1);
        true_index_via = [];
        for i = 1:length(ind_angles)-1
            true_index_via = [true_index_via, ind_angles(i):step_path:(ind_angles(i+1)-1)];
        end
        true_index_via = [true_index_via, ind_angles(end)];
        true_index_via = [ step_path:step_path:true_index_via(1), true_index_via];
        
        via = path(true_index_via(:),:);
        via = [via; goal];

        q = (via*cell).';
        q_ref = homtrans(inv(RTr),q);
        sqrt( (youbotPos(1)-q_ref(1,1))^2 + (youbotPos(2)-q_ref(2,1))^2)
        if sqrt( (youbotPos(1)-q_ref(1,1))^2 + (youbotPos(2)-q_ref(2,1))^2) < 0.2 && size(q_ref,2) > 1
            q_ref = q_ref(:,2:end);
        end
        fprintf('Computing a new path finished \n');
        cnt = 1;
        fsm = 'rotate';
        fprintf('Rotate \n');
        
    elseif strcmp(fsm, 'rotate'),
        
        RTrAbsToRobot = [1 0 youbotPos(1); 0 1 youbotPos(2); 0 0 1];
        targetRobot = homtrans(inv(RTrAbsToRobot), q_ref(:,cnt));
        theta = -atan2(targetRobot(1),targetRobot(2));
        errRot = angdiff(theta, youbotEuler(3));
        if errRot < 0
            errRot = errRot+pi;
        else
            errRot = errRot-pi;
        end
        rotVel = (kp_r*errRot + ki_r*errRot*timestep + (kd_r*(errRot -prevErrRot)/timestep));
        if (abs(rotVel) < .1/180*pi) && (abs(angdiff(prevOri, youbotEuler(3))) < .01/180*pi),
            rotVel = 0;
            h = youbot_drive(vrep, h, forwBackVel, leftRightVel, rotVel);
            prevErrRot = 0;
            cnt_rotate = cnt_rotate + 1;
            fsm = 'drive';
            fprintf('Drive \n');
        end
        prevOri = youbotEuler(3);
        prevErrRot = errRot;

    elseif strcmp(fsm, 'drive'),
        
        RTrAbsToRobot = [cos(youbotEuler(3)) sin(youbotEuler(3)) youbotPos(1);-sin(youbotEuler(3)) cos(youbotEuler(3)) youbotPos(2);0 0 1];
        nextPDrive = homtrans(inv(RTrAbsToRobot), q_ref(:,cnt));
        errDr = nextPDrive(2);
        forwBackVel = (kp_d*errDr + ki_d*errDr*timestep + (kd_d*(errDr -prevErrDr)/timestep));
        if (abs(forwBackVel) < 0.1)
            forwBackVel = 0;
            h = youbot_drive(vrep, h, forwBackVel, leftRightVel, rotVel);
            if cnt < length(via)
                cnt = cnt +1;
                sqrt( (youbotPos(1)-q_ref(1,cnt))^2 + (youbotPos(2)-q_ref(2,cnt))^2)
                if sqrt( (youbotPos(1)-q_ref(1,cnt))^2 + (youbotPos(2)-q_ref(2,cnt))^2) < 0.01 && cnt ~= length(via)
                    cnt = cnt + 1;
                end
                prevErrDr = 0;
                fsm = 'rotate';
                fprintf('Rotate \n');
            else
                fsm = 'finished';
            end
        end
        prevErrDr = errDr;
        
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