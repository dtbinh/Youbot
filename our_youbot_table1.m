% The logic of the code is the following :
% The robot navigates (navigate mode). 
% When it has finished its current path, it looks if
% there is a new basket or not. If so it, the mode findBasket is on and the
% robot goes to see it (or them if there is 2). 
% It takes a picture and then goes back in the navigate mode.
% When it has finished to navigate, it stops
%
function our_youbot_table1(struct_tables_input, map, map_plot_input, map_plot_precise_input)
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
%startingJoints = [0,10*pi/180,52.42*pi/180,72.68*pi/180,0];
% In this demo, we move the arm to a preset pose:
pickupJoints = [90*pi/180, 19.6*pi/180, 113*pi/180, -41*pi/180, 0*pi/180];
 
% Tilt of the Rectangle22 box
r22tilt = -44.56/180*pi;
 
 
% Parameters for controlling the youBot's wheels:
forwBackVel = 0; % Speed according to x axis
leftRightVel = 0; % Speed according to y axis
rotVel = 0; % Speed around z axis
prevErrRot = 0;
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
RTr = [-1 0 7.5; 0 -1 7.5; 0 0 1];
StructEl = ones(8,8);
StructSeen = ones(4,4);
side_map = 15;
cell_map = 0.125;
prev_goal = [1,120];
map = zeros(side_map/cell_map, side_map/cell_map);
map_seen = true(side_map/cell_map, side_map/cell_map);
map_plot = false(side_map/cell_map, side_map/cell_map);
cell_map_precise = cell_map/2;
map_plot_precise = false(side_map/cell_map_precise, side_map/cell_map_precise);

path = [];
step_path = ceil(0.5/cell_map);
cnt_path = 0;

navigate = true;
findBasket = false;

%state = 'navigate';
state = 'findObjectsTable1';
%state = 'grasp';

% Create the handle for the basket
h_basket = handle_basket(2, 0);
%len_old_centers = 2;
%cnt_basket = 0;
%nb_new_centers = 0;
%prev_centers = [];

% create the handle for the path
h_path = handle_path();

% create the handle for the table 1
h_objects_table1 = handle_objects_table1();

cnt_picture = 0;
cnt_drive = 0;

% We create a structure : struct_baskets that contains the center and the
% blob of the basket circles
struct_baskets = struct('centers_xy',zeros(5,2),'pixelsBlob',[],'nbRepresentative',zeros(5,1), ...
    'Seen',false(5,1));
struct_baskets.pixelsBlob = cell(1,5);

% We create a structure : struct_tables that contains the center and the
% blob of the tables circles
struct_tables = struct('centers_xy',zeros(2,2),'pixelsBlob',[],'nbRepresentative',zeros(2,1), ...
    'Seen',false(2,1), 'right', false(2,1));
struct_tables.pixelsBlob = cell(1,2);

 % We get back the initial position of the youbot
[res, youbotPos] = vrep.simxGetObjectPosition(id, h.ref, -1,...
    vrep.simx_opmode_buffer); %-1 is to retrieve the absolute position of the object
vrchk(vrep, res, true);
youbot_abs_start = ceil( (homtrans(RTr,[youbotPos(1); youbotPos(2)]))/cell_map);
youbot_start = youbotPos;
while true,
    tic
    if vrep.simxGetConnectionId(id) == -1,
        error('Lost connection to remote API.');
    end
    
    % We get back all the positions
    [res, youbotPos] = vrep.simxGetObjectPosition(id, h.ref, -1,...
        vrep.simx_opmode_buffer); %-1 is to retrieve the absolute position of the object
    vrchk(vrep, res, true);
    [res, youbotEuler] = vrep.simxGetObjectOrientation(id, h.ref, -1,...
        vrep.simx_opmode_buffer);
    vrchk(vrep, res, true);
    
    youbot_abs = ceil( (homtrans(RTr,[youbotPos(1); youbotPos(2)]))/cell_map);
    
    % Read data from the Hokuyo sensor:
    [pts, contacts] = youbot_hokuyo(vrep, h, vrep.simx_opmode_buffer);
    %We receive the seen points and the obstacles as coordinates in comparison to the youbot
    youbot2absTrans = transl(youbotPos) * trotx(youbotEuler(1)) * troty(youbotEuler(2)) * trotz(youbotEuler(3));
    pts_abs = homtrans(youbot2absTrans, pts);
    pts_final = homtrans(RTr,pts_abs(1:2,:));
    pts_final = ceil(pts_final/cell_map);
    pts_final_precise = ceil(homtrans(RTr,pts_abs(1:2,:))/cell_map_precise);
    
    % To make sure that the hukoyo data belongs to the matrix
    min_tmp = min(pts_final,floor(side_map/cell_map));
    pts_final = max(min_tmp, 1);
    
    % To make sure that the hukoyo data belongs to the matrix
    min_tmp_precise = min(pts_final_precise,floor(side_map/cell_map_precise));
    pts_final_precise = max(min_tmp_precise, 1);
      
    % We update the map
    % Map 2 is a trick to dilate only ones the wall
    map2 = zeros(size(map));
    map2(sub2ind(size(map), pts_final(2,contacts), pts_final(1,contacts))) = 1; % % http://nl.mathworks.com/company/newsletters/articles/matrix-indexing-in-matlab.html
    map2 = idilate(map2, StructEl);
    index_dilate = ((map2-map) == 1);
    map(index_dilate) = 1;
    map_plot(sub2ind(size(map), pts_final(2,contacts), pts_final(1,contacts))) = true;
    map_plot_precise(sub2ind(size(map_plot_precise), pts_final_precise(2,contacts), pts_final_precise(1,contacts))) = true;

    % We transfo the hukoyo data in the abs reference
    hokuyo1_abs = homtrans(youbot2absTrans,h.hokuyo1Pos.');
    hokuyo2_abs = homtrans(youbot2absTrans,h.hokuyo2Pos.');
    hokuyo1_map = ceil(homtrans(RTr, [hokuyo1_abs(1);hokuyo1_abs(2)])/cell_map);
    hokuyo2_map = ceil(homtrans(RTr, [hokuyo2_abs(1);hokuyo2_abs(2)])/cell_map);
    
    % We compute the box around the robot
    x_left_robot = max(1, (youbot_abs(1) - (5/cell_map)) );
    x_right_robot = min(length(map), (youbot_abs(1) + (5/cell_map)) );
    y_up_robot = min(length(map), (youbot_abs(2) + (5/cell_map)) );
    y_down_robot = max(1,(youbot_abs(2) - (5/cell_map)) );
    
    % We compute where we see the points in the map 
    [X,Y] = meshgrid(x_left_robot:1:x_right_robot, y_down_robot:1:y_up_robot); 
    X = reshape(X, 1, []);
    Y = reshape(Y, 1, []);

    in = inpolygon(X,Y, [hokuyo1_map(1) pts_final(1,:) hokuyo2_map(1)],...
                    [hokuyo1_map(2) pts_final(2,:) hokuyo2_map(2)]);                
   
    map_seen(sub2ind(size(map_seen), Y(in), X(in))) = 0;
    map_seen(sub2ind(size(map_seen), hokuyo1_map(2), hokuyo1_map(1))) = 0;
    map_seen(sub2ind(size(map_seen), hokuyo2_map(2), hokuyo2_map(1))) = 0;
    map_seen(sub2ind(size(map_seen), youbot_abs(2), youbot_abs(1))) = 0;
    map_seen(index_dilate) = 0;
    
    map_seen = imerode(map_seen,StructSeen);
    map_seen = idilate(map_seen,StructSeen);
    
    % We plot the map
    [row_map_seen, col_map_seen] = find(map_seen == 1);
    [row, col] = find(map(:,:) == 1);
    subplot(121)
    %plot(col,row,'.k');
    if ~(isempty(h_path.path))
        %hold on;
        plot(h_path.path(:,1),h_path.path(:,2),'.r');
        hold on;
        plot(h_path.via(h_path.cnt,1),h_path.via(h_path.cnt,2), '.g');
    end
    plot(col,row,'.k');
    %hold on;
    plot(youbot_abs(1),youbot_abs(2),'db');
    hold off;
    axis square
    xlim([1,side_map/cell_map]);
    ylim([1,side_map/cell_map]);
    
    subplot(122)
    plot(col_map_seen,row_map_seen,'.r');
    xlim([1,side_map/cell_map]);
    ylim([1,side_map/cell_map]);
    
    axis square
    drawnow;
    
    if strcmp(state, 'navigate'),
        if strcmp(fsm, 'path'),
            if cnt_path < 5 % During 3 timesteps we put all speed to 0 to be sure the robot stop
                forwBackVel = 0; % Robot must not move while computing because we loose VREP comm during this time
                leftRightVel = 0;
                rotVel = 0;
                h = youbot_drive(vrep, h, forwBackVel, leftRightVel, rotVel);
                cnt_path = cnt_path+1;
            else %  We compute a new path
                start = youbot_abs;
                path=[];
                if(sum(map_seen(:)) < 10) && strcmp(state, 'navigate') && h_basket.len_old_centers >= 7 % If map totally seen & in navigate mode and not seen the 5 baskets + the 2 tables in the map
                    map_seen(:) = 0;
                    cnt_finished = 0;
                    state = 'finished';
                else
                    fprintf('Computing a new path \n');
                    % Use of our path planner
                    fprintf('Begin to find the next goal and compute a path \n');
                    [goal, y_block, x_block, len_block] = findGoal(map_seen, prev_goal, cell_map);
                    prev_goal=goal;
                    %save('MapCose.mat','map','start','goal');
                    [path,~] = ourPathPlannerBest(map, goal, start);
                    fprintf('Path computed \n');
                    fct_give_path(start, goal, map, youbotPos, path, RTr, step_path, cell_map, h_path)
                    fprintf('Computing a new path finished \n');
                    fprintf('Rotate \n');
                    prevErrRot = 0;
                    fsm = 'rotate';
                    fprintf('Look if there is a new basket by the way \n');
                    [struct_tables, struct_baskets, stateReturn] = fct_see_basket(map_plot,youbot_abs_start, youbot_abs,  h_basket, struct_tables, struct_baskets);
                    if strcmp(stateReturn, 'findBasket'),
                        state = 'findBasket';
                        cnt_path = 0;
                        fsm = 'path';
                        %save('BuildMap.mat','map_plot', 'map');
                    end
                end
            end
        elseif strcmp(fsm, 'rotate'),
            forwBackVel = 0;
            leftRightVel = 0;
            [rotVel,errRot, fsmReturn] = fct_rotate(youbotPos, youbotEuler, cell_map, map, h_path, prevErrRot);
            prevErrRot = errRot;
            if strcmp(fsmReturn, 'path'),
                fsm = 'path';
            elseif strcmp(fsmReturn, 'drive'),
                fsm = 'drive';
                prevErrDr = 0;
            end
        elseif strcmp(fsm, 'drive'),
            [rotVel, errRot, forwBackVel, leftRightVel, errDr, fsmReturn] = fct_drive(youbotPos, youbotEuler, h_path, cell_map, prevErrRot, prevErrDr);
            prevErrRot = errRot;
            prevErrDr = errDr;
            if strcmp(fsmReturn, 'finished_curr_path'),
                cnt_finished_path = 0;
                forwBackVel = 0;
                leftRightVel = 0;
                rotVel = 0;
                fsm = 'finished_curr_path';
            end
            block = map_seen( (1+(len_block*(y_block-1))):(len_block*y_block) , (1+(len_block*(x_block-1))):(len_block*x_block));
            if ( h_path.cnt < length(h_path.true_index_via) && any(map(sub2ind(size(map), h_path.path((h_path.true_index_via(h_path.cnt):h_path.true_index_via(h_path.cnt+1)),2), h_path.path((h_path.true_index_via(h_path.cnt):h_path.true_index_via(h_path.cnt+1)),1)))) ) ...
            ||  ~any(block(:))
        
                cnt_path = 0;
                %map(youbot_abs(2),youbot_abs(1)) = 0; % HEREEEEEE
                fsm = 'path';
                forwBackVel = 0;
                leftRightVel = 0;
                prevErrRot = 0;
            end
        elseif strcmp(fsm, 'finished_curr_path'),
            forwBackVel = 10;
            leftRightVel = 0;
            rotVel = 0;
            cnt_finished_path = cnt_finished_path + 1;
            if cnt_finished_path >= 2 
                forwBackVel = 0;
                fprintf('Finished the current path and continue to explore the map \n');
                cnt_path = 0;
                fsm = 'path';
            end
        end
            
        
    elseif strcmp(state, 'findBasket'),
        if strcmp(fsm, 'path'),
            if cnt_path < 5 % During 3 timesteps we put all speed to 0 to be sure the robot stop
                forwBackVel = 0; % Robot must not move while computing because we loose VREP comm during this time
                leftRightVel = 0;
                rotVel = 0;
                h = youbot_drive(vrep, h, forwBackVel, leftRightVel, rotVel);
                cnt_path = cnt_path+1;
            else
                start = youbot_abs;
                fprintf('Begin to find the next basket and compute a path \n');
                goal = [struct_baskets.centers_xy(h_basket.cnt_basket,1),struct_baskets.centers_xy(h_basket.cnt_basket,2)];
                map_modif = map;
                map_modif(struct_baskets.pixelsBlob{1,h_basket.cnt_basket}) = 0;
                %save('MapCose.mat','map','start','goal');
                [path,~] = ourPathPlannerBest(map_modif, goal, start, map);
                fprintf('Path computed \n');
                fct_give_path(start, goal, map, youbotPos, path, RTr, step_path, cell_map, h_path)
                fprintf('Computing a new path finished \n');
                fprintf('Rotate \n');
                prevErrRot = 0;
                fsm = 'rotate';
            end
        elseif strcmp(fsm, 'rotate'),
            forwBackVel = 0;
            leftRightVel = 0;
            [rotVel,errRot, fsmReturn] = fct_rotate(youbotPos, youbotEuler, cell_map, map_modif, h_path, prevErrRot);
            prevErrRot = errRot;
            if strcmp(fsmReturn, 'path'),
                fsm = 'path';
            elseif strcmp(fsmReturn, 'drive'),
                fsm = 'drive';
                prevErrDr = 0;
            end
        elseif strcmp(fsm, 'drive'),
            [rotVel, errRot, forwBackVel, leftRightVel, errDr, fsmReturn] = fct_drive(youbotPos, youbotEuler, h_path, cell_map, prevErrRot, prevErrDr);
            prevErrRot = errRot;
            prevErrDr = errDr;
            if strcmp(fsmReturn, 'finished_curr_path'),
                cnt_finished_path = 0;
                forwBackVel = 0;
                leftRightVel = 0;
                rotVel = 0;
                fsm = 'finished_curr_path';
            end
            if ( h_path.cnt < length(h_path.true_index_via) && any(map(sub2ind(size(map), h_path.path((h_path.true_index_via(h_path.cnt):h_path.true_index_via(h_path.cnt+1)),2), h_path.path((h_path.true_index_via(h_path.cnt):h_path.true_index_via(h_path.cnt+1)),1)))) )
                cnt_path = 0;
                %map(youbot_abs(2),youbot_abs(1)) = 0; % HEREEEEEE
                fsm = 'path';
                forwBackVel = 0;
                leftRightVel = 0;
                prevErrRot = 0;
            end
        elseif strcmp(fsm, 'finished_curr_path'),
            forwBackVel = 10;
            leftRightVel = 0;
            rotVel = 0;
            cnt_finished_path = cnt_finished_path + 1;
            if cnt_finished_path >= 10 
                forwBackVel = 0;
                [errRot, rotVel] = youbot_rotate(youbotPos(1), youbotPos(2), youbotEuler(3), h_path.goal_transfo(1), h_path.goal_transfo(2), prevErrRot);
                prevErrRot = errRot;
                if (abs(errRot) < 0.15) %&& (abs(angdiff(prevOri, youbotEuler(3))) < 0.04),
                    rotVel = 0;
                    fprintf('Finished the current path and take a picture of the basket \n');
                    res = vrep.simxSetFloatSignal(id, 'rgbd_sensor_scan_angle', pi/3,...
                        vrep.simx_opmode_oneshot_wait); %pi/2 before
                    vrchk(vrep, res);
                    res = vrep.simxSetIntegerSignal(id, 'handle_rgb_sensor', 1,...
                        vrep.simx_opmode_oneshot_wait);
                    vrchk(vrep, res);
                    fprintf('Capturing image...\n');
                    [res resolution image] = ...
                        vrep.simxGetVisionSensorImage2(id, h.rgbSensor, 0,...
                        vrep.simx_opmode_oneshot_wait);
                    vrchk(vrep, res);
                    fprintf('Captured %i pixels.\n', resolution(1)*resolution(2));
                    image_name = strcat('Picture',num2str(h_basket.cnt_basket),'.png');
                    imwrite(image,image_name);
                    curr_cnt_basket = h_basket.cnt_basket;
                    size(h_basket.centers_basket,1)
                    if h_basket.cnt_basket == size(h_basket.centers_basket,1) % if the robot has gone to all the baskets it has discovered
                        state = 'navigate'; 
                    else % if not
                        h_basket.cnt_basket = h_basket.cnt_basket + 1;
                    end
                    cnt_path = 0;
                    fsm = 'path';
                end
            end
            
        end
    
    elseif strcmp(state, 'findObjectsTable1'),
        % If first time in it
        if h_objects_table1.cnt_objects_table1 == 0
            [centers_8targets, ~, ~] = fctHoughFindBaskets(map_plot_precise_input);
            centers_ref = homtrans(inv(RTr),((centers_8targets*cell_map_precise).'));
            centers_ref = centers_ref.';
            fct_8targets_table1(h_objects_table1, struct_tables_input, centers_ref, youbot_start); % youbot_start in ref of the room
            SEObject = strel('disk', 3);
            map_circle = imdilate(map_plot_input,SEObject);
            % We put the arm up to avoid that it is in the field of view of
            % the sensor
            res = vrep.simxSetJointTargetPosition(id, h.armJoints(2), 10*pi/180,...
                                              vrep.simx_opmode_oneshot); % On mets les 5 joints dans la position souhaitée
            vrchk(vrep, res, true);
            res = vrep.simxSetJointTargetPosition(id, h.armJoints(4), 0,...
                                              vrep.simx_opmode_oneshot); % On mets les 5 joints dans la position souhaitée
            vrchk(vrep, res, true);
            pause(1.5)
            fsm = 'path';
            
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% TO DELETE LATER  the next part with state = 'grasp'; %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            state = 'grasp';
            
            
        else % else we go to the 8 points to take a picture
            if strcmp(fsm, 'path'),
                if cnt_path < 5 % During 3 timesteps we put all speed to 0 to be sure the robot stop
                    forwBackVel = 0; % Robot must not move while computing because we loose VREP comm during this time
                    leftRightVel = 0;
                    rotVel = 0;
                    h = youbot_drive(vrep, h, forwBackVel, leftRightVel, rotVel);
                    cnt_path = cnt_path+1;
                else
                    start = youbot_abs;
                    fprintf('Begin to find the path to go to pick an object \n');
                    goal = h_objects_table1.target_xy_map(h_objects_table1.cnt_objects_table1,:);
                    %[path,mapPath_cost] = ourPathPlannerBest(map_modif, goal, start, map);
                    [path,~] = ourPathPlannerBest(map_circle, goal, start);
                    fprintf('Path computed \n');
                    fct_give_path(start, goal, map_circle, youbotPos, path, RTr, step_path, cell_map, h_path)
                    fprintf('Computing a new path finished \n');
                    fprintf('Rotate \n');
                    prevErrRot = 0;
                    fsm = 'rotate';
                end
            elseif strcmp(fsm, 'rotate'),
                forwBackVel = 0;
                leftRightVel = 0;
                [rotVel,errRot, fsmReturn] = fct_rotate(youbotPos, youbotEuler, cell_map, map, h_path, prevErrRot);
                prevErrRot = errRot;
                if strcmp(fsmReturn, 'path'),
                    fsm = 'path';
                elseif strcmp(fsmReturn, 'drive'),
                    fsm = 'drive';
                    prevErrDr = 0;
                end
            elseif strcmp(fsm, 'drive'),
                [rotVel, errRot, forwBackVel, leftRightVel, errDr, fsmReturn] = fct_drive(youbotPos, youbotEuler, h_path, cell_map, prevErrRot, prevErrDr);
                prevErrRot = errRot;
                prevErrDr = errDr;
                if strcmp(fsmReturn, 'finished_curr_path'),
                    cnt_finished_path = 0;
                    forwBackVel = 0;
                    leftRightVel = 0;
                    rotVel = 0;
                    cnt_finished_curr_path = 0;
                    fsm = 'finished_curr_path';
                end
            elseif strcmp(fsm, 'finished_curr_path'),
                forwBackVel = 0;
                leftRightVel = 0;
                rotVel = 0;
                cnt_finished_curr_path = cnt_finished_curr_path +1;
                if cnt_finished_curr_path > 10
                    [rotVel, forwBackVel, leftRightVel, errDr, fsmReturn] = fct_move_precise(youbotPos, youbotEuler, ...
                        h_objects_table1.center_table_right_abs, h_objects_table1.target_xy_abs(h_objects_table1.cnt_objects_table1,:), prevErrDr);
                    prevErrDr = errDr;
                    if strcmp(fsmReturn, 'finished'),
                        fsm = 'takePicture';
                        cnt_picture = 0;
                    end
                end
            elseif strcmp(fsm, 'takePicture'),
                  cnt_picture = cnt_picture + 1;
                  rotVel = 0;
                  forwBackVel = 0;
                  leftRightVel = 0;
                  if cnt_picture > 15
                      fct_picture_table(id, vrep, h, h_objects_table1);
                    if h_objects_table1.cnt_objects_table1 < 8 % If not taken the 8 pictures, go to take the next one
                        h_objects_table1.cnt_objects_table1 = h_objects_table1.cnt_objects_table1 + 1;
                        cnt_path = 0;
                        fsm = 'path';
                    else
                        cnt_finished = 0;
                        state = 'finished';
                        % We put the arm to its starting configuration 
                        for i = 1:5,
                            res = vrep.simxSetJointTargetPosition(id, h.armJoints(i),...
                                startingJoints(i),...
                                vrep.simx_opmode_oneshot);
                            vrchk(vrep, res, true);
                        end
                        pause(1.5)
                    end
                  end
            end
            
        end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Attention : To change and to move in function of armRef (youBot_ref)
% rather than ref (youBot_center) when going close to the target
% See youbot_init
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    elseif strcmp(state, 'grasp'),
        if isempty(h_objects_table1.centers_objects)
            fprintf('In grasp \n');
            fct_give_centers_table1(h_objects_table1) % To complete for the future
            SEObject = strel('disk', 3);
            map_circle = imdilate(map_plot_input,SEObject);
        else % else we go to the 8 points to take a picture
            if strcmp(fsm, 'path'),
                if cnt_path < 5 % During 3 timesteps we put all speed to 0 to be sure the robot stop
                    forwBackVel = 0; % Robot must not move while computing because we loose VREP comm during this time
                    leftRightVel = 0;
                    rotVel = 0;
                    h = youbot_drive(vrep, h, forwBackVel, leftRightVel, rotVel);
                    cnt_path = cnt_path+1;
                else
                    start = youbot_abs;
                    fprintf('Begin to find the path to go to pick an object \n');
                    % We find the closest tagert among the 8 defines to the
                    % center
                    if h_objects_table1.cnt_objects_table1 ==  0 % Normally not ok but to be sure
                        fct_8targets_table1(h_objects_table1, struct_tables, centers_abs, start)
                        h_objects_table1.cnt_objects_table1 = 1;
                    end
                    d = sqrt((h_objects_table1.target_xy_abs(:,1)-h_objects_table1.centers_objects(h_objects_table1.cnt_objects_table1,1)).^2 + ...
                        (h_objects_table1.target_xy_abs(:,2)-h_objects_table1.centers_objects(h_objects_table1.cnt_objects_table1,2)).^2);
                    [~,idx_goal] = sort(d);
                    goal = h_objects_table1.target_xy_map(idx_goal(1),:);
                    %[path,mapPath_cost] = ourPathPlannerBest(map_modif, goal, start, map);
                    [path,~] = ourPathPlannerBest(map_circle, goal, start);
                    fprintf('Path computed \n');
                    fct_give_path(start, goal, map_circle, youbotPos, path, RTr, step_path, cell_map, h_path)
                    fprintf('Computing a new path finished \n');
                    fprintf('Rotate \n');
                    prevErrRot = 0;
                    fsm = 'rotate';
                end
            elseif strcmp(fsm, 'rotate'),
                forwBackVel = 0;
                leftRightVel = 0;
                [rotVel,errRot, fsmReturn] = fct_rotate(youbotPos, youbotEuler, cell_map, map, h_path, prevErrRot);
                prevErrRot = errRot;
                if strcmp(fsmReturn, 'path'),
                    fsm = 'path';
                elseif strcmp(fsmReturn, 'drive'),
                    fsm = 'drive';
                    prevErrDr = 0;
                end
            elseif strcmp(fsm, 'drive'),
                fprintf('In drive \n');
                [rotVel, errRot, forwBackVel, leftRightVel, errDr, fsmReturn] = fct_drive(youbotPos, youbotEuler, h_path, cell_map, prevErrRot, prevErrDr);
                prevErrRot = errRot;
                prevErrDr = errDr;
                if strcmp(fsmReturn, 'finished_curr_path'),
                    cnt_finished_path = 0;
                    forwBackVel = 0;
                    leftRightVel = 0;
                    rotVel = 0;
                    cnt_finished_curr_path = 0;
                    fsm = 'finished_curr_path_rot';
                    fprintf('Go to orientate with the target \n');
                end
 
            elseif strcmp(fsm, 'finished_curr_path_rot'), % To re-oriente correctly before move precise
                forwBackVel = 0;
                leftRightVel = 0;
                rotVel = 0;
                cnt_finished_curr_path = cnt_finished_curr_path + 1;
                if cnt_finished_curr_path > 10
                    %[rotVel,errRot, fsmReturn] = fct_rotate(youbotPos, youbotEuler, cell_map, map, h_path, prevErrRot);
                    [rotVel] = youbot_rotate_precise(youbotPos(1), youbotPos(2), youbotEuler(3), ...
                        h_objects_table1.centers_objects(h_objects_table1.cnt_objects_table1,1), ...
                        h_objects_table1.centers_objects(h_objects_table1.cnt_objects_table1,2));
                    if abs(rotVel) < 0.02
                        fsm = 'finished_curr_path';
                        cnt_finished_curr_path = 0;
                        prevErrDr = 0;
                        fprintf('Go to approach to the target \n');
                        h_objects_table1.centers_objects(h_objects_table1.cnt_objects_table1,:)
                    end
                end
            elseif strcmp(fsm, 'finished_curr_path'),
                forwBackVel = 0;
                leftRightVel = 0;
                rotVel = 0;
                cnt_finished_curr_path = cnt_finished_curr_path +1;
                if cnt_finished_curr_path > 10
                    [res t] = vrep.simxGetObjectPosition(id, h.armRef, -1,...
                        vrep.simx_opmode_oneshot_wait); % We get back the position of the tip compared to the armRef
                    vrchk(vrep, res, true);
                    [res o] = vrep.simxGetObjectOrientation(id, h.armRef, -1,...
                        vrep.simx_opmode_oneshot_wait); % We get back the position of the tip compared to the armRef
                    vrchk(vrep, res, true);
                    [rotVel, forwBackVel, leftRightVel, errDr, fsmReturn] = fct_move_precise_grasp(youbotPos, youbotEuler, ...
                        h_objects_table1.centers_objects(h_objects_table1.cnt_objects_table1,:), h_objects_table1.centers_objects(h_objects_table1.cnt_objects_table1,:), prevErrDr);
                    prevErrDr = errDr;
                    if strcmp(fsmReturn, 'finished'),
                        fprintf('Close enough from target and snapshot_move \n');
                        fsm = 'snapshot_move';
                        cnt_snapshot_move = 0;
                        res = vrep.simxSetIntegerSignal(id, 'km_mode', 2,...
                                       vrep.simx_opmode_oneshot_wait); % km_mode = 2 => robot will try to move the *position* of ptip to the *position* of ptarget.
                        [res tpos] = vrep.simxGetObjectPosition(id, h.ptip, h.armRef,...
                                         vrep.simx_opmode_buffer);
                        vrchk(vrep, res, true);
                        res = vrep.simxSetObjectPosition(id, h.ptarget, h.armRef, [tpos(1) tpos(2) 0.25],...
                            vrep.simx_opmode_oneshot);  % We update the target
                        vrchk(vrep, res, true);
                        fsm = 'snapshot';
                    end
                end
                
            elseif strcmp(fsm, 'snapshot_move'),
                cnt_snapshot_move = cnt_snapshot_move+ 1;
                if cnt_snapshot_move == 10
                    [res t] = vrep.simxGetObjectPosition(id, h.armRef, -1,...
                        vrep.simx_opmode_oneshot_wait); % We get back the position of the tip compared to the armRef
                    vrchk(vrep, res, true);
                    [res o] = vrep.simxGetObjectOrientation(id, h.armRef, -1,...
                        vrep.simx_opmode_oneshot_wait); % We get back the position of the tip compared to the armRef
                    vrchk(vrep, res, true);

                    refRoom2armRef = transl(t) * trotx(o(1)) * troty(o(2)) * trotz(o(3));
                    myTarget = refRoom2armRef\[h_objects_table1.centers_objects(h_objects_table1.cnt_objects_table1,:).';1];
                    res = vrep.simxSetIntegerSignal(id, 'km_mode', 2,...
                        vrep.simx_opmode_oneshot_wait);
                    vrchk(vrep, res, true);

                    res = vrep.simxSetObjectPosition(id, h.ptarget, h.armRef, [myTarget(1) myTarget(2) 0.38],...
                        vrep.simx_opmode_oneshot);  % We update the target
                    vrchk(vrep, res, true);
                    cnt_extend1 = 0;
                    fsm = 'extend1';
                end
             
            elseif strcmp(fsm, 'extend1'),
                cnt_extend1 = cnt_extend1 + 1;
                if cnt_extend1 == 15
                    res = vrep.simxSetIntegerSignal(id, 'km_mode', 2,...
                        vrep.simx_opmode_oneshot_wait);
                    vrchk(vrep, res, true);
                    
                    res = vrep.simxSetObjectPosition(id, h.ptarget, h.armRef, [myTarget(1) myTarget(2) myTarget(3)],...
                        vrep.simx_opmode_oneshot);  % We update the target
                    vrchk(vrep, res, true);
                    fsm = 'extend2';
                end
             
            elseif strcmp(fsm, 'extend2'),
                fprintf('In extend \n');
                [res tpos] = vrep.simxGetObjectPosition(id, h.ptip, h.armRef,...
                    vrep.simx_opmode_buffer); % We get back the position of the tip compared to the armRef
                vrchk(vrep, res, true);
                if norm(tpos-[myTarget(1) myTarget(2) myTarget(3)]) < .002,
                    res = vrep.simxSetIntegerSignal(id, 'km_mode', 2,...
                        vrep.simx_opmode_oneshot_wait); % km_mode = 2 => robot will try to move the *position* of ptip to the *position* of ptarget.
                    fsm = 'grasp';
                end
            elseif strcmp(fsm, 'reachout'),
                [res tpos] = vrep.simxGetObjectPosition(id, h.ptip, h.armRef,...
                    vrep.simx_opmode_buffer);
                vrchk(vrep, res, true);
                
                tpos(1) = tpos(1)+.03;
                res = vrep.simxSetObjectPosition(id, h.ptarget, h.armRef, tpos,...
                    vrep.simx_opmode_oneshot);  % We update the target
                vrchk(vrep, res, true);
                fsm = 'grasp';
            elseif strcmp(fsm, 'grasp'),
                res = vrep.simxSetIntegerSignal(id, 'gripper_open', 0,...
                    vrep.simx_opmode_oneshot_wait); % The gripper closes and applies a constant force inwards
                vrchk(vrep, res);
                pause(2);
                res = vrep.simxSetObjectPosition(id, h.ptarget, h.armRef, [myTargetUp(1) myTargetUp(2) 0.50],...
                    vrep.simx_opmode_oneshot);  % We update the target
                vrchk(vrep, res, true);
                
                res = vrep.simxSetIntegerSignal(id, 'km_mode', 0,...
                    vrep.simx_opmode_oneshot_wait); % The arm joints are in position-control mode. Use simxSetJointTargetPosition to change the arm's configuration.
                fsm = 'backoff';
            elseif strcmp(fsm, 'backoff'),
                for i = 1:5,
                    res = vrep.simxSetJointTargetPosition(id, h.armJoints(i),...
                        startingJoints(i),...
                        vrep.simx_opmode_oneshot); % We put back all the joints in their starting positions
                    vrchk(vrep, res, true);
                end
                [res tpos] = vrep.simxGetObjectPosition(id, h.ptip, h.armRef,...
                    vrep.simx_opmode_buffer);
                vrchk(vrep, res, true);
                if norm(tpos-homeGripperPosition) < .02,
                    [res endGripperPosition] = vrep.simxGetObjectPosition(id, ...
                        h.ptip,...
                        h.armRef,...
                        vrep.simx_opmode_buffer);
                    vrchk(vrep, res, true);
                    endGripperPosition
                    homeGripperPosition
                    res = vrep.simxSetIntegerSignal(id, 'gripper_open', 1,...
                        vrep.simx_opmode_oneshot_wait); % If closed enough from starting position, the gripper is opened
                    vrchk(vrep, res);
                end
                if norm(tpos-homeGripperPosition) < .002,
                    a = 5;
                    state = 'finished';
                    cnt_finished = 0;
                end
            end
            
        end
        
    elseif strcmp(state, 'finished'),
        forwBackVel = 0;
        leftRightVel = 0;
        rotVel = 0;
        map_seen(:) = 0;
        if cnt_finished == 3
            fprintf('Finished to navigate \n');
            %save('BuildMap.mat','map_plot', 'map');
            %save('Structures.mat','struct_tables','struct_baskets');
            break,
        end
        cnt_finished = cnt_finished + 1;
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