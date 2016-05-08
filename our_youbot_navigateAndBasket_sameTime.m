% The logic of the code is the following :
% The robot navigates (navigate mode). 
% When it has finished its current path, it looks if
% there is a new basket or not. If so it, the mode findBasket is on and the
% robot goes to see it (or them if there is 2). 
% It takes a picture and then goes back in the navigate mode.
% When it has finished to navigate, it stops
%
function our_youbot_navigateAndBasket_sameTime
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

path = [];
step_path = ceil(1.25/cell_map);
cnt_path = 0;

navigate = true;
findBasket = false;

len_old_centers = 2;
cnt_basket = 0;
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
nb_new_centers = 0;
prev_centers = [];
 % We get back the initial position of the youbot
[res, youbotPos] = vrep.simxGetObjectPosition(id, h.ref, -1,...
    vrep.simx_opmode_buffer); %-1 is to retrieve the absolute position of the object
vrchk(vrep, res, true);
youbot_abs_start = ceil( (homtrans(RTr,[youbotPos(1); youbotPos(2)]))/cell_map);

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
    
    % To make sure that the hukoyo data belongs to the matrix
    min_tmp = min(pts_final,floor(side_map/cell_map));
    pts_final = max(min_tmp, 1);
      
    % We update the map
    % Map 2 is a trick to dilate only ones the wall
    map2 = zeros(size(map));
    map2(sub2ind(size(map), pts_final(2,contacts), pts_final(1,contacts))) = 1; % % http://nl.mathworks.com/company/newsletters/articles/matrix-indexing-in-matlab.html
    map2 = idilate(map2, StructEl);
    index_dilate = ((map2-map) == 1);
    map(index_dilate) = 1;
    map_plot(sub2ind(size(map), pts_final(2,contacts), pts_final(1,contacts))) = true;

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
    if ~(isempty(path))
        %hold on;
        plot(path(:,1),path(:,2),'.r');
        hold on;
        plot(via(cnt,1),via(cnt,2), '.g');
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
            if(sum(map_seen(:)) < 10) && navigate && len_old_centers >= 7 % If map totally seen & in navigate mode and not seen the 5 baskets + the 2 tables in the map
                map_seen(:) = 0;
                cnt_finished = 0;
                fsm = 'finished';
            else
                fprintf('Computing a new path \n'); 
                if ~findBasket % if not in findBasket mode, look if not a new basket
                    fprintf('Look if there is a new basket \n');
                    [centers, CC, HoughMap] = fctHoughFindBaskets(map_plot);
                    % If there is a new blob found 
                    if size(centers,1) > len_old_centers 
                        save('BuildMapProb.mat','map_plot', 'map');
                        forwBackVel = 0;
                        cnt_basket = cnt_basket + 1;
                        len_old_centers = size(centers,1);
                        d = sqrt((centers(:,1)-youbot_abs_start(1)).^2 + (centers(:,2)-youbot_abs_start(2)).^2);
                        [~,I] = sort(d);
                        % The 2 closest to the starting position are the
                        % tables
                        struct_tables.centers_xy(1,:) = centers(I(1),:);
                        struct_tables.pixelsBlob{1,1} = CC.PixelIdxList{1,I(1)};
                        struct_tables.centers_xy(2,:) = centers(I(2),:);
                        struct_tables.pixelsBlob{1,2} = CC.PixelIdxList{1,I(2)};
                        % We look which table is the right one (with the
                        % objects upwards)
                        if struct_tables.centers_xy(1,1) < struct_tables.centers_xy(2,1) % if the x index of first table is smaller than second =>
                            struct_tables.right(1) = false;
                            struct_tables.right(2) = true;
                        else
                            struct_tables.right(2) = false;
                            struct_tables.right(1) = true;
                        end
                        % Then it remains the baskets
                        centers_basket = centers(I(3:end),:);
                        idx = I(3:end);
                        for i = 1:size(centers_basket,1)
                            if isempty(prev_centers)
                                x_curr_centers = centers_basket(i,1);
                                y_curr_centers = centers_basket(i,2);
                                nb_new_centers = nb_new_centers + 1;
                                prev_centers = [prev_centers; [x_curr_centers,y_curr_centers]];
                                struct_baskets.centers_xy(nb_new_centers,:) = [x_curr_centers,y_curr_centers];
                                struct_baskets.pixelsBlob{1,nb_new_centers} = CC.PixelIdxList{1,idx(i)};
                            end
                            x_curr_centers = centers_basket(i,1)*ones(size(prev_centers,1),1);
                            y_curr_centers = centers_basket(i,2)*ones(size(prev_centers,1),1);
                            curr_centers = [x_curr_centers,y_curr_centers];
                            distances = sqrt(sum(bsxfun(@minus, curr_centers, prev_centers).^2, 2));
                            if any(distances < 10) % if there is a NOT new centers
                                % To overwrtie in the structure
                                idx_old_centers = find(distances < 10)
                                struct_baskets.centers_xy(idx_old_centers,:) = [x_curr_centers(1),y_curr_centers(1)];
                                struct_baskets.pixelsBlob{1,idx_old_centers} = CC.PixelIdxList{1,idx(i)};
                            else % there is a new centers
                                nb_new_centers = nb_new_centers + 1;
                                % To add in the stucture
                                prev_centers = [prev_centers; [x_curr_centers(1),y_curr_centers(1)]];
                                struct_baskets.centers_xy(nb_new_centers,:) = [x_curr_centers(1),y_curr_centers(1)];
                                struct_baskets.pixelsBlob{1,nb_new_centers} = CC.PixelIdxList{1,idx(i)};
                            end
                        end
                        findBasket = true;
                        navigate = false;
                        fprintf('Find a basket close to me and go see it \n');
                    end
                end
                % Use of our path planner
                if navigate
                    fprintf('Begin to find the next goal and compute a path \n');
                    [goal, y_block, x_block, len_block] = findGoal(map_seen, prev_goal, cell_map);
                    prev_goal=goal
                    save('MapCose.mat','map','start','goal');
                    [path,mapPath_cost] = ourPathPlannerBest(map, goal, start);
                    fprintf('Path computed \n');
                elseif findBasket
                    fprintf('Begin to find the next basket and compute a path \n');
                    goal = [struct_baskets.centers_xy(cnt_basket,1),struct_baskets.centers_xy(cnt_basket,2)];
                    map_modif = map;
                    map_modif(struct_baskets.pixelsBlob{1,cnt_basket}) = 0;
                    save('MapCose.mat','map','start','goal');
                    [path,mapPath_cost] = ourPathPlannerBest(map_modif, goal, start, map);
                    fprintf('Path computed \n');
                end
                if size(path, 1)>2 % If there is at least 2 elements in the path, we only keep the via points and points every 1.25m
                    angles = (path(2:end,2)-path(1:end-1,2))./(path(2:end,1)-path(1:end-1,1));
                    ind_angles = [false(1); angles(2:end) ~= angles(1:end-1)];
                    ind_angles = find(ind_angles == 1);
                    true_index_via = [];
                    if(isempty(ind_angles))
                        true_index_via = [step_path:step_path:length(path)-1];
                        true_index_via = [true_index_via, length(path)];
                        via = path(true_index_via(:),:);
                    else
                        for i = 1:length(ind_angles)-1
                            true_index_via = [true_index_via, ind_angles(i):step_path:(ind_angles(i+1)-1)];
                        end
                        if length(ind_angles) == 1
                            true_index_via = [ind_angles];
                        end
                        true_index_via = [step_path:step_path:true_index_via(1),true_index_via, ind_angles(end), true_index_via(end)+step_path:step_path:(length(path)-1), length(path)];
                        via = path(true_index_via(:),:);
                        true_index_via = [1, true_index_via]; % Trick to know the path between previous via and next one
                        if map(start(2),start(1)) == 1 % Trick to avoid problem if we started in a wall.
                            true_index_via(1) = [];
                        end
                    end
                elseif size(path, 1)==2
                    via = path(end,:);
                    true_index_via = [2];
                end
                q = double((via*cell_map).');
                q_ref = homtrans(inv(RTr),q);
                goal_transfo = homtrans(inv(RTr),((goal*cell_map).'));
                cnt = 1;
                if size(q_ref,2) > 1 && sqrt( (youbotPos(1)-q_ref(1,1))^2 + (youbotPos(2)-q_ref(2,1))^2) < 0.2
                    q_ref = q_ref(:,2:end);
                    via = via(2:end,:);
                    true_index_via = true_index_via(2:end);
                end
                fprintf('Computing a new path finished \n');
                fprintf('Rotate \n');
                fsm = 'rotate';
            end
        end
    elseif strcmp(fsm, 'rotate'), % Can delete this if we do not want that the youbot align the first time, just need to change fsm = 'drive' 3 lines before
        forwBackVel = 0;
        if sqrt( (youbotPos(1)-q_ref(1,cnt))^2 + (youbotPos(2)-q_ref(2,cnt))^2 ) < (2*cell_map) && cnt < size(via, 1)
            cnt = cnt +1;
        end
        [errRot, rotVel] = youbot_rotate(youbotPos(1), youbotPos(2), youbotEuler(3), q_ref(1,cnt), q_ref(2,cnt), prevErrRot);
        if (abs(errRot) < 0.15) %&& (abs(angdiff(prevOri, youbotEuler(3))) < 0.04),
            rotVel = 0;
            h = youbot_drive(vrep, h, forwBackVel, leftRightVel, rotVel);

            if cnt < length(true_index_via) && any(map(sub2ind(size(map), path((true_index_via(cnt):true_index_via(cnt+1)),2), path((true_index_via(cnt):true_index_via(cnt+1)),1)))) 
                cnt_path = 0;
                %map(youbot_abs(2),youbot_abs(1)) = 0; % HEREEEEEE
                path((true_index_via(cnt):true_index_via(cnt+1)),:)
                fsm = 'path';
            else
                prevErrRot = 0;
                cnt_drive = 0;
                fsm = 'drive';
                fprintf('Drive \n');
            end
        end
        prevErrRot = errRot;

    elseif strcmp(fsm, 'drive'),
        [errRot, rotVel] = youbot_rotate(youbotPos(1), youbotPos(2), youbotEuler(3), q_ref(1,cnt), q_ref(2,cnt), prevErrRot);
        prevErrRot = errRot;
        [curr_dist_target, errDr, forwBackVel, leftRightVel] = youbot_2velocities(youbotPos(1), youbotPos(2), q_ref(1,cnt), q_ref(2,cnt), youbotEuler(3), prevErrDr);
        prevErrDr = errDr;
        %cnt_drive = cnt_drive + 1;
        % If close enough => go to next point 
        if (abs(curr_dist_target) < 2.2*sqrt(2)*cell_map) %&& cnt < size(via, 1)) || ((abs(curr_dist_target) < cell_map) && cnt == size(via, 1)) 
            forwBackVel = 0;
            leftRightVel = 0;
            if cnt < size(via, 1) % if no last point go to next one
                cnt = cnt +1;
                prevErrDr = 0;
                % If the next point is too closed we go to 2 points after
                if sqrt( (youbotPos(1)-q_ref(1,cnt))^2 + (youbotPos(2)-q_ref(2,cnt))^2 ) < 0.2 && (cnt+1) < size(via, 1) 
                    cnt = cnt +1;
                    fprintf('Go to next next point because next point to closed  \n');
                end
                fprintf('Next point \n');
                %[errRot, ~] = youbot_rotate(youbotPos(1), youbotPos(2), youbotEuler(3), q_ref(1,cnt), q_ref(2,cnt), prevErrRot);
                %if abs(errRot) > 2*0.1745 %Too big angular deviation with next point => Rotate to align with this next point
                %    fsm = 'rotate';
                %    fprintf('Rotate because must align with next point \n');
                %    fprintf('Rotate \n');   
                %end
            else % if last one => finished the current path
                cnt_finished_path = 0;
                forwBackVel = 0;
                leftRightVel = 0;
                fsm = 'finished_curr_path';
            end
%         elseif abs(errRot) > 2*(0.1745) % If too big angular deviation during motion
%              forwBackVel = 0;
%              h = youbot_drive(vrep, h, forwBackVel, leftRightVel, rotVel);
%              fprintf('Rotate because too big deviation \n');
%              fsm = 'rotate';
%              %curr_dist_target;
%              fprintf('Rotate \n');

        end

        % If obstacle between current via and next one OR if the current block is completely seen & not in findbasket=> compute a new
        % path (ATTENTION : block condition only in navigate)
        block = map_seen( (1+(len_block*(y_block-1))):(len_block*y_block) , (1+(len_block*(x_block-1))):(len_block*x_block));
        if ( cnt < length(true_index_via) && any(map(sub2ind(size(map), path((true_index_via(cnt):true_index_via(cnt+1)),2), path((true_index_via(cnt):true_index_via(cnt+1)),1)))) ) ...
            || (~any(block(:)) && ~findBasket)
            cnt_path = 0;
            %map(youbot_abs(2),youbot_abs(1)) = 0; % HEREEEEEE
            fsm = 'path';
            forwBackVel = 0;
            leftRightVel = 0;
            prevErrRot = 0;
        end

%         if cnt_drive > 10 && ~findBasket
%             [centers, CC, HoughMap] = fctHoughFindBaskets(map_plot);
%             % If there is a new blob found 
%             if length(centers) > len_old_centers 
%                 save('BuildMapProb.mat','map_plot', 'map');
%                 forwBackVel = 0;
%                 d = sqrt((centers(:,1)-youbot_abs_start(1)).^2 + (centers(:,2)-youbot_abs_start(2)).^2);
%                 [~,I] = sort(d);
%                 centers = centers(I(3:end),:);
%                 index_centers = I(3:end);
%                 len_old_centers = length(centers);
%                 cnt_basket = cnt_basket + 1;
%                 findBasket = true;
%                 navigate = false;
%                 cnt_drive = 0;
%                 fprintf('Find a basket close to me and go see it \n');
%                 fsm = 'path';
%             end
%         end


    elseif strcmp(fsm, 'finished_curr_path'),
        forwBackVel = 10;
        leftRightVel = 0;
        rotVel = 0;
        cnt_finished_path = cnt_finished_path + 1;
        if cnt_finished_path >= 2 && navigate % if in navigate, remains in it
            forwBackVel = 0;
            fprintf('Finished the current path and continue to explore the map \n');
            fsm = 'path';
        elseif cnt_finished_path >= 10 && findBasket % if in findBasket, we have to take a picture
            forwBackVel = 0;
            [errRot, rotVel] = youbot_rotate(youbotPos(1), youbotPos(2), youbotEuler(3), goal_transfo(1), goal_transfo(2), prevErrRot);
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
                image_name = strcat('Picture',num2str(cnt_basket),'.png');
                imwrite(image,image_name);
                cnt_basket = cnt_basket
                size(centers_basket,1)
                if cnt_basket == size(centers_basket,1) % if the robot has gone to all the baskets it has discovered
                    findBasket = false;
                    navigate = true;
                else % if not
                    cnt_basket = cnt_basket + 1;
                end
                fsm = 'path';
            end
            prevErrRot = errRot;
        end

    elseif strcmp(fsm, 'finished'),
        forwBackVel = 0;
        leftRightVel = 0;
        rotVel = 0;
        map_seen(:) = 0;
        if cnt_finished == 3
            fprintf('Finished to navigate \n');
            save('BuildMap.mat','map_plot', 'map');
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