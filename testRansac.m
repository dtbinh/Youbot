function [pts] =  youbot_grasp_comments()
%function youbot_grasp_comments()
% youbot Illustrates the V-REP Matlab bindings.

% (C) Copyright Renaud Detry 2013.
% Distributed under the GNU General Public License.
% (See http://www.gnu.org/copyleft/gpl.html)
   
  disp('Program started');
  %Use the following line if you had to recompile remoteApi
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
  forwBackVel = 0;
  leftRightVel = 0;
  rotVel = 0;
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

  plotData = true;
  if plotData,
    subplot(211)
    drawnow;
    [X,Y] = meshgrid(-5:.25:5,-5.5:.25:2.5);
    X = reshape(X, 1, []);
    Y = reshape(Y, 1, []);
  end

  % Make sure everything is settled before we start
  pause(2);

  [res homeGripperPosition] = ...
   vrep.simxGetObjectPosition(id, h.ptip,...
                              h.armRef,...
                              vrep.simx_opmode_buffer);
  vrchk(vrep, res, true);
  fsm = 'rotate';

  while true,
    tic
    if vrep.simxGetConnectionId(id) == -1,
      error('Lost connection to remote API.');
    end

    [res youbotPos] = vrep.simxGetObjectPosition(id, h.ref, -1,...
                                                vrep.simx_opmode_buffer);
    vrchk(vrep, res, true);
    [res youbotEuler] = vrep.simxGetObjectOrientation(id, h.ref, -1,...
                                                     vrep.simx_opmode_buffer);
    vrchk(vrep, res, true);

    if plotData,
      % Read data from the Hokuyo sensor:
      [pts contacts] = youbot_hokuyo(vrep, h, vrep.simx_opmode_buffer);

      in = inpolygon(X, Y, [h.hokuyo1Pos(1) pts(1,:) h.hokuyo2Pos(1)],...
                    [h.hokuyo1Pos(2) pts(2,:) h.hokuyo2Pos(2)]);

      subplot(211)
      plot(X(in), Y(in),'.g', pts(1,contacts), pts(2,contacts), '*r',...
          [h.hokuyo1Pos(1) pts(1,:) h.hokuyo2Pos(1)],...
          [h.hokuyo1Pos(2) pts(2,:) h.hokuyo2Pos(2)], 'r',...
          0, 0, 'ob',...
          h.hokuyo1Pos(1), h.hokuyo1Pos(2), 'or',...
          h.hokuyo2Pos(1), h.hokuyo2Pos(2), 'or');
      axis([-5.5 5.5 -5.5 2.5]);
      axis equal;
      drawnow;
    end
    angl = -pi/2;

    if strcmp(fsm, 'rotate'),
      rotVel = angdiff(angl, youbotEuler(3));
      if (abs(angdiff(angl, youbotEuler(3))) < .1/180*pi) && (abs(angdiff(prevOri, youbotEuler(3))) < .01/180*pi),
        rotVel = 0;
        fsm = 'drive';
      end
      prevOri = youbotEuler(3);
    elseif strcmp(fsm, 'drive'),
      forwBackVel = -(youbotPos(1)+3.167);

      if (youbotPos(1)+3.167 < .001) && (abs(youbotPos(1)-prevLoc) < .001),
        forwBackVel = 0;
        vrep.simxSetObjectOrientation(id, h.rgbdCasing, h.ref,...
                                     [0 0 pi/4], vrep.simx_opmode_oneshot);  
        for i = 1:5,
          res = vrep.simxSetJointTargetPosition(id, h.armJoints(i), pickupJoints(i),...
                                               vrep.simx_opmode_oneshot); % On mets les 5 joints dans la position souhaitée
          vrchk(vrep, res, true);
        end
        if plotData,
          fsm = 'snapshot';
        else,
          fsm = 'extend';
        end
      end
      prevLoc = youbotPos(1);
    elseif strcmp(fsm, 'snapshot'),
      % Read data from the range camera

      % Reading a 3D image costs a lot to VREP (vrep has to simulate the image). It
      % also requires a lot of bandwidth, and processing a 3D point cloud (for
      % instance, to find one of the boxes or cylinders that the robot has to
      % grasp) will take a long time in Matlab. In general, you will only want to
      % capture a 3D image at specific times, for instance when you believe you're
      % facing one of the tables.

      % Reduce the view angle to better see the objects
      res = vrep.simxSetFloatSignal(id, 'rgbd_sensor_scan_angle', pi/8,...
                                   vrep.simx_opmode_oneshot_wait);
      vrchk(vrep, res);
      % Ask the sensor to turn itself on, take A SINGLE 3D IMAGE,
      % and turn itself off again
      res = vrep.simxSetIntegerSignal(id, 'handle_xyz_sensor', 1,...
                                     vrep.simx_opmode_oneshot_wait);
      vrchk(vrep, res);

      fprintf('Capturing point cloud...\n');
      pts = youbot_xyz_sensor(vrep, h, vrep.simx_opmode_oneshot_wait);
      % Each column of pts has [x;y;z;distancetosensor]
      % Here, we only keep points within 1 meter, to focus on the table
      pts = pts(1:3,pts(4,:)<1);
      %pts(1,:) = -pts(1,:);
      size(pts)
      %pts = pts(:,(pts(2,:) > -0.038)); % seems to be x z y
      subplot(223)
      plot3(pts(1,:), pts(3,:), pts(2,:), '*');
      xlabel('pts(1,:) = x');
      ylabel('pts(3,:) = z');
      zlabel('pts(2,:) = y');
      hold on;
      %plot3(0.3259, 0.2951, -0.0010, 'r*');
      plot3(-0.0010, 0.3259, 0.2951, 'r*');
      %plot3(0.1238,-0.3015,0.0693,'g*');
      plot3(0.1882,-0.4180,0.1716,'g*');
      hold off;
      %axis equal;
      %view([-169 -46]);

      % Save the pointcloud to pc.xyz.
      % (pc.xyz can be displayed with meshlab.sf.net).
      fileID = fopen('pc.xyz','w');
      fprintf(fileID,'%f %f %f\n',pts);
      fclose(fileID);
      fprintf('Read %i 3D points, saved to pc.xyz.\n', max(size(pts)));
      save('pts.mat','pts');

      % Read data from the RGB camera

      % This is very similar to reading from the 3D camera. The comments in the 3D
      % camera section directly apply to this section.

      res = vrep.simxSetIntegerSignal(id, 'handle_rgb_sensor', 1,...
                                     vrep.simx_opmode_oneshot_wait);
      vrchk(vrep, res);
      fprintf('Capturing image...\n');
      [res resolution image] = ...
       vrep.simxGetVisionSensorImage2(id, h.rgbSensor, 0,...
                                      vrep.simx_opmode_oneshot_wait);
      vrchk(vrep, res);
      fprintf('Captured %i pixels.\n', resolution(1)*resolution(2));
      subplot(224)
      imshow(image);
      drawnow;
      
      [res t] = vrep.simxGetObjectPosition(id, h.xyzSensor, h.armRef,...
                                             vrep.simx_opmode_oneshot_wait); % We get back the position of the tip compared to the armRef 
      vrchk(vrep, res, true);
      [res o] = vrep.simxGetObjectOrientation(id, h.xyzSensor, h.armRef,...
                                             vrep.simx_opmode_oneshot_wait); % We get back the position of the tip compared to the armRef 
      vrchk(vrep, res, true);
      
      RyM = transl(t) * trotx(o(1)) * troty(o(2)) * trotz(o(3));
      [center,~] = ourRansac(pts);
      %M = [1 0 0 t(1); 0 cos(pi/2) -sin(pi/2) t(2); 0 sin(pi/2) cos(pi/2) t(3); 0 0 0 1]; % pose from youBot_ref to xyz_repere
      %beta = pi/8;
      %Ry = [cos(beta) 0 sin(beta) 0; 0 1 0 0; -sin(beta) 0 cos(beta) 0; 0 0 0 1]; % pose from xyz_repere to xyz_repere modif of angle of view
      %RyM = M*Ry
      %myTarget = RyM*[center.';1]
      myTarget = RyM*[center.';1]
      youbot2absTrans = transl(youbotPos) * trotx(youbotEuler(1)) * troty(youbotEuler(2)) * trotz(youbotEuler(3));
      myTarget_abs = youbot2absTrans*myTarget
      prof_target_abs = youbot2absTrans*[0.3259; -0.0010; 0.2951; 1]
      a = 5;
      
      
      res = vrep.simxSetIntegerSignal(id, 'km_mode', 2,...
                                       vrep.simx_opmode_oneshot_wait);
      vrchk(vrep, res, true);
      
      res = vrep.simxSetObjectPosition(id, h.ptarget, h.armRef, [myTarget(1) myTarget(2) myTarget(3)],...
                                      vrep.simx_opmode_oneshot);  % We update the target
      vrchk(vrep, res, true);
      
      fsm = 'extend';
    elseif strcmp(fsm, 'extend'),
      [res tpos] = vrep.simxGetObjectPosition(id, h.ptip, h.armRef,...
                                             vrep.simx_opmode_buffer); % We get back the position of the tip compared to the armRef 
      vrchk(vrep, res, true);
      if norm(tpos-[myTarget(1) myTarget(2) myTarget(3)]) < .002,
        res = vrep.simxSetIntegerSignal(id, 'km_mode', 2,...
                                       vrep.simx_opmode_oneshot_wait); % km_mode = 2 => robot will try to move the *position* of ptip to the *position* of ptarget.
        fsm = 'reachout';
      end
    elseif strcmp(fsm, 'reachout'),
      [res tpos] = vrep.simxGetObjectPosition(id, h.ptip, h.armRef,...
                                             vrep.simx_opmode_buffer);
      vrchk(vrep, res, true);
      %tpos(1)
      if tpos(1) > .39,
        fsm = 'grasp';
      end

      tpos(1) = tpos(1)+.01;
      res = vrep.simxSetObjectPosition(id, h.ptarget, h.armRef, tpos,...
                                      vrep.simx_opmode_oneshot);  % We update the target
      vrchk(vrep, res, true);
    elseif strcmp(fsm, 'grasp'),
      res = vrep.simxSetIntegerSignal(id, 'gripper_open', 0,...
                                     vrep.simx_opmode_oneshot_wait); % The gripper closes and applies a constant force inwards
      vrchk(vrep, res);
      pause(2);
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
        res = vrep.simxSetIntegerSignal(id, 'gripper_open', 1,...
                                       vrep.simx_opmode_oneshot_wait); % If closed enough from starting position, the gripper is opened
        vrchk(vrep, res);
      end
      if norm(tpos-homeGripperPosition) < .002,
        fsm = 'finished';
      end
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