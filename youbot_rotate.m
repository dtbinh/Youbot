function [errRot, rotVel] = youbot_rotate(youbotPos1, youbotPos2, youbotEuler3, q_ref_x, q_ref_y, prevErrRot)
    timestep = .05;
    ku_r = 5;
    Tu_r = 1.8;
    kp_r = 0.6*ku_r;
    ki_r = 2*kp_r/Tu_r;
    kd_r = kp_r*Tu_r/8;
    TrAbsToRobot = [1 0 youbotPos1; 0 1 youbotPos2; 0 0 1];
    targetRobot = homtrans(inv(TrAbsToRobot), [q_ref_x; q_ref_y]);
    theta = -atan2(targetRobot(1),targetRobot(2));
    errRot = angdiff(theta, youbotEuler3);
    if errRot < 0
        errRot = errRot+pi;
    else
        errRot = errRot-pi;
    end
    rotVel = (kp_r*errRot + ki_r*errRot*timestep + (kd_r*(errRot -prevErrRot)/timestep));
end