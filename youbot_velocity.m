function [curr_dist_target, errDr, forwBackVel] = youbot_velocity(youbotPos1, youbotPos2, q_ref_x, q_ref_y, prevErrDr)
    timestep = .05;
    ku_d = 6.7;
    Tu_d = 2;
    kp_d = 0.6*ku_d;
    ki_d = 2*kp_d/Tu_d;
    kd_d = kp_d*Tu_d/8;    
    curr_dist_target = sqrt( (youbotPos1-q_ref_x)^2 + (youbotPos2-q_ref_y)^2 );
    errDr = -curr_dist_target;
    forwBackVel = 0.5*(kp_d*errDr + ki_d*errDr*timestep + (kd_d*(errDr -prevErrDr)/timestep));
end