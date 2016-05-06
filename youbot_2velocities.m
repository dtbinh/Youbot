function [curr_dist_target, errDr, forwBackVel,leftRightVel] = youbot_2velocities(youbotPos1, youbotPos2, q_ref_x, q_ref_y, Euler3, prevErrDr)
    timestep = .05;
    ku_d = 6.7;
    Tu_d = 2;
    kp_d = 0.6*ku_d;
    ki_d = 2*kp_d/Tu_d;
    kd_d = kp_d*Tu_d/8;    
    curr_dist_target = sqrt( (youbotPos1-q_ref_x)^2 + (youbotPos2-q_ref_y)^2 );
    errDr = -curr_dist_target;
    %forwBackVel = 0.35*(kp_d*errDr + ki_d*errDr*timestep + (kd_d*(errDr -prevErrDr)/timestep));
    M = se2(youbotPos1, youbotPos2, Euler3);
    P = homtrans(inv(M), [q_ref_x;q_ref_y]);
    forwBackVel = P(2);
    leftRightVel = P(1);
    
end