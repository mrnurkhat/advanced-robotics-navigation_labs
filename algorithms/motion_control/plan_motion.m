function [public_vars] = plan_motion(target, read_only_vars, public_vars, is_finish)
    pose = public_vars.estimated_pose;
    max_w = public_vars.max_angular_vel;
    speed = public_vars.desired_speed;
    Lh = public_vars.look_ahead_dist;

    dx = target(1) - pose(1);
    dy = target(2) - pose(2);
    dir = pose(3);

    if is_finish
        dist_to_finish = hypot(dx, dy);
        w = 0;

        if dist_to_finish < public_vars.finish_treshold
            speed = 0; 
        end
    else
        y_local = -dx * sin(dir) + dy * cos(dir);
        gamma = 2 * y_local / Lh^2;

        w = gamma * speed;
        w = max(-max_w, min(max_w, w));
    end

    speed_r = speed + w * read_only_vars.agent_drive.interwheel_dist / 2;
    speed_l = speed - w * read_only_vars.agent_drive.interwheel_dist / 2;

    public_vars.desired_speed = speed;
    public_vars.motion_vector = [speed_r, speed_l];
end