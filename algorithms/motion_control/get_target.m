function [target, public_vars] = get_target(public_vars)
    pose = public_vars.estimated_pose;
    path = public_vars.path;
    target_idx = public_vars.curr_waypoint_idx;
    Lh = public_vars.look_ahead_dist;
    
    is_last_waypoint = true;
    for i = target_idx:size(path, 1)
        dx = pose(1) - path(i, 1);
        dy = pose(2) - path(i, 2);
        D = hypot(dx, dy);
    
        if D > Lh 
            target_idx = i;
            is_last_waypoint = false;
            break;
        end
    end
    
    if is_last_waypoint
        target_idx = size(path, 1);
    end
    
    public_vars.curr_waypoint_idx = target_idx;
    target = path(target_idx, :);
end

