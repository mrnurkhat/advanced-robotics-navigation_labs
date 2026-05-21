function target = follow_corridor(read_only_vars, public_vars, max_r)
    pose = public_vars.estimated_pose;

    d_front = read_only_vars.lidar_distances(1);
    d_left = min(read_only_vars.lidar_distances(2:4));
    d_right = min(read_only_vars.lidar_distances(6:8));
    Lh = public_vars.look_ahead_dist;

    d_left = min(max_r, d_left);
    d_right = min(max_r, d_right);

    y_local = (d_left - d_right) / 2;
    x_local = min(Lh, d_front);

    target(1) = pose(1) + x_local * cos(pose(3)) - y_local * sin(pose(3));
    target(2) = pose(2) + x_local * sin(pose(3)) + y_local * cos(pose(3));
end