function waypoints = coridor_following(lidar_meas, Lh)
    left_dist = lidar_meas(7);
    right_dist = lidar_meas(3);
    forward_dist = lidar_meas(1);

    center = (left_dist + right_dist) / 2;

    waypoints = center;
end