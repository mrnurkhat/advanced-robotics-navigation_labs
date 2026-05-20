function [measurement] = compute_lidar_measurement(map, pose, lidar_config, lidar_range)
% This function predicts what the sensor would "see" from the particle's perspective
% by performing ray casting against the known map boundaries

x = pose(1);
y = pose(2);
dir = pose(3);
    
num_rays = length(lidar_config);
measurement = zeros(1, num_rays);

for i = 1:num_rays
    % Absolute ray direction
    global_ray_dir = lidar_config(i) + dir;

    % Perform ray casting to find all geometric intersections with the all defined walls
    all_intersections = ray_cast([x, y], map.walls, global_ray_dir);

    % Filter out NaN values
    valid_hits = all_intersections(~any(isnan(all_intersections), 2), :); 
    
    if isempty(valid_hits)
    % If no intersection is found
        measurement(i) = lidar_range;
    else
        % Compute Euclidean distances from the particle to all hit points
        distances = hypot(valid_hits(:, 1) - x, valid_hits(:, 2) - y); 

        % Distance to the closest obstacle
        measurement(i) = min(distances);
    end
end

end

