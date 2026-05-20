function [path] = astar(read_only_vars, public_vars)
% x, y - horizontal & vertical world coordinates
% i, j - horizontal & vertical grid coordinates
% 
% (vertical_grid, horizontal_grid) = world2Grid(horizontal_world, vertical_world)
% (horizontal_world, vertical_world) = grid2World(vertical_grid, horizontal_grid)

occupancy_grid_dim = size(read_only_vars.discrete_map.map);
occupancy_grid = read_only_vars.discrete_map.map;
ds = read_only_vars.map.discretization_step;

occupancy_grid = dilate_map(occupancy_grid);

start_x = public_vars.estimated_pose(1);
start_y = public_vars.estimated_pose(2);

[start_j, start_i] = world2Grid(start_x, start_y, read_only_vars);

goal_x = read_only_vars.map.goal(1);
goal_y = read_only_vars.map.goal(2);

goal_i = read_only_vars.discrete_map.goal(1); 
goal_j = read_only_vars.discrete_map.goal(2);

closed_list = false(occupancy_grid_dim);

g_score = inf(occupancy_grid_dim);
h_score = inf(occupancy_grid_dim);
f_score = inf(occupancy_grid_dim);
parent = zeros([occupancy_grid_dim, 2]);

g_score(start_j, start_i) = 0;
h_score(start_j, start_i) = hypot(goal_y - start_y, goal_x - start_x);
f_score(start_j, start_i) = g_score(start_j, start_i) + h_score(start_j, start_i);

open_list = [start_j, start_i, f_score(start_j, start_i)];

path = [];

while ~isempty(open_list)
    [~, idx] = min(open_list(:, 3));
    curr_node = open_list(idx, :);
    open_list(idx, :) = [];

    curr_j = curr_node(1);
    curr_i = curr_node(2);

    if curr_j == goal_j && curr_i == goal_i
        tmp_j = curr_j;
        tmp_i = curr_i;

        while tmp_j ~= 0 && tmp_i ~= 0
            [wx, wy] = grid2World(tmp_j, tmp_i, read_only_vars);
            path = [wx, wy; path];
            pj = parent(tmp_j, tmp_i, 1);
            pi = parent(tmp_j, tmp_i, 2);
            
            tmp_j = pj;
            tmp_i = pi;
        end

        return;
    end

    closed_list(curr_j, curr_i) = true;

    for di = -1:1
        for dj = -1:1
            if di == 0 && dj == 0
                continue;
            end
            
            nj = curr_j + dj;
            ni = curr_i + di;
            
            out_of_map = nj < 1 || nj > occupancy_grid_dim(1) || ni < 1 || ni > occupancy_grid_dim(2); 
            if out_of_map
                continue;
            end

            is_obstacle = (occupancy_grid(nj, ni) == 100);

            if is_obstacle || closed_list(nj, ni)
                continue;
            end
            
            dist = hypot(di, dj) * ds;
            tentative_g = g_score(curr_j, curr_i) + dist + occupancy_grid(nj, ni);

            if tentative_g < g_score(nj, ni)
                g_score(nj, ni) = tentative_g;

                [nx, ny] = grid2World(nj, ni, read_only_vars);
                h_val = hypot(goal_x - nx, goal_y - ny);

                f_score(nj, ni) = g_score(nj, ni) + h_val;
                parent(nj, ni, :) = [curr_j, curr_i];

                in_open = any(open_list(:, 1) == nj & open_list(:, 2) == ni);

                if ~in_open
                    open_list = [open_list; nj, ni, f_score(nj, ni)];
                else
                    idx_in_open = (open_list(:, 1) == nj & open_list(:, 2) == ni);
                    open_list(idx_in_open, 3) = f_score(nj, ni);
                end
            end
        end
    end
end

if isempty(path)
    disp("Path not found.");
end

end

