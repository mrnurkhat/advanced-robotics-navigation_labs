function [public_vars] = plan_motion(read_only_vars, public_vars)
    pose = public_vars.estimated_pose;
    max_w = public_vars.max_angular_vel;

    cluster_ids = public_vars.particles(:, 4);
    num_active_clusters = length(unique(cluster_ids(cluster_ids > 0)));

    if num_active_clusters > 1
        speed = 0.15; 
        is_last_waypoint = false;

        d_front = read_only_vars.lidar_distances(1);
        d_left = read_only_vars.lidar_distances(3);  
        d_right = read_only_vars.lidar_distances(7);

        max_r = 3.0;
        d_left = min(d_left, max_r);
        d_right = min(d_right, max_r);

        % Генерируем локальную точку по центру коридора
        y_loc = (d_left - d_right) / 2.0;
        x_loc = min(public_vars.look_ahead_dist, d_front * 0.7); % Тормозим перед стеной

        % Переводим в глобальные координаты относительно текущей (даже неверной) позы
        target(1) = pose(1) + x_loc * cos(pose(3)) - y_loc * sin(pose(3));
        target(2) = pose(2) + x_loc * sin(pose(3)) + y_loc * cos(pose(3));

    else
        % СОСТОЯНИЕ Б: УВЕРЕННАЯ НАВИГАЦИЯ
        % Вызываем ваш старый код только тогда, когда точно знаем, где мы!
        speed = public_vars.desired_speed;
        [target, public_vars] = get_target(public_vars);
        is_last_waypoint = (public_vars.curr_waypoint_idx == size(public_vars.path, 1));
    end

    % --- 3. ВАШ КЛАССИЧЕСКИЙ PURE PURSUIT (Работает для обоих состояний!) ---
    dx = target(1) - pose(1);
    dy = target(2) - pose(2);
    dir = pose(3);

    if is_last_waypoint && num_active_clusters == 1
        dist_to_finish = hypot(dx, dy);
        w = 0;

        if dist_to_finish < public_vars.finish_treshold
            speed = 0; % Финиш!
        end
    else
        y_local = -dx * sin(dir) + dy * cos(dir);
        gamma = 2 * y_local / public_vars.look_ahead_dist^2;

        w = gamma * speed;
        w = max(-max_w, min(max_w, w));
    end

    % Расчет скоростей колес
    speed_r = speed + w * read_only_vars.agent_drive.interwheel_dist / 2;
    speed_l = speed - w * read_only_vars.agent_drive.interwheel_dist / 2;

    public_vars.motion_vector = [speed_r, speed_l];
end