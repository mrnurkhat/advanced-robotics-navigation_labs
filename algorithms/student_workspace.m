function [public_vars] = student_workspace(read_only_vars, public_vars)
    if read_only_vars.counter == 1
        % Motion planning parameters
        public_vars.look_ahead_dist = 0.2;
        public_vars.curr_waypoint_idx = 1;
        public_vars.max_angular_vel = 1.5; 
        public_vars.finish_treshold = 0.1;

        % Adaptive Particle filter parameters
        public_vars.particles_count = 500;
        public_vars.motion_noise = 0.5; 
        public_vars.sensor_noise = 0.6; 
        public_vars.lidar_range = 15;
        public_vars.alpha_slow = 0.001;
        public_vars.alpha_fast = 0.1;

        % Path smoothing parameters
        public_vars.tolerance = 0.1;
        public_vars.weight_data = 0.9;
        public_vars.weight_smooth = 0.1;
        public_vars.path = [];

        public_vars.localization_source = 'NONE';
        public_vars.kf_enabled = false;
        public_vars.pf_enabled = false;

        public_vars.estimated_pose = [0, 0, 0];
    end

    has_gnss = ~any(isnan(read_only_vars.gnss_position), 'all');

    if has_gnss
        public_vars.localization_source = 'KF';
        valid_gnss_count = sum(~isnan(read_only_vars.gnss_history(:, 1)));

        if valid_gnss_count >= 100
            if ~public_vars.kf_enabled
                public_vars = init_kalman_filter(read_only_vars, public_vars);
                public_vars.kf_enabled = true;
            end
        else
            public_vars.localization_source = 'WAITING_GNSS';
        end
    else
        public_vars.localization_source = 'PF';

        if ~public_vars.pf_enabled
            public_vars = init_particle_filter(read_only_vars, public_vars);
            public_vars.pf_enabled = true;
        end
    end

    target = public_vars.estimated_pose(1:2); 
    is_finish = false;

    switch public_vars.localization_source
        case 'WAITING_GNSS'
            public_vars.desired_speed = 0;

        case 'KF'
            [public_vars.mu, public_vars.sigma] = update_kalman_filter(read_only_vars, public_vars);
            public_vars.estimated_pose = estimate_pose(public_vars, read_only_vars);

            if isempty(public_vars.path) || ~mod(read_only_vars.counter, 5)
                public_vars.path = plan_path(read_only_vars, public_vars);
            end

            public_vars.desired_speed = 0.5;
            [target, public_vars] = get_target(public_vars);
            is_finish = (public_vars.curr_waypoint_idx == size(public_vars.path, 1));

        case 'PF'
            public_vars = update_particle_filter(read_only_vars, public_vars);
            public_vars.estimated_pose = estimate_pose(public_vars, read_only_vars);

            cluster_ids = public_vars.particles(:, 4);
            num_active_clusters = length(unique(cluster_ids(cluster_ids > 0)));

            if num_active_clusters > 1
                public_vars.desired_speed = 0.1;
                target = follow_corridor(read_only_vars, public_vars, 5);
                is_finish = false;
            elseif num_active_clusters == 1
                if isempty(public_vars.path) || ~mod(read_only_vars.counter, 5)
                    public_vars.path = plan_path(read_only_vars, public_vars);
                end
                public_vars.desired_speed = 0.3;
                [target, public_vars] = get_target(public_vars);
                is_finish = (public_vars.curr_waypoint_idx == size(public_vars.path, 1));
            else
                public_vars.desired_speed = 0.0;
                target = public_vars.estimated_pose(1:2);
            end
    end

    public_vars = plan_motion(target, read_only_vars, public_vars, is_finish);
end


