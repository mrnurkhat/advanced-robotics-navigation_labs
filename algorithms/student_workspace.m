function [public_vars] = student_workspace(read_only_vars, public_vars)
    
    if (read_only_vars.counter == 1)
        % Motion planning parameters
        public_vars.look_ahead_dist = 0.2;
        public_vars.curr_waypoint_idx = 1;  
        public_vars.desired_speed = 0.3; 
        public_vars.max_angular_vel = 1.0; 
        public_vars.finish_treshold = 0.1; 

        % Particle filter parameters
        public_vars.particles_count = 500;
        public_vars.motion_noise = 0.8; 
        public_vars.sensor_noise = 1.2; 
        public_vars.lidar_range = 15;
     
        public_vars = init_particle_filter(read_only_vars, public_vars);

        % Path smoothing parameters
        public_vars.tolerance = 0.1;
        public_vars.weight_data = 0.75;
        public_vars.weight_smooth = 0.25;
        public_vars.path = [];
    end
    
    public_vars = update_particle_filter(read_only_vars, public_vars);
    public_vars.estimated_pose = estimate_pose(public_vars, read_only_vars);
    cluster_ids = public_vars.particles(:, 4);
    num_active_clusters = length(unique(cluster_ids(cluster_ids > 0)));
   
    if num_active_clusters > 1
        public_vars.desired_speed = 0.1; 
        
    elseif num_active_clusters == 1
        public_vars.desired_speed = 0.4;
        
        public_vars.path = plan_path(read_only_vars, public_vars);
        
    else
        % СТАТУС 3: ПОТЕРЯ ЛОКАЛИЗАЦИИ (Все частицы умерли / num_clusters == 0)
        % Робот ослеп или его унесло. 
        public_vars.desired_speed = 0.0; % Экстренная остановка!
        % Здесь можно запустить init_particle_filter заново (Global Recovery)
    end
    
    % 13. Вычисляем команды на моторы (V, W)
    public_vars = plan_motion(read_only_vars, public_vars);
end