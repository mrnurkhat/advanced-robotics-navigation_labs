function [public_vars] = init_particle_filter(read_only_vars, public_vars)
    N = public_vars.particles_count;
    x_min = read_only_vars.map.limits(1) + 0.8;
    y_min = read_only_vars.map.limits(2) + 0.8;
    x_max = read_only_vars.map.limits(3) - 0.8;
    y_max = read_only_vars.map.limits(4) - 0.8;
    
    % particles(1) - horizontal axis
    % particles(2) - vertical axis
    % particles(3) - direction 
    % particles(4) - cluster idx
    particles = zeros(N, 4);

    valid_particles = 0;
    dilated_grid = dilate_map(read_only_vars.discrete_map.map);

    while valid_particles < N 
        rand_x = x_min + (x_max - x_min) * rand();
        rand_y = y_min + (y_max - y_min) * rand();
        
        if is_free_space([rand_x, rand_y], read_only_vars, dilated_grid, 25) && check_gnss_denied_zone(rand_x, rand_y, read_only_vars.map.gnss_denied)
            valid_particles = valid_particles + 1;
            rand_dir = -pi + 2 * pi * rand();

            particles(valid_particles, 1) = rand_x;
            particles(valid_particles, 2) = rand_y;
            particles(valid_particles, 3) = rand_dir;
            particles(valid_particles, 4) = 0;
        end
    end

    public_vars.particles = particles;
    public_vars.weights = ones(N, 1) / N;

    public_vars.w_slow = 0.0;
    public_vars.w_fast = 0.0;
end
