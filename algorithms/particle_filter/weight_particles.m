function [weights] = weight_particles(particle_measurements, lidar_distances, public_vars)
    % Evaluates particles by comparing predicted and actual sensor data
    N = size(particle_measurements, 1);
    sigma = public_vars.sensor_noise;
    weights = zeros(N, 1);
    
    % Uniform random noise
    base_prob = 0.01; 
    
    for i = 1:N
        errors = lidar_distances - particle_measurements(i, :);
        
        p_beams = exp(-errors.^2 / (2 * sigma^2));
        p_beams = p_beams + base_prob;
        
        weights(i) = prod(p_beams);
    end
end