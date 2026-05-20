function [new_particles] = resample_particles(particles, weights, num_particles)
    % Performs systematic resampling to maintain particle diversity.
    % It replaces low-weight particles with copies of high-weight ones.
    
    new_particles = zeros(num_particles, size(particles, 2));
    
    cdf = cumsum(weights);
    cdf(end) = 1.0; 
    
    step = 1 / num_particles;
    current_tooth = rand() * step;
    
    new_idx = 1;
    idx = 1;
    num_input_particles = length(weights);
    
    while new_idx <= num_particles
        if cdf(idx) >= current_tooth
            current_tooth = current_tooth + step;
            new_particles(new_idx, :) = particles(idx, :);
            new_idx = new_idx + 1;
        else
            idx = min(idx + 1, num_input_particles); 
        end
    end
end