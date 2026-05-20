function [weights] = weight_particles(particle_measurements, lidar_distances, public_vars)
    % Evaluates particles by comparing predicted and actual sensor data
    N = size(particle_measurements, 1);
    sigma = public_vars.sensor_noise;
    weights = zeros(N, 1);
    
    % Базовая вероятность (Uniform random noise)
    % Предотвращает underflow и делает фильтр устойчивым к динамическим препятствиям
    base_prob = 0.01; 
    
    for i = 1:N
        errors = lidar_distances - particle_measurements(i, :);
        
        % Считаем Гауссиану (от 0 до 1) для каждого луча
        p_beams = exp(-errors.^2 / (2 * sigma^2));
        
        % Смешиваем идеальную модель с базовым шумом
        p_beams = p_beams + base_prob;
        
        % Совокупный вес - это ПРОИЗВЕДЕНИЕ вероятностей 8 лучей
        weights(i) = prod(p_beams);
    end
end