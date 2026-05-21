function [estimated_pose, public_vars] = estimate_pose(public_vars, read_only_vars)
    % Fallback по умолчанию, если ни одна система еще не активна
    estimated_pose = [0, 0, 0];

    % --- 1. РЕЖИМ УЛИЦЫ (Kalman Filter) ---
    if strcmp(public_vars.active_system, 'KF')
        estimated_pose = [public_vars.mu(1), public_vars.mu(2), public_vars.mu(3)];
        
        % ВАЖНЫЙ ТРЮК: Запоминаем позу KF как "стабильную" для PF. 
        % Это обеспечит идеальный бесшовный переход при въезде в здание!
        public_vars.last_stable_pose = estimated_pose; 
        return;
    end
    
    % --- 2. РЕЖИМ ПОМЕЩЕНИЯ (Particle Filter) ---
    if strcmp(public_vars.active_system, 'PF')
        particles = public_vars.particles;
        weights   = public_vars.weights;
        
        cluster_ids = particles(:, 4);
        valid_clusters = unique(cluster_ids(cluster_ids > 0));
        
        if isempty(valid_clusters)
            estimated_pose = compute_weighted_mean(particles, weights);
            public_vars.last_stable_pose = estimated_pose;
            return;
        end
        
        num_c = length(valid_clusters);
        cluster_centers = zeros(num_c, 3);
        cluster_weights = zeros(num_c, 1);
        
        for k = 1:num_c
            c_id = valid_clusters(k);
            mask = (cluster_ids == c_id);
            c_particles = particles(mask, :);
            c_weights = weights(mask);
            
            cluster_weights(k) = sum(c_weights);
            
            % Нормализуем для локального среднего
            norm_w = c_weights / cluster_weights(k);
            cluster_centers(k, :) = compute_weighted_mean(c_particles, norm_w);
        end
        
        best_idx = 1;
        
        % ГИСТЕРЕЗИС (Липкая гипотеза)
        if isfield(public_vars, 'last_stable_pose') && ~isempty(public_vars.last_stable_pose)
            last_pose = public_vars.last_stable_pose;
            
            distances = sqrt((cluster_centers(:,1) - last_pose(1)).^2 + ...
                             (cluster_centers(:,2) - last_pose(2)).^2);
                             
            [min_dist, closest_idx] = min(distances);
            
            total_w = sum(cluster_weights);
            if min_dist < 1.0 && (cluster_weights(closest_idx) / total_w) > 0.1
                best_idx = closest_idx;
            else
                [~, best_idx] = max(cluster_weights);
            end
        else
            [~, best_idx] = max(cluster_weights);
        end
        
        % Финальная оценка и сохранение
        estimated_pose = cluster_centers(best_idx, :);
        public_vars.last_stable_pose = estimated_pose;
        return;
    end
    
    % --- 3. РЕЖИМ ОЖИДАНИЯ ('WAITING_GNSS' или 'NONE') ---
    % Возвращаем последнюю известную позу, чтобы робот не "телепортировался" в (0,0)
    if isfield(public_vars, 'last_stable_pose') && ~isempty(public_vars.last_stable_pose)
        estimated_pose = public_vars.last_stable_pose;
    end
end

function mean_pose = compute_weighted_mean(p, w)
    x = sum(w .* p(:,1));
    y = sum(w .* p(:,2));
    dir = atan2(sum(w .* sin(p(:,3))), sum(w .* cos(p(:,3))));
    mean_pose = [x, y, dir];
end