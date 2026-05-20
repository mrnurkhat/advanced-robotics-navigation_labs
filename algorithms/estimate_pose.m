function [estimated_pose, public_vars] = estimate_pose(public_vars, read_only_vars)
    if ~any(isnan(read_only_vars.gnss_position), 'all')
        estimated_pose = [public_vars.mu(1), public_vars.mu(2), public_vars.mu(3)];
        return;
    end
    
    particles = public_vars.particles;
    weights   = public_vars.weights;
    
    cluster_ids = particles(:, 4);
    valid_clusters = unique(cluster_ids(cluster_ids > 0));
    
    if isempty(valid_clusters)
        estimated_pose = compute_weighted_mean(particles, weights);
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
    
    % 2. ГИСТЕРЕЗИС (Липкая гипотеза)
    % Проверяем, была ли у нас стабильная поза на прошлом шаге
    if isfield(public_vars, 'last_stable_pose') && ~isempty(public_vars.last_stable_pose)
        last_pose = public_vars.last_stable_pose;
        
        % Ищем кластер, физически ближайший к прошлой позе
        distances = sqrt((cluster_centers(:,1) - last_pose(1)).^2 + ...
                         (cluster_centers(:,2) - last_pose(2)).^2);
                         
        [min_dist, closest_idx] = min(distances);
        
        % Если ближайший кластер находится в пределах разумного радиуса (например, 1 метр)
        % И его вес не совсем мусорный (например, больше 10% от общей суммы)
        total_w = sum(cluster_weights);
        if min_dist < 1.0 && (cluster_weights(closest_idx) / total_w) > 0.1
            % ОСТАЕМСЯ НА ТЕКУЩЕЙ ГИПОТЕЗЕ, даже если она не максимальная по весу!
            best_idx = closest_idx;
        else
            % Если старый кластер исчез или проиграл с разгромом, берем глобальный максимум
            [~, best_idx] = max(cluster_weights);
        end
    else
        % Если истории нет, берем максимальный кластер
        [~, best_idx] = max(cluster_weights);
    end
    
    % 3. Финальная оценка и сохранение
    estimated_pose = cluster_centers(best_idx, :);
    public_vars.last_stable_pose = estimated_pose; % Запоминаем для следующего кадра
end

% Вспомогательная функция, чтобы не дублировать код
function mean_pose = compute_weighted_mean(p, w)
    x = sum(w .* p(:,1));
    y = sum(w .* p(:,2));
    dir = atan2(sum(w .* sin(p(:,3))), sum(w .* cos(p(:,3))));
    mean_pose = [x, y, dir];
end