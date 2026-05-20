function cluster_ids = cluster_particles(particles, eps, min_neighbours)
    N = size(particles, 1);
    cluster_ids = zeros(N, 1);
    visited = false(N, 1);
    cluster_num = 0;
    
    for i = 1:N
        if visited(i)
            continue;
        end
        visited(i) = true;
        
        neighbors = region_query(particles, i, eps);
        
        if length(neighbors) < min_neighbours
            cluster_ids(i) = -1; % Noise
        else
            cluster_num = cluster_num + 1;
            cluster_ids(i) = cluster_num;
            
            k = 1;
            while k <= length(neighbors)
                j = neighbors(k);
                
                if ~visited(j)
                    visited(j) = true;
                    new_neighbours = region_query(particles, j, eps);
                    
                    if length(new_neighbours) >= min_neighbours
                        neighbors = unique([neighbors, new_neighbours]); 
                    end
                end
                
                if cluster_ids(j) == 0
                    cluster_ids(j) = cluster_num;
                end
                k = k + 1;
            end
        end
    end
end

function neighbours = region_query(particles, i, eps)
    from = particles(i, :); 
    w_dir = 0.8;

    dx = particles(:, 1) - from(1);
    dy = particles(:, 2) - from(2);
    ddir = atan2(sin(particles(:, 3) - from(3)), cos(particles(:, 3) - from(3)));

    distances = sqrt(dx.^2 + dy.^2 + (w_dir * ddir).^2);
    neighbours = find(distances < eps)'; 
end