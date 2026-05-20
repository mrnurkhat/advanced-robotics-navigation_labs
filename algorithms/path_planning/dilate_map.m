function dilated_map = dilate_map(map)
    [rows, cols] = size(map);
    dilated_map = zeros(rows, cols);

    dilated_map(map ~= 0) = 100;
    [wall_y, wall_x] = find(map ~= 0);
    
    radiuses = [1, 2, 3, 4];
    costs = [100, 75, 50, 25]; 
    
    for r_idx = length(radiuses):-1:1 
        R = radiuses(r_idx);
        current_cost = costs(r_idx);
        
        mask = [];
        for dj = -R:R
            for di = -R:R
                if hypot(dj, di) <= R
                    mask = [mask; dj, di];
                end
            end
        end

        for k = 1:length(wall_x)
            nj = wall_y(k) + mask(:,1);
            ni = wall_x(k) + mask(:,2);
            
            valid = (nj >= 1 & nj <= rows & ni >= 1 & ni <= cols);
            idx = nj(valid) + (ni(valid) - 1) * rows;

            dilated_map(idx) = max(dilated_map(idx), current_cost);
        end
    end
end