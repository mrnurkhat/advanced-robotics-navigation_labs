function free = is_free_space(point, read_only_vars, dilated_grid, threshold)
    [row, col] = world2Grid(point(1), point(2), read_only_vars);

    if dilated_grid(row, col) > threshold
        free = false;
        return;
    end
    
    free = true;
end