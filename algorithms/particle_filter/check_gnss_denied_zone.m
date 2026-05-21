function is_inside = check_gnss_denied_zone(x, y, gnss_denied)
    is_inside = false;

    for k = 1:size(gnss_denied,1)
        xv = gnss_denied(k,1:2:end);
        yv = gnss_denied(k,2:2:end);

        if inpolygon(x, y, xv, yv)
            is_inside = true;
            return;
        end
    end
end