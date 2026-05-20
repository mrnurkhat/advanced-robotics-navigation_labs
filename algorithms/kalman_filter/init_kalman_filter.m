function [public_vars] = init_kalman_filter(read_only_vars, public_vars)
%INIT_KALMAN_FILTER Summary of this function goes here

gnss_mean = mean(read_only_vars.gnss_history);
gnss_cov = cov(read_only_vars.gnss_history);
initial_dir_uncertainty = 10;

public_vars.mu = [gnss_mean(1); gnss_mean(2); 0];
public_vars.sigma = blkdiag(gnss_cov, initial_dir_uncertainty);

public_vars.kf.C = [1 0 0; 0 1 0];
public_vars.kf.R = diag([3e-5, 3e-5, 3e-5]);
public_vars.kf.Q = gnss_cov;

end

