%% Initialization
close all;  
clc;

addpath('Data')
addpath('Scripts')
addpath('Functions')
addpath('Classes')

% Load the configuration parameters for the algorithms
config

% Load the data
% load_precomputed_data = true;
% load_data


%% SVD estimation
clc;
index = 14000;
delta_index = 1;

ls1  = laserscans{index};
ls2  = laserscans{index+delta_index};

dtheta_tot = 0;

for i = index:index+delta_index
    dtheta_tot = dtheta_tot + odometries{i}.dtheta;
end

alpha = svd_estimation(ls1, ls2);
fprintf("SVD estimation = %f\n", alpha);
fprintf("sum of odometries = %f\n\n", dtheta_tot);

dtheta_tot = zeros(length(laserscans)-delta_index,1);

alphas = [];
alphas_idx = [];
alpha_old = 0;
for i = 1:delta_index:length(laserscans)-delta_index
    ls1  = laserscans{i};
    ls2  = laserscans{i+delta_index};

    alpha = wrapToPi(svd_estimation(ls1, ls2));
    alphas = [alphas, alpha_old + alpha];
    alphas_idx = [alphas_idx, i];
    alpha_old = alpha_old + alpha;
end

figure(), clf, hold on;
plot(alphas_idx, alphas);
plot(th);
plot(gt(:, 4));
legend(['SVD estimate'; 'EKF estimate'; 'Ground truth']);
grid on;
xlabel('time index');
ylabel('theta [rad]');

% plot(1:1:length(alpha),alpha);

% hold on
% plot(1:1:length(alpha),dtheta_tot);