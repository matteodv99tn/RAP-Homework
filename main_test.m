%% Initialization
close all;
clear;  
clc;

addpath('Data')
addpath('Scripts')
addpath('Functions')
addpath('Classes')

% Load the configuration parameters for the algorithms
config

% Load the data
load_precomputed_data = true;
load_data


%% SVD estimation
clc;
index = 14000;
delta_index = 5;

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

for i = 1:length(laserscans) - delta_index
    ls1  = laserscans{i};
    ls2  = laserscans{i+delta_index};
    index = i;
    for j = index:index+delta_index
        dtheta_tot(i) = dtheta_tot(i) + odometries{j}.dtheta; 
    end

    alpha(i) = svd_estimation(ls1, ls2);

end

plot(1:1:length(alpha),alpha);
hold on
plot(1:1:length(alpha),dtheta_tot);