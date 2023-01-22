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
index = 4000;
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