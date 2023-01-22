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

ls1  = laserscans{index};
ls2  = laserscans{index+1};
odo1 = odometries{index};
odo2 = odometries{index+1};

alpha = svd_estimation(ls1, ls2)

odo1.dtheta
odo2.dtheta