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
load_data


% Plot the raw data
plot_animation = true; % flat used to show (or not) an animation of the pointcloud
pre_compute_features = true; % flag used to pre-compute the features (or not)
plot_raw_data
