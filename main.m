%% Initialization
close all;
clear;  
clc;

addpath('Data')
addpath('Scripts')
addpath('Functions')

% Load the configuration parameters for the algorithms
config

% Load the data
load_data

% Plot the raw data
plot_animation = false; % flat used to show (or not) an animation of the pointcloud
plot_raw_data