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

% scan = laserscans{1};
% scan.extract_feature();     
% 
% figure(1), clf, hold on;
% plot(scan)
