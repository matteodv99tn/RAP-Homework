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

ls = laserscans{8200};

obj = Laserscan(ls.r, (ls.Theta) .* 3.14 / 180);

extracted_features = obj.extract_feature();

disp(size(extracted_features, 1));



