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


%% Test extract_features
n_scan      = 1;
data        = zeros(2, 361);
data(1,:)   = laserscans{n_scan}.xscan(:);
data(2,:)   = laserscans{n_scan}.yscan(:);
R           = [0, -1; 1, 0];
data        = R * data;


%for i = 1:N_laserscans
%    extracted_feature = extract_feature(laserscans{i},conf);
% end
extracted_feature = extract_feature(laserscans{n_scan},conf);
figure(4), clf, hold on;
plot(data(1,:), data(2,:), '.');

plot([-1, 0, 1 -1], [-1, 2, -1, -1], 'r');
plot(0, 0, 'or');
title(['Laserscan ' num2str(n_scan) ' - time ', num2str(laserscans{n_scan}.t)]);

set(gca,'DataAspectRatio',[1 1 1]);
grid on;