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

ls  = laserscans{1};                                % choose a laserscan 
obj = Laserscan(ls.r, (ls.Theta) .* 3.14 / 180);    % build the object

extracted_features = obj.extract_feature();         % extract the features

%figure(1), clf, hold on;
%plot(obj)

obj.features = extracted_features;
figure(2), clf, hold on;
plot(obj);
