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
obj.features = extracted_features;

figure(1), clf, hold on;
plot(obj)
for k = 1:size(extracted_features, 1)
    p1 = obj.cartesian(extracted_features(k, 1), :);
    p2 = obj.cartesian(extracted_features(k, 2), :);
    plot([p1(1), p2(1)], [p1(2), p2(2)], 'o-r', 'LineWidth', 1);
end
